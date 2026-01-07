// SPDX-License-Identifier: GPL-2.0-only
//
// ModuOS SQRM GPU module (QXL driver). This source is GPLv2 as part of ModuOS.
// It uses QXL interface headers under LICENSES/NTSoftware-QXLHeaders/mod_MIT.txt
// (LicenseRef-NTSoftware-Modified-MIT-QXLHeaders).

#include <stdint.h>
#include <stddef.h>

#include "../../../../sdk/sqrm_sdk.h"

// QXL protocol headers (provided by repo)
#include "qxl_dev.h"
#include "qxl_surface.h"
#include "qxl_mem.h"

#define COM1_PORT 0x3F8

const sqrm_module_desc_t sqrm_module_desc = {
    .abi_version = 1,
    .type = SQRM_TYPE_GPU,
    .name = "qxl_gpu",
};

// Global state for flush()
static const sqrm_kernel_api_t *g_api;
static pci_device_t *g_dev;
static uint16_t g_io_base;
static volatile QXLRom *g_rom;
static volatile QXLRam *g_ram_hdr;
static uint64_t g_ram_phys;
static uint64_t g_ram_size;
static void *g_ram_virt;

static uint64_t g_vram_phys;
static uint64_t g_vram_size;
static void *g_vram_virt;

static framebuffer_t g_fb;

static inline int tri_edge(int ax, int ay, int bx, int by, int cx, int cy) {
    return (cx-ax)*(by-ay) - (cy-ay)*(bx-ax);
}

static inline void qxl_io_write(uint32_t cmd, uint32_t val) {
    // QXL I/O registers are 32-bit and addressed as io_base + (cmd * 4).
    // NOTE: The IO BAR size printed by our PCI code is not reliable for IO sizing.
    g_api->outl((uint16_t)(g_io_base + (uint16_t)(cmd * 4u)), val);
}

static int qxl_set_mode(uint32_t width, uint32_t height, uint32_t bpp);
static int qxl_enumerate_modes(gfx_mode_t *out_modes, uint32_t max_modes);

static void qxl_flush(const framebuffer_t *fb, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    (void)fb;
    if (!g_ram_hdr) return;

    // Clamp
    if (x >= g_fb.width || y >= g_fb.height) return;
    if (w == 0 || h == 0) return;
    if (x + w > g_fb.width) w = g_fb.width - x;
    if (y + h > g_fb.height) h = g_fb.height - y;

    // QXLRam contains update_surface and update_area_rect
    g_ram_hdr->update_surface = 0;
    g_ram_hdr->update_area_rect.left = (int32_t)x;
    g_ram_hdr->update_area_rect.top = (int32_t)y;
    g_ram_hdr->update_area_rect.right = (int32_t)(x + w);
    g_ram_hdr->update_area_rect.bottom = (int32_t)(y + h);

    qxl_io_write(QXL_IO_UPDATE_AREA_ASYNC, 0);
}

static int qxl_enumerate_modes(gfx_mode_t *out_modes, uint32_t max_modes) {
    if (!g_rom || !out_modes || max_modes == 0) return -1;

    const QXLMode *modes = (const QXLMode *)((const uint8_t*)g_rom + g_rom->modes_offset);

    uint32_t n = 0;
    for (uint32_t i = 0; i < 64 && n < max_modes; i++) {
        const QXLMode *m = &modes[i];
        if (m->x_res == 0 || m->y_res == 0 || m->bits == 0) break;

        // Only expose modes we can actually set (current driver supports 32bpp only).
        if (m->bits != 32) continue;

        // Ensure the mode fits in the surface0/draw area.
        uint64_t bytes = (uint64_t)m->stride * (uint64_t)m->y_res;
        if (g_rom->surface0_area_size && bytes > (uint64_t)g_rom->surface0_area_size) continue;

        out_modes[n].width = m->x_res;
        out_modes[n].height = m->y_res;
        out_modes[n].bpp = m->bits;
        n++;
    }

    return (int)n;
}

static int map_find_regions(const sqrm_kernel_api_t *api, pci_device_t *dev,
                            volatile QXLRom **out_rom, void **out_ram_bar, void **out_vram_bar,
                            uint64_t *out_ram_phys, uint64_t *out_ram_size,
                            uint64_t *out_vram_phys, uint64_t *out_vram_size) {
    // Robust ROM detection:
    // Scan MMIO BARs and pick the one whose first u32 matches QXL_ROM_MAGIC.

    int rom_bar = -1;
    volatile QXLRom *r = NULL;

    for (int i = 0; i < 6; i++) {
        if (dev->bar_type[i] != 0) continue; // MMIO only
        if (dev->bar_size[i] < 0x1000) continue;

        uint64_t phys = (uint64_t)(dev->bar[i] & ~0xFULL);
        void *m = api->ioremap_guarded ? api->ioremap_guarded(phys, dev->bar_size[i]) : api->ioremap(phys, dev->bar_size[i]);
        if (!m) continue;

        volatile uint32_t *u = (volatile uint32_t*)m;
        if (*u == QXL_ROM_MAGIC) {
            rom_bar = i;
            r = (volatile QXLRom*)m;
            break;
        }
    }

    if (rom_bar < 0 || !r) return -1;

    // Find two large MMIO bars for RAM/VRAM
    int big[2] = {-1, -1};
    int bi = 0;
    for (int i = 0; i < 6 && bi < 2; i++) {
        if (i == rom_bar) continue;
        if (dev->bar_type[i] != 0) continue;
        if (dev->bar_size[i] >= (16u * 1024u * 1024u)) {
            big[bi++] = i;
        }
    }
    if (big[0] < 0 || big[1] < 0) return -4;

    // Map both and detect which contains QXLRam header at rom->ram_header_offset
    uint64_t phys0 = (uint64_t)(dev->bar[big[0]] & ~0xFULL);
    uint64_t phys1 = (uint64_t)(dev->bar[big[1]] & ~0xFULL);

    void *bar0 = api->ioremap_guarded ? api->ioremap_guarded(phys0, dev->bar_size[big[0]]) : api->ioremap(phys0, dev->bar_size[big[0]]);
    void *bar1 = api->ioremap_guarded ? api->ioremap_guarded(phys1, dev->bar_size[big[1]]) : api->ioremap(phys1, dev->bar_size[big[1]]);
    if (!bar0 || !bar1) return -5;

    uint32_t off = r->ram_header_offset;
    volatile QXLRam *h0 = (volatile QXLRam*)((uint8_t*)bar0 + off);
    volatile QXLRam *h1 = (volatile QXLRam*)((uint8_t*)bar1 + off);

    if (h0->magic == QXL_RAM_MAGIC) {
        *out_rom = r;
        *out_ram_bar = bar0;
        *out_vram_bar = bar1;
        *out_ram_phys = phys0;
        *out_ram_size = dev->bar_size[big[0]];
        *out_vram_phys = phys1;
        *out_vram_size = dev->bar_size[big[1]];
        return 0;
    }
    if (h1->magic == QXL_RAM_MAGIC) {
        *out_rom = r;
        *out_ram_bar = bar1;
        *out_vram_bar = bar0;
        *out_ram_phys = phys1;
        *out_ram_size = dev->bar_size[big[1]];
        *out_vram_phys = phys0;
        *out_vram_size = dev->bar_size[big[0]];
        return 0;
    }

    return -6;
}

int sqrm_module_init(const sqrm_kernel_api_t *api) {
    if (!api || api->abi_version != 1) return -1;
    g_api = api;

    if (!api->pci_find_device || !api->gfx_register_framebuffer || !api->outl || !api->ioremap) {
        if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] Missing required kernel API hooks\n");
        return -2;
    }

    pci_device_t *dev = api->pci_find_device(0x1b36, QXL_DEVICE_ID_STABLE);
    if (!dev) {
        if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] QXL PCI device not found\n");
        return -3;
    }
    g_dev = dev;

    if (api->pci_enable_memory_space) api->pci_enable_memory_space(dev);
    if (api->pci_enable_io_space) api->pci_enable_io_space(dev);
    if (api->pci_enable_bus_mastering) api->pci_enable_bus_mastering(dev);

    // IO base is on IO BAR (typically BAR3 in your QEMU logs)
    int io_bar = -1;
    for (int i = 0; i < 6; i++) {
        if (dev->bar_type[i] == 1 && dev->bar_size[i] != 0) { io_bar = i; break; }
    }
    if (io_bar < 0) {
        if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] No IO BAR found\n");
        return -4;
    }
    g_io_base = (uint16_t)(dev->bar[io_bar] & ~0x3u);

    // Map ROM + RAM + VRAM and locate RAM header
    void *ram_bar = NULL;
    void *vram_bar = NULL;
    uint64_t ram_phys = 0;
    uint64_t ram_size = 0;
    uint64_t vram_phys = 0;
    uint64_t vram_size = 0;
    volatile QXLRom *rom = NULL;
    int mr = map_find_regions(api, dev, &rom, &ram_bar, &vram_bar, &ram_phys, &ram_size, &vram_phys, &vram_size);
    if (mr != 0) {
        if (api->com_write_string) {
            api->com_write_string( COM1_PORT, "[SQRM-QXL] Failed to map/identify ROM/RAM/VRAM (rc=");
            // tiny itoa
            char b[16];
            int v = mr;
            int p = 0;
            if (v < 0) { b[p++]='-'; v = -v; }
            if (v == 0) b[p++]='0';
            char tmp[16]; int tp=0;
            while (v>0) { tmp[tp++] = '0' + (v%10); v/=10; }
            while (tp>0) b[p++] = tmp[--tp];
            b[p++] = ')'; b[p++]='\n'; b[p]=0;
            api->com_write_string( COM1_PORT, b);
        }
        return -5;
    }
    g_rom = rom;

    g_ram_hdr = (volatile QXLRam*)((uint8_t*)ram_bar + rom->ram_header_offset);

    g_ram_phys = ram_phys;
    g_ram_size = ram_size;
    g_ram_virt = ram_bar;

    g_vram_phys = vram_phys;
    g_vram_size = vram_size;
    g_vram_virt = vram_bar;

    uint8_t slot_id = rom->slots_start; // use first valid slot id

    if (api->com_write_string) {
        api->com_write_string( COM1_PORT, "[SQRM-QXL] ROM slots_start=");
        char b[8]; b[0] = '0' + (slot_id / 10); b[1] = '0' + (slot_id % 10); b[2]=0;
        api->com_write_string( COM1_PORT, b);
        api->com_write_string( COM1_PORT, " slots_end=");
        b[0] = '0' + (rom->slots_end / 10); b[1] = '0' + (rom->slots_end % 10); b[2]=0;
        api->com_write_string( COM1_PORT, b);
        api->com_write_string( COM1_PORT, " slot_gen_bits=");
        b[0] = '0' + (rom->slot_gen_bits / 10); b[1] = '0' + (rom->slot_gen_bits % 10); b[2]=0;
        api->com_write_string( COM1_PORT, b);
        api->com_write_string( COM1_PORT, " slot_id_bits=");
        b[0] = '0' + (rom->slot_id_bits / 10); b[1] = '0' + (rom->slot_id_bits % 10); b[2]=0;
        api->com_write_string( COM1_PORT, b);
        api->com_write_string( COM1_PORT, " slot_generation=");
        b[0] = '0' + (rom->slot_generation / 10); b[1] = '0' + (rom->slot_generation % 10); b[2]=0;
        api->com_write_string( COM1_PORT, b);
        api->com_write_string( COM1_PORT, "\n");
    }

    // Register a memslot covering VRAM so the device can translate QXL addresses.
    // IMPORTANT: QXLRam.mem_slot_start/end are *guest physical addresses*.
    // Place the slot descriptor right after the QXLRam header in the RAM BAR.
    uint64_t slot_off = (uint64_t)rom->ram_header_offset + (uint64_t)sizeof(QXLRam);
    slot_off = (slot_off + 7ULL) & ~7ULL;

    QXLMemSlot *slot = (QXLMemSlot*)((uint8_t*)ram_bar + slot_off);

    // memslot covers RAM (where surface0/draw area lives)
    slot->mem_start = g_ram_phys;
    slot->mem_end = g_ram_phys + g_ram_size;
    slot->generation = (uint64_t)rom->slot_generation;
    slot->high_bits = 0;

    // Tell device where the memslot table lives (guest physical)
    g_ram_hdr->mem_slot_start = ram_phys + slot_off;
    g_ram_hdr->mem_slot_end = ram_phys + slot_off + sizeof(QXLMemSlot);

    // Use async variant and wait for IO_CMD interrupt flag.
    g_ram_hdr->int_pending = 0;
    g_ram_hdr->int_mask = QXL_INTERRUPT_IO_CMD;
    qxl_io_write(QXL_IO_MEMSLOT_ADD_ASYNC, (uint32_t)slot_id);
    for (volatile int spin = 0; spin < 10000000; spin++) {
        if (g_ram_hdr->int_pending & QXL_INTERRUPT_IO_CMD) break;
    }

    if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] SET_MODE NATIVE\n");
    qxl_io_write(QXL_IO_SET_MODE, QXL_MODE_NATIVE);

    // Create primary surface in the RAM draw area (surface0 area).
    // QXL ROM provides a mode table; pick the best 32bpp mode that fits surface0.

    const QXLMode *modes = (const QXLMode *)((const uint8_t*)g_rom + g_rom->modes_offset);
    uint32_t mode_count = 0;
    // modes are stored in ROM region; assume contiguous until we hit an invalid entry.
    // We cap scan to 64 to avoid runaway.
    for (uint32_t i = 0; i < 64; i++) {
        if (modes[i].x_res == 0 || modes[i].y_res == 0 || modes[i].bits == 0) break;
        mode_count++;
    }

    uint32_t width = 1024, height = 768, bpp = 32, pitch = 1024 * 4;
    uint32_t best_id = 0;
    uint64_t best_area = 0;

    for (uint32_t i = 0; i < mode_count; i++) {
        const QXLMode *m = &modes[i];
        if (m->bits != 32) continue;
        if (m->stride < m->x_res * 4) continue;

        uint64_t bytes = (uint64_t)m->stride * (uint64_t)m->y_res;
        if (g_rom->surface0_area_size && bytes > (uint64_t)g_rom->surface0_area_size) continue;

        uint64_t area = (uint64_t)m->x_res * (uint64_t)m->y_res;
        if (area > best_area) {
            best_area = area;
            best_id = m->id;
            width = m->x_res;
            height = m->y_res;
            pitch = m->stride;
        }
    }

    const uint64_t fb_bytes = (uint64_t)pitch * (uint64_t)height;

    if (api->com_write_string) {
        api->com_write_string( COM1_PORT, "[SQRM-QXL] Selected mode id=");
        char buf[16];
        int v = (int)best_id; int p = 0;
        if (v == 0) buf[p++]='0';
        else { char t[16]; int tp=0; while (v>0) { t[tp++]='0'+(v%10); v/=10; } while (tp>0) buf[p++]=t[--tp]; }
        buf[p]=0;
        api->com_write_string( COM1_PORT, buf);
        api->com_write_string( COM1_PORT, " ");
        api->com_write_string( COM1_PORT, "res=");
        // width
        v = (int)width; p = 0; if (v==0) buf[p++]='0'; else { char t[16]; int tp=0; while (v>0) { t[tp++]='0'+(v%10); v/=10; } while(tp>0) buf[p++]=t[--tp]; } buf[p]=0; api->com_write_string( COM1_PORT, buf);
        api->com_write_string( COM1_PORT, "x");
        v = (int)height; p = 0; if (v==0) buf[p++]='0'; else { char t[16]; int tp=0; while (v>0) { t[tp++]='0'+(v%10); v/=10; } while(tp>0) buf[p++]=t[--tp]; } buf[p]=0; api->com_write_string( COM1_PORT, buf);
        api->com_write_string( COM1_PORT, " pitch=");
        v = (int)pitch; p = 0; if (v==0) buf[p++]='0'; else { char t[16]; int tp=0; while (v>0) { t[tp++]='0'+(v%10); v/=10; } while(tp>0) buf[p++]=t[--tp]; } buf[p]=0; api->com_write_string( COM1_PORT, buf);
        api->com_write_string( COM1_PORT, "\n");
    }

    uint64_t fb_off = (uint64_t)rom->draw_area_offset;
    if (rom->surface0_area_size && fb_bytes > (uint64_t)rom->surface0_area_size) {
        if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] surface0_area_size too small for requested mode\n");
        return -7;
    }

    void *fb_ptr = (void*)((uint8_t*)g_ram_virt + fb_off);
    uint64_t fb_phys = g_ram_phys + fb_off;

    // Fill create_surface fields in RAM header and trigger CREATE_PRIMARY
    g_ram_hdr->create_surface_id = 0;
    g_ram_hdr->create_surface.width = width;
    g_ram_hdr->create_surface.height = height;
    g_ram_hdr->create_surface.stride = (int32_t)pitch;
    g_ram_hdr->create_surface.format = QXL_SURF_FMT_32_xRGB;
    g_ram_hdr->create_surface.position = 0;
    g_ram_hdr->create_surface.flags = 0;
    g_ram_hdr->create_surface.type = 0;
    // Primary surface memory for CREATE_PRIMARY is a guest physical address.
    g_ram_hdr->create_surface.mem = fb_phys;

    if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] CREATE_PRIMARY\n");
    // Create + attach primary using async commands and wait for completion flag.
    g_ram_hdr->int_pending = 0;
    g_ram_hdr->int_mask = QXL_INTERRUPT_IO_CMD;
    qxl_io_write(QXL_IO_CREATE_PRIMARY_ASYNC, 0);
    for (volatile int spin = 0; spin < 10000000; spin++) {
        if (g_ram_hdr->int_pending & QXL_INTERRUPT_IO_CMD) break;
    }

    g_ram_hdr->int_pending = 0;
    g_ram_hdr->int_mask = QXL_INTERRUPT_IO_CMD;
    qxl_io_write(QXL_IO_ATTACH_PRIMARY, 0);
    for (volatile int spin = 0; spin < 10000000; spin++) {
        if (g_ram_hdr->int_pending & QXL_INTERRUPT_IO_CMD) break;
    }

    // Force an update
    qxl_io_write(QXL_IO_UPDATE_AREA_ASYNC, 0);

    // Register framebuffer with kernel
    g_fb.addr = fb_ptr;
    g_fb.phys_addr = fb_phys;
    g_fb.size_bytes = fb_bytes;
    g_fb.width = width;
    g_fb.height = height;
    g_fb.pitch = pitch;
    g_fb.bpp = (uint8_t)bpp;
    g_fb.fmt = FB_FMT_XRGB8888;

    sqrm_gpu_device_t gpu = {
        .fb = g_fb,
        .flush = qxl_flush,
        .set_mode = qxl_set_mode,
        .enumerate_modes = qxl_enumerate_modes,
        .shutdown = NULL,
    };

    int rc = api->gfx_register_framebuffer(&gpu);
    if (rc != 0) {
        if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] gfx_register_framebuffer failed\n");
        return -6;
    }

    // Force one full update to ensure the first framebuffer contents become visible.
    qxl_flush(&g_fb, 0, 0, width, height);

    if (api->com_write_string) api->com_write_string( COM1_PORT, "[SQRM-QXL] Primary surface registered\n");
    return 0;
}
