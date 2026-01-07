#include "../../../../sdk/sqrm_sdk.h"
#include "vmsvga.h"

// Minimal VMSVGA SQRM GPU module
//
// IMPORTANT: this is an initial bring-up driver for VMs.
// It targets VirtualBox (Graphics Controller = VMSVGA) and VMware.
//
// Strategy:
//  1) Find PCI device 15ad:0405
//  2) Enable IO + MEM + bus mastering
//  3) Use IO BAR to talk to SVGA registers
//  4) Use MMIO/VRAM BAR as linear framebuffer
//  5) Program mode: 1024x768x32
//  6) Implement flush() by submitting FIFO UPDATE

static const sqrm_kernel_api_t *g_api;
static pci_device_t *g_pci;

static uint16_t g_io_base = 0;
static volatile uint32_t *g_fb = 0;
static volatile uint32_t *g_fifo = 0;
static uint32_t g_fifo_words = 0;

static sqrm_gpu_device_t g_dev;

static void com(const char *s) {
    // SQRM API uses raw port numbers; COM1 is 0x3F8.
    if (g_api && g_api->com_write_string) g_api->com_write_string(0x3F8, s);
}

static inline void svga_out(uint32_t index, uint32_t value) {
    g_api->outl((uint16_t)(g_io_base + SVGA_INDEX_PORT_OFF), index);
    g_api->outl((uint16_t)(g_io_base + SVGA_VALUE_PORT_OFF), value);
}

static inline uint32_t svga_in(uint32_t index) {
    g_api->outl((uint16_t)(g_io_base + SVGA_INDEX_PORT_OFF), index);
    return g_api->inl((uint16_t)(g_io_base + SVGA_VALUE_PORT_OFF));
}

static void com_hex32(uint32_t v) {
    if (!g_api || !g_api->com_write_string) return;
    char buf[11];
    buf[0] = '0'; buf[1] = 'x';
    const char *h = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++) {
        buf[2+i] = h[(v >> (28 - 4*i)) & 0xF];
    }
    buf[10] = 0;
    g_api->com_write_string(0x3F8, buf);
}

static int svga_negotiate_id(void) {
    uint32_t before = svga_in(SVGA_REG_ID);
    com("[VMSVGA] REG_ID initial="); com_hex32(before); com("\n");

    // Try highest supported ID, fall back.
    uint32_t ids[] = { SVGA_ID_2, SVGA_ID_1, SVGA_ID_0 };
    for (int i = 0; i < 3; i++) {
        com("[VMSVGA] write REG_ID="); com_hex32(ids[i]); com("\n");
        svga_out(SVGA_REG_ID, ids[i]);
        uint32_t id = svga_in(SVGA_REG_ID);
        com("[VMSVGA] read  REG_ID="); com_hex32(id); com("\n");

        // Accept exact match OR any supported ID in the expected range.
        if (id == ids[i]) return 0;
        if (id >= SVGA_ID_0 && id <= SVGA_ID_2) return 0;
    }

    return -1;
}

static void svga_wait_for_fifo(void) {
    // Basic sync
    svga_out(SVGA_REG_SYNC, 1);
    while (svga_in(SVGA_REG_BUSY)) {
        // spin
    }
}

static int fifo_init(void) {
    if (!g_fifo || g_fifo_words < 16) return -1;

    // SVGA FIFO pointers are BYTE OFFSETS from the start of the FIFO region.
    // Reserve first 16 dwords (64 bytes) for FIFO registers.
    uint32_t min = 16 * 4;
    uint32_t max = g_fifo_words * 4;

    g_fifo[SVGA_FIFO_MIN] = min;
    g_fifo[SVGA_FIFO_MAX] = max;
    g_fifo[SVGA_FIFO_NEXT_CMD] = min;
    g_fifo[SVGA_FIFO_STOP] = min;
    return 0;
}

static void fifo_write(const void *src, uint32_t bytes) {
    if (!g_fifo || bytes == 0) return;

    // FIFO pointers are byte offsets.
    uint32_t next = g_fifo[SVGA_FIFO_NEXT_CMD];
    uint32_t stop = g_fifo[SVGA_FIFO_STOP];
    uint32_t min = g_fifo[SVGA_FIFO_MIN];
    uint32_t max = g_fifo[SVGA_FIFO_MAX];

    // Very simple space handling: if not enough room, sync until FIFO is drained.
    // (Correct solution would check space with wrap handling.)
    while (stop != next) {
        svga_wait_for_fifo();
        stop = g_fifo[SVGA_FIFO_STOP];
    }

    const uint32_t *dw = (const uint32_t*)src;
    uint32_t count = bytes / 4;

    for (uint32_t i = 0; i < count; i++) {
        volatile uint32_t *dest = (volatile uint32_t *)((volatile uint8_t*)g_fifo + next);
        *dest = dw[i];
        next += 4;
        if (next >= max) next = min;
    }

    g_fifo[SVGA_FIFO_NEXT_CMD] = next;
}

static void vmsvga_flush(const framebuffer_t *fb, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    (void)fb;
    if (!g_fifo) return;

    static int printed = 0;
    if (!printed) {
        printed = 1;
        com("[VMSVGA] flush called\n");
    }

    svga_fifo_update_t u;
    u.cmd = SVGA_CMD_UPDATE;
    u.x = x;
    u.y = y;
    u.w = w;
    u.h = h;

    fifo_write(&u, sizeof(u));

    // Avoid synchronizing on every update; this can be extremely slow on some hypervisors.
    // The host will process FIFO commands asynchronously.
    static uint32_t sync_backoff = 0;
    sync_backoff++;
    if ((sync_backoff & 0x3Fu) == 0) {
        svga_wait_for_fifo();
    }
}

static int set_mode_1024_768_32(void) {
    // Disable while programming
    svga_out(SVGA_REG_ENABLE, 0);

    svga_out(SVGA_REG_WIDTH, 1024);
    svga_out(SVGA_REG_HEIGHT, 768);
    svga_out(SVGA_REG_BITS_PER_PIXEL, 32);
    svga_out(SVGA_REG_DEPTH, 32);

    svga_out(SVGA_REG_ENABLE, 1);
    svga_out(SVGA_REG_CONFIG_DONE, 1);

    uint32_t bpl = svga_in(SVGA_REG_BYTES_PER_LINE);
    if (!bpl) bpl = 1024 * 4;

    // Setup framebuffer descriptor
    g_dev.fb.addr = (void*)g_fb;
    g_dev.fb.width = 1024;
    g_dev.fb.height = 768;
    g_dev.fb.pitch = bpl;
    g_dev.fb.bpp = 32;
    g_dev.fb.fmt = FB_FMT_UNKNOWN;
    g_dev.fb.red_pos = 16; g_dev.fb.red_mask_size = 8;
    g_dev.fb.green_pos = 8; g_dev.fb.green_mask_size = 8;
    g_dev.fb.blue_pos = 0; g_dev.fb.blue_mask_size = 8;

    return 0;
}

static sqrm_module_desc_t sqrm_module_desc = {
    .abi_version = 1,
    .type = SQRM_TYPE_GPU,
    .name = "vmsvga",
};

int sqrm_module_init(const sqrm_kernel_api_t *api) {
    g_api = api;
    if (!api || api->abi_version != 1) return -1;

    if (!api->pci_find_device || !api->outl || !api->inl) {
        com("[VMSVGA] Missing PCI or IO port capabilities in SQRM API\n");
        return -1;
    }

    g_pci = api->pci_find_device(VMSVGA_VENDOR_VMWARE, VMSVGA_DEVICE_SVGA2);
    if (!g_pci) {
        com("[VMSVGA] Device 15ad:0405 not found\n");
        return -1;
    }

    com("[VMSVGA] Found VMware SVGA II\n");

    // Enable IO space BEFORE touching SVGA index/value ports (BAR0).
    api->pci_enable_io_space(g_pci);
    api->pci_enable_memory_space(g_pci);
    api->pci_enable_bus_mastering(g_pci);
    com("[VMSVGA] PCI IO space enabled\n");

    // Dump BAR table for debugging (VBox vs VMware differences)
    com("[VMSVGA] PCI BAR table:\n");
    for (int bi = 0; bi < 6; bi++) {
        com("[VMSVGA]  BAR");
        char d[2] = { (char)('0' + bi), 0 };
        com(d);
        com(": ");
        if (g_pci->bar[bi] == 0) {
            com("<none>\n");
            continue;
        }
        if (g_pci->bar_type[bi] == PCI_BAR_IO) {
            com("IO base=");
            com_hex32((uint32_t)(g_pci->bar[bi] & ~0x3));
        } else {
            com("MEM base=");
            com_hex32((uint32_t)(g_pci->bar[bi] & ~0xFULL));
        }
        com(" size=");
        com_hex32((uint32_t)g_pci->bar_size[bi]);
        com("\n");
    }


    // BAR0 is usually IO (SVGA registers)
    if (g_pci->bar_type[0] != PCI_BAR_IO) {
        com("[VMSVGA] Unexpected BAR0 type (expected IO)\n");
        return -1;
    }
    g_io_base = (uint16_t)(g_pci->bar[0] & ~0x3);
    com("[VMSVGA] IO base=0x");
    if (g_api && g_api->com_write_string) {
        // tiny hex print
        char hx[8] = {0};
        const char *hex = "0123456789ABCDEF";
        hx[0] = hex[(g_io_base >> 12) & 0xF];
        hx[1] = hex[(g_io_base >> 8) & 0xF];
        hx[2] = hex[(g_io_base >> 4) & 0xF];
        hx[3] = hex[(g_io_base >> 0) & 0xF];
        hx[4] = 0;
        g_api->com_write_string(0x3F8, hx);
        g_api->com_write_string(0x3F8, "\n");
    }

    // Touch a register to "wake" device on some hypervisors
    (void)svga_in(SVGA_REG_CAPABILITIES);

    if (svga_negotiate_id() != 0) {
        com("[VMSVGA] SVGA ID negotiation failed\n");
        return -1;
    }

    // BAR1 is VRAM aperture (framebuffer)
    if (g_pci->bar_type[1] == PCI_BAR_IO) {
        com("[VMSVGA] Unexpected BAR1 type (expected MEM)\n");
        return -1;
    }

    uint64_t bar1_phys = (uint64_t)(g_pci->bar[1] & ~0xFULL);
    uint64_t bar1_size = (uint64_t)g_pci->bar_size[1];
    if (!bar1_size) bar1_size = 16 * 1024 * 1024;

    void *bar1 = api->ioremap_guarded ? api->ioremap_guarded(bar1_phys, bar1_size) : api->ioremap(bar1_phys, bar1_size);
    if (!bar1) {
        com("[VMSVGA] Failed to map BAR1\n");
        return -1;
    }

    // Framebuffer base is BAR1 + FB_OFFSET
    uint32_t fb_off = svga_in(SVGA_REG_FB_OFFSET);
    com("[VMSVGA] FB_OFFSET="); com_hex32(fb_off); com("\n");
    if ((uint64_t)fb_off >= bar1_size) {
        com("[VMSVGA] FB_OFFSET outside BAR1\n");
        return -1;
    }
    g_fb = (volatile uint32_t*)((uint8_t*)bar1 + fb_off);

    // FIFO setup:
    // Preferred: SVGA_REG_MEM_START/MEM_SIZE describes a dedicated FIFO region.
    // If missing, fall back to scanning PCI memory BARs and force-initializing FIFO there.

    uint32_t mem_start = svga_in(SVGA_REG_MEM_START);
    uint32_t mem_size  = svga_in(SVGA_REG_MEM_SIZE);
    com("[VMSVGA] MEM_START="); com_hex32(mem_start); com(" MEM_SIZE="); com_hex32(mem_size); com("\n");

    auto int use_fifo_region(void *mf, uint64_t bytes, const char *where) {
        if (!mf || bytes < 4096) return -1;
        g_fifo = (volatile uint32_t*)mf;
        g_fifo_words = (uint32_t)(bytes / 4);
        if (fifo_init() != 0) {
            g_fifo = 0;
            g_fifo_words = 0;
            return -1;
        }
        com("[VMSVGA] FIFO initialized via ");
        com(where);
        com("\n");
        // Tell hardware FIFO config is ready
        svga_out(SVGA_REG_CONFIG_DONE, 1);
        return 0;
    }

    // 1) MEM_START/MEM_SIZE
    if (mem_start && mem_size) {
        void *mf = api->ioremap_guarded ? api->ioremap_guarded((uint64_t)mem_start, (uint64_t)mem_size)
                                       : api->ioremap((uint64_t)mem_start, (uint64_t)mem_size);
        if (use_fifo_region(mf, mem_size, "MEM_START") != 0) {
            com("[VMSVGA] FIFO init via MEM_START failed\n");
        }
    }

    // 2) BAR scan fallback
    if (!g_fifo) {
        for (int bi = 0; bi < 6; bi++) {
            if (g_pci->bar_type[bi] == PCI_BAR_IO) continue;
            if (g_pci->bar[bi] == 0) continue;
            uint64_t phys = (uint64_t)(g_pci->bar[bi] & ~0xFULL);
            uint64_t size = (uint64_t)g_pci->bar_size[bi];
            if (!size) continue;
            if (bi == 1) continue; // skip VRAM aperture

            com("[VMSVGA] Using BAR");
            char d[2] = { (char)('0' + bi), 0 };
            com(d);
            com(" as FIFO region\n");

            void *mf = api->ioremap_guarded ? api->ioremap_guarded(phys, size) : api->ioremap(phys, size);
            char w[8] = { 'B','A','R', (char)('0'+bi), 0,0,0,0 };
            if (use_fifo_region(mf, size, w) == 0) break;
        }

        if (!g_fifo) {
            com("[VMSVGA] FIFO NOT FOUND (flush disabled)\n");
        }
    }

    g_dev.flush = g_fifo ? vmsvga_flush : NULL;
    g_dev.enumerate_modes = NULL;
    g_dev.set_mode = NULL;
    g_dev.shutdown = NULL;

    if (set_mode_1024_768_32() != 0) {
        com("[VMSVGA] Mode set failed\n");
        return -1;
    }

    // Register
    if (!api->gfx_register_framebuffer) {
        com("[VMSVGA] Missing gfx_register_framebuffer in SQRM API\n");
        return -1;
    }

    // Debug: print framebuffer descriptor we are registering
    com("[VMSVGA] registering fb addr=");
    com_hex32((uint32_t)(uintptr_t)g_dev.fb.addr);
    com(" w=");
    { char tmp[16]; uint32_t v=g_dev.fb.width; int j=0; char b[16]; if(v==0){b[j++]='0';} else {char t[16]; int k=0; while(v&&k<15){t[k++]=(char)('0'+(v%10)); v/=10;} while(k--) b[j++]=t[k];} b[j]=0; com(b);} 
    com(" h=");
    { char tmp[16]; uint32_t v=g_dev.fb.height; int j=0; char b[16]; if(v==0){b[j++]='0';} else {char t[16]; int k=0; while(v&&k<15){t[k++]=(char)('0'+(v%10)); v/=10;} while(k--) b[j++]=t[k];} b[j]=0; com(b);} 
    com(" pitch=");
    { char tmp[16]; uint32_t v=g_dev.fb.pitch; int j=0; char b[16]; if(v==0){b[j++]='0';} else {char t[16]; int k=0; while(v&&k<15){t[k++]=(char)('0'+(v%10)); v/=10;} while(k--) b[j++]=t[k];} b[j]=0; com(b);} 
    com(" bpp=");
    { char tmp[16]; uint32_t v=g_dev.fb.bpp; int j=0; char b[16]; if(v==0){b[j++]='0';} else {char t[16]; int k=0; while(v&&k<15){t[k++]=(char)('0'+(v%10)); v/=10;} while(k--) b[j++]=t[k];} b[j]=0; com(b);} 
    com("\n");

    int rc = api->gfx_register_framebuffer(&g_dev);
    if (rc == 0) {
        com("[VMSVGA] Registered framebuffer\n");

        // Clear the framebuffer once to black and let the kernel (FBCON) draw after that.
        // Direct contiguous fill (32bpp).
        volatile uint32_t *pix = (volatile uint32_t*)g_dev.fb.addr;
        uint32_t total = g_dev.fb.width * g_dev.fb.height;
        for (uint32_t i = 0; i < total; i++) {
            pix[i] = 0x00000000;
        }

        // Trigger one full-screen update
        if (g_dev.flush) g_dev.flush(&g_dev.fb, 0, 0, g_dev.fb.width, g_dev.fb.height);
        svga_wait_for_fifo();
    } else { 
        com("[VMSVGA] gfx_register_framebuffer failed\n");
    }

    return rc;
}
