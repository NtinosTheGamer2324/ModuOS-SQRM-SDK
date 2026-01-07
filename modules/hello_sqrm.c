#include <stdint.h>
#include <stddef.h>

#include "../../sdk/sqrm_sdk.h"
#define COM1_PORT 0x3F8

static const sqrm_module_desc_t sqrm_module_desc = {
    .abi_version = 1,
    .type = SQRM_TYPE_USB,
    .name = "hello",
};

// Entry point called by the kernel module loader.
int sqrm_module_init(const sqrm_kernel_api_t *api) {
    if (!api || !api->com_write_string) return -1;
    api->com_write_string(COM1_PORT, "[SQRM-HELLO] hello.sqrm loaded!\n");
    return 0;
}
