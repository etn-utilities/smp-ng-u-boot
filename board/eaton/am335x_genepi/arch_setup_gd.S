#include <config.h>
#include <asm-offsets.h>
#include <asm/arch/omap.h>
#include <asm/omap_common.h>
#include <linux/linkage.h>

/* When this is called, we do have the link register, but no BSS (so no statics and no global data). We do have a
 * stack, but we don't really need it here. */
ENTRY(arch_setup_gd)
	str	r9, [r0, #(GD_OMAP_BOOT_DEVICE_OFFSET)]
	mov	r9, r0
	mov	pc, lr
ENDPROC(arch_setup_gd)

