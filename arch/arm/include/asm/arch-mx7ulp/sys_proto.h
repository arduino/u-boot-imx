/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 */

#ifndef _SYS_PROTO_MX7ULP_H_
#define _SYS_PROTO_MX7ULP_H_

#include <asm/mach-imx/sys_proto.h>

#define BT0CFG_LPBOOT_MASK 0x1
#define BT0CFG_DUALBOOT_MASK 0x2

enum bt_mode {
	LOW_POWER_BOOT,		/* LP_BT = 1 */
	DUAL_BOOT,		/* LP_BT = 0, DUAL_BT = 1 */
	SINGLE_BOOT		/* LP_BT = 0, DUAL_BT = 0 */
};

enum boot_device get_boot_device(void);

#define M4_BOOT_REG		(SIM0_RBASE + 0x70)
#define M4_BASE			(TCML_BASE)
#define M4_SIZE			(SZ_128K + SZ_64K)
#define M4_ENTRY_OFFSET		0x1004
#define M4_WATERMARK_OFFSET	0x1000
#define M4_WATERMARK(fw)						\
	({								\
		const char *__p = (const char *) (fw);			\
		u32 __watermark = *(u32 *) (__p + M4_WATERMARK_OFFSET);	\
		__watermark;						\
	})

#define M4_ENTRY(fw)							\
	({								\
		const char *__p = (const char *) (fw);			\
		u32 __entry = *(u32 *) (__p + M4_ENTRY_OFFSET);		\
		__entry;						\
	})

#define M4_FW_VALID(x) (((x) == 0x402000d1) || ((x) == 0x412000d1))


int boot_mode_getprisec(void);
void boot_mode_enable_secondary(bool enable);
int boot_mode_is_closed(void);

#endif /* _SYS_PROTO_MX7ULP_H_ */
