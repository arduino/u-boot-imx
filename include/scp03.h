// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2021, Foundries.IO
 *
 */

#ifndef _SCP03_H
#define _SCP03_H

#include <common.h>

/* 
 * Requests to OPTEE to enable or provision the Secure Channel Protocol on its
 * Secure Element
 *
 *  If key provisioning is requested, OPTEE shall generate new SCP03 keys and
 *  write them to the Secure Element.
 */
int tee_enable_scp03(void);
int tee_provision_scp03(void);
#endif /* _SCP03_H */
