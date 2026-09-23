/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Samuel Holland <samuel@sholland.org>
 */

#ifndef __IRQCHIP_FDT_IRQCHIP_PLIC_H__
#define __IRQCHIP_FDT_IRQCHIP_PLIC_H__

#include <sbi/sbi_types.h>

/**
 * Save the PLIC priority state
 * @param priority pointer to the memory region for the saved priority
 * @param num size of the memory region including interrupt source 0
 */
void fdt_plic_priority_save(u8 *priority, u32 num);

/**
 * Restore the PLIC priority state
 * @param priority pointer to the memory region for the saved priority
 * @param num size of the memory region including interrupt source 0
 */
void fdt_plic_priority_restore(const u8 *priority, u32 num);

void fdt_plic_context_save(bool smode, u32 *enable, u32 *threshold, u32 num);

void fdt_plic_context_restore(bool smode, const u32 *enable, u32 threshold,
			      u32 num);

/* Synchronize the current hart's PLIC routing with the active S-world. */
int fdt_plic_set_current_world_state(bool tee_world);

/* Complete an M-context secure interrupt after OP-TEE returns from FIQ. */
void fdt_plic_secure_irq_complete(void);

void thead_plic_restore(void);

#endif
