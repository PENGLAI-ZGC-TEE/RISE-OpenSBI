/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#ifndef __IRQCHIP_PLIC_H__
#define __IRQCHIP_PLIC_H__

#include <sbi/sbi_types.h>

struct plic_data {
	unsigned long addr;
	unsigned long size;
	unsigned long num_src;
};

/* So far, priorities on all consumers of these functions fit in 8 bits. */
void plic_priority_save(const struct plic_data *plic, u8 *priority, u32 num);

void plic_priority_restore(const struct plic_data *plic, const u8 *priority,
			   u32 num);

void plic_set_priority(const struct plic_data *plic, u32 source, u32 val);

void plic_context_save(const struct plic_data *plic, int context_id,
		       u32 *enable, u32 *threshold, u32 num);

void plic_context_restore(const struct plic_data *plic, int context_id,
			  const u32 *enable, u32 threshold, u32 num);

int plic_context_init(const struct plic_data *plic, int context_id,
		      bool enable, u32 threshold);

void plic_set_threshold(const struct plic_data *plic, u32 context_id, u32 val);

int plic_warm_irqchip_init(const struct plic_data *plic,
			   int m_cntx_id, int s_cntx_id);

int plic_cold_irqchip_init(const struct plic_data *plic);

u32 plic_claim(const struct plic_data *plic, u32 context_id);
void plic_complete(const struct plic_data *plic, u32 context_id, u32 irq);

void plic_enable_irq(const struct plic_data *plic, u32 context_id,
		     u32 irq, bool enable);

bool plic_get_sec_src(const struct plic_data *plic, u32 irq);
void plic_set_sec_src(const struct plic_data *plic, u32 irq, bool secure);

u32 plic_get_world_state(const struct plic_data *plic, u32 context_id);
void plic_set_world_state(const struct plic_data *plic, u32 context_id, u32 ws);

#endif
