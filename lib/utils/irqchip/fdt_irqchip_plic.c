/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <libfdt.h>
#include <sbi/riscv_asm.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_irqchip.h>
#include <sbi/sbi_domain_context.h>
#include <sbi/sbi_scratch.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/irqchip/fdt_irqchip.h>
#include <sbi_utils/mpxy/fdt_mpxy_opteed.h>
#include <sbi_utils/irqchip/plic.h>

static unsigned long plic_ptr_offset;

#define plic_get_hart_data_ptr(__scratch)				\
	sbi_scratch_read_type((__scratch), void *, plic_ptr_offset)

#define plic_set_hart_data_ptr(__scratch, __plic)			\
	sbi_scratch_write_type((__scratch), void *, plic_ptr_offset, (__plic))

static unsigned long plic_mcontext_offset;

#define plic_get_hart_mcontext(__scratch)				\
	(sbi_scratch_read_type((__scratch), long, plic_mcontext_offset) - 1)

#define plic_set_hart_mcontext(__scratch, __mctx)			\
	sbi_scratch_write_type((__scratch), long, plic_mcontext_offset, (__mctx) + 1)

static unsigned long plic_scontext_offset;

#define plic_get_hart_scontext(__scratch)				\
	(sbi_scratch_read_type((__scratch), long, plic_scontext_offset) - 1)

#define plic_set_hart_scontext(__scratch, __sctx)			\
	sbi_scratch_write_type((__scratch), long, plic_scontext_offset, (__sctx) + 1)

struct plic_sec_irq_record {
	u32 irq;
	u32 sec;
	u32 ws;
	u32 mctx;
	u32 hartid;
	u32 pending;
	u64 count;
};

static struct plic_sec_irq_record sec_irq_records[SBI_HARTMASK_MAX_BITS];

struct plic_secure_cfg {
	u32 *irqs;
	u32 count;
};

static struct plic_secure_cfg plic_secure_cfg;

static void plic_sec_dump_record(const char *tag,
				    const struct plic_sec_irq_record *rec)
{
	if (!rec)
		return;

	sbi_printf("plic-sec: %s irq=%u pending=%u count=%llu ws=%u sec=%u mctx=%u hart=%u\n",
		   tag, rec->irq, rec->pending,
		   (unsigned long long)rec->count, rec->ws, rec->sec,
		   rec->mctx, rec->hartid);
}

static int plic_secure_irqfn(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	long mctx = plic_get_hart_mcontext(scratch);
	u32 irq, ws, sec;
	u32 hartid = current_hartid();
	u32 hartindex = sbi_hartid_to_hartindex(hartid);
	struct plic_sec_irq_record *rec = NULL;
	unsigned long fiq_entry;
	struct sbi_domain *tdomain;

	if (!plic || mctx < 0)
		return SBI_ENODEV;

	irq = plic_claim(plic, mctx);
	if (!irq)
		return SBI_ENOENT;

	sec = plic_get_sec_src(plic, irq);
	ws = plic_get_world_state(plic, mctx);

	if (sec && hartindex < SBI_HARTMASK_MAX_BITS) {
		rec = &sec_irq_records[hartindex];
		rec->irq = irq;
		rec->sec = sec;
		rec->ws = ws;
		rec->mctx = mctx;
		rec->hartid = hartid;
		rec->pending = 1;
		rec->count++;
		sbi_printf("plic-sec: hart%u mctx=%u irq=%u sec=%u ws=%u count=%llu\n",
			   rec->hartid, rec->mctx, rec->irq, rec->sec, rec->ws,
			   (unsigned long long)rec->count);
	}

	if (!sec) {
		sbi_printf("plic-sec: irq=%u not marked secure (ws=%u) -> complete\n",
			   irq, ws);
		plic_complete(plic, mctx, irq);
		return SBI_ENOENT;
	}

	tdomain = opteed_get_tdomain();
	fiq_entry = opteed_get_fiq_entry();
	if (!tdomain || !fiq_entry) {
		sbi_printf("plic-sec: OP-TEE not ready, complete irq=%u\n", irq);
		if (rec)
			rec->pending = 0;
		plic_complete(plic, mctx, irq);
		return 0;
	}

	sbi_printf("plic-sec: enter tee fiq irq=%u entry=0x%lx\n",
		   irq, fiq_entry);
	sbi_domain_context_set_mepc(tdomain, fiq_entry);
	sbi_domain_context_enter(tdomain);
	if (rec)
		plic_sec_dump_record("exit tee call", rec);

	return 0;
}

void fdt_plic_secure_irq_complete(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	long mctx = plic_get_hart_mcontext(scratch);
	u32 hartid = current_hartid();
	u32 hartindex = sbi_hartid_to_hartindex(hartid);
	struct plic_sec_irq_record *rec;

	if (!plic || mctx < 0 || hartindex >= SBI_HARTMASK_MAX_BITS)
		return;

	rec = &sec_irq_records[hartindex];
	if (!rec->pending) {
		plic_sec_dump_record("complete called but not pending", rec);
		return;
	}

	plic_complete(plic, mctx, rec->irq);
	rec->pending = 0;

	plic_sec_dump_record("tee return -> complete", rec);

	/* Avoid interrupt storm if OP-TEE doesn't clear device IRQ */
	if (rec->count == 1) {
		plic_enable_irq(plic, mctx, rec->irq, false);
		sbi_printf("plic-sec: masked secure irq %u on mctx=%ld to stop storm\n",
			   rec->irq, mctx);
	}
}

void fdt_plic_priority_save(u8 *priority, u32 num)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	plic_priority_save(plic_get_hart_data_ptr(scratch), priority, num);
}

void fdt_plic_priority_restore(const u8 *priority, u32 num)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	plic_priority_restore(plic_get_hart_data_ptr(scratch), priority, num);
}

void fdt_plic_context_save(bool smode, u32 *enable, u32 *threshold, u32 num)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	plic_context_save(plic_get_hart_data_ptr(scratch),
			  smode ? plic_get_hart_scontext(scratch) :
				  plic_get_hart_mcontext(scratch),
			  enable, threshold, num);
}

void fdt_plic_context_restore(bool smode, const u32 *enable, u32 threshold,
			      u32 num)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	plic_context_restore(plic_get_hart_data_ptr(scratch),
			     smode ? plic_get_hart_scontext(scratch) :
				     plic_get_hart_mcontext(scratch),
			     enable, threshold, num);
}

static int irqchip_plic_warm_init(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	long mctx = plic_get_hart_mcontext(scratch);

	int rc = plic_warm_irqchip_init(plic,
					mctx,
					plic_get_hart_scontext(scratch));
	if (rc)
		return rc;

	/* Enable secure IRQs for M-mode context on this hart */
	if (plic && plic_secure_cfg.irqs && plic_secure_cfg.count && mctx >= 0) {
		for (u32 i = 0; i < plic_secure_cfg.count; i++)
			plic_enable_irq(plic, mctx, plic_secure_cfg.irqs[i], true);
		/* Allow all priorities in M-context */
		plic_set_threshold(plic, mctx, 0);
	}

	return 0;
}

static int irqchip_plic_update_hartid_table(const void *fdt, int nodeoff,
					    struct plic_data *pd)
{
	const fdt32_t *val;
	u32 phandle, hwirq, hartid;
	struct sbi_scratch *scratch;
	int i, err, count, cpu_offset, cpu_intc_offset;

	val = fdt_getprop(fdt, nodeoff, "interrupts-extended", &count);
	if (!val || count < sizeof(fdt32_t))
		return SBI_EINVAL;
	count = count / sizeof(fdt32_t);

	for (i = 0; i < count; i += 2) {
		phandle = fdt32_to_cpu(val[i]);
		hwirq = fdt32_to_cpu(val[i + 1]);

		cpu_intc_offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (cpu_intc_offset < 0)
			continue;

		cpu_offset = fdt_parent_offset(fdt, cpu_intc_offset);
		if (cpu_offset < 0)
			continue;

		err = fdt_parse_hart_id(fdt, cpu_offset, &hartid);
		if (err)
			continue;

		scratch = sbi_hartid_to_scratch(hartid);
		if (!scratch)
			continue;

		plic_set_hart_data_ptr(scratch, pd);
		switch (hwirq) {
		case IRQ_M_EXT:
			plic_set_hart_mcontext(scratch, i / 2);
			break;
		case IRQ_S_EXT:
			plic_set_hart_scontext(scratch, i / 2);
			break;
		}
	}

	return 0;
}

static int irqchip_plic_cold_init(const void *fdt, int nodeoff,
				  const struct fdt_match *match)
{
	int rc;
	struct plic_data *pd;
	const fdt32_t *val;
	int len;
	u32 secure_cnt;

	if (!plic_ptr_offset) {
		plic_ptr_offset = sbi_scratch_alloc_type_offset(void *);
		if (!plic_ptr_offset)
			return SBI_ENOMEM;
	}

	if (!plic_mcontext_offset) {
		plic_mcontext_offset = sbi_scratch_alloc_type_offset(long);
		if (!plic_mcontext_offset)
			return SBI_ENOMEM;
	}

	if (!plic_scontext_offset) {
		plic_scontext_offset = sbi_scratch_alloc_type_offset(long);
		if (!plic_scontext_offset)
			return SBI_ENOMEM;
	}

	pd = sbi_zalloc(sizeof(*pd));
	if (!pd)
		return SBI_ENOMEM;

	rc = fdt_parse_plic_node(fdt, nodeoff, pd);
	if (rc)
		goto fail_free_data;

	if (match->data) {
		void (*plic_plat_init)(struct plic_data *) = match->data;
		plic_plat_init(pd);
	}

	rc = plic_cold_irqchip_init(pd);
	if (rc)
		goto fail_free_data;

	rc = irqchip_plic_update_hartid_table(fdt, nodeoff, pd);
	if (rc)
		goto fail_free_data;

	/* Parse secure IRQ list from DT and program sec_src + priority */
	val = fdt_getprop(fdt, nodeoff, "riscv,secure-irqs", &len);
	if (val && len >= 4) {
		secure_cnt = len / sizeof(fdt32_t);
		plic_secure_cfg.irqs = sbi_zalloc(sizeof(u32) * secure_cnt);
		if (!plic_secure_cfg.irqs) {
			rc = SBI_ENOMEM;
			goto fail_free_data;
		}
		plic_secure_cfg.count = 0;
		for (u32 i = 0; i < secure_cnt; i++) {
			u32 irq = fdt32_to_cpu(val[i]);
			if (irq == 0 || irq > pd->num_src)
				continue;
			plic_secure_cfg.irqs[plic_secure_cfg.count++] = irq;
			plic_set_sec_src(pd, irq, true);
			/* Give secure IRQ a non-zero priority */
			plic_set_priority(pd, irq, 1);
		}
		if (plic_secure_cfg.count) {
			sbi_printf("plic-sec: configured %u secure irqs\n",
				   plic_secure_cfg.count);
		}
	}

	/* Register external IRQ handler for secure IRQs if configured */
	if (plic_secure_cfg.count)
		sbi_irqchip_set_irqfn(plic_secure_irqfn);

	return 0;

fail_free_data:
	sbi_free(pd);
	return rc;
}

#define THEAD_PLIC_CTRL_REG 0x1ffffc

static void thead_plic_plat_init(struct plic_data *pd)
{
	writel_relaxed(BIT(0), (char *)pd->addr + THEAD_PLIC_CTRL_REG);
}

void thead_plic_restore(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);

	thead_plic_plat_init(plic);
}

static const struct fdt_match irqchip_plic_match[] = {
	{ .compatible = "andestech,nceplic100" },
	{ .compatible = "riscv,plic0" },
	{ .compatible = "sifive,plic-1.0.0" },
	{ .compatible = "thead,c900-plic",
	  .data = thead_plic_plat_init },
	{ /* sentinel */ }
};

struct fdt_irqchip fdt_irqchip_plic = {
	.match_table = irqchip_plic_match,
	.cold_init = irqchip_plic_cold_init,
	.warm_init = irqchip_plic_warm_init,
	.exit = NULL,
};
