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
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_irqchip.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_domain_context.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/irqchip/fdt_irqchip.h>
#include <sbi_utils/irqchip/fdt_irqchip_plic.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/mpxy/fdt_mpxy_opteed.h>

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

struct plic_secure_config {
	u32 *irqs;
	u32 count;
};

struct plic_secure_irq_state {
	u32 irq;
	bool pending;
};

static struct plic_secure_config secure_config;
static struct plic_secure_irq_state secure_irq_state[SBI_HARTMASK_MAX_BITS];

/*
 * Non-secure (REE) interrupt sources that are allowed to preempt the secure
 * world. These are enabled on the M-context so that, while a hart runs in the
 * TEE world, the secure-PLIC routes them to M-mode (to_M) and lets OpenSBI
 * suspend the running secure thread. In the REE world the same sources route
 * to the S-context (to_S) and are handled by Linux directly, so enabling them
 * on the M-context is harmless there.
 */
static struct plic_secure_config preempt_config;

/*
 * Per-hart marker: the foreign-interrupt preemption path masked this hart's
 * machine external interrupt (mie.MEIE) and it must be re-enabled on the next
 * transition back to the REE world. See plic_foreign_intr_preempt() and
 * fdt_plic_set_current_world_state().
 */
static bool plic_foreign_preempt_meie_masked[SBI_HARTMASK_MAX_BITS];

/*
 * A non-secure (foreign) interrupt reached the M-context while this hart was
 * running in the secure (TEE) world, i.e. it preempted the secure world.
 *
 * Deliver a *virtual* S-mode foreign interrupt into the running OP-TEE so that
 * it suspends the current secure thread via its existing foreign-interrupt
 * handler and returns control to OpenSBI through its MPXY "complete" ecall
 * (OPTEE_ABI_RETURN_RPC_FOREIGN_INTR). OpenSBI then switches the world back to
 * REE (fdt_plic_set_current_world_state) and Linux services the interrupt.
 *
 * Mechanism and contract:
 *  - We do NOT claim the pending PLIC source. Once the world flips to REE the
 *    secure-PLIC re-routes the still-pending source to the S-context, where
 *    Linux claims it through the normal vector. The secure thread is later
 *    resumed by the standard OP-TEE "return from RPC" path.
 *  - mie.MEIE is masked until that world switch, otherwise the still-asserted
 *    MEIP would immediately re-trap to M-mode the instant we mret into the
 *    OP-TEE S-mode handler (M-mode interrupts are always enabled while the hart
 *    executes in S-mode).
 *  - We enter OP-TEE through its dedicated foreign-interrupt entry vector
 *    (opteed_get_foreign_intr_entry(), slot 9 of the OP-TEE vector table),
 *    presenting the interrupted context exactly as a hardware trap would
 *    (XEPC, XSTATUS.SPP). The entry vector synthesizes the foreign-classified
 *    cause itself, so OpenSBI does not depend on OP-TEE's native/foreign
 *    interrupt classification.
 */
static int plic_foreign_intr_preempt(struct sbi_scratch *scratch)
{
	struct sbi_trap_context *tcntx = sbi_trap_get_context(scratch);
	u32 hartindex = current_hartindex();
	unsigned long mstatus, prev_priv, entry;
	struct sbi_trap_regs *regs;

	if (!tcntx || hartindex >= SBI_HARTMASK_MAX_BITS)
		return SBI_EINVAL;
	regs = &tcntx->regs;

	/* Mask MEIE until the world switch de-routes the source from M. */
	csr_clear(CSR_MIE, MIP_MEIP);
	plic_foreign_preempt_meie_masked[hartindex] = true;

	/*
	 * Only preempt when OP-TEE is running a secure thread that can actually
	 * be suspended:
	 *   - OP-TEE explicitly unmasks foreign interrupts through sie.SEIE, and
	 *   - a kernel-mode context has global S-interrupts enabled.
	 *
	 * OP-TEE keeps sie.SEIE masked while running a fast call, on a temporary
	 * stack, in a critical section, or without an active secure thread. Checking
	 * only mstatus.SIE is insufficient because those states can still have the
	 * global S-mode interrupt bit set.
	 * Otherwise defer: leave the source pending with MEIE masked; it is
	 * delivered to Linux at the next transition to the REE world, which
	 * matches the pre-existing "deferred to REE" behaviour.
	 */
	if (!(csr_read(CSR_SIE) & MIP_SEIP))
		return 0;

	prev_priv = (regs->mstatus & MSTATUS_MPP) >> MSTATUS_MPP_SHIFT;
	if (prev_priv == PRV_S && !(regs->mstatus & MSTATUS_SIE))
		return 0;

	/* OP-TEE must have registered its entry vector table; else defer. */
	entry = opteed_get_foreign_intr_entry();
	if (!entry)
		return 0;

	/*
	 * Present the interrupted context to OP-TEE's dedicated foreign-interrupt
	 * entry vector as a hardware trap would: XEPC = interrupted PC,
	 * XSTATUS.SPP = interrupted privilege (SPIE <- SIE, SIE cleared). The
	 * entry vector itself synthesizes the foreign-classified cause, so OpenSBI
	 * stays decoupled from the platform's interrupt classification.
	 *
	 * sstatus is a view of mstatus, and the trap exit path restores mstatus
	 * from regs->mstatus before mret. The SIE/SPIE/SPP updates must therefore
	 * be applied to regs->mstatus; writing CSR_SSTATUS here would be lost.
	 * sepc is a separate CSR that the trap exit path does not touch.
	 */
	csr_write(CSR_SEPC, regs->mepc);

	mstatus = regs->mstatus;
	if (prev_priv == PRV_S)
		mstatus |= MSTATUS_SPP;
	else
		mstatus &= ~MSTATUS_SPP;
	if (mstatus & MSTATUS_SIE)
		mstatus |= MSTATUS_SPIE;
	else
		mstatus &= ~MSTATUS_SPIE;
	mstatus &= ~MSTATUS_SIE;

	/* Redirect the mret target to OP-TEE's foreign-interrupt entry vector. */
	mstatus = (mstatus & ~MSTATUS_MPP) |
		  ((unsigned long)PRV_S << MSTATUS_MPP_SHIFT);
	regs->mstatus = mstatus;
	regs->mepc = entry;

	return 0;
}

static int plic_secure_irqfn(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	struct plic_secure_irq_state *state;
	struct sbi_domain *tdomain;
	unsigned long fiq_entry;
	long mctx = plic_get_hart_mcontext(scratch);
	u32 hartindex = current_hartindex();
	u32 irq;
	int rc;

	if (!plic || mctx < 0 || hartindex >= SBI_HARTMASK_MAX_BITS)
		return SBI_ENODEV;

	/*
	 * While this hart runs in the secure (TEE) world, every machine external
	 * interrupt is - by the Nanhu secure-PLIC routing rules - a non-secure
	 * (foreign) interrupt that preempted the secure world: secure sources
	 * match the world state and are delivered to the S-context instead. Hand
	 * such interrupts to the foreign-interrupt preemption path.
	 */
	if (plic_secure_enabled(plic) && plic_get_world_state(plic, hartindex))
		return plic_foreign_intr_preempt(scratch);

	irq = plic_claim(plic, mctx);
	if (!irq)
		return SBI_ENOENT;

	if (!plic_get_sec_src(plic, irq)) {
		plic_complete(plic, mctx, irq, false);
		return 0;
	}

	state = &secure_irq_state[hartindex];
	if (state->pending) {
		plic_complete(plic, mctx, irq, true);
		return SBI_EALREADY;
	}

	state->irq = irq;
	state->pending = true;
	tdomain = opteed_get_tdomain();
	fiq_entry = opteed_get_fiq_entry();
	if (!tdomain || !fiq_entry) {
		plic_complete(plic, mctx, irq, true);
		state->pending = false;
		return 0;
	}

	rc = fdt_plic_set_current_world_state(true);
	if (rc)
		goto fail_complete;

	/* set_mepc() stores entry - 4; compensate for the FIQ vector slot. */
	rc = sbi_domain_context_set_mepc(tdomain, fiq_entry + 4);
	if (rc)
		goto fail_restore_world;
	rc = sbi_domain_context_enter(tdomain);
	if (rc)
		goto fail_restore_world;

	return 0;

fail_restore_world:
	fdt_plic_set_current_world_state(false);
fail_complete:
	plic_complete(plic, mctx, irq, true);
	state->pending = false;
	return rc;
}

void fdt_plic_secure_irq_complete(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	u32 hartindex = current_hartindex();
	struct plic_secure_irq_state *state;
	long mctx = plic_get_hart_mcontext(scratch);

	if (!plic || mctx < 0 || hartindex >= SBI_HARTMASK_MAX_BITS)
		return;

	state = &secure_irq_state[hartindex];
	if (state->pending) {
		plic_complete(plic, mctx, state->irq, true);
		state->pending = false;
	}

	fdt_plic_set_current_world_state(false);
}

int fdt_plic_set_current_world_state(bool tee_world)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct plic_data *plic = plic_get_hart_data_ptr(scratch);
	long sctx = plic_get_hart_scontext(scratch);
	u32 hartindex = current_hartindex();
	int rc;

	if (!plic || !plic_secure_enabled(plic))
		return 0;
	if (tee_world && sctx >= 0) {
		for (u32 i = 0; i < secure_config.count; i++)
			plic_enable_irq(plic, sctx, secure_config.irqs[i], true);
	}

	rc = plic_set_world_state(plic, hartindex, tee_world);

	/*
	 * On the way back to REE, re-enable the machine external interrupt that
	 * the foreign-interrupt preemption path masked. The pending non-secure
	 * source is now routed to the S-context (Linux) and no longer re-traps
	 * to M-mode, so it is safe to accept M external interrupts again (e.g.
	 * REE secure IRQs that enter OP-TEE through the FIQ path).
	 */
	if (!tee_world && hartindex < SBI_HARTMASK_MAX_BITS &&
	    plic_foreign_preempt_meie_masked[hartindex]) {
		plic_foreign_preempt_meie_masked[hartindex] = false;
		csr_set(CSR_MIE, MIP_MEIP);
	}

	return rc;
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
	int rc;

	rc = plic_warm_irqchip_init(plic, mctx,
				    plic_get_hart_scontext(scratch));
	if (rc)
		return rc;

	if (!secure_config.count || mctx < 0)
		return 0;

	for (u32 i = 0; i < secure_config.count; i++)
		plic_enable_irq(plic, mctx, secure_config.irqs[i], true);
	/*
	 * Enable the REE sources that may preempt the TEE on the M-context. The
	 * routing rules keep them invisible to M while in REE (they go to the
	 * S-context there), so this only takes effect in the TEE world.
	 */
	for (u32 i = 0; i < preempt_config.count; i++)
		plic_enable_irq(plic, mctx, preempt_config.irqs[i], true);
	plic_set_threshold(plic, mctx, 0);
	if (plic_get_hart_scontext(scratch) >= 0) {
		for (u32 i = 0; i < secure_config.count; i++)
			plic_enable_irq(plic, plic_get_hart_scontext(scratch),
					secure_config.irqs[i], true);
	}

	return plic_set_world_state(plic, current_hartindex(), true);
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
	const fdt32_t *secure_irqs;
	int secure_irqs_len;
	const fdt32_t *preempt_irqs;
	int preempt_irqs_len;

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

	secure_irqs = fdt_getprop(fdt, nodeoff, "riscv,secure-irqs",
				  &secure_irqs_len);
	if (secure_irqs && secure_irqs_len >= sizeof(*secure_irqs)) {
		u32 count = secure_irqs_len / sizeof(*secure_irqs);

		secure_config.irqs = sbi_zalloc(count * sizeof(*secure_config.irqs));
		if (!secure_config.irqs) {
			rc = SBI_ENOMEM;
			goto fail_free_data;
		}

		for (u32 i = 0; i < count; i++) {
			u32 irq = fdt32_to_cpu(secure_irqs[i]);

			if (!irq || irq > pd->num_src)
				continue;
			secure_config.irqs[secure_config.count++] = irq;
			plic_set_sec_src(pd, irq, true);
			plic_set_priority(pd, irq, 1);
		}
	}

	if (secure_config.count) {
		rc = plic_secure_configure(pd, true, true);
		if (rc)
			goto fail_free_secure_config;
		sbi_irqchip_set_irqfn(plic_secure_irqfn);
	}

	preempt_irqs = fdt_getprop(fdt, nodeoff, "riscv,ree-preempt-irqs",
				   &preempt_irqs_len);
	if (secure_config.count && preempt_irqs &&
	    preempt_irqs_len >= sizeof(*preempt_irqs)) {
		u32 count = preempt_irqs_len / sizeof(*preempt_irqs);

		preempt_config.irqs = sbi_zalloc(count *
						 sizeof(*preempt_config.irqs));
		if (!preempt_config.irqs) {
			rc = SBI_ENOMEM;
			goto fail_free_secure_config;
		}

		for (u32 i = 0; i < count; i++) {
			u32 irq = fdt32_to_cpu(preempt_irqs[i]);

			/* Preempt sources must be non-secure REE interrupts. */
			if (!irq || irq > pd->num_src || plic_get_sec_src(pd, irq))
				continue;
			preempt_config.irqs[preempt_config.count++] = irq;
			plic_set_priority(pd, irq, 1);
		}
	}

	return 0;

fail_free_secure_config:
	sbi_free(secure_config.irqs);
	secure_config.irqs = NULL;
	secure_config.count = 0;
	sbi_free(preempt_config.irqs);
	preempt_config.irqs = NULL;
	preempt_config.count = 0;

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
