/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) IPADS@SJTU 2023. All rights reserved.
 */

#include <sbi/sbi_error.h>
#include <sbi/riscv_locks.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_hsm.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_trap.h>

/** Context representation for a hart within a domain */
struct hart_context {
	/** Trap-related states such as GPRs, mepc, and mstatus */
	struct sbi_trap_context trap_ctx;

	/** Supervisor status register */
	unsigned long sstatus;
	/** Supervisor interrupt enable register */
	unsigned long sie;
	/** Supervisor trap vector base address register */
	unsigned long stvec;
	/** Supervisor scratch register for temporary storage */
	unsigned long sscratch;
	/** Supervisor exception program counter register */
	unsigned long sepc;
	/** Supervisor cause register */
	unsigned long scause;
	/** Supervisor trap value register */
	unsigned long stval;
	/** Supervisor interrupt pending register */
	unsigned long sip;
	/** Supervisor address translation and protection register */
	unsigned long satp;
	/** Counter-enable register */
	unsigned long scounteren;
	/** Supervisor environment configuration register */
	unsigned long senvcfg;

	/** Reference to the owning domain */
	struct sbi_domain *dom;
	/** Previous context (caller) to jump to during context exits */
	struct hart_context *prev_ctx;
	/** Is context initialized and runnable */
	bool initialized;
};

struct domain_context_priv {
	/** Contexts for possible HARTs indexed by hartindex */
	struct hart_context *hartindex_to_context_table[SBI_HARTMASK_MAX_BITS];
};

static struct sbi_domain_data dcpriv = {
	.data_size = sizeof(struct domain_context_priv),
};

static bool trace_active[SBI_HARTMASK_MAX_BITS];
static const char *trace_reason[SBI_HARTMASK_MAX_BITS];

void sbi_domain_context_trace_set(bool active, const char *reason)
{
	u32 hartindex = current_hartindex();

	if (hartindex >= SBI_HARTMASK_MAX_BITS)
		return;

	trace_active[hartindex] = active;
	trace_reason[hartindex] = active ? reason : NULL;
}

bool sbi_domain_context_trace_active(void)
{
	u32 hartindex = current_hartindex();

	return hartindex < SBI_HARTMASK_MAX_BITS && trace_active[hartindex];
}

const char *sbi_domain_context_trace_reason(void)
{
	u32 hartindex = current_hartindex();

	if (hartindex >= SBI_HARTMASK_MAX_BITS || !trace_reason[hartindex])
		return "unknown";

	return trace_reason[hartindex];
}

static void trace_regs(const char *tag, const char *reason,
		       const char *from, const char *to,
		       const struct sbi_trap_regs *regs)
{
	if (!regs)
		return;

	sbi_printf("[SBI-DOM] %s reason=%s from=%s to=%s mepc=0x%lx mstatus=0x%lx ra=0x%lx sp=0x%lx gp=0x%lx tp=0x%lx\n",
		   tag, reason, from, to, regs->mepc, regs->mstatus,
		   regs->ra, regs->sp, regs->gp, regs->tp);
	sbi_printf("[SBI-DOM] GPR reason=%s a0=0x%lx a1=0x%lx a2=0x%lx a3=0x%lx a4=0x%lx a5=0x%lx a6=0x%lx a7=0x%lx\n",
		   reason, regs->a0, regs->a1, regs->a2, regs->a3,
		   regs->a4, regs->a5, regs->a6, regs->a7);
	sbi_printf("[SBI-DOM] GPR reason=%s s0=0x%lx s1=0x%lx s2=0x%lx s3=0x%lx s4=0x%lx s5=0x%lx s6=0x%lx s7=0x%lx s8=0x%lx s9=0x%lx s10=0x%lx s11=0x%lx\n",
		   reason, regs->s0, regs->s1, regs->s2, regs->s3,
		   regs->s4, regs->s5, regs->s6, regs->s7, regs->s8,
		   regs->s9, regs->s10, regs->s11);
	sbi_printf("[SBI-DOM] GPR reason=%s t0=0x%lx t1=0x%lx t2=0x%lx t3=0x%lx t4=0x%lx t5=0x%lx t6=0x%lx\n",
		   reason, regs->t0, regs->t1, regs->t2, regs->t3,
		   regs->t4, regs->t5, regs->t6);
}

static unsigned long trace_read_scounteren(struct sbi_scratch *scratch)
{
	if (sbi_hart_priv_version(scratch) < SBI_HART_PRIV_VER_1_10)
		return 0;

	return csr_read(CSR_SCOUNTEREN);
}

static unsigned long trace_read_senvcfg(struct sbi_scratch *scratch)
{
	if (sbi_hart_priv_version(scratch) < SBI_HART_PRIV_VER_1_12)
		return 0;

	return csr_read(CSR_SENVCFG);
}

static inline struct hart_context *hart_context_get(struct sbi_domain *dom,
						    u32 hartindex)
{
	struct domain_context_priv *dcp = sbi_domain_data_ptr(dom, &dcpriv);

	return (dcp && hartindex < SBI_HARTMASK_MAX_BITS) ?
		dcp->hartindex_to_context_table[hartindex] : NULL;
}

static void hart_context_set(struct sbi_domain *dom, u32 hartindex,
			     struct hart_context *hc)
{
	struct domain_context_priv *dcp = sbi_domain_data_ptr(dom, &dcpriv);

	if (dcp && hartindex < SBI_HARTMASK_MAX_BITS) {
		dcp->hartindex_to_context_table[hartindex] = hc;
	}
}

/** Macro to obtain the current hart's context pointer */
#define hart_context_thishart_get()					\
	hart_context_get(sbi_domain_thishart_ptr(),			\
			 sbi_hartid_to_hartindex(current_hartid()))

/**
 * Switches the HART context from the current domain to the target domain.
 * This includes changing domain assignments and reconfiguring PMP, as well
 * as saving and restoring CSRs and trap states.
 *
 * @param ctx pointer to the current HART context
 * @param dom_ctx pointer to the target domain context
 */
static void switch_to_next_domain_context(struct hart_context *ctx,
					  struct hart_context *dom_ctx)
{
	u32 hartindex = current_hartindex();
	struct sbi_trap_context *trap_ctx;
	struct sbi_domain *current_dom = ctx->dom;
	struct sbi_domain *target_dom = dom_ctx->dom;
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	unsigned int pmp_count = sbi_hart_pmp_count(scratch);
	const char *reason = sbi_domain_context_trace_reason();
	bool trace = sbi_domain_context_trace_active();
	unsigned long exp_sstatus = dom_ctx->sstatus;
	unsigned long exp_sepc = dom_ctx->sepc;
	unsigned long exp_satp = dom_ctx->satp;
	unsigned long exp_mepc = dom_ctx->trap_ctx.regs.mepc;

	/* Assign current hart to target domain */
	spin_lock(&current_dom->assigned_harts_lock);
	sbi_hartmask_clear_hartindex(hartindex, &current_dom->assigned_harts);
	spin_unlock(&current_dom->assigned_harts_lock);

	sbi_update_hartindex_to_domain(hartindex, target_dom);

	spin_lock(&target_dom->assigned_harts_lock);
	sbi_hartmask_set_hartindex(hartindex, &target_dom->assigned_harts);
	spin_unlock(&target_dom->assigned_harts_lock);

	/* Reconfigure PMP settings for the new domain */
	for (int i = 0; i < pmp_count; i++) {
		pmp_disable(i);
	}
	sbi_hart_pmp_configure(scratch);

	if (trace) {
		trap_ctx = sbi_trap_get_context(scratch);
		sbi_printf("[SBI-DOM] SAVE reason=%s from=%s to=%s sstatus=0x%lx sie=0x%lx stvec=0x%lx sscratch=0x%lx sepc=0x%lx scause=0x%lx stval=0x%lx satp=0x%lx scounteren=0x%lx senvcfg=0x%lx trap_mepc=0x%lx\n",
			   reason, current_dom->name, target_dom->name,
			   csr_read(CSR_SSTATUS), csr_read(CSR_SIE),
			   csr_read(CSR_STVEC), csr_read(CSR_SSCRATCH),
			   csr_read(CSR_SEPC), csr_read(CSR_SCAUSE),
			   csr_read(CSR_STVAL), csr_read(CSR_SATP),
			   trace_read_scounteren(scratch),
			   trace_read_senvcfg(scratch),
			   trap_ctx ? trap_ctx->regs.mepc : 0);
		trace_regs("SAVE-REGS", reason, current_dom->name,
			   target_dom->name, trap_ctx ? &trap_ctx->regs : NULL);
	}

	/* Save current CSR context and restore target domain's CSR context */
	ctx->sstatus	= csr_swap(CSR_SSTATUS, dom_ctx->sstatus);
	ctx->sie	= csr_swap(CSR_SIE, dom_ctx->sie);
	ctx->stvec	= csr_swap(CSR_STVEC, dom_ctx->stvec);
	ctx->sscratch	= csr_swap(CSR_SSCRATCH, dom_ctx->sscratch);
	ctx->sepc	= csr_swap(CSR_SEPC, dom_ctx->sepc);
	ctx->scause	= csr_swap(CSR_SCAUSE, dom_ctx->scause);
	ctx->stval	= csr_swap(CSR_STVAL, dom_ctx->stval);
	ctx->satp	= csr_swap(CSR_SATP, dom_ctx->satp);
	if (sbi_hart_priv_version(scratch) >= SBI_HART_PRIV_VER_1_10)
		ctx->scounteren = csr_swap(CSR_SCOUNTEREN, dom_ctx->scounteren);
	if (sbi_hart_priv_version(scratch) >= SBI_HART_PRIV_VER_1_12)
		ctx->senvcfg	= csr_swap(CSR_SENVCFG, dom_ctx->senvcfg);

	/* Save current trap state and restore target domain's trap state */
	trap_ctx = sbi_trap_get_context(scratch);
	sbi_memcpy(&ctx->trap_ctx, trap_ctx, sizeof(*trap_ctx));
	sbi_memcpy(trap_ctx, &dom_ctx->trap_ctx, sizeof(*trap_ctx));

	if (trace) {
		bool ok = csr_read(CSR_SSTATUS) == exp_sstatus &&
			  csr_read(CSR_SEPC) == exp_sepc &&
			  csr_read(CSR_SATP) == exp_satp &&
			  trap_ctx->regs.mepc == exp_mepc;

		sbi_printf("[SBI-DOM] RESTORE reason=%s target=%s sstatus=0x%lx sie=0x%lx stvec=0x%lx sscratch=0x%lx sepc=0x%lx scause=0x%lx stval=0x%lx satp=0x%lx scounteren=0x%lx senvcfg=0x%lx trap_mepc=0x%lx\n",
			   reason, target_dom->name, csr_read(CSR_SSTATUS),
			   csr_read(CSR_SIE), csr_read(CSR_STVEC),
			   csr_read(CSR_SSCRATCH), csr_read(CSR_SEPC),
			   csr_read(CSR_SCAUSE), csr_read(CSR_STVAL),
			   csr_read(CSR_SATP), trace_read_scounteren(scratch),
			   trace_read_senvcfg(scratch), trap_ctx->regs.mepc);
		trace_regs("RESTORE-REGS", reason, current_dom->name,
			   target_dom->name, &trap_ctx->regs);
		sbi_printf("[SBI-DOM] CHECK reason=%s restore=%s result=%s sepc=0x%lx sstatus=0x%lx satp=0x%lx trap_mepc=0x%lx\n",
			   reason, target_dom->name, ok ? "OK" : "CHANGED",
			   csr_read(CSR_SEPC), csr_read(CSR_SSTATUS),
			   csr_read(CSR_SATP), trap_ctx->regs.mepc);
	}

	/* Mark current context structure initialized because context saved */
	ctx->initialized = true;

	/* If target domain context is not initialized or runnable */
	if (!dom_ctx->initialized) {
		/* Startup boot HART of target domain */
		if (current_hartid() == target_dom->boot_hartid)
			sbi_hart_switch_mode(target_dom->boot_hartid,
					     target_dom->next_arg1,
					     target_dom->next_addr,
					     target_dom->next_mode,
					     false);
		else
			sbi_hsm_hart_stop(scratch, true);
	}
}

/**
 * Set domain entry point
 * @param dom pointer to domain
 * @param entry_point new entry point of domain
 *
 * @return 0 on success and negative error code on failure
 */
int sbi_domain_context_set_mepc(struct sbi_domain *dom, unsigned long entry_point)
{
	struct hart_context *dom_ctx = hart_context_get(dom, current_hartindex());

	/* Validate the domain context existence */
	if (!dom_ctx)
		return SBI_EINVAL;

	dom_ctx->trap_ctx.regs.mepc = entry_point - 4;

	return 0;
}

int sbi_domain_context_enter(struct sbi_domain *dom)
{
	struct hart_context *ctx = hart_context_thishart_get();
	struct hart_context *dom_ctx = hart_context_get(dom, current_hartindex());

	/* Validate the domain context existence */
	if (!dom_ctx)
		return SBI_EINVAL;

	/* Update target context's previous context to indicate the caller */
	dom_ctx->prev_ctx = ctx;

	switch_to_next_domain_context(ctx, dom_ctx);

	return 0;
}

int sbi_domain_context_exit(void)
{
	u32 hartindex = current_hartindex();
	struct sbi_domain *dom;
	struct hart_context *ctx = hart_context_thishart_get();
	struct hart_context *dom_ctx, *tmp;

	/*
	 * If it's first time to call `exit` on the current hart, no
	 * context allocated before. Loop through each domain to allocate
	 * its context on the current hart if valid.
	 */
	if (!ctx) {
		sbi_domain_for_each(dom) {
			if (!sbi_hartmask_test_hartindex(hartindex,
							 dom->possible_harts))
				continue;

			dom_ctx = sbi_zalloc(sizeof(struct hart_context));
			if (!dom_ctx)
				return SBI_ENOMEM;

			/* Bind context and domain */
			dom_ctx->dom				   = dom;
			hart_context_set(dom, hartindex, dom_ctx);
		}

		ctx = hart_context_thishart_get();
	}

	dom_ctx = ctx->prev_ctx;

	/* If no previous caller context */
	if (!dom_ctx) {
		/* Try to find next uninitialized user-defined domain's context */
		sbi_domain_for_each(dom) {
			if (dom == &root || dom == sbi_domain_thishart_ptr())
				continue;

			tmp = hart_context_get(dom, hartindex);
			if (tmp && !tmp->initialized) {
				dom_ctx = tmp;
				break;
			}
		}
	}

	/* Take the root domain context if fail to find */
	if (!dom_ctx)
		dom_ctx = hart_context_get(&root, hartindex);

	switch_to_next_domain_context(ctx, dom_ctx);

	return 0;
}

int sbi_domain_context_init(void)
{
	return sbi_domain_register_data(&dcpriv);
}

void sbi_domain_context_deinit(void)
{
	sbi_domain_unregister_data(&dcpriv);
}
