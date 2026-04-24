/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Intel Corporation. All rights reserved.
 */

#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_mpxy.h>
#include <libfdt.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>
#include <sbi_utils/mpxy/fdt_mpxy_opteed.h>
#include <sbi_utils/irqchip/fdt_irqchip_plic.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_trap.h>

#if __riscv_xlen == 64
#define SHMEM_PHYS_ADDR(_hi, _lo) (_lo)
#elif __riscv_xlen == 32
#define SHMEM_PHYS_ADDR(_hi, _lo) (((u64)(_hi) << 32) | (_lo))
#else
#error "Undefined XLEN"
#endif

#define INVALID_ADDR		(-1U)
#define IS_SHMEM_ADDR_VALID(ms) \
		((ms)->shmem.shmem_addr_lo != INVALID_ADDR && \
		 (ms)->shmem.shmem_addr_hi != INVALID_ADDR)

/** Get hart shared memory base address */
static inline void *hart_shmem_base(struct hart_mpxy_state *ms)
{
	return (void *)(unsigned long)SHMEM_PHYS_ADDR(ms->shmem.shmem_addr_hi,
						ms->shmem.shmem_addr_lo);
}

/** SPD TEE MPXY Message IDs */
enum mpxy_opteed_message_id {
	OPTEED_MSG_COMMUNICATE = 0x01,
	OPTEED_MSG_COMPLETE = 0x02,
};

struct abi_entry_vectors {
	uint32_t yield_abi_entry;
	uint32_t fast_abi_entry;
};

struct abi_entry_vectors *entry_vector_table = NULL;

#define ABI_ENTRY_TYPE_FAST		1
#define ABI_ENTRY_TYPE_YIELD		0
#define FUNCID_TYPE_SHIFT		31
#define FUNCID_TYPE_MASK		0x1
#define GET_ABI_ENTRY_TYPE(id)		(((id) >> FUNCID_TYPE_SHIFT) & \
					 FUNCID_TYPE_MASK)

/* Defined in optee_os/core/arch/riscv/include/tee/teeabi_opteed.h */
#define TEEABI_OPTEED_RETURN_CALL_DONE 0xBE000000
#define TEEABI_OPTEED_RETURN_FIQ_DONE  0xBE000006

static char opteed_domain_name[64];
static struct sbi_domain *tdomain, *udomain;
static bool opteed_skip_regs_update[SBI_HARTMASK_MAX_BITS];

static void opteed_dump_trap_regs(const char *tag)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct sbi_trap_context *tcntx = sbi_trap_get_context(scratch);
	struct sbi_trap_regs *regs = tcntx ? &tcntx->regs : NULL;
	unsigned long mtvec, mscratch, mcause, mtval, mie, mip;

	if (!regs) {
		sbi_printf("opteed-mpxy: %s trapctx=null\n", tag);
		return;
	}

	mtvec = csr_read(CSR_MTVEC);
	mscratch = csr_read(CSR_MSCRATCH);
	mcause = csr_read(CSR_MCAUSE);
	mtval = csr_read(CSR_MTVAL);
	mie = csr_read(CSR_MIE);
	mip = csr_read(CSR_MIP);

	sbi_printf("opteed-mpxy: %s ra=0x%lx sp=0x%lx gp=0x%lx tp=0x%lx\n",
		   tag, regs->ra, regs->sp, regs->gp, regs->tp);
	sbi_printf("opteed-mpxy: %s t0=0x%lx t1=0x%lx t2=0x%lx t3=0x%lx\n",
		   tag, regs->t0, regs->t1, regs->t2, regs->t3);
	sbi_printf("opteed-mpxy: %s t4=0x%lx t5=0x%lx t6=0x%lx\n",
		   tag, regs->t4, regs->t5, regs->t6);
	sbi_printf("opteed-mpxy: %s s0=0x%lx s1=0x%lx s2=0x%lx s3=0x%lx\n",
		   tag, regs->s0, regs->s1, regs->s2, regs->s3);
	sbi_printf("opteed-mpxy: %s s4=0x%lx s5=0x%lx s6=0x%lx s7=0x%lx\n",
		   tag, regs->s4, regs->s5, regs->s6, regs->s7);
	sbi_printf("opteed-mpxy: %s s8=0x%lx s9=0x%lx s10=0x%lx s11=0x%lx\n",
		   tag, regs->s8, regs->s9, regs->s10, regs->s11);
	sbi_printf("opteed-mpxy: %s a0=0x%lx a1=0x%lx a2=0x%lx a3=0x%lx\n",
		   tag, regs->a0, regs->a1, regs->a2, regs->a3);
	sbi_printf("opteed-mpxy: %s a4=0x%lx a5=0x%lx a6=0x%lx a7=0x%lx\n",
		   tag, regs->a4, regs->a5, regs->a6, regs->a7);
	sbi_printf("opteed-mpxy: %s mepc=0x%lx mstatus=0x%lx mtvec=0x%lx\n",
		   tag, regs->mepc, regs->mstatus, mtvec);
	sbi_printf("opteed-mpxy: %s mscratch=0x%lx mcause=0x%lx mtval=0x%lx\n",
		   tag, mscratch, mcause, mtval);
	sbi_printf("opteed-mpxy: %s mie=0x%lx mip=0x%lx\n",
		   tag, mie, mip);
}

static void opteed_dump_smode_csrs(const char *tag)
{
	unsigned long sstatus = csr_read(CSR_SSTATUS);
	unsigned long sie = csr_read(CSR_SIE);
	unsigned long stvec = csr_read(CSR_STVEC);
	unsigned long sscratch = csr_read(CSR_SSCRATCH);
	unsigned long sepc = csr_read(CSR_SEPC);
	unsigned long satp = csr_read(CSR_SATP);

	sbi_printf("opteed-mpxy: %s sepc=0x%lx sstatus=0x%lx sie=0x%lx\n",
		   tag, sepc, sstatus, sie);
	sbi_printf("opteed-mpxy: %s sscratch=0x%lx satp=0x%lx stvec=0x%lx\n",
		   tag, sscratch, satp, stvec);
}

bool opteed_entry_ready(void)
{
	return !!entry_vector_table;
}

struct sbi_domain *opteed_get_tdomain(void)
{
	return tdomain;
}

unsigned long opteed_get_fiq_entry(void)
{
	unsigned long base = (unsigned long)entry_vector_table;

	if (!entry_vector_table)
		return 0;

	/* vector_fiq_entry is the 7th slot (index 6), each 4 bytes */
	return base + 6 * 4;
}

bool opteed_consume_skip_regs_update(void)
{
	u32 hartidx = current_hartindex();
	bool skip;

	if (hartidx >= SBI_HARTMASK_MAX_BITS)
		return false;

	skip = opteed_skip_regs_update[hartidx];
	opteed_skip_regs_update[hartidx] = false;

	return skip;
}

static int opteed_domain_setup(void *fdt, int nodeoff, const struct fdt_match *match)
{
	const u32 *prop_instance;
	int len, offset;

	prop_instance = fdt_getprop(fdt, nodeoff, "opensbi-domain-instance", &len);
	if (!prop_instance || len < 4)
		return SBI_EINVAL;

	offset = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*prop_instance));
	if (offset < 0)
		return SBI_EINVAL;

	strncpy(opteed_domain_name, fdt_get_name(fdt, offset, NULL),
		sizeof(opteed_domain_name));
	opteed_domain_name[sizeof(opteed_domain_name) - 1] = '\0';

	return 0;
}

static struct sbi_domain *__get_tdomain(void)
{
	struct sbi_domain *dom = NULL;
	sbi_domain_for_each(dom) {
		if (!sbi_strcmp(dom->name, opteed_domain_name)) {
			return dom;
		}
	}

	return NULL;
}

static struct sbi_domain *__get_udomain(void)
{
	struct sbi_domain *dom = NULL;
	sbi_domain_for_each(dom) {
		if (!sbi_strcmp(dom->name, "untrusted-domain")) {
			return dom;
		}
	}

	return NULL;
}

static int sbi_ecall_tee_domain_enter(unsigned long entry_point)
{
	sbi_domain_context_set_mepc(tdomain, entry_point);
	sbi_domain_context_enter(tdomain);
	return 0;
}

static int sbi_ecall_tee_domain_exit(void)
{
	sbi_domain_context_exit();
	return 0;
}

static int mpxy_opteed_send_message(struct sbi_mpxy_channel *channel,
			    u32 msg_id, void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len)
{
	u32 hartidx = current_hartindex();
	struct hart_mpxy_state *ms;
	void *shmem_base;
	u32 funcid_type;

	if (hartidx < SBI_HARTMASK_MAX_BITS)
		opteed_skip_regs_update[hartidx] = false;

	if (msg_id == OPTEED_MSG_COMMUNICATE) {
		/* Get per-hart MPXY share memory with tdomain */
		ms = hart_mpxy_state_get(tdomain, hartidx);
		shmem_base = hart_shmem_base(ms);

		if(!IS_SHMEM_ADDR_VALID(ms)) {
			sbi_printf("%s: hart%d trusted domain MPXY shared memory is invalid\n",
				   __func__,
				   current_hartid());
			return SBI_EINVAL;
		}

		sbi_memcpy(shmem_base, msgbuf, msg_len);

		funcid_type = GET_ABI_ENTRY_TYPE(((ulong *)shmem_base)[0]);
		sbi_ecall_tee_domain_enter((funcid_type == ABI_ENTRY_TYPE_FAST) ?
					   (ulong)&entry_vector_table->fast_abi_entry :
					   (ulong)&entry_vector_table->yield_abi_entry);
	} else if (msg_id == OPTEED_MSG_COMPLETE) {
		/* Get per-hart MPXY share memory with udomain */
		ms = hart_mpxy_state_get(udomain, hartidx);
		sbi_printf("opteed-mpxy: COMPLETE func=0x%lx hart=%u\n",
			   ((ulong *)msgbuf)[0], current_hartid());
		opteed_dump_trap_regs("pre-exit");
		opteed_dump_smode_csrs("pre-exit");

		if(!IS_SHMEM_ADDR_VALID(ms)) {
			if (((ulong *)msgbuf)[0] == TEEABI_OPTEED_RETURN_CALL_DONE) {
				/* RETURN_INIT_DONE */
				entry_vector_table = (struct abi_entry_vectors *)(((ulong *)msgbuf)[1]);
				sbi_printf("Registered OP-TEE entry table: %#lx\n", (ulong)entry_vector_table);
			}
		} else {
			if (!resp_len) {
				sbi_printf("ERROR %s: send data without response.\n", __func__);
				return SBI_EINVAL;
			}
			/* tx has a0~a4. Just skip a0 and copy a1~a4 here */
			sbi_memcpy(hart_shmem_base(ms),
				   &(((ulong *)msgbuf)[1]),
				   msg_len - sizeof(ulong));
			*resp_len = msg_len - sizeof(ulong);
		}

		sbi_ecall_tee_domain_exit();
		if (((ulong *)msgbuf)[0] == TEEABI_OPTEED_RETURN_FIQ_DONE &&
		    hartidx < SBI_HARTMASK_MAX_BITS)
			opteed_skip_regs_update[hartidx] = true;
		opteed_dump_trap_regs("post-exit");
		opteed_dump_smode_csrs("post-exit");
		/*
		 * Only FIQ return should complete the pending secure IRQ.
		 * Normal OP-TEE call/return traffic also uses COMPLETE and
		 * would otherwise spam misleading logs here.
		 */
		if (((ulong *)msgbuf)[0] == TEEABI_OPTEED_RETURN_FIQ_DONE)
			fdt_plic_secure_irq_complete();
	} else {
		sbi_printf("%s: message id %d not supported by channel%d\n",
			   __func__, msg_id, channel->channel_id);
		return SBI_EINVAL;
	}

	return 0;
}

static int mpxy_opteed_init(void *fdt, int nodeoff,
			  const struct fdt_match *match)
{
	struct sbi_mpxy_channel *channel;
	const fdt32_t *val;
	u32 channel_id;
	int rc, len;

	/* Allocate context for MPXY channel */
	channel = sbi_zalloc(sizeof(*channel));
	if (!channel)
		return SBI_ENOMEM;

	/* Setup domain for OP-TEE dispatcher */
	rc = opteed_domain_setup(fdt, nodeoff, match);
	if (rc) {
		sbi_free(channel);
		return 0;
	}

	val = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (len > 0 && val)
		channel_id = fdt32_to_cpu(*val);
	else
		sbi_panic("Failed to get riscv,sbi-mpxy-channel-id");

	channel->channel_id = channel_id;
	channel->send_message = mpxy_opteed_send_message;
	channel->attrs.msg_proto_id = SBI_MPXY_MSGPROTO_TEE_ID;
	channel->attrs.msg_data_maxlen = PAGE_SIZE;

	rc = sbi_mpxy_register_channel(channel);
	if (rc) {
		sbi_free(channel);
		return rc;
	}

	tdomain = __get_tdomain();
	udomain = __get_udomain();

	return 0;
}

static const struct fdt_match mpxy_opteed_match[] = {
	{ .compatible = "riscv,sbi-mpxy-opteed", .data = NULL },
	{},
};

struct fdt_mpxy fdt_mpxy_opteed = {
	.match_table = mpxy_opteed_match,
	.init = mpxy_opteed_init,
};
