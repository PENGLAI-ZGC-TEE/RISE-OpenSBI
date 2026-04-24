/*
 * SPDX-License-Identifier: BSD-2-Clause
 */
#ifndef __SBI_UTILS_MPXY_FDT_MPXY_OPTEED_H__
#define __SBI_UTILS_MPXY_FDT_MPXY_OPTEED_H__

#include <sbi/sbi_types.h>

struct sbi_domain;

/* Return true if OP-TEE entry table is ready */
bool opteed_entry_ready(void);

/* Return OP-TEE trusted domain pointer (NULL if unavailable) */
struct sbi_domain *opteed_get_tdomain(void);

/* Return OP-TEE FIQ entry address, or 0 if unavailable */
unsigned long opteed_get_fiq_entry(void);

/*
 * Return whether the current hart should skip the generic ecall register
 * update on exit, and clear the one-shot request after consuming it.
 */
bool opteed_consume_skip_regs_update(void);

#endif /* __SBI_UTILS_MPXY_FDT_MPXY_OPTEED_H__ */
