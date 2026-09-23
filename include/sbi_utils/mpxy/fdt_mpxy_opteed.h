/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef __SBI_UTILS_MPXY_FDT_MPXY_OPTEED_H__
#define __SBI_UTILS_MPXY_FDT_MPXY_OPTEED_H__

#include <sbi/sbi_types.h>

struct sbi_domain;

struct sbi_domain *opteed_get_tdomain(void);
unsigned long opteed_get_fiq_entry(void);
bool opteed_consume_skip_regs_update(void);

#endif
