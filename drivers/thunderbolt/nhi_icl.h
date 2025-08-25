// SPDX-License-Identifier: GPL-2.0
/*
 * Ice Lake NHI specific operations
 *
 * Copyright (C) 2019, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#ifndef NHI_ICL_H_
#define NHI_ICL_H_

int icl_nhi_suspend(struct tb_nhi *nhi);
int icl_nhi_suspend_noirq(struct tb_nhi *nhi, bool wakeup);
int icl_nhi_resume(struct tb_nhi *nhi);
void icl_nhi_shutdown(struct tb_nhi *nhi);

#endif