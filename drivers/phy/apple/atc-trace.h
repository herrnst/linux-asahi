// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Type-C PHY driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM appletypecphy

#if !defined(_APPLETYPECPHY_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _APPLETYPECPHY_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>

#define show_mux_state(state)                                                 \
	__print_symbolic(state.mode, { TYPEC_STATE_SAFE, "USB Safe State" }, \
			 { TYPEC_STATE_USB, "USB" })

#define show_mux_svid(svid)                                                 	\
	__print_symbolic(svid,							\
			 { 0, "none" },						\
			 { USB_TYPEC_TBT_SID, "Thunderbolt" },			\
			 { USB_TYPEC_DP_SID, "DisplayPort" }			\
			)

TRACE_EVENT(atcphy_mux_set, TP_PROTO(struct typec_mux_state *state),
	    TP_ARGS(state),
	    TP_STRUCT__entry(
		__field_struct(struct typec_mux_state, state)
		__field(unsigned long, svid)),
	    TP_fast_assign(__entry->state = *state; __entry->svid = state->alt ? state->alt->svid : 0;),
	    TP_printk("state: %s, svid: %s", show_mux_state(__entry->state), show_mux_svid(__entry->svid)));


#define show_sw_orientation(orientation)                                  \
	__print_symbolic(orientation, { TYPEC_ORIENTATION_NONE, "none" }, \
			 { TYPEC_ORIENTATION_NORMAL, "normal" },          \
			 { TYPEC_ORIENTATION_REVERSE, "reverse" })

TRACE_EVENT(atcphy_sw_set, TP_PROTO(enum typec_orientation orientation),
	    TP_ARGS(orientation),
	    TP_STRUCT__entry(__field(enum typec_orientation, orientation)),
	    TP_fast_assign(__entry->orientation = orientation;),
	    TP_printk("orientation: %s",
		      show_sw_orientation(__entry->orientation)));
#endif

/* This part must be outside protection */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE atc-trace
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/phy/apple
#include <trace/define_trace.h>
