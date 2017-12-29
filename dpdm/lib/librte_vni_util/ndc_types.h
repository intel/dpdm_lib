/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017- Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _NDC_TYPES_H_
#define _NDC_TYPES_H_
#include "vni_share_types.h"
#include <rte_dev.h>

#ifndef MAXI_INTERFACE_NAME
#define MAXI_INTERFACE_NAME 256
#endif

#ifdef VNI_DEBUG
#define RTE_VNI_DEBUG_TRACE(...) \
	rte_pmd_debug_trace(__func__, __VA_ARGS__)
#else
#define RTE_VNI_DEBUG_TRACE(...)
#endif

#ifdef VF_FLAG_DEBUG
#define RTE_VF_FLAG_TRACE(...) \
	fprintf(stdout, __VA_ARGS__)
#else
#define RTE_VF_FLAG_TRACE(...)
#endif
	
#define SEND_NLH(nl_data)		(struct nlmsghdr *)(nl_data->send_data)
#define SEND_CMD(nl_data)	(netdev_cmd_info *)NLMSG_DATA(SEND_NLH(nl_data))
#define MSG_NLHDR(msg)		(struct nlmsghdr *)((msg)->msg_iov->iov_base)
#define MSG_NETDEV_CMD(msg)	(netdev_cmd_info *)NLMSG_DATA(MSG_NLHDR(msg))
#define RECV_NLH(nl_data)	(struct nlmsghdr *)(nl_data->recv_data)
#define RECV_CMD(nl_data)	(netdev_cmd_info *)NLMSG_DATA(RECV_NLH(nl_data))

#define TYPE_ALIGN(leng,size)	((leng+size-1)&(~(size-1)))

#define INT_ALIGN(leng)		TYPE_ALIGN(leng, sizeof(int))
#define DOUBLE_ALIGN(leng)	TYPE_ALIGN(leng, sizeof(double))

typedef enum _ndc_status {
	ndc_sucess,
	ndc_no_socket,
	ndc_no_netdev,
	ndc_no_inf,
	ndc_error,
} rte_ndc_status;

struct netlink_data;

struct netlink_data* find_nl_with_inf_name(char *inf_name);
const char *cmd_name(netdev_cmd_type cmd);
#endif /* _NDC_TYPES_H_ */