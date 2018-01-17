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
#ifndef _RTE_VF_FLAGS_H
#define _RTE_VF_FLAGS_H

/* VF_FLAG define
 * 31: PF/VF (0/1)
 * 30:24: vf-index
 * 23:21: op-code
 *	0: VF-feature bit set	(bit[0:15]: feature)
 *	1: VF Rx mode			(bit[0:15]: rxmode feature)
 *	2: VF get statistics	(output: /sys/kernel/vni_manage/client_cache)
 *	3: VF vlan filter main	(multi-step op; bit[0:15]: vlan-id)
 *	4: VF vlan insert		(bit[0:15]: vlan-id)
 *	5: VF rate				(bit[0:15]: rx_rate)
 *	6: VF queue rate limit	(multi-step op; bit[0:15]: rx_rate)
 *
 * 20:18: code sequence (only for multi-step op)
 * 17:16: reserved
 * 15:0: feature bits/rxmode feature/rx_rate/16-bit-vlan-id
 *
 */

#define VF_FLAG_IS_VF(dword)		(dword & 0x80000000)
#define VF_FLAG_VF_INDEX(dword)		((dword >> 24) & 0x7F)
#define VF_FLAG_OP_CODE(dword)		((dword >> 21) & 0x7)
#define VF_FLAG_OP_SEQ(dword)		((dword >> 18) & 0x7)
#define VF_FLAG_FEATURES(dword)		(dword & 0xFFFF)

/* VF op code */
#define VF_FLAG_CODE_VF_FEATURE		0
#define VF_FLAG_CODE_RXMODE_SET		1
#define VF_FLAG_CODE_GET_STATS		2
#define VF_FLAG_CODE_VLAN_FILT		3
#define VF_FLAG_CODE_VLAN_INSERT	4
#define VF_FLAG_CODE_VF_RATE		5
#define VF_FLAG_CODE_VF_Q_RATE		6

/* VF featue bit */
#define VF_FLAG_SPLIT_DROP_EN_BIT	0
#define VF_FLAG_MAC_PROMISC_BIT		1
#define VF_FLAG_ALLMULTI_BIT		2
#define VF_FLAG_BROADCAST_BIT		3
#define VF_FLAG_SPOOFCHK_BIT		4
#define VF_FLAG_LINK_STATE_BIT		5
#define VF_FLAG_PING_VFS_BIT		6
#define VF_FLAG_MAC_ANTISPOOF_BIT	7
#define VF_FLAG_VLAN_ANTISPOOF_BIT	8
#define VF_FLAG_VLAN_STRIPQ_BIT		9
#define VF_FLAG_RX_QUEUE_EN_BIT		10
#define VF_FLAG_TX_QUEUE_EN_BIT		11
#define VF_FLAG_REST_STATISTICS_BIT	12
#define VF_FLAG_VF_BIT_MAXI			(VF_FLAG_REST_STATISTICS_BIT+1)
#define VF_FLAG_TRUSTED_BIT			15
#define VF_FLAG_MASK				((1 << VF_FLAG_VF_BIT_MAXI) - 1)

#define VF_FLAG_MODE_BIT			15
#define VF_FLAG_MODE(dword)			(dword & (1 << VF_FLAG_MODE_BIT))

/* Private flags on VF configuration setting */
static const char rte_priv_flags_strings[][ETH_GSTRING_LEN] = {
	"split-drop.untag.b0",
	"promisc.hash-mc.b1",
	"allmulti.hash-uc.b2",
	"broadcast.b3",
	"spoofchk.mc.b4",
	"link-state.b5",
	"pf-ping-vf.b6",
	"mac-antispoof.b7",
	"vlan-antispoof.b8",
	"vlan-stripq.b9",
	"rx-queue-en.b10",
	"tx-queue-en.b11",
	"reset-stats.b12",
	"b13",
	"b14",
	"trusted.mode.b15",
	"b16",
	"b17",
	"seq0",
	"seq1",
	"seq2",
	"op-code0",
	"op-code1",
	"op-code2",
	"vf-id0",
	"vf-id1",
	"vf-id2",
	"vf-id3",
	"vf-id4",
	"vf-id5",
	"vf-id6",
	"vf-flag",
};
#define RTE_PRIV_FLAGS_STR_LEN \
	(sizeof(rte_priv_flags_strings)/sizeof(rte_priv_flags_strings[0]))

#endif /* _RTE_VF_FLAGS_H */