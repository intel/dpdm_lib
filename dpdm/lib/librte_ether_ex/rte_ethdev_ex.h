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
#ifndef _RTE_ETHDEV_EX_H_
#define _RTE_ETHDEV_EX_H_

#include <inttypes.h>
#include <rte_manage.h>
#include <vni_share_types.h>

#define RTE_MAX_VF_COUNT	256

struct eth_dev_ethtool_ops;
struct eth_dev_netdev_ops;
struct rte_eth_dev;

/*struct eth_ifconfig_ops; */
union mask {
	uint64_t q_mask;
	uint16_t s_mask[4];
};

struct rte_eth_dev_ex {
	struct netdev_priv_data netdev_data;
	const struct eth_dev_ethtool_ops *dev_ethtool_ops;
	const struct eth_dev_netdev_ops *dev_netdev_ops;
	struct common_vf_info vf_info[RTE_MAX_VF_COUNT];

	unsigned int is_vf;
    unsigned int priv_flag; /* use by "ethtool --set-priv-flag */
	unsigned int dev_iff_flag; /* match kernel IFF flag */
	unsigned int dev_state;
	unsigned int vf_flags[RTE_MAX_VF_COUNT]; /* per-vf private flag */
	union mask vf_mask[RTE_MAX_VF_COUNT];
	union mask queue_mask;

/* additional information available from netdev */
	unsigned int nr_uc_addr;
	uint8_t *uc_addr_list;
	unsigned int nr_mc_addr;
	uint8_t *mc_addr_list;
};

struct rte_eth_drv_info {
	struct rte_pci_driver *driver;
	const char *drv_name;
};

void rte_eth_get_dev_ex_by_port(port_t port_id, struct rte_eth_dev **dev,
    struct rte_eth_dev_ex **dev_ex);

#endif /* _RTE_ETHDEV_EX_H_ */
