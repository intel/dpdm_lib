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
#include "rte_ethdev.h"
#include "ethdev_netdev_types.h"

#include "rte_netdev_ops.h"
#include "vni_share_types.h"
#include "ndc_netdev_proc.h"

int is_netdev_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_netdev_init) &&
		(cmd < vni_ethtool_set_setting))
		return 1;
	return 0;
}

static void rte_stats_to_common_stats64(struct common_stats64 *common_stats,
	struct rte_eth_stats *rte_stats)
{
	common_stats->rx_packets = rte_stats->ipackets;
	common_stats->tx_packets = rte_stats->opackets;
	common_stats->rx_bytes = rte_stats->ibytes;
	common_stats->tx_bytes = rte_stats->obytes;
	common_stats->rx_errors = rte_stats->ierrors;
	common_stats->tx_errors = rte_stats->oerrors;
	common_stats->rx_missed_errors = rte_stats->imissed;
	common_stats->rx_dropped = rte_stats->rx_nombuf;
}

#ifdef USE_32BIT_STATS
static void rte_stats_to_common_stats(struct common_stats *common_stats,
	struct rte_eth_stats *rte_stats)
{
	common_stats->rx_packets = (uint32_t)rte_stats->ipackets;
	common_stats->tx_packets = (uint32_t)rte_stats->opackets;
	common_stats->rx_bytes = (uint32_t)rte_stats->ibytes;
	common_stats->tx_bytes = (uint32_t)rte_stats->obytes;
	common_stats->rx_errors = (uint32_t)rte_stats->ierrors;
	common_stats->tx_errors = (uint32_t)rte_stats->oerrors;
	common_stats->rx_missed_errors = (uint32_t)rte_stats->imissed;
	common_stats->rx_dropped = (uint32_t)rte_stats->rx_nombuf;
}
#endif

void netdev_cmd_proc(netdev_cmd_info *cmd_info, netdev_cmd_info *send_cmd_info)
{
	//struct rte_dev_pkt_buff pkt_buff;
	struct rte_eth_stats rte_stats;
	struct common_vf_info rte_ivf;
	struct netdev_priv_data netdev_data;
	//unsigned char *buf;

	switch(cmd_info->cmd){
	case vni_netdev_init:
		send_cmd_info->status = rte_netdev_init(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_uninit:
		send_cmd_info->status = rte_netdev_uninit(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_open:
		send_cmd_info->status = rte_netdev_open(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_stop:
		send_cmd_info->status = rte_netdev_stop(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_change_rx_flags:
		send_cmd_info->status = rte_netdev_change_rx_flags(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int, sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_set_rx_mode:
		send_cmd_info->status = rte_netdev_set_rx_mode(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_set_mac_addr:
		send_cmd_info->status = rte_netdev_set_mac_address(cmd_info->port_id,
			(void *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_validate_addr:
		send_cmd_info->status = rte_netdev_validate_addr(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;		
	case vni_netdev_do_ioctl:
		send_cmd_info->status = rte_netdev_do_ioctl(cmd_info->port_id,
			(struct rte_dev_ifreq *)cmd_info->data,
			GET_DATA(cmd_info->data, sizeof(struct rte_dev_ifreq), int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_change_mtu:
		send_cmd_info->status = rte_netdev_change_mtu(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int, sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;		
	case vni_netdev_tx_timeout:
		send_cmd_info->status = rte_netdev_tx_timeout(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;		
	case vni_netdev_get_stats64:
		send_cmd_info->status = rte_netdev_get_stats64(cmd_info->port_id,
			&rte_stats);
		rte_stats_to_common_stats64(
			(struct common_stats64 *)send_cmd_info->data, &rte_stats);
		send_cmd_info->data_length = sizeof(struct common_stats64);
		break;
	case vni_netdev_get_stats:
		send_cmd_info->status = rte_netdev_get_stats(cmd_info->port_id,
			&rte_stats);
		rte_stats_to_common_stats64(
			(struct common_stats64 *)send_cmd_info->data, &rte_stats);
		send_cmd_info->data_length = sizeof(struct common_stats64);
		break;
	case vni_netdev_vlan_rx_add_vid: /* skip __be16 prot (2 bytes) from netdev_op */
		send_cmd_info->status = rte_netdev_vlan_rx_add_vid(cmd_info->port_id,
			GET_DATA(cmd_info->data, 2, uint16_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;		
	case vni_netdev_vlan_rx_kill_vid:
		send_cmd_info->status = rte_netdev_vlan_rx_kill_vid(cmd_info->port_id,
			GET_DATA(cmd_info->data, 2, uint16_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;		
	case vni_netdev_set_vf_mac:
		send_cmd_info->status = rte_netdev_set_vf_mac_addr(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)),
			(uint8_t *)(&cmd_info->data[sizeof(int)]));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_set_vf_vlan: /* CAVEAT: make sure queue-id<-->vf-id match */
		send_cmd_info->status = rte_netdev_set_vf_vlan(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)),/* queue */
			GET_DATA(cmd_info->data, sizeof(int), uint16_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)), /* vlan */
			GET_DATA(cmd_info->data, sizeof(int)+sizeof(uint16_t), uint8_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_set_vf_rate:
		send_cmd_info->status = rte_netdev_set_vf_rate(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)),
			GET_DATA(cmd_info->data, sizeof(int), int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_set_vf_spoofchk:
		send_cmd_info->status = rte_netdev_set_vf_spoofchk(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)), 
			GET_DATA(cmd_info->data, sizeof(int), int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_get_vf_config:
		send_cmd_info->status = rte_netdev_get_vf_config(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)), &rte_ivf);
		COPY_DATA(send_cmd_info->data, &rte_ivf, 
			sizeof(struct common_vf_info));
		send_cmd_info->data_length = sizeof(struct common_vf_info );
		break;
	case vni_netdev_set_vf_trust:
		send_cmd_info->status = rte_netdev_set_vf_trust(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)),
			GET_DATA(cmd_info->data, sizeof(int), int,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_netdev_fix_features:
		send_cmd_info->status = rte_netdev_fix_features(cmd_info->port_id,
			GET_PTR(cmd_info->data, 0, uint64_t));
		*GET_PTR(send_cmd_info->data, 0, uint64_t) = GET_DATA(cmd_info->data, 0, uint64_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info));
		send_cmd_info->data_length = sizeof(uint64_t);
		break;
	case vni_netdev_set_features:
		send_cmd_info->status = rte_netdev_set_features(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, uint32_t,
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	default:
		send_cmd_info->status = -ENOTSUP;
		break;
	}
}
