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
#ifndef _RTE_NETDEV_TYPES_H_
#define _RTE_NETDEV_TYPES_H_

#include "ethdev_netdev_types.h"

struct rte_eth_dev;
struct rte_eth_stats;
struct common_vf_info;

typedef int (*eth_netdev_init_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_uninit_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_open_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_stop_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_start_xmit_t)(struct rte_eth_dev *dev, struct rte_dev_pkt_buff *skb);
typedef int (*eth_netdev_select_queue_t)(struct rte_eth_dev *dev, struct rte_dev_pkt_buff *skb);
typedef int (*eth_netdev_change_rx_flags_t)(struct rte_eth_dev *dev, int flags);
typedef int (*eth_netdev_set_rx_mode_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_set_multicast_list_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_set_mac_address_t)(struct rte_eth_dev *dev, void *addr);
typedef int (*eth_netdev_validate_addr_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_do_ioctl_t)(struct rte_eth_dev *dev, struct rte_dev_ifreq *ifr, int cmd);
typedef int (*eth_netdev_set_config_t)(struct rte_eth_dev *dev, struct rte_dev_ifmap *map);
typedef int (*eth_netdev_change_mtu_t)(struct rte_eth_dev *dev, int new_mtu);
typedef int (*eth_netdev_tx_timeout_t) (struct rte_eth_dev *dev);

typedef int (*eth_netdev_get_stats64_t)(struct rte_eth_dev *dev, struct rte_eth_stats *ret_stats);
typedef int (*eth_netdev_get_stats_t)(struct rte_eth_dev *dev, struct rte_eth_stats *ret_stats);

typedef int (*eth_netdev_vlan_rx_add_vid_t)(struct rte_eth_dev *dev, unsigned short vid);
typedef int (*eth_netdev_vlan_rx_kill_vid_t)(struct rte_eth_dev *dev, unsigned short vid);
typedef int (*eth_netdev_set_all_queues_drop_en_t)(struct rte_eth_dev *dev, uint8_t on);

/* PF set VF configuraiton */
typedef int (*eth_netdev_is_vf_enabled_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_set_vf_mac_addr_t)(struct rte_eth_dev *dev, int vf_id, uint8_t *mac_addr);
typedef int (*eth_netdev_set_vf_promisc_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_allmulticast_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_broadcast_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_vlan_t)(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id,
									  uint8_t qos);
typedef int (*eth_netdev_set_vf_rate_t)(struct rte_eth_dev *dev, int vf_id, int max_tx_rate);
typedef int (*eth_netdev_get_vf_config_t)(struct rte_eth_dev *dev, int vf_id,
									  struct common_vf_info *ivi);
typedef int (*eth_netdev_set_vf_spoofchk_t)(struct rte_eth_dev *dev, int vf_id, int enable);
typedef int (*eth_netdev_set_vf_link_state_t)(struct rte_eth_dev *dev, int vf_id, int link);
typedef int (*eth_netdev_ping_vfs_t)(struct rte_eth_dev *dev, int vf_id);
typedef int (*eth_netdev_set_vf_trust_t)(struct rte_eth_dev *dev, int vf_id, int setting);
typedef int (*eth_netdev_set_vf_vlan_anti_spoof_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_mac_anti_spoof_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_vlan_anti_spoof_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_vlan_filter_t)(struct rte_eth_dev *dev, uint64_t vf_mask, uint16_t vlan,
											 uint8_t on);
typedef int (*eth_netdev_set_vf_vlan_insert_t)(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id);
typedef int (*eth_netdev_set_vf_rate_limit_t)(struct rte_eth_dev *dev, int vf_id, uint16_t tx_rate,
											  uint64_t q_msk);
typedef int (*eth_netdev_set_vf_vlan_stripq_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_rxmode_t)(struct rte_eth_dev *dev, int vf_id, uint16_t rx_mask, uint8_t on);
typedef int (*eth_netdev_set_vf_rx_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_set_vf_tx_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);
typedef int (*eth_netdev_get_vf_stats_t)(struct rte_eth_dev *dev, int vf_id, struct rte_eth_stats *stats);
typedef int (*eth_netdev_reset_vf_stats_t)(struct rte_eth_dev *dev, int vf_id);
typedef int (*eth_netdev_set_vf_split_drop_en_t)(struct rte_eth_dev *dev, int vf_id, uint8_t on);

typedef int (*eth_netdev_set_tx_loopback_t)(struct rte_eth_dev *dev, uint8_t on);
typedef int (*eth_netdev_macsec_enable_t)(struct rte_eth_dev *dev, uint8_t en, uint8_t rp);
typedef int (*eth_netdev_macsec_disable_t)(struct rte_eth_dev *dev);
typedef int (*eth_netdev_macsec_config_txsc_t)(struct rte_eth_dev *dev, uint8_t *mac);
typedef int (*eth_netdev_macsec_config_rxsc_t)(struct rte_eth_dev *dev, uint8_t *mac, uint16_t pi);
typedef int (*eth_netdev_macsec_select_txsa_t)(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
											   uint32_t pn, uint8_t *key);
typedef int (*eth_netdev_macsec_select_rxsa_t)(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
											   uint32_t pn, uint8_t *key);

typedef int (*eth_netdev_fix_features_t)(struct rte_eth_dev *dev, uint64_t *features);
typedef int (*eth_netdev_set_features_t)(struct rte_eth_dev *dev, uint64_t features);

/* Advance Netdev function */
typedef int (*eth_netdev_udp_tunnel_add_t)(struct rte_eth_dev *dev, struct rte_dev_udp_tunnel_info *ti);
typedef int (*eth_netdev_udp_tunnel_del_t)(struct rte_eth_dev *dev, struct rte_dev_udp_tunnel_info *ti);

struct eth_dev_netdev_ops {
	eth_netdev_init_t		init;
	eth_netdev_uninit_t		uninit;
	eth_netdev_open_t		open;
	eth_netdev_stop_t		stop;
	eth_netdev_start_xmit_t	start_xmit;
	eth_netdev_select_queue_t		select_queue;
	eth_netdev_change_rx_flags_t	change_rx_flag;
	eth_netdev_set_rx_mode_t		set_rx_mode;
	eth_netdev_set_multicast_list_t set_multicast_list;
	eth_netdev_set_mac_address_t	set_mac_addr;
	eth_netdev_validate_addr_t		validate_addr;
	eth_netdev_do_ioctl_t			do_ioctl;
	eth_netdev_set_config_t			set_config;
	eth_netdev_change_mtu_t			change_mtu;
	eth_netdev_tx_timeout_t			tx_timeout;
	eth_netdev_get_stats64_t		get_stats64;
	eth_netdev_get_stats_t			get_stats;
	eth_netdev_vlan_rx_add_vid_t	vlan_rx_add_vid;
	eth_netdev_vlan_rx_kill_vid_t	vlan_rx_kill_vid;
	eth_netdev_set_all_queues_drop_en_t set_all_queues_drop_en;

	/* PF set VF configuration */
	eth_netdev_is_vf_enabled_t		is_vf_enabled;
	eth_netdev_set_vf_mac_addr_t	set_vf_mac_addr;
	eth_netdev_set_vf_promisc_t		set_vf_promisc;
	eth_netdev_set_vf_allmulticast_t	set_vf_allmulticast;
	eth_netdev_set_vf_broadcast_t	set_vf_broadcast;
	eth_netdev_set_vf_vlan_t		set_vf_vlan;
	eth_netdev_set_vf_rate_t		set_vf_rate;
	eth_netdev_get_vf_config_t		get_vf_config;
	eth_netdev_set_vf_spoofchk_t	set_vf_spoofchk;
	eth_netdev_set_vf_link_state_t	set_vf_link_state;
	eth_netdev_ping_vfs_t			ping_vfs;
	eth_netdev_set_vf_trust_t		set_vf_trust;
	eth_netdev_set_vf_mac_anti_spoof_t	set_vf_mac_anti_spoof;
	eth_netdev_set_vf_vlan_anti_spoof_t	set_vf_vlan_anti_spoof;
	eth_netdev_set_vf_vlan_filter_t	set_vf_vlan_filter;
	eth_netdev_set_vf_vlan_insert_t	set_vf_vlan_insert;
	eth_netdev_set_vf_rate_limit_t	set_vf_rate_limit;
	eth_netdev_set_vf_vlan_stripq_t	set_vf_vlan_stripq;
	eth_netdev_set_vf_rxmode_t		set_vf_rxmode;
	eth_netdev_set_vf_rx_t			set_vf_rx;
	eth_netdev_set_vf_tx_t			set_vf_tx;
	eth_netdev_get_vf_stats_t		get_vf_stats;
	eth_netdev_reset_vf_stats_t		reset_vf_stats;
	eth_netdev_set_vf_split_drop_en_t set_vf_split_drop_en;

	eth_netdev_set_tx_loopback_t	set_tx_loopback;
	eth_netdev_macsec_enable_t		macsec_enable;
	eth_netdev_macsec_disable_t		macsec_disable;
	eth_netdev_macsec_config_txsc_t	macsec_config_txsc;
	eth_netdev_macsec_config_rxsc_t	macsec_config_rxsc;
	eth_netdev_macsec_select_txsa_t	macsec_select_txsa;
	eth_netdev_macsec_select_rxsa_t	macsec_select_rxsa;

	eth_netdev_fix_features_t		fix_features;
	eth_netdev_set_features_t		set_features;

#ifdef RTE_ADVANCE_NETDEV_OPS
/* L2+ protocoal support: geneve and udp */
	eth_netdev_udp_tunnel_add_t		udp_tunnel_add;
	eth_netdev_udp_tunnel_del_t		udp_tunnel_del;
/* bridge mode stup */
/* l2 forwarding database */

#endif
	unsigned char		capability[16];
};

#endif /* _RTE_NETDEV_TYPES_H_ */
