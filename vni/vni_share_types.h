/*
 *   Copyright(c) 2017- Intel Corporation. All rights reserved.
 */
#ifndef _VNI_SHARE_TYPES_H
#define _VNI_SHARE_TYPES_H

#define MAXI_MANAGEMENT_DEV	256
#define MAXI_INTERFACE_NAME	256
#define MAXI_REQ_DATA		4*1600	/* maximum # of NIC registers */

#define NETLINK_VNI 31		/* (MAX_LINKS-1) */

#define GET_DATA(base, offset, type, length) length>=(offset+sizeof(type))?(*(type *)(&base[offset])):((type) 0)
#define GET_PTR(base, offset, type) (type *)(&base[offset])
#define MIN(a,b) a>b?b:a
#define COPY_DATA(dst, src, data_size)	memcpy(dst, src, MIN(sizeof(dst), MIN(sizeof(src), data_size)))
#define INIT_DATA(base, value, size)	memset(base, value, MIN(sizeof(base), size))

#ifndef ETH_GSTRING_LEN
#define ETH_GSTRING_LEN 32
#endif

typedef enum netdev_cmd_type {
	vni_manage = 0,
	vni_manage_clean_inf,
	vni_manage_add_inf,
	vni_manage_del_inf,
	vni_manage_ack_kernel,
	vni_manage_ack_us,
	vni_manage_test,
	vni_netdev_init,
	vni_netdev_uninit,
	vni_netdev_open,
	vni_netdev_stop,
	vni_netdev_start_xmit,
	vni_netdev_change_rx_flags,
	vni_netdev_set_rx_mode,
	vni_netdev_set_mac_addr,
	vni_netdev_validate_addr,
	vni_netdev_do_ioctl,
	vni_netdev_change_mtu,
	vni_netdev_tx_timeout,
	vni_netdev_get_stats64,
	vni_netdev_get_stats,
	vni_netdev_vlan_rx_add_vid,
	vni_netdev_vlan_rx_kill_vid,
	vni_netdev_set_vf_mac,
	vni_netdev_set_vf_vlan,
	vni_netdev_set_vf_rate,
	vni_netdev_set_vf_spoofchk,
	vni_netdev_get_vf_config,
	vni_netdev_set_vf_trust,
	vni_netdev_fix_features,
	vni_netdev_set_features,
	vni_ethtool_set_setting,
	vni_ethtool_get_setting,
	vni_ethtool_get_drvinfo,
	vni_ethtool_get_reg_len,
	vni_ethtool_get_reg,
	vni_ethtool_get_wol,
	vni_ethtool_set_wol,
	vni_ethtool_get_msglevel,
	vni_ethtool_set_msglevel,
	vni_ethtool_nway_reset,
	vni_ethtool_get_link,
	vni_ethtool_get_eeprom_len,
	vni_ethtool_get_eeprom,
	vni_ethtool_set_eeprom,
	vni_ethtool_get_coalesce,
	vni_ethtool_set_coalesce,
	vni_ethtool_get_ringparam,
	vni_ethtool_set_ringparam,
	vni_ethtool_get_pauseparam,
	vni_ethtool_set_pauseparam,
	vni_ethtool_self_test,
	vni_ethtool_get_strings,
	vni_ethtool_set_phys_id,
	vni_ethtool_get_stats,
	vni_ethtool_begin,
	vni_ethtool_complete,
	vni_ethtool_get_priv_flags,
	vni_ethtool_set_priv_flags,
	vni_ethtool_get_sset_count,
	vni_ethtool_get_rxnfc,
	vni_ethtool_set_rxnfc,
	vni_ethtool_flash_device,
	vni_ethtool_reset,
	vni_ethtool_get_rxfh_key_size,
	vni_ethtool_get_rxfh_indir_size,
	vni_ethtool_get_rxfh,
	vni_ethtool_set_rxfh,
	vni_ethtool_get_channels,
	vni_ethtool_set_channels,
	vni_ethtool_get_dump_flag,
	vni_ethtool_get_dump_data,
	vni_ethtool_set_dump,
	vni_ethtool_get_ts_info,
	vni_ethtool_get_module_info,
	vni_ethtool_get_module_eeprom,
	vni_ethtool_get_eee,
	vni_ethtool_set_eee,
	vni_ethtool_get_tunable,
	vni_ethtool_set_tunable,
	vni_ethtool_get_per_queue_coalesce,
	vni_ethtool_set_per_queue_coalesce,
	vni_invalid
} netdev_cmd_type;

typedef enum {
	msg_created,
	msg_sent,
	msg_recv,
	msg_done
} vni_msg_status;

struct inf_req {
	/* interface name = format("%s%2d", base_name, port_id) */
	pid_t app_pid;
	char base_name[MAXI_INTERFACE_NAME];
	int num_of_ports;
};

typedef struct netdev_cmd_info {
	void *host;
	pid_t app_pid;
	unsigned short port_id;
	netdev_cmd_type cmd;
	int status;
	int data_length;
	unsigned char data[0];
} netdev_cmd_info;

struct common_stats {
	unsigned int rx_packets;
	unsigned int tx_packets;
	unsigned int rx_bytes;
	unsigned int tx_bytes;
	unsigned int rx_errors;
	unsigned int tx_errors;
	unsigned int rx_missed_errors;
	unsigned int rx_dropped;
};

struct common_stats64 {
	unsigned long long rx_packets;
	unsigned long long tx_packets;
	unsigned long long rx_bytes;
	unsigned long long tx_bytes;
	unsigned long long rx_errors;
	unsigned long long tx_errors;
	unsigned long long rx_missed_errors;
	unsigned long long rx_dropped;
};

struct netdev_priv_data {
	unsigned long long hw_features;
	unsigned long long features;
	unsigned char perm_addr[32];
	unsigned char dev_addr[6];
	unsigned char addr_len;
	unsigned int mtu;
	unsigned int flags;
};

#endif /* _VNI_SHARE_TYPES_H */