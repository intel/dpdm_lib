/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2017- Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */
#include <linux/netdevice.h>
#include <linux/version.h>
#include "vni.h"

/* utility function */
static void common_stats64_to_rtnl_stats(struct rtnl_link_stats64  *rtnl_stats,
	struct common_stats64 *common_stats)
{
	rtnl_stats->rx_packets = common_stats->rx_packets;
	rtnl_stats->tx_packets = common_stats->tx_packets;
	rtnl_stats->rx_bytes = common_stats->rx_bytes;
	rtnl_stats->tx_bytes = common_stats->tx_bytes;
	rtnl_stats->rx_errors = common_stats->rx_errors;
	rtnl_stats->tx_errors = common_stats->tx_errors;
	rtnl_stats->rx_missed_errors = common_stats->rx_missed_errors;
	rtnl_stats->rx_dropped = common_stats->rx_dropped;
}

static void common_stats64_to_netdev_stats(struct net_device_stats  *netdev_stats,
	struct common_stats64 *common_stats)
{
	netdev_stats->rx_packets = common_stats->rx_packets;
	netdev_stats->tx_packets = common_stats->tx_packets;
	netdev_stats->rx_bytes = common_stats->rx_bytes;
	netdev_stats->tx_bytes = common_stats->tx_bytes;
	netdev_stats->rx_errors = common_stats->rx_errors;
	netdev_stats->tx_errors = common_stats->tx_errors;
	netdev_stats->rx_missed_errors = common_stats->rx_missed_errors;
	netdev_stats->rx_dropped = common_stats->rx_dropped;
}

static int
vni_init(struct net_device *dev)
{
	struct netdev_priv_data *netdev_data = netdev_priv(dev);

	set_get_stats64_enable(1);
	netdev_data->features = dev->features;
	netdev_data->hw_features = dev->hw_features;
	vni_log("%s: init called\n", dev->name);
	return 0;
}

static void
vni_uninit(struct net_device *dev)
{
	vni_log("%s: uninit called\n", dev->name);
	return;
}

static int
vni_open(struct net_device *dev)
{
	vni_log("%s: open called\n", dev->name);
	return 0;
}

static int
vni_stop(struct net_device *dev)
{
	vni_log("%s: stop called\n", dev->name);
	return 0;
}

static netdev_tx_t
vni_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	return k2u_link_1var_noupdate(dev, vni_netdev_start_xmit, skb,
		sizeof(struct sk_buff));
}

static void
vni_change_rx_flags(struct net_device *dev, int flags)
{
	int status = k2u_link_1var_noupdate(dev, vni_netdev_change_rx_flags, &flags,
		sizeof(int));
	if (status < 0)
		vni_elog("vni: netdev_change_rx_flags (inf: %s)failed with error code: %d\n",
		dev->name, status);
}

static void
vni_set_rx_mode(struct net_device *dev)
{
	int status = k2u_link(dev, vni_netdev_set_rx_mode);

	if (status < 0)
		vni_elog("vni: netdev_set_rx_mode (inf: %s)failed with error code: %d\n",
		dev->name, status);
}

static int
vni_set_mac_addr(struct net_device *dev, void *addr)
{
	return k2u_link_1var_noupdate(dev, vni_netdev_set_mac_addr, addr, 6);
}

static int
vni_validate_addr(struct net_device *dev)
{
	return k2u_link(dev, vni_netdev_validate_addr);
}

static int
vni_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_do_ioctl,
		sizeof(struct ifreq) + sizeof(int));
	if (!k2u_cmd_info) 
		return -1;
		
	memcpy(k2u_cmd_info->data, ifr, sizeof(struct ifreq));
	memcpy((u8 *)((u8*)k2u_cmd_info->data+sizeof(struct ifreq)), &cmd, sizeof(int));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}

static int
vni_change_mtu(struct net_device *dev, int new_mtu)
{
	return k2u_link_1var_noupdate(dev, vni_netdev_change_mtu, &new_mtu, sizeof(int));
}

static void
vni_tx_timeout(struct net_device *dev)
{
	int status = k2u_link(dev, vni_netdev_tx_timeout);
	if (status < 0)
		vni_elog("vni: netdev_tx_timeout (inf: %s)failed with error code: %d\n",
		dev->name, status);
}

#ifdef NEW_GET_STATS64
static void
vni_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	netdev_cmd_info *u2k_cmd_info;
	unsigned char port_id;
	
	if (get_get_stats64_enable() == 0 ||
		is_inf_closing(dev) ||
		(get_info_from_inf(dev->name,&port_id) == 0))
		return;

	u2k_cmd_info = k2u_link_1var_other(dev, vni_netdev_get_stats64,
		stats, sizeof(struct common_stats64));
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0) {
		vni_elog("netdev_get_stats (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else
		common_stats64_to_rtnl_stats(stats, u2k_cmd_info->data);
}
#else
static struct rtnl_link_stats64*
vni_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	static struct rtnl_link_stats64 ret_stats;
	netdev_cmd_info *u2k_cmd_info;
	unsigned short port_id;

	if (get_get_stats64_enable() == 0 ||
		is_inf_closing(dev) ||
		(get_info_from_inf(dev->name,&port_id) == 0)) {
		memset(&ret_stats, 0, sizeof(struct rtnl_link_stats64));
		return &ret_stats;
	}
	
	u2k_cmd_info = k2u_link_1var_other(dev, vni_netdev_get_stats64,
		stats, sizeof(struct common_stats64));
	if(!u2k_cmd_info)
		return &ret_stats;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: netdev_get_stats64 (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else
		common_stats64_to_rtnl_stats(&ret_stats, (struct common_stats64 *)u2k_cmd_info->data);
	
	return &ret_stats; 
}
#endif

static struct net_device_stats*
vni_get_stats(struct net_device *dev)
{
	static struct net_device_stats ret_stats;

	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev, vni_netdev_get_stats,
		&ret_stats, sizeof(struct common_stats64));
	if(!u2k_cmd_info)
		return &ret_stats;

	common_stats64_to_netdev_stats(&ret_stats, (struct common_stats64 *)u2k_cmd_info->data);
	if (u2k_cmd_info->status < 0)
		vni_elog("vni: netdev_get_stats (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	
	return &ret_stats; 
}

static int
vni_vlan_rx_add_vid(struct net_device *dev, __be16 prot, u16 vid)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_vlan_rx_add_vid,
		sizeof(__be16) + sizeof(u16));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &prot, sizeof(__be16));
	buf += sizeof(__be16);
	
	memcpy(buf, &vid, sizeof(u16));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}

static int
vni_vlan_rx_kill_vid(struct net_device *dev, __be16 prot, u16 vid)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_vlan_rx_kill_vid,
		sizeof(__be16) + sizeof(u16));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &prot, sizeof(__be16));
	buf += sizeof(__be16);
	
	memcpy(buf, &vid, sizeof(u16));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}

static int
vni_set_vf_mac_addr(struct net_device *dev, int vf, u8* mac)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_mac,
		sizeof(int) + 6);
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, mac, 6);

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}

#ifdef NEW_SET_VF_VLAN
static int
vni_set_vf_vlan(struct net_device *dev, int vf, u16 vlan,
					u8 qos, __be16 prot)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_vlan,
		sizeof(int) + sizeof(u16) + sizeof(u8) + sizeof(__be16));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &vlan, sizeof(u16));
	buf += sizeof(u16);

	memcpy(buf, &qos, sizeof(u8));
	buf += sizeof(u8);
	
	memcpy(buf, &prot, sizeof(__be16));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#else
static int
vni_set_vf_vlan(struct net_device *dev, int vf, u16 vlan,
					u8 qos)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_vlan,
		sizeof(int) + sizeof(u16) + sizeof(u8));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &vlan, sizeof(u16));
	buf += sizeof(u16);

	memcpy(buf, &qos, sizeof(u8));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#endif

#ifdef SUPPORT_SET_VF_RATE
#ifdef NEW_SET_VF_RATE
static int
vni_set_vf_rate(struct net_device *dev, int vf, int min_tx_rate, int max_tx_rate)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_rate,
		sizeof(int) + sizeof(int) + sizeof(int));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &min_tx_rate, sizeof(int));
	buf += sizeof(int);

	memcpy(buf, &max_tx_rate, sizeof(int));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#else
static int
vni_set_vf_rate(struct net_device *dev, int vf, int rate)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_rate,
		sizeof(int) + sizeof(int));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &rate, sizeof(int));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#endif
#endif

static int
vni_set_vf_spoofchk(struct net_device *dev, int vf, bool setting)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_spoofchk,
		sizeof(int) + sizeof(bool));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &setting, sizeof(bool));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}

#ifdef SUPPORT_SET_VF_TRUST
static int
vni_set_vf_trust(struct net_device *dev, int vf, bool setting)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	unsigned char *buf;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_set_vf_trust,
		sizeof(int) + sizeof(bool));
	if (!k2u_cmd_info)
		return -1;
	buf = (unsigned char *)k2u_cmd_info->data;
		
	memcpy(buf, &vf, sizeof(int));
	buf += sizeof(int);
	
	memcpy(buf, &setting, sizeof(bool));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#endif

static int
vni_get_vf_config(struct net_device *dev, int vf,
	struct ifla_vf_info *ivf)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	struct common_vf_info *us_vf_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_netdev_get_vf_config,
		sizeof(int) + sizeof(bool));		
	if (!k2u_cmd_info)
		return -1;
	memcpy(k2u_cmd_info->data, &vf, sizeof(int));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;
	
	if(u2k_cmd_info->status) {
		vni_elog("vni: netdev_get_config (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else {
		us_vf_info = (void *)u2k_cmd_info->data;
		ivf->vf = vf;
		memcpy(ivf->mac, us_vf_info->mac, MAC_ADDR_LEN);
		ivf->vlan = us_vf_info->vlan;
		ivf->qos = us_vf_info->qos;
		ivf->spoofchk = us_vf_info->spoofchk;
		ivf->linkstate = us_vf_info->linkstate;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
		ivf->min_tx_rate = us_vf_info->min_tx_rate;
		ivf->max_tx_rate = us_vf_info->max_tx_rate;
#endif
	}

	return u2k_cmd_info->status;
}

static netdev_features_t
vni_fix_features(struct net_device *dev, netdev_features_t features)
{
	netdev_features_t result, status;
	netdev_features_t rte_flags = rte_features(features);
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev, vni_netdev_fix_features,
		&rte_flags, sizeof(netdev_features_t));

	if (!u2k_cmd_info || u2k_cmd_info->status < 0) {
		vni_elog("vni: netdev_fix_features (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
		return -1;
	}
	status = *(netdev_features_t*)u2k_cmd_info->data;
	result = netdev_features(status);

	vni_log("featues size is %d in:%llx out:%llx\n",
		(int)(sizeof(netdev_features_t)), features, result);
	return result;
}

static int
vni_set_features(struct net_device *dev, netdev_features_t features)
{
	netdev_features_t rte_flags = rte_features(features);

	return k2u_link_1var_noupdate(dev, vni_netdev_set_features,
		&rte_flags, sizeof(netdev_features_t));
}

static struct net_device_ops vni_netdev_ops =
{
	.ndo_init		= vni_init,
	.ndo_uninit		= vni_uninit,
	.ndo_open		= vni_open,
	.ndo_stop		= vni_stop,
	.ndo_start_xmit		= vni_start_xmit,
	.ndo_change_rx_flags= vni_change_rx_flags,
	.ndo_set_rx_mode	= vni_set_rx_mode,
	.ndo_set_mac_address= vni_set_mac_addr,
	.ndo_validate_addr	= vni_validate_addr,
	.ndo_do_ioctl		= vni_do_ioctl,
	.ndo_change_mtu		= vni_change_mtu,
	.ndo_tx_timeout		= vni_tx_timeout,
	.ndo_get_stats64	= vni_get_stats64,
	.ndo_get_stats		= vni_get_stats,
	.ndo_vlan_rx_add_vid	= vni_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= vni_vlan_rx_kill_vid,
	.ndo_set_vf_mac			= vni_set_vf_mac_addr,
	.ndo_set_vf_vlan		= vni_set_vf_vlan,
#ifdef SUPPORT_SET_VF_RATE
	.ndo_set_vf_rate		= vni_set_vf_rate,
#endif
	.ndo_set_vf_spoofchk	= vni_set_vf_spoofchk,
	.ndo_get_vf_config		= vni_get_vf_config,
#ifdef SUPPORT_SET_VF_TRUST
	.ndo_set_vf_trust		= vni_set_vf_trust,
#endif
	.ndo_fix_features		= vni_fix_features,
	.ndo_set_features		= vni_set_features,
};

void
dev_add_netdev_ops(struct net_device *dev)
{
	dev->netdev_ops = &vni_netdev_ops;
}

static struct net_device_ops dummy_netdev_ops = {
	.ndo_init		= vni_init,
	.ndo_uninit		= vni_uninit,
	.ndo_open		= vni_open,
	.ndo_stop		= vni_stop,
};

void dev_add_dummy_netdev(struct net_device *dev)
{
	dev->netdev_ops = &dummy_netdev_ops;
}

