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
#include <errno.h>
#include <sys/queue.h>
#include <linux/ethtool.h>
#include <string.h>
#include <rte_ethdev.h>
#include <rte_dpdm.h>
#include <vni_netdev_flags.h>
#include <rte_pci.h>
#include <rte_version.h>
#include <vni_share_types.h>
#include <rte_vf_flags.h>

#include "i40e_logs.h"
#include "base/i40e_prototype.h"
#include "base/i40e_adminq_cmd.h"
#include "base/i40e_type.h"

#include "i40e_rxtx.h"
#ifndef I40E_MAC_X722_VF
#define I40E_MAC_X722_VF (I40E_MAC_VF+2)
#endif
#include "i40e_ethdev.h"
#include "i40e_pf.h"
#include "i40e_common.h"

static int eth_i40evf_dev_init_ex(struct rte_eth_dev *dev);

/* ethtool_op APIs */
static int i40evf_get_reg_length(struct rte_eth_dev *dev);
static int i40evf_get_regs(struct rte_eth_dev *dev, struct rte_dev_ethtool_reg *reg_info, void *data);

/* netdev_op APIs */
static int i40evf_dev_start(struct rte_eth_dev *dev);
static int i40evf_dev_stop(struct rte_eth_dev *dev);
static int i40evf_set_features(struct rte_eth_dev *dev, uint64_t features);
static int i40evf_change_rx_flag(struct rte_eth_dev *dev, int flags);

/* helper routines */
static int
i40evf_netdev_flag_sync(struct rte_eth_dev *dev, unsigned int change,
	unsigned int new_flag)
{
	unsigned int supported[] = {RTE_IFF_BROADCAST, RTE_IFF_LOOPBACK,
		RTE_IFF_PROMISC, RTE_IFF_ALLMULTI, RTE_IFF_MULTICAST};
	int i;

	for (i=0; i < sizeof(supported)/sizeof(unsigned int); i++){
		if (change & supported[i]){
			switch(supported[i]){
			case RTE_IFF_BROADCAST:
				PMD_DRV_LOG(INFO, "Un-supported IFF flag\n");
				break;
			case RTE_IFF_LOOPBACK:
				PMD_DRV_LOG(INFO, "Un-supported IFF flag\n");
				break;
			case RTE_IFF_PROMISC:
				if (new_flag & RTE_IFF_PROMISC)
					dev->dev_ops->promiscuous_enable(dev);
				else
					dev->dev_ops->promiscuous_disable(dev);
				break;
			case RTE_IFF_ALLMULTI:
				if (new_flag & RTE_IFF_ALLMULTI)
					dev->dev_ops->allmulticast_enable(dev);
				else
					dev->dev_ops->allmulticast_disable(dev);
				break;
			case RTE_IFF_MULTICAST:
			default:
				PMD_DRV_LOG(INFO, "Un-supported IFF flag\n");
				break;
			}
		}
	}
	return 0;
}

/* ethtool implementation */
static int 
i40evf_get_setting(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_cmd *cmd)
{
	struct i40e_vf *vf = I40EVF_DEV_PRIVATE_TO_VF(dev->data->dev_private);

	cmd->supported = 0;
	cmd->autoneg = AUTONEG_DISABLE;
	cmd->transceiver = XCVR_DUMMY1;
	cmd->port = PORT_NONE;
	/* Set speed and duplex */
	switch (vf->link_speed) {
	case I40E_LINK_SPEED_40GB:
		i40e_link_speed_set(cmd, SPEED_40000);
		break;
	case I40E_LINK_SPEED_25GB:
		i40e_link_speed_set(cmd, SPEED_25000);
		break;
	case I40E_LINK_SPEED_20GB:
		i40e_link_speed_set(cmd, SPEED_20000);
		break;
	case I40E_LINK_SPEED_10GB:
		i40e_link_speed_set(cmd, SPEED_10000);
		break;
	case I40E_LINK_SPEED_1GB:
		i40e_link_speed_set(cmd, SPEED_1000);
		break;
	case I40E_LINK_SPEED_100MB:
		i40e_link_speed_set(cmd, SPEED_100);
		break;
	default:
		break;
	}
	cmd->duplex = DUPLEX_FULL;

	return 0;
}

static int
i40evf_set_setting(struct rte_eth_dev *dev,
			     struct rte_dev_ethtool_cmd *cmd)
{
	return -1;
}

static int
i40evf_get_reg_length(struct rte_eth_dev *dev)
{
	return 0;
}

static int
i40evf_get_regs(struct rte_eth_dev *dev, 
	struct rte_dev_ethtool_reg *reg_info, void *data)
{
	return -1;
}

static int
i40evf_get_eeprom_length(struct rte_eth_dev *dev)
{
	return 0;
}
	
static int
i40evf_get_drvinfo(struct rte_eth_dev *dev, struct rte_dev_ethtool_drvinfo *drvinfo)
{
	struct rte_eth_dev_info dev_info;
	const char driver_name[] = "i40evf\0";
	struct rte_pci_device *pci_dev = NULL;
	int status = 0;
#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	const struct rte_bus *bus = NULL;
#endif

	if (drvinfo == NULL)
		return  -EINVAL;
#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	if (dev->dev_ops->fw_version_get) {
		status = dev->dev_ops->fw_version_get(dev, drvinfo->fw_version, 
			sizeof(drvinfo->fw_version));

		if (status < 0)
			PMD_DRV_LOG(ERR, "Get firmware version error: (%s)\n", strerror(-status));
		else {
			if (status > 0)
				PMD_DRV_LOG(ERR, "Insufficient buffer size of firmware version"
				"minimum size is %d", status);
		}
	} else {
		snprintf(drvinfo->fw_version, 4, "N/A");
	}
#else
	snprintf(drvinfo->fw_version, 4, "N/A");
#endif

	memset(&dev_info, 0, sizeof(struct rte_eth_dev_info));
	dev->dev_ops->dev_infos_get(dev, &dev_info);
	if (dev_info.driver_name)
		memcpy(drvinfo->driver, dev_info.driver_name, strlen(dev_info.driver_name)+1);
	else
		memcpy(drvinfo->driver, driver_name, strlen(driver_name) + 1);

	memcpy(drvinfo->version, rte_version(), strlen(rte_version())+1);

#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	if (dev_info.device)
		bus = rte_bus_find_by_device(dev_info.device);
	if (bus && !strcmp(bus->name, "pci"))
		pci_dev = RTE_DEV_TO_PCI(dev_info.device);
#else
	pci_dev = dev_info.pci_dev;
#endif
	
	if (pci_dev)
		snprintf(drvinfo->bus_info, sizeof(drvinfo->bus_info),
			"%04x:%02x:%02x.%x",
			pci_dev->addr.domain,
			pci_dev->addr.bus,
			pci_dev->addr.devid,
			pci_dev->addr.function);
	else
		snprintf(drvinfo->bus_info, sizeof(drvinfo->bus_info),
			"%04x:%02x:%02x.%x", 0, 0, 0, 0);

	drvinfo->regdump_len = i40evf_get_reg_length(dev);
	drvinfo->eedump_len = i40evf_get_eeprom_length(dev);
	drvinfo->n_stats = sizeof(struct rte_eth_stats)/sizeof(uint64_t);
	drvinfo->testinfo_len = 0;
	drvinfo->n_priv_flags = RTE_PRIV_FLAGS_STR_LEN;

	return status;
}

static int
i40evf_begin(struct rte_eth_dev *dev)
{
	/* DPDK should be always on */
	return 0;
}

static int
i40evf_complete(struct rte_eth_dev *dev)
{
	/* No close of DPDK, just close the interface */
	return 0;
}

static int 
i40evf_get_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	/* update attributes that can be changed run-time */
	memcpy(&dev_ex->netdev_data.perm_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN);
	memcpy(&dev_ex->netdev_data.dev_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN);
	dev_ex->netdev_data.mtu = dev->data->mtu;
    dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
    dev_ex->netdev_data.type = 1; /* ARPHRD_ETHER */

	memcpy((void *)netdev_data, &dev_ex->netdev_data, sizeof(struct netdev_priv_data));
	return 0;
}

static int 
i40evf_set_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	unsigned int change, old_flag;

	old_flag = dev_ex->netdev_data.flags;
	change = old_flag ^ netdev_data->flags;

	memcpy((void *)&dev_ex->netdev_data, netdev_data, sizeof(struct netdev_priv_data));
	if (memcmp(netdev_data->perm_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN))
		dev->dev_ops->mac_addr_set(dev, (void *)netdev_data->perm_addr);

	if (netdev_data->mtu != dev->data->mtu) {
		/* mtu change may require extra safe-guard step */
		if (dev->dev_ops->mtu_set)
			return (*dev->dev_ops->mtu_set)(dev, netdev_data->mtu);
		else
			dev->data->mtu = netdev_data->mtu;
	}

	if (change)
		return i40evf_netdev_flag_sync(dev, change, netdev_data->flags);

	return 0;
}

static const struct eth_dev_ethtool_ops i40evf_ethtool_ops = {
	.get_netdev_data = i40evf_get_netdev_data,
	.set_netdev_data = i40evf_set_netdev_data,
	.get_setting = i40evf_get_setting,
	.set_setting = i40evf_set_setting,
	.get_regs_len = i40evf_get_reg_length,
	.get_regs = i40evf_get_regs,
	.get_drvinfo = i40evf_get_drvinfo,
	/* pseudo function */
	.begin	= i40evf_begin,
	.complete		= i40evf_complete,
};
/* netdev op implementation */
static int
i40evf_dev_start(struct rte_eth_dev *dev)
{
	return dev->dev_ops->dev_start(dev);
}

static int
i40evf_dev_stop(struct rte_eth_dev *dev)
{
	dev->dev_ops->dev_stop(dev);
	return 0;
}

static int
i40evf_fix_features(struct rte_eth_dev *dev __rte_unused, uint64_t *features)
{
	uint64_t fix_up = *features;
	*features = fix_up;

	return 0;
}

static int
i40evf_set_features(struct rte_eth_dev *dev, uint64_t features)
{
	if (features & RTE_NETIF_F_HW_VLAN_RX)
		dev->dev_ops->vlan_offload_set(dev, 1);
	else
		dev->dev_ops->vlan_offload_set(dev, 0);

	return 0;
}

static int
i40evf_change_rx_flag(struct rte_eth_dev *dev, int flags)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	if (dev->dev_ops->promiscuous_enable) {
		if (flags & RTE_IFF_PROMISC) {
			dev->dev_ops->promiscuous_enable(dev);
			dev_ex->dev_iff_flag |= RTE_IFF_PROMISC;
		} else {
			dev->dev_ops->promiscuous_disable(dev);
			dev_ex->dev_iff_flag &= ~RTE_IFF_PROMISC;
		}
	}
	
	if (dev->dev_ops->allmulticast_enable) {
		if (flags & RTE_IFF_ALLMULTI) {
			dev->dev_ops->allmulticast_enable(dev);
			dev_ex->dev_iff_flag |= RTE_IFF_ALLMULTI;
		} else {
			dev->dev_ops->allmulticast_disable(dev);
			dev_ex->dev_iff_flag &= ~RTE_IFF_ALLMULTI;
		}
	}

	return 0;
}

static int
i40evf_dev_init(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	dev_ex->netdev_data.features = RTE_NETIF_F_SG |
		RTE_NETIF_F_IP_CSUM | RTE_NETIF_F_IPV6_CSUM |
		RTE_NETIF_F_HW_VLAN_TX | RTE_NETIF_F_HW_VLAN_RX |
		RTE_NETIF_F_TSO | RTE_NETIF_F_TSO6 |
		RTE_NETIF_F_LRO | RTE_NETIF_F_GRO |
		RTE_NETIF_F_HW_VLAN_FILTER | RTE_NETIF_F_SCTP_CSUM |
		RTE_NETIF_F_HIGHDMA;
	dev_ex->netdev_data.hw_features = dev_ex->netdev_data.features;
	dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
	memcpy(&dev_ex->netdev_data.perm_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN);
	return 0;
}

static int
i40evf_dev_uninit(struct rte_eth_dev *dev __rte_unused)
{
	return 0;
}

static const struct eth_dev_netdev_ops i40evf_netdev_ops = {
	.init				= i40evf_dev_init,
	.uninit				= i40evf_dev_uninit,
	.open				= i40evf_dev_start,
	.stop				= i40evf_dev_stop,
	.fix_features		= i40evf_fix_features,
	.set_features		= i40evf_set_features,
	.change_rx_flag		= i40evf_change_rx_flag,
};

static int
eth_i40evf_dev_init_ex(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex;

	dev_ex = rte_eth_get_devex_by_dev(dev);
	dev_ex->dev_ethtool_ops = &i40evf_ethtool_ops;
	dev_ex->dev_netdev_ops = &i40evf_netdev_ops;
	dev_ex->is_vf		= true;

	return 0;
}
RTE_PMD_EX_REGISTER_PCI(net_i40e_vf, eth_i40evf_dev_init_ex);
