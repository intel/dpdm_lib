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
#include <string.h>
#include <rte_ethdev.h>
#include <rte_dpdm.h>
#include <vni_netdev_flags.h>
#include <rte_pci.h>
#include <vni_share_types.h>
#include <rte_vf_flags.h>

#include "ixgbe_logs.h"
#include "base/ixgbe_api.h"
#include "base/ixgbe_vf.h"
#include "base/ixgbe_common.h"
#include "base/ixgbe_type.h"
#include "base/ixgbe_phy.h"
#include "ixgbe_ethdev.h"

static int eth_ixgbevf_dev_init_ex(struct rte_eth_dev *dev);
static int eth_ixgbevf_dev_uninit_ex(struct rte_eth_dev *dev);

/* ethtool_op APIs */
static int ixgbevf_get_reg_length(struct rte_eth_dev *dev);
static int ixgbevf_get_regs(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_reg *reg_info, void *data);
static int ixgbevf_begin(struct rte_eth_dev *dev);
static int ixgbevf_complete(struct rte_eth_dev *dev);
static int ixgbevf_get_netdev_data(struct rte_eth_dev *dev,
	struct netdev_priv_data *netdev_data);
static int ixgbevf_set_netdev_data(struct rte_eth_dev *dev,
	struct netdev_priv_data *netdev_data);

/* netdev_op APIs */
static int ixgbevf_dev_start(struct rte_eth_dev *dev);
static int ixgbevf_dev_stop(struct rte_eth_dev *dev);
static int ixgbevf_fix_features(struct rte_eth_dev *dev, uint64_t *features);
static int ixgbevf_set_features(struct rte_eth_dev *dev, uint64_t features);
static int ixgbevf_change_rx_flag(struct rte_eth_dev *dev, int flags);

#define READ_REG_COUNT 0
#define READ_REGISTERS 1

static int
ixgbevf_get_link(struct rte_eth_dev *dev)
{
	struct rte_eth_link link, *dst, *src;

	link.link_status = 0;
	dst = &link;
	if (dev->data->dev_conf.intr_conf.lsc != 0) {
		src = &(dev->data->dev_link);
		rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
			*(uint64_t *)src);
	}
	else {		
		dev->dev_ops->link_update(dev, 1);
		*dst = dev->data->dev_link;
	}

	return dst->link_status;
}


static uint32_t
ixgbevf_read_reg_set(struct ixgbe_hw *hw, struct rte_dev_reg_set *reg_set, uint32_t *reg,
	uint32_t set_count)
{
	uint32_t i, j,total = 0;
	uint32_t count, stride;

	for(i = 0; i < set_count; i++) {
		count = reg_set[i].count;
		stride = reg_set[i].stride;
		if (count > 1) {
			for(j = 0; j < count; j++)
				reg[total+j] = IXGBE_READ_REG(hw, reg_set[i].base_addr + j*stride);
		} else {
			reg[total] = IXGBE_READ_REG(hw, reg_set[i].base_addr);
		}
		total += count;
	}
	return total;
}

static uint32_t
ixgbevf_registers(struct rte_eth_dev *dev, int mode, struct rte_dev_ethtool_reg *reginfo,
	void *data)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;
	uint32_t total = 0, count;
	uint32_t *regs = data;
	uint32_t seg0_addr_set[] = {
		IXGBE_VFSTATUS, IXGBE_VFLINKS, IXGBE_VFRXMEMWRAP, IXGBE_VFFRTIMER,
		IXGBE_VTEICS, IXGBE_VTEICS, IXGBE_VTEIMC, IXGBE_VTEIAC,
		IXGBE_VTEIAM, IXGBE_VTEITR(0), IXGBE_VTIVAR(0), IXGBE_VTIVAR_MISC
	};

	struct rte_dev_reg_set seg2_addr_set [] = {/* Receive DMA */
		{IXGBE_VFRDBAL(0), 2, 0x40}, {IXGBE_VFRDBAH(0), 2, 0x40},
		{IXGBE_VFRDLEN(0), 2, 0x40}, {IXGBE_VFRDH(0), 2, 0x40},
		{IXGBE_VFRDT(0), 2, 0x40}, {IXGBE_VFRXDCTL(0), 2, 0x40},
		{IXGBE_VFSRRCTL(0), 2, 0x40}
	};

	struct rte_dev_reg_set seg3_addr_set [] = { /* Receive */
		{IXGBE_VFPSRTYPE, 1, 0}
	};

	struct rte_dev_reg_set seg4_addr_set[] = { /* Transmit */
		{IXGBE_VFTDBAL(0), 2, 0x40}, {IXGBE_VFTDBAH(0), 2, 0x40},
		{IXGBE_VFTDLEN(0), 2, 0x40}, {IXGBE_VFTDH(0), 2, 0x40},
		{IXGBE_VFTDT(0), 2, 0x40}, {IXGBE_VFTXDCTL(0), 2, 0x40},
		{IXGBE_VFTDWBAL(0), 2, 0x40}, {IXGBE_VFTDWBAH(0), 2, 0x40}
	};


	count = (uint32_t)(sizeof(seg0_addr_set)/sizeof(uint32_t) +
		sizeof(seg2_addr_set)/sizeof(struct rte_dev_reg_set) +
		sizeof(seg3_addr_set)/sizeof(struct rte_dev_reg_set) +
		sizeof(seg4_addr_set)/sizeof(struct rte_dev_reg_set));

	if (mode == READ_REG_COUNT)
		return count;

	if ((reginfo == NULL) || (data == NULL))
		return EINVAL;

	if (reginfo->len < count) {
		PMD_INIT_LOG(ERR,"Expect # of registers (= %d) is less than available registers (%d)",
			reginfo->len, count);
		return EINVAL;
	}
	/* IXGBE_VFCTRL is a Write Only register, so just return 0 */
	regs[0] = 0x0;
	total = 1;

	for(i = 0; i < sizeof(seg0_addr_set)/sizeof(uint32_t); i++)
		regs[total++] = IXGBE_READ_REG(hw, seg0_addr_set[i]);

	total += ixgbevf_read_reg_set(hw, seg2_addr_set, &regs[total], sizeof(seg2_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbevf_read_reg_set(hw, seg3_addr_set, &regs[total], sizeof(seg3_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbevf_read_reg_set(hw, seg4_addr_set, &regs[total], sizeof(seg4_addr_set)/sizeof(struct rte_dev_reg_set));

	return total;
}

static int
ixgbevf_netdev_flag_sync(struct rte_eth_dev *dev, unsigned int change,
	unsigned int new_flag)
{
	unsigned int supported[] = {RTE_IFF_BROADCAST, RTE_IFF_LOOPBACK,
		RTE_IFF_PROMISC, RTE_IFF_ALLMULTI, RTE_IFF_MULTICAST};
	int i;

	for (i=0; i < sizeof(supported)/sizeof(unsigned int); i++){
		if (change & supported[i]){
			switch(supported[i]){
			case RTE_IFF_BROADCAST:
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
ixgbevf_get_reg_length(struct rte_eth_dev *dev)
{
	return ixgbevf_registers(dev, READ_REG_COUNT, NULL, NULL);
}

static int
ixgbevf_get_regs(struct rte_eth_dev *dev,
	      struct rte_dev_ethtool_reg *reginfo, void *data)
{
	return ixgbevf_registers(dev, READ_REGISTERS, reginfo, data);
}

static int
ixgbevf_get_drvinfo(struct rte_eth_dev *dev, struct rte_dev_ethtool_drvinfo *drvinfo)
{
	struct rte_eth_dev_info dev_info;
	const char driver_name[] = "ixgbevf\0";
	struct rte_pci_device *pci_dev;
	int status;
#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	const struct rte_bus *bus = NULL;
#endif

	if (drvinfo == NULL)
		return  -EINVAL;
#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	if (dev->dev_ops->fw_version_get)
		dev->dev_ops->fw_version_get(dev, drvinfo->fw_version, 
			sizeof(drvinfo->fw_version));
	else
		snprintf(drvinfo->fw_version, 4, "N/A");
#else
	snprintf(drvinfo->fw_version, 4, "N/A");
#endif
	memset(&dev_info, 0, sizeof(struct rte_eth_dev_info));
#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	dev_info.device = dev->device;
#endif
	dev->dev_ops->dev_infos_get(dev, &dev_info);
	if (dev_info.driver_name)
		memcpy(drvinfo->driver, dev_info.driver_name,
		strlen(dev_info.driver_name)+1);
	else
		memcpy(drvinfo->driver, driver_name, strlen(driver_name)+1);

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

	drvinfo->regdump_len = ixgbevf_get_reg_length(dev);
	drvinfo->eedump_len = 0;
	drvinfo->n_stats = sizeof(struct rte_eth_stats)/sizeof(uint64_t);
	drvinfo->testinfo_len = 0;
	drvinfo->n_priv_flags = RTE_PRIV_FLAGS_STR_LEN;

	return 0;
}

static int 
ixgbevf_begin(struct rte_eth_dev *dev)
{
	return 0;
}

static int
ixgbevf_complete(struct rte_eth_dev *dev)
{
	return 0;
}

static int 
ixgbevf_get_netdev_data(struct rte_eth_dev *dev,
	struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* update attributes that can be changed run-time */
	memcpy(&dev_ex->netdev_data.perm_addr, hw->mac.perm_addr, ETHER_ADDR_LEN);
	memcmp(netdev_data->dev_addr, hw->mac.perm_addr, ETHER_ADDR_LEN);
	dev_ex->netdev_data.mtu = dev->data->mtu;
    dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
    dev_ex->netdev_data.type = 1; /* ARPHRD_ETHER */
	memcpy((void *)netdev_data, &dev_ex->netdev_data, sizeof(struct netdev_priv_data));
	return 0;
}

static int
ixgbevf_set_netdev_data(struct rte_eth_dev *dev,
	struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	unsigned int change, old_flag;

	old_flag = dev_ex->netdev_data.flags;
	change = old_flag ^ netdev_data->flags;

	memcpy((void *)&dev_ex->netdev_data, netdev_data, sizeof(struct netdev_priv_data));
	if (memcmp(netdev_data->perm_addr, hw->mac.perm_addr, ETHER_ADDR_LEN))
		dev->dev_ops->mac_addr_set(dev, (void *)netdev_data->perm_addr);

	if (netdev_data->mtu != dev->data->mtu) {
		/* mtu change may require extra safe-guard step */
		if (dev->dev_ops->mtu_set)
			return (*dev->dev_ops->mtu_set)(dev, netdev_data->mtu);
		else
			dev->data->mtu = netdev_data->mtu;
	}

	if (change)
		return ixgbevf_netdev_flag_sync(dev, change, netdev_data->flags);
		
	return 0;
}

static const struct eth_dev_ethtool_ops ixgbevf_ethtool_ops = {
	.get_netdev_data = ixgbevf_get_netdev_data,
	.set_netdev_data = ixgbevf_set_netdev_data,
	.get_regs_len = ixgbevf_get_reg_length,
	.get_regs = ixgbevf_get_regs,
	.get_drvinfo = ixgbevf_get_drvinfo,
    .get_link = ixgbevf_get_link,
	/* pseudo function */
	.begin			= ixgbevf_begin,
	.complete		= ixgbevf_complete,
};

/* netdev op implementation */
static int
ixgbevf_dev_start(struct rte_eth_dev *dev)
{
	return dev->dev_ops->dev_start(dev);
}

static int
ixgbevf_dev_stop(struct rte_eth_dev *dev)
{
	dev->dev_ops->dev_stop(dev);
	return 0;
}

static int
ixgbevf_fix_features(struct rte_eth_dev *dev __rte_unused, uint64_t *features)
{
	uint64_t fix_up = *features;
	/* TODO when DCB is on, RXVLAN stripping is also on */
	*features = fix_up;

	return 0;
}

static int
ixgbevf_set_features(struct rte_eth_dev *dev, uint64_t features)
{
	if (dev->dev_ops->vlan_offload_set) {
		if (features & RTE_NETIF_F_HW_VLAN_RX)
			dev->dev_ops->vlan_offload_set(dev, 1);
		else
			dev->dev_ops->vlan_offload_set(dev, 0);
	}

	return 0;
}

static int
ixgbevf_validate_addr(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (is_valid_assigned_ether_addr((ether_addr_t *)hw->mac.addr))
		return 1;
	return 0;
}

static int
ixgbevf_change_mtu(struct rte_eth_dev *dev, int mtu)
{
	return dev->dev_ops->mtu_set(dev, mtu);
}

static int
ixgbevf_get_stats64(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
{
	int i;
	struct rte_eth_stats stats;

	dev->dev_ops->stats_get(dev, &stats);
	/* add old stats */
	out_stats->ipackets += stats.ipackets;
	out_stats->opackets += stats.opackets;
	out_stats->ibytes += stats.ibytes;
	out_stats->obytes += stats.obytes;
	out_stats->imissed += stats.imissed;
	out_stats->ierrors += stats.ierrors;
	out_stats->oerrors += stats.oerrors;
	out_stats->rx_nombuf += stats.rx_nombuf;

	return 0;
}

static int
ixgbevf_get_stats(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
{
	dev->dev_ops->stats_get(dev, out_stats);
	return 0;
}

static int
ixgbevf_change_rx_flag(struct rte_eth_dev *dev, int flags)
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
ixgbevf_dev_init(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	dev_ex->netdev_data.features = RTE_NETIF_F_SG |
		RTE_NETIF_F_IP_CSUM | RTE_NETIF_F_IPV6_CSUM |
		RTE_NETIF_F_HW_VLAN_TX | RTE_NETIF_F_HW_VLAN_RX |
		RTE_NETIF_F_TSO | RTE_NETIF_F_TSO6 |
		RTE_NETIF_F_LRO | RTE_NETIF_F_GRO |
		RTE_NETIF_F_HW_VLAN_FILTER | RTE_NETIF_F_SCTP_CSUM |
		RTE_NETIF_F_HIGHDMA;
	dev_ex->netdev_data.hw_features = dev_ex->netdev_data.features;
	dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
	memcpy(&dev_ex->netdev_data.perm_addr, hw->mac.perm_addr, ETHER_ADDR_LEN);

	return 0;
}

static int
ixgbevf_dev_uninit(struct rte_eth_dev *dev)
{
	return 0;
}

static const struct eth_dev_netdev_ops ixgbevf_netdev_ops = {
	.init				= ixgbevf_dev_init,
	.uninit				= ixgbevf_dev_uninit,
	.open				= ixgbevf_dev_start,
	.stop				= ixgbevf_dev_stop,
	.validate_addr		= ixgbevf_validate_addr,
	.change_mtu			= ixgbevf_change_mtu,
	.get_stats64		= ixgbevf_get_stats64,
	.get_stats			= ixgbevf_get_stats,
	.fix_features		= ixgbevf_fix_features,
	.set_features		= ixgbevf_set_features,
	.change_rx_flag		= ixgbevf_change_rx_flag,

};

static int
eth_ixgbevf_dev_init_ex(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex;

	dev_ex = rte_eth_get_devex_by_dev(dev);
	dev_ex->dev_ethtool_ops = &ixgbevf_ethtool_ops;
	dev_ex->dev_netdev_ops = &ixgbevf_netdev_ops;
	dev_ex->is_vf		= true;

	return 0;
}
RTE_PMD_EX_REGISTER_PCI(net_ixgbe_vf, eth_ixgbevf_dev_init_ex);
