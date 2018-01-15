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
#include "rte_ethdev.h"
#include "rte_ethdev_ex.h"
#include "rte_manage.h"
#include "vni_netdev_flags.h"
#include "rte_vf_flags.h"
#include "rte_netdev_ops.h"

extern int rte_eth_dev_is_valid_port(port_t);

int 
rte_netdev_init(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->init, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->init)(dev);
}

int
rte_netdev_uninit(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->uninit, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->uninit)(dev);
}

int
rte_netdev_open(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->open, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->open)(dev);
}

int
rte_netdev_stop(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->stop, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->stop)(dev);
}

int
rte_netdev_start_xmit(port_t port_id, struct rte_dev_pkt_buff *skb)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->start_xmit, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->start_xmit)(dev, skb);
}

int
rte_netdev_select_queue(port_t port_id, struct rte_dev_pkt_buff *skb)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->select_queue, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->select_queue)(dev, skb);
}

int
rte_netdev_change_rx_flags(port_t port_id, int flags)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->change_rx_flag, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->change_rx_flag)(dev, flags);
}

int
rte_netdev_set_rx_mode(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_rx_mode, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_rx_mode)(dev);
}

int
rte_netdev_set_multicast_list(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_multicast_list, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_multicast_list)(dev);
}

int
rte_netdev_set_mac_address(port_t port_id, void *addr)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_mac_addr, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_mac_addr)(dev, addr);
}

int
rte_netdev_validate_addr(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->validate_addr, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->validate_addr)(dev);
}

int
rte_netdev_do_ioctl(port_t port_id, struct rte_dev_ifreq *ifr, int cmd)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->do_ioctl, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->do_ioctl)(dev, ifr, cmd);
}

int
rte_netdev_set_config(port_t port_id, struct rte_dev_ifmap *map)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_config, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_config)(dev, map);
}

int
rte_netdev_change_mtu(port_t port_id, int new_mtu)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->change_mtu, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->change_mtu)(dev, new_mtu);
}

int
rte_netdev_tx_timeout(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->tx_timeout, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->tx_timeout)(dev);
}

int
rte_netdev_get_stats64(port_t port_id, struct rte_eth_stats *in_stats)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->get_stats64, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->get_stats64)(dev, in_stats);

	return status;
}

int
rte_netdev_get_stats(port_t port_id, struct rte_eth_stats *ret)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->get_stats, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->get_stats)(dev, ret);

	return status;
}


int
rte_netdev_vlan_rx_add_vid(port_t port_id, unsigned short vid)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->vlan_rx_add_vid, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->vlan_rx_add_vid)(dev, vid);
}

int
rte_netdev_vlan_rx_kill_vid(port_t port_id, unsigned short vid)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->vlan_rx_kill_vid, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->vlan_rx_kill_vid)(dev, vid);
}

int
rte_netdev_set_all_queues_drop_en(port_t port_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_all_queues_drop_en, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_all_queues_drop_en)(dev, on);
}

/* PF set VF configuraiton */
int
rte_netdev_is_vf_enabled(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->is_vf_enabled, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->is_vf_enabled)(dev);
}

int
rte_netdev_set_vf_mac_addr(port_t port_id, int vf_id, uint8_t *mac)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_mac_addr, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_mac_addr)(dev, vf_id, mac);
}

int
rte_netdev_set_vf_promisc(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_promisc, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->set_vf_promisc)(dev, vf_id, on);
	
	if (status >= 0) {
		dev_ex->dev_iff_flag &= ~RTE_IFF_PROMISC;
		if (on)
			dev_ex->dev_iff_flag |= RTE_IFF_PROMISC;
	}

	return status;
}

int
rte_netdev_set_vf_allmulticast(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_allmulticast, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->set_vf_allmulticast)(dev, vf_id, on);

	if (status >= 0) {
		dev_ex->dev_iff_flag &= ~RTE_IFF_ALLMULTI;
		if (on)
			dev_ex->dev_iff_flag |= RTE_IFF_ALLMULTI;
	}

	return status;
}

int
rte_netdev_set_vf_broadcast(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_broadcast, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->set_vf_broadcast)(dev, vf_id, on);

	if (status >= 0) {
		dev_ex->dev_iff_flag &= ~RTE_IFF_BROADCAST;
		if (on)
			dev_ex->dev_iff_flag |= RTE_IFF_BROADCAST;
	}

	return status;
}

int
rte_netdev_set_vf_vlan(port_t port_id, int vf_id, uint16_t vlan_id,
									  uint8_t qos)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_vlan, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_vlan)(dev, vf_id, vlan_id, qos);
}

int
rte_netdev_set_vf_rate(port_t port_id, int vf_id, int max_tx_rate)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_rate, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_rate)(dev, vf_id, max_tx_rate);
}

int
rte_netdev_get_vf_config(port_t port_id, int vf_id,
	struct common_vf_info *ivi)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->get_vf_config, -ENOTSUP);
	status = (*dev_ex->dev_netdev_ops->get_vf_config)(dev, vf_id, &dev_ex->vf_info[vf_id]);
	memcpy(ivi, &dev_ex->vf_info[vf_id], sizeof(struct common_vf_info));

	return status;
}

int
rte_netdev_set_vf_spoofchk(port_t port_id, int vf_id, int enable)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_spoofchk, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_spoofchk)(dev, vf_id, enable);
}

int
rte_netdev_set_vf_link_state(port_t port_id, int vf_id, int link)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_link_state, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_link_state)(dev, vf_id, link);
}

int
rte_netdev_ping_vfs(port_t port_id, int vf_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->ping_vfs, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->ping_vfs)(dev, vf_id);
}

int
rte_netdev_set_vf_trust(port_t port_id, int vf_id, int setting)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	if (setting)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_TRUSTED_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_TRUSTED_BIT);

	return 0;
}

int
rte_netdev_set_vf_mac_anti_spoof(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_mac_anti_spoof, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_mac_anti_spoof)(dev, vf_id, on);
}

int
rte_netdev_set_vf_vlan_anti_spoof(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_vlan_anti_spoof, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_vlan_anti_spoof)(dev, vf_id, on);
}

int
rte_netdev_set_vf_vlan_filter(port_t port_id, uint64_t vf_mask, uint16_t vlan_id,
	uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;
	uint64_t bit_mask = 1;
	int i;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	RET_ERR_ON_VF_ENABLE(port_id);

	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	for(i = 0; i < 64; i++) {
		if (vf_mask & (bit_mask << i)) {
			RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, i);
		}
	}
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_vlan_filter, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_vlan_filter)(dev, vf_mask, vlan_id, on);
}

int
rte_netdev_set_vf_vlan_insert(port_t port_id, int vf_id, uint16_t vlan_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_vlan_insert, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_vlan_insert)(dev, vf_id, vlan_id);
}

int
rte_netdev_set_vf_rate_limit(port_t port_id, int vf_id, uint16_t tx_rate,
	uint64_t q_mask)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_rate_limit, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_rate_limit)(dev, vf_id, tx_rate, q_mask);
}

int rte_netdev_set_vf_vlan_stripq(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_vlan_stripq, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_vlan_stripq)(dev, vf_id, on);
}

int
rte_netdev_set_vf_rxmode(port_t port_id, int vf_id, uint16_t rx_mask, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_rxmode, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_rxmode)(dev, vf_id, rx_mask, on);
}

int
rte_netdev_set_vf_rx(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_rx, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_rx)(dev, vf_id, on);
}

int
rte_netdev_set_vf_tx(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_tx, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_tx)(dev, vf_id, on);
}

int
rte_netdev_get_vf_stats(port_t port_id, int vf_id, struct rte_eth_stats *stats)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->get_vf_stats, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->get_vf_stats)(dev, vf_id, stats);
}

int
rte_netdev_reset_vf_stats(port_t port_id, int vf_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->reset_vf_stats, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->reset_vf_stats)(dev, vf_id);
}

int
rte_netdev_set_vf_split_drop_en(port_t port_id, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
	struct rte_pci_device *pci_dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	RET_ERR_ON_VF_ENABLE(port_id);
	RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id);

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_vf_split_drop_en, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_vf_split_drop_en)(dev, vf_id, on);
}

int
rte_netdev_set_tx_loopback(port_t port_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_tx_loopback, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_tx_loopback)(dev, on);
}

int
rte_netdev_macsec_enable(port_t port_id, uint8_t en, uint8_t rp)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_enable, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_enable)(dev, en, rp);
}

int
rte_netdev_macsec_disable(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_disable, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_disable)(dev);
}

int
rte_netdev_macsec_config_txsc(port_t port_id, uint8_t *mac)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_config_txsc, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_config_txsc)(dev, mac);
}

int
rte_netdev_macsec_config_rxsc(port_t port_id, uint8_t *mac, uint16_t pi)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_config_rxsc, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_config_rxsc)(dev, mac, pi);
}

int
rte_netdev_macsec_select_txsa(port_t port_id, uint8_t idx, uint8_t an,
		uint32_t pn, uint8_t *key)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_select_txsa, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_select_txsa)(dev, idx, an, pn, key);
}

int
rte_netdev_macsec_select_rxsa(port_t port_id, uint8_t idx, uint8_t an,
		uint32_t pn, uint8_t *key)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops || (dev_ex->is_vf))
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->macsec_select_rxsa, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->macsec_select_rxsa)(dev, idx, an, pn, key);
}

int
rte_netdev_fix_features(port_t port_id, uint64_t *features)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->fix_features, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->fix_features)(dev, features);
}

int
rte_netdev_set_features(port_t port_id, uint64_t features)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_netdev_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_netdev_ops->set_features, -ENOTSUP);
	return (*dev_ex->dev_netdev_ops->set_features)(dev, features);
}

