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
#ifndef _RTE_NETDEV_OPS_H_
#define _RTE_NETDEV_OPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <linux/types.h>
#include <rte_manage.h>
#include <rte_netdev_types.h>

// RTE Netdev API
/**
 * Initialize a port (system level, pci, initialization)
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_init(port_t port_id);

/**
 * Uninitialize a port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_uninit(port_t port_id);

/**
 * Start a port (configure port based upon configuration parameters)
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param on
 *    1 - set the queue drop enable bit for all pools.
 *    0 - reset the queue drop enable bit for all pools.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_open(port_t port_id);

/**
 * Stop a port (no traffic go through rx/tx queues)
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_stop(port_t port_id);

int rte_netdev_start_xmit(port_t port_id, struct rte_dev_pkt_buff *skb);
int rte_netdev_select_queue(port_t port_id, struct rte_dev_pkt_buff *skb);
int rte_netdev_change_rx_flags(port_t port_id, int flags);
/**
 * Set port receive mode: unicast/multicast/vlan promiscuous mode and uc/mc address list
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_rx_mode(port_t port_id);

int rte_netdev_set_multicast_list(port_t port_id);

/**
 * Set port default MAC address
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param addr
 *    The 48-bit MAC address to be assigned to port
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_mac_address(port_t port_id, void *addr);

/**
 * Return if default MAC address is valid
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *	 - (1) if sucessful and default MAC is a valid address
 *   - (0) if successful and default MAC is not valid
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_validate_addr(port_t port_id);

int rte_netdev_do_ioctl(port_t port_id, struct rte_dev_ifreq *ifr, int cmd);

/**
 * set all queues drop enable bit
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param on
 *    1 - set the queue drop enable bit for all pools.
 *    0 - reset the queue drop enable bit for all pools.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_change_mtu(port_t port_id, int new_mtu);

/**
 * Callback when Tx has not made any progress to avoid Tx hang
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_tx_timeout(port_t port_id);

/**
 * Retrieve the accumulated port statistics
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param stats
 *    The last accumulated statistics
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_get_stats64(port_t port_id, struct rte_eth_stats *stats);

/**
 * Retrieve the statistics since last call
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param stats
 *    Contain statistic since last retrieval
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_get_stats(port_t port_id, struct rte_eth_stats *stats);

/**
 * Add a new vlan-id for filtering
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vid
 *    The new vlan to be added for vlan filtering.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_vlan_rx_add_vid(port_t port_id, unsigned short vid);

/**
 * Remove the provided vlan-id from vlan filtering list
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vid
 *    The new vlan to be deleted from vlan filtering
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_vlan_rx_kill_vid(port_t port_id, unsigned short vid);

/**
 * Enable/disable packets to be dropped when port is running out of descriptors
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param on
 *    1 - enable packet dropping when run out of descriptors.
 *    0 - disable packet dropping when run out of descriptors.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_all_queues_drop_en(port_t port_id, uint8_t on);

/* Manage VF configurations through PF */

/**
 * Query if SR-IOV is enabled by PF
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 *
 * @return
 *   - (1) if SR-IOV is enabled.
 *   - (0) if sR_IOV is disabled.
 *   - (-ENODEV) if port identifier is invalid.
 *   - (-ENOTSUP) if hardware doesn't support tunnel type.
 */
int rte_netdev_is_vf_enabled(port_t port_id);

/**
 * Set the VF MAC address.
 *
 * PF should set MAC address before VF initialized, if PF sets the MAC
 * address after VF initialized, new MAC address won't be effective until
 * VF reinitialize.
 *
 * This will remove all existing MAC filters.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF id.
 * @param mac_addr
 *   VF MAC address.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if *vf* or *mac_addr* is invalid.
 */
int rte_netdev_set_vf_mac_addr(port_t port_id, int vf_id, uint8_t *mac);

/**
 * Enable VF unicast promiscuous mode.
 *
 * The feature may not be available for all devices.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param on
 *   1: enable 0: disable
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_promisc(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable VF multicast promiscuous mode.
 *
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param on
 *   1: enable 0: disable
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_allmulticast(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable VF broadcast mode.
 *
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param on
 *   1: enable 0: disable
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_broadcast(port_t port_id, int vf_id, uint8_t on);

/**
 * Set VF vlan
 *
 * A vlan has 16 bits. Among them [11:0]: vlan tag (vlan tag 0 is reserved
 * for special device), [14:12]: qos and [15]
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param vlan_id
 *   12-bit vlan tag
 * @param qos
 *   3-bit QoS
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_vlan(port_t port_id, int vf_id, uint16_t vlan_id,
									  uint8_t qos);

/**
 * Confgure VF (Tx) rate
 *
 * Set VF Tx transmit speed
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param max_tx_rate
 *   Maximum Tx speed
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_rate(port_t port_id, int vf_id, int max_tx_rate);


/**
 * Retrieve VF device configuration
 *
 * Retrieve selected VF configuration parameters
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param ivi
 *   The VF configuration data structure
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_get_vf_config(port_t port_id, int vf_id,
									  struct rte_dev_ifla_vf_info *ivi);

/**
 * Confgure VF spoof check
 *
 * Enable/disable VF spoof check
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param enable
 *   -1: enable spoof check 0: disable spoof check
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_spoofchk(port_t port_id, int vf_id, int enable);

/**
 * Confgure VF link state
 *
 * Set VF device link state up/down
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param link
 *   -1: up 0: disable down
 *
 * @return
 *   - 0: Success.
 */
int rte_netdev_set_vf_link_state(port_t port_id, int vf_id, int link);

/**
 * Notify VF when PF link status changes.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf
 *   VF id.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if *vf* invalid.
 */
int rte_netdev_ping_vfs(port_t port_id, int vf_id);

/**
 * Confgure VF as a trust/untrust agent
 *
 * Enable/disable VF spoof check
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param vf_id
 *   VF index. An VF can have multiple queue.
 * @param setting
 *   -1: set VF as trusted 0: set VF as not trusted
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_trust(port_t port_id, int vf_id, int enable);

/**
 * Enable/Disable vf MAC-base anti-spoof
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf
 *    ID specifying VF.
 * @param vlan_id
 *    0 - Disable VF's vlan insert.
 *    n - Enable; n is inserted as the vlan id.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_mac_anti_spoof(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable/Disable vf VLAN-base anti-spoof
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf
 *    ID specifying VF.
 * @param vlan_id
 *    0 - Disable VF's vlan insert.
 *    n - Enable; n is inserted as the vlan id.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_vlan_anti_spoof(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable/Disable VF VLAN filter
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vlan_id
 *    ID specifying VLAN
 * @param vf_mask
 *    Mask to filter VF's
 * @param on
 *    0 - Disable VF's VLAN filter.
 *    1 - Enable VF's VLAN filter.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 *   - (-ENOTSUP) not supported by firmware.
 */
int rte_netdev_set_vf_vlan_filter(port_t port_id, uint64_t vf_mask, uint16_t vlan,
											 uint8_t on);

/**
 * Enable/Disable vf vlan insertion
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf
 *    ID specifying VF.
 * @param vlan_id
 *    0 - Disable VF's vlan insert.
 *    n - Enable; n is inserted as the vlan id.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_vlan_insert(port_t port_id, int vf_id, uint16_t vlan_id);

/**
 * Set the rate limitation for a vf on an Ethernet device.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf
 *   VF id.
 * @param tx_rate
 *   The tx rate allocated from the total link speed for this VF id.
 * @param q_msk
 *   The queue mask which need to set the rate.
 * @return
 *   - (0) if successful.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 *   - (-ENODEV) if *port_id* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_rate_limit(port_t port_id, int vf_id, uint16_t tx_rate,
											  uint64_t q_msk);

/**
 * Enable/Disable vf vlan strip for all queues in a pool
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf
 *    ID specifying VF.
 * @param on
 *    1 - Enable VF's vlan strip on RX queues.
 *    0 - Disable VF's vlan strip on RX queues.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_vlan_stripq(port_t port_id, int vf_id, uint8_t on);

/**
 * Set RX L2 Filtering mode of a VF of an Ethernet device.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf
 *   VF id.
 * @param rx_mask
 *    The RX mode mask, which is one or more of accepting Untagged Packets,
 *    packets that match the PFUTA table, Broadcast and Multicast Promiscuous.
 *    ETH_VMDQ_ACCEPT_UNTAG,ETH_VMDQ_ACCEPT_HASH_UC,
 *    ETH_VMDQ_ACCEPT_BROADCAST and ETH_VMDQ_ACCEPT_MULTICAST will be used
 *    in rx_mode.
 * @param on
 *    1 - Enable a VF RX mode.
 *    0 - Disable a VF RX mode.
 * @return
 *   - (0) if successful.
 *   - (-ENOTSUP) if hardware doesn't support.
 *   - (-ENODEV) if *port_id* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_rxmode(port_t port_id, int vf_id, uint16_t rx_mask, uint8_t on);

/**
 * Enable or disable a VF traffic receive of an Ethernet device.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf
 *   VF id.
 * @param on
 *    1 - Enable a VF traffic receive.
 *    0 - Disable a VF traffic receive.
 * @return
 *   - (0) if successful.
 *   - (-ENOTSUP) if hardware doesn't support.
 *   - (-ENODEV) if *port_id* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_rx(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable or disable a VF traffic transmit of the Ethernet device.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param vf
 *   VF id.
 * @param on
 *    1 - Enable a VF traffic transmit.
 *    0 - Disable a VF traffic transmit.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port_id* invalid.
 *   - (-ENOTSUP) if hardware doesn't support.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_tx(port_t port_id, int vf_id, uint8_t on);

/**
 * Get VF's statistics
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf_id
 *    VF on which to get.
 * @param stats
 *    A pointer to a structure of type *rte_eth_stats* to be filled with
 *    the values of device counters for the following set of statistics:
 *   - *ipackets* with the total of successfully received packets.
 *   - *opackets* with the total of successfully transmitted packets.
 *   - *ibytes*   with the total of successfully received bytes.
 *   - *obytes*   with the total of successfully transmitted bytes.
 *   - *ierrors*  with the total of erroneous received packets.
 *   - *oerrors*  with the total of failed transmitted packets.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_get_vf_stats(port_t port_id, int vf_id, struct rte_eth_stats *stats);

/**
 * Clear VF's statistics
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf_id
 *    VF on which to get.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_reset_vf_stats(port_t port_id, int vf_id);

/**
 * set drop enable bit in the VF split rx control register
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param vf_id
 *    ID specifying VF.
 * @param on
 *    1 - set the drop enable bit in the split rx control register.
 *    0 - reset the drop enable bit in the split rx control register.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_vf_split_drop_en(port_t port_id, int vf_id, uint8_t on);

/**
 * Enable/Disable TX loopback on all the PF and VFs.
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param on
 *    1 - Enable TX loopback.
 *    0 - Disable TX loopback.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_tx_loopback(port_t port_id, uint8_t on);

/**
 * Enable MACsec offload.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param en
 *    1 - Enable encryption (encrypt and add integrity signature).
 *    0 - Disable encryption (only add integrity signature).
 * @param rp
 *    1 - Enable replay protection.
 *    0 - Disable replay protection.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 */
int rte_netdev_macsec_enable(port_t port_id, uint8_t en, uint8_t rp);

/**
 * Disable MACsec offload.
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 */
int rte_netdev_macsec_disable(port_t port_id);

/**
 * Configure Tx SC (Secure Connection).
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param mac
 *   The MAC address on the local side.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 */
int rte_netdev_macsec_config_txsc(port_t port_id, uint8_t *mac);

/**
 * Configure Rx SC (Secure Connection).
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param mac
 *   The MAC address on the remote side.
 * @param pi
 *   The PI (port identifier) on the remote side.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 */
int rte_netdev_macsec_config_rxsc(port_t port_id, uint8_t *mac, uint16_t pi);

/**
 * Enable Tx SA (Secure Association).
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param idx
 *   The SA to be enabled (0 or 1).
 * @param an
 *   The association number on the local side.
 * @param pn
 *   The packet number on the local side.
 * @param key
 *   The key on the local side.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_macsec_select_txsa(port_t port_id, uint8_t idx, uint8_t an,
		uint32_t pn, uint8_t *key);

/**
 * Enable Rx SA (Secure Association).
 *
 * @param port
 *   The port identifier of the Ethernet device.
 * @param idx
 *   The SA to be enabled (0 or 1)
 * @param an
 *   The association number on the remote side.
 * @param pn
 *   The packet number on the remote side.
 * @param key
 *   The key on the remote side.
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-ENOTSUP) if hardware doesn't support this feature.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_macsec_select_rxsa(port_t port_id, uint8_t idx, uint8_t an,
		uint32_t pn, uint8_t *key);

/**
 * Fixup netdev features used for set_features function to setup configurations
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param features
 *	  [IN]The featues determined by kernel (or third-party application) on which
 *		features need to set or reset.
 *	  [OUT]Fix up the input value and return to calling application.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_fix_features(port_t port_id, uint64_t *features);

/**
 * Setup device based upon the provided 64-bit feature list
 *
 * @param port
 *    The port identifier of the Ethernet device.
 * @param features
 *    A 64-bit feature list
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_netdev_set_features(port_t port_id, uint64_t features);

int rte_netdev_udp_tunnel_add(port_t port_id, struct rte_dev_udp_tunnel_info *ti);
int rte_netdev_udp_tunnel_del(port_t port_id, struct rte_dev_udp_tunnel_info *ti);

#ifdef __cplusplus
 }
#endif

#endif // _RTE_NETDEV_OPS_H_
