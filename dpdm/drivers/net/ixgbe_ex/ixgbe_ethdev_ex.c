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
#include <linux/ethtool.h>
#include <sys/queue.h>
#include <string.h>
#include <rte_ethdev.h>
#include <rte_dpdm.h>
#include <vni_netdev_flags.h>
#include <rte_pci.h>
#include <generic/rte_atomic.h>
#include <rte_version.h>
#include <vni_share_types.h>
#include <rte_vf_flags.h>

#include "ixgbe_logs.h"
#include "base/ixgbe_api.h"
#include "base/ixgbe_vf.h"
#include "base/ixgbe_common.h"
#include "ixgbe_ethdev.h"
#include "ixgbe_bypass.h"
#include "ixgbe_rxtx.h"
#include "base/ixgbe_type.h"
#include "base/ixgbe_phy.h"
#include "base/ixgbe_mbx.h"
#if (RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0)) && \
	(RTE_VERSION > RTE_VERSION_NUM(16, 11, 0, 0))
#include "rte_pmd_ixgbe.h"
#endif

#ifndef IXGBE_SECTX_MINSECIFG_MASK
#define IXGBE_SECTX_MINSECIFG_MASK		0x0000000F
#define IXGBE_MACSEC_PNTHRSH            0xFFFFFE00
#endif

static int eth_ixgbe_dev_init_ex(struct rte_eth_dev *dev);
static unsigned int ixgbe_dev_get_iff_flag(struct rte_eth_dev *dev);

/* ethtool_op APIs */
static int ixgbe_get_reg_length(struct rte_eth_dev *dev);
static int ixgbe_get_regs(struct rte_eth_dev *dev, struct rte_dev_ethtool_reg *reg_info, void *data);
static int ixgbe_get_strings(struct rte_eth_dev *dev, uint32_t stringset, uint8_t *ctx);
static int ixgbe_get_sset_count(struct rte_eth_dev *dev, int val);

/* netdev_op APIs */
static int ixgbe_dev_start(struct rte_eth_dev *dev);
static int ixgbe_dev_stop(struct rte_eth_dev *dev);
static int ixgbe_change_rx_flag(struct rte_eth_dev *dev, int flags);
static int ixgbe_set_rx_mode(struct rte_eth_dev *dev);
static int ixgbe_set_mac_addr(struct rte_eth_dev *dev, void *mac_addr);
static int ixgbe_validate_addr(struct rte_eth_dev *dev);
static int ixgbe_change_mtu(struct rte_eth_dev *dev, int mtu);
static int ixgbe_get_stats64(struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static int ixgbe_get_stats(struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static int ixgbe_vlan_rx_add_vid(struct rte_eth_dev *dev, uint16_t vlan_id);
static int ixgbe_vlan_rx_kill_vid(struct rte_eth_dev *dev, uint16_t vlan_id);
static int ixgbe_set_features(struct rte_eth_dev *dev, uint64_t features);
static int ixgbe_fix_features(struct rte_eth_dev *dev, uint64_t *features);
static int ixgbe_set_all_queues_drop_en(struct rte_eth_dev *dev, uint8_t on);
static int ixgbe_set_tx_loopback(struct rte_eth_dev *dev, uint8_t on);

/* PF manage VF routines */
static int ixgbe_is_vf_enabled(struct rte_eth_dev *dev);
static int ixgbe_set_vf_mac_addr(struct rte_eth_dev *dev, int vf_id, uint8_t *mac);
static int ixgbe_set_vf_promisc(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_allmulticast(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_broadcast(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_vlan(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id,
							 uint8_t qos);
static int ixgbe_set_vf_rate(struct rte_eth_dev *dev, int vf_id, int max_tx_rate);
static int ixgbe_set_vf_tx_rate(struct rte_eth_dev *dev, int vf_id, int min_tx_rate,
								int max_tx_rate);
static int ixgbe_get_vf_config(struct rte_eth_dev *dev, int vf_id,
							struct common_vf_info *ivi);
static int ixgbe_set_vf_spoofchk(struct rte_eth_dev *dev, int vf_id, int enable);
static int ixgbe_set_vf_link_state(struct rte_eth_dev *dev, int vf_id, int link);
static int ixgbe_ping_vfs(struct rte_eth_dev *dev, int vf_id);
static int ixgbe_set_vf_mac_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_vlan_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_vlan_filter(struct rte_eth_dev *dev, uint64_t vf_mask, uint16_t vlan,
									uint8_t on);
static int ixgbe_set_vf_vlan_insert(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id);
static int ixgbe_set_vf_rate_limits(struct rte_eth_dev *dev, int vf_id, uint16_t tx_rate,
								   uint64_t q_msk);
static int ixgbe_set_vf_vlan_stripq(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_rxmode(struct rte_eth_dev *dev, int vf_id, uint16_t rx_mask, uint8_t on);
static int ixgbe_set_vf_rx(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_set_vf_tx(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int ixgbe_get_vf_stats(struct rte_eth_dev *dev, int vf_id, struct rte_eth_stats *stats);
static int ixgbe_reset_vf_stats(struct rte_eth_dev *dev, int vf_id);
static int ixgbe_set_vf_split_drop_en(struct rte_eth_dev *dev, int vf_id, uint8_t on);

/* MACSec */
static int ixgbe_macsec_enable(struct rte_eth_dev *dev, uint8_t en, uint8_t rp);
static int ixgbe_macsec_disable(struct rte_eth_dev *dev);
static int ixgbe_macsec_config_txsc(struct rte_eth_dev *dev, uint8_t *mac);
static int ixgbe_macsec_config_rxsc(struct rte_eth_dev *dev, uint8_t *mac, uint16_t pi);
static int ixgbe_macsec_select_txsa(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
									uint32_t pn, uint8_t *key);
static int ixgbe_macsec_select_rxsa(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
									uint32_t pn, uint8_t *key);
/* helper routines */
#if RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0) 
int ixgbe_vt_check(struct ixgbe_hw *hw)
{
	uint32_t value;

	/* if Virtualization Technology is enabled */
	value = IXGBE_READ_REG(hw, IXGBE_VT_CTL);
	if (!(value & IXGBE_VT_CTL_VT_ENABLE)) {
		PMD_INIT_LOG(ERR, "VT must be enabled for this setting");
		return -1;
	}

	return 0;
}
#endif

bool ixgbe_vlan_strip_enabled(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		struct ixgbe_rx_queue *rxq = dev->data->rx_queues[i];
		u8 reg_idx = rxq->reg_idx;
		u32 vlnctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(reg_idx));
		if (vlnctrl & IXGBE_RXDCTL_VME)
			return true;
	}
	return false;
}

int 
ixgbe_enable_port_vlan(struct rte_eth_dev *dev,
	int vf_id, uint16_t vlan_id, uint8_t qos)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_vf_info *vfinfo;
	uint32_t value, q_per_pool;
	int ret, i;

	/* Rx vlan setting */
	ret = hw->mac.ops.set_vfta(hw, vlan_id, vf_id, true, false);
	if (ret)
		return ret;
	/* Tx vlan setting */
	value = vlan_id | (qos << 12) | IXGBE_VMVIR_VLANA_DEFAULT;
	IXGBE_WRITE_REG(hw, IXGBE_VMVIR(vf_id), value);

	/* L2 Filtering setting   */
	/* Reject untagged packets*/
	value = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id)) | IXGBE_VMOLR_BAM;
	value &= ~IXGBE_VMOLR_AUPE;
	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), value);

	vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	if (vfinfo[vf_id].spoofchk_enabled)
		hw->mac.ops.set_vlan_anti_spoofing(hw, true, vf_id);
	vfinfo[vf_id].vlan_count++;

	/* enable hide vlan on X550 */
	if (hw->mac.type >= ixgbe_mac_X550) {
		q_per_pool = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
		for(i = vf_id*q_per_pool; i < (vf_id+1)*q_per_pool; i++) {
			value = IXGBE_QDE_WRITE | IXGBE_QDE_ENABLE | IXGBE_QDE_HIDE_VLAN;
			value |= (i << IXGBE_QDE_IDX_SHIFT);
			IXGBE_WRITE_REG(hw, IXGBE_QDE, value);
		}
	}

	vfinfo[vf_id].default_vf_vlan_id = vlan_id;
	RTE_LOG(INFO, PMD, 
		"Setting VF %d with VLAN %x, QOS 0x%x\n", vf_id, vlan_id, qos);

	return 0;
}

int ixgbe_disable_port_vlan(struct rte_eth_dev *dev, int vf_id)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_vf_info *vfinfo;
	uint32_t value, q_per_pool;
	int ret, i;

	vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));

	/* Rx VLAN setting */
	ret = hw->mac.ops.set_vfta(hw, vfinfo[vf_id].default_vf_vlan_id, vf_id, false, false);
	if (ret)
		return ret;

	/* Tx VLAN setting */
	IXGBE_WRITE_REG(hw, IXGBE_VMVIR(vf_id), 0);

	/* L2 Filtering setting */
	value = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id));
	value |= IXGBE_VMOLR_BAM | IXGBE_VMOLR_AUPE;
	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), value);

	hw->mac.ops.set_vlan_anti_spoofing(hw, false, vf_id);
	if (vfinfo[vf_id].vlan_count)
		vfinfo[vf_id].vlan_count--;
	/* disable hide vlan on X550 */
	if (hw->mac.type >= ixgbe_mac_X550) {
		q_per_pool = RTE_ETH_DEV_SRIOV(dev).nb_q_per_pool;
		for(i = vf_id*q_per_pool; i < (vf_id+1)*q_per_pool; i++) {
			value = IXGBE_QDE_WRITE | IXGBE_QDE_ENABLE;
			value |= i << IXGBE_QDE_IDX_SHIFT;
			IXGBE_WRITE_REG(hw, IXGBE_QDE, value);
		}
	}
	vfinfo[vf_id].default_vf_vlan_id = 0;

	return 0;
}

int
ixgbe_atomic_read_link_status(struct rte_eth_dev *dev,
	struct rte_eth_link *link)
{
	struct rte_eth_link *dst = link;
	struct rte_eth_link *src = &(dev->data->dev_link);

	if (rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
		*(uint64_t *)src) == 0)
		return -1;

	return 0;
}

static int
ixgbe_fw_version_get(struct rte_eth_dev *dev, char *fw_version, size_t fw_size)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	u16 eeprom_verh, eeprom_verl;
	u32 etrack_id;
	int ret;

	ixgbe_read_eeprom(hw, 0x2e, &eeprom_verh);
	ixgbe_read_eeprom(hw, 0x2d, &eeprom_verl);

	etrack_id = (eeprom_verh << 16) | eeprom_verl;
	ret = snprintf(fw_version, fw_size, "0x%08x", etrack_id);

	ret += 1; /* add the size of '\0' */
	if (fw_size < (u32)ret)
		return ret;
	else
		return 0;
}

inline uint32_t ixgbe_queue_per_pool(struct ixgbe_hw *hw)
{
	if (hw->mac.type == ixgbe_mac_82598EB)
		return hw->mac.max_rx_queues /
		ETH_16_POOLS;
	else
		return hw->mac.max_rx_queues /
		ETH_64_POOLS;
}

static uint32_t
ixgbe_read_reg_set(struct ixgbe_hw *hw, struct rte_dev_reg_set *reg_set, uint32_t *reg,
	uint32_t set_count)
{
	uint32_t i, j,total = 0;
	uint32_t count, stride;

	for(i = 0; i < set_count; i++) {
		count = reg_set[i].count;
		if (reg) {
			stride = reg_set[i].stride;
			if (count > 1) {
				for(j = 0; j < count; j++)
					reg[total+j] = IXGBE_READ_REG(hw, reg_set[i].base_addr + j*stride);
			} else {
				reg[total] = IXGBE_READ_REG(hw, reg_set[i].base_addr);
			}
		}
		total += count;
	}
	return total;
}

static int
ixgbe_set_broadcast(struct rte_eth_dev *dev, uint8_t on)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);

	if (on)
		fctrl |= IXGBE_FCTRL_BAM;
	else
		fctrl &= (~IXGBE_FCTRL_BAM);

	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

	return 0;
}

static int
ixgbe_netdev_flag_sync(struct rte_eth_dev *dev, unsigned int change,
	unsigned int new_flag)
{
	unsigned int supported[] = {RTE_IFF_BROADCAST, RTE_IFF_LOOPBACK,
		RTE_IFF_PROMISC, RTE_IFF_ALLMULTI, RTE_IFF_MULTICAST};
	int i;

	for (i=0; i < sizeof(supported)/sizeof(unsigned int); i++){
		if (change & supported[i]){
			switch(supported[i]){
			case RTE_IFF_BROADCAST:
				ixgbe_set_broadcast(dev, (new_flag & RTE_IFF_BROADCAST != 0));
				break;
			case RTE_IFF_LOOPBACK:
				ixgbe_set_tx_loopback(dev, (new_flag & RTE_IFF_LOOPBACK != 0));
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

#define READ_REG_COUNT 0
#define READ_REGISTERS 1
/*
* mode: 0: get # of registers 
*		1: read registers and save the data in "data"
*/
static uint32_t
ixgbe_registers(struct rte_eth_dev *dev, int mode, void *data)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;
	uint32_t total = 0, count;
	uint32_t *regs = data;
	uint32_t seg0_addr_set[] = {
		IXGBE_CTRL, IXGBE_STATUS, IXGBE_CTRL_EXT, IXGBE_ESDP,
		IXGBE_EODSDP, IXGBE_LEDCTL, IXGBE_FRTIMER, IXGBE_TCPTIMER,
		IXGBE_EEC, IXGBE_EERD, IXGBE_FLA, IXGBE_EEMNGCTL,
		IXGBE_EEMNGDATA, IXGBE_FLMNGCTL, IXGBE_FLMNGDATA, IXGBE_FLMNGCNT,
		IXGBE_FLOP, IXGBE_GRC,  IXGBE_EICR, IXGBE_EICS,
		IXGBE_EIMS, IXGBE_EIMC, IXGBE_EIAC, IXGBE_EIAM,
		IXGBE_EITR(0), IXGBE_IVAR(0), IXGBE_MSIXT, IXGBE_MSIXPBA,
		IXGBE_PBACL(0), IXGBE_GPIE, IXGBE_PFCTOP, IXGBE_FCTTV(0),
		IXGBE_FCTTV(1), IXGBE_FCTTV(2), IXGBE_FCTTV(3)
	};

	struct rte_dev_reg_set seg1_addr_set [] = {
		{IXGBE_FCRTL(0), 8, 4}, {IXGBE_FCRTH(0), 8, 4}
	};

	struct rte_dev_reg_set seg2_addr_set [] = {/* Receive DMA */
		{IXGBE_FCRTV, 1, 0}, {IXGBE_TFCS, 1, 0},
		{IXGBE_RDBAL(0), 64, 0x40}, {IXGBE_RDBAH(0), 64, 0x40},
		{IXGBE_RDLEN(0), 64, 0x40}, {IXGBE_RDH(0), 64, 0x40},
		{IXGBE_RDT(0), 64, 0x40}, {IXGBE_RXDCTL(0), 64, 0x40},
		{IXGBE_SRRCTL(0), 16, 4}, {IXGBE_DCA_RXCTRL(0), 16, 4},
		{IXGBE_RDRXCTL, 1, 0}, {IXGBE_RXPBSIZE(0), 8, 4},
		{IXGBE_RXCTRL, 1, 0}, {IXGBE_DROPEN, 1, 0}
	};

	struct rte_dev_reg_set seg3_addr_set [] = { /* Receive */
		{IXGBE_RXCSUM, 1, 0}, {IXGBE_RFCTL, 1, 0},
		{IXGBE_RAL(0), 16, 8}, {IXGBE_RAH(0), 16, 8},
		{IXGBE_PSRTYPE(0), 1, 0}, {IXGBE_FCTRL, 1, 0},
		{IXGBE_VLNCTRL, 1, 0}, {IXGBE_MCSTCTRL, 1, 0},
		{IXGBE_MRQC, 1, 0}, {IXGBE_VMD_CTL, 1, 0},
		{IXGBE_IMIR(0), 8, 4}, {IXGBE_IMIREXT(0), 8, 4},
		{IXGBE_IMIRVP, 1, 0}
	};

	struct rte_dev_reg_set seg4_addr_set[] = { /* Transmit */
		{IXGBE_TDBAL(0), 32, 0x40}, {IXGBE_TDBAH(0), 32, 0x40},
		{IXGBE_TDLEN(0), 32, 0x40}, {IXGBE_TDH(0), 32, 0x40},
		{IXGBE_TDT(0), 32, 0x40}, {IXGBE_TXDCTL(0), 32, 0x40},
		{IXGBE_TDWBAL(0), 32, 0x40}, {IXGBE_TDWBAH(0), 32, 0x40},
		{IXGBE_DTXCTL, 1, 0}, {IXGBE_DCA_TXCTRL(0), 16, 0x40},
		{IXGBE_TIPG, 1, 0}, {IXGBE_TXPBSIZE(0), 8, 4},
		{IXGBE_MNGTXMAP, 1, 0},
	};

	uint32_t seg5_addr_set[] = { /* Wake Up */
		IXGBE_WUC, IXGBE_WUFC, IXGBE_WUS, IXGBE_IPAV,
		IXGBE_IP4AT, IXGBE_IP6AT, IXGBE_WUPL, IXGBE_WUPM,
		IXGBE_FHFT(0)
	};

	struct rte_dev_reg_set seg6_addr_set[] = { /* DCB */
		{IXGBE_RMCS, 1, 0}, {IXGBE_DPMCS, 1, 0},
		{IXGBE_PDPMCS, 1, 0}, {IXGBE_RUPPBMR, 1, 0},
		{IXGBE_RT2CR(0), 8, 4}, {IXGBE_RT2SR(0), 8, 4},
		{IXGBE_TDTQ2TCCR(0), 8, 0x40}, {IXGBE_TDTQ2TCSR(0), 8, 0x40},
		{IXGBE_TDPT2TCCR(0), 8, 4}, {IXGBE_TDPT2TCSR(0), 8, 4}
	};

	struct rte_dev_reg_set seg7_addr_set[] = { /* Statistics */
		{IXGBE_CRCERRS, 1, 0}, {IXGBE_ILLERRC, 1, 0},
		{IXGBE_ERRBC, 1, 0}, {IXGBE_MSPDC, 1, 0},
		{IXGBE_MPC(0), 8, 4}, {IXGBE_MLFC, 1, 0},
		{IXGBE_MRFC, 1, 0}, {IXGBE_RLEC, 1, 0},
		{IXGBE_LXONTXC, 1, 0}, {IXGBE_LXONRXC, 1, 0},
		{IXGBE_LXOFFTXC, 1, 0}, {IXGBE_LXOFFRXC, 1, 0},
		{IXGBE_PXONTXC(0), 8, 4}, {IXGBE_PXONRXC(0), 8, 4},
		{IXGBE_PXOFFTXC(0), 8, 4}, {IXGBE_PXOFFRXC(0), 8, 4},
		{IXGBE_PRC64, 1, 0}, {IXGBE_PRC127, 1, 0},
		{IXGBE_PRC255, 1, 0}, {IXGBE_PRC511, 1, 0},
		{IXGBE_PRC1023, 1, 0}, {IXGBE_PRC1522, 1, 0},
		{IXGBE_GPRC, 1, 0}, {IXGBE_BPRC, 1, 0},
		{IXGBE_MPRC, 1, 0}, {IXGBE_GPTC, 1, 0},
		{IXGBE_GORCL, 2, 4}, {IXGBE_GOTCL, 2, 4}, /* may need to combine h/l */
		{IXGBE_RNBC(0), 8, 4}, {IXGBE_RUC, 1, 0},
		{IXGBE_RFC, 1, 0}, {IXGBE_ROC, 1, 0},
		{IXGBE_RJC, 1, 0}, {IXGBE_MNGPRC, 1, 0},
		{IXGBE_MNGPDC, 1, 0}, {IXGBE_MNGPTC, 1, 0},
		{IXGBE_TORL, 2, 4}, {IXGBE_TPR, 1, 0},
		{IXGBE_TPT, 1, 0}, {IXGBE_PTC64, 1, 0},
		{IXGBE_PTC127, 1, 0}, {IXGBE_PTC255, 1, 0},
		{IXGBE_PTC511, 1, 0}, {IXGBE_PTC1023, 1, 0},
		{IXGBE_PTC1522, 1, 0}, {IXGBE_MPTC, 1, 0},
		{IXGBE_BPTC, 1, 0}, {IXGBE_XEC, 1, 0},
		{IXGBE_QPRC(0), 16, 0x40}, {IXGBE_QPTC(0), 16, 0x40},
		{IXGBE_QBRC(0), 16, 0x40}, {IXGBE_QBTC(0), 16, 0x40}
	};

	uint32_t seg8_addr_set[] = { /* MAC */
		IXGBE_PCS1GCFIG, IXGBE_PCS1GLCTL, IXGBE_PCS1GLSTA, IXGBE_PCS1GDBG0,
		IXGBE_PCS1GDBG1, IXGBE_PCS1GANA, IXGBE_PCS1GANLP, IXGBE_PCS1GANNP,
		IXGBE_PCS1GANLPNP, IXGBE_HLREG0, IXGBE_HLREG1, IXGBE_PAP,
		IXGBE_MACA, IXGBE_APAE, IXGBE_ARD, IXGBE_AIS,
		IXGBE_MSCA, IXGBE_MSRWD, IXGBE_MLADD, IXGBE_MHADD,
		IXGBE_TREG, IXGBE_PCSS1, IXGBE_PCSS2, IXGBE_XPCSS,
		IXGBE_SERDESC, IXGBE_MACS, IXGBE_AUTOC, IXGBE_LINKS,
		IXGBE_AUTOC2, IXGBE_AUTOC3, IXGBE_ANLP1, IXGBE_ANLP2,
		IXGBE_ATLASCTL
	};

	struct rte_dev_reg_set seg9_addr_set[] = { /* Diagnostic */
		{IXGBE_RDSTATCTL, 1, 0}, {IXGBE_RDSTAT(0), 8, 4},
		{IXGBE_RDHMPN, 1, 0}, {IXGBE_RIC_DW(0), 4, 4},
		{IXGBE_RDPROBE, 1, 0}, 
		/*add dummy 9 reg to match legacy TDSTATCTL and TDSTAT*/
		{IXGBE_RDPROBE, 1, 0}, 
		{IXGBE_RDPROBE, 1, 0}, {IXGBE_RDPROBE, 1, 0}, 
		{IXGBE_RDPROBE, 1, 0}, {IXGBE_RDPROBE, 1, 0}, 
		{IXGBE_RDPROBE, 1, 0}, {IXGBE_RDPROBE, 1, 0},
		{IXGBE_RDPROBE, 1, 0}, {IXGBE_RDPROBE, 1, 0}, 			
		{IXGBE_TDHMPN, 1, 0},
		{IXGBE_TIC_DW(0), 4, 4}, {IXGBE_TDPROBE, 1, 0},
		{IXGBE_TXBUFCTRL, 1, 0}, {IXGBE_TXBUFDATA0, 1, 0},
		{IXGBE_TXBUFDATA1, 1, 0}, {IXGBE_TXBUFDATA2, 1, 0},
		{IXGBE_TXBUFDATA3, 1, 0}, {IXGBE_RXBUFCTRL, 1, 0},
		{IXGBE_RXBUFDATA0, 1, 0}, {IXGBE_RXBUFDATA1, 1, 0},
		{IXGBE_RXBUFDATA2, 1, 0}, {IXGBE_RXBUFDATA3, 1, 0},
		{IXGBE_PCIE_DIAG(0), 8, 4}, {IXGBE_RFVAL, 1, 0},
		{IXGBE_MDFTC1, 1, 0}, {IXGBE_MDFTC2, 1, 0},
		{IXGBE_MDFTFIFO1, 1, 0}, {IXGBE_MDFTFIFO2, 1, 0},
		{IXGBE_MDFTS, 1, 0}, {IXGBE_PCIEECCCTL, 1, 0},
		{IXGBE_PBTXECC, 1, 0}, {IXGBE_PBRXECC, 1, 0},
		{IXGBE_MFLCN, 1, 0}
	};

	count = sizeof(seg0_addr_set)/sizeof(uint32_t);
	count += ixgbe_read_reg_set(hw, seg1_addr_set, NULL, sizeof(seg1_addr_set)/sizeof(struct rte_dev_reg_set));
	count += ixgbe_read_reg_set(hw, seg2_addr_set, NULL, sizeof(seg2_addr_set)/sizeof(struct rte_dev_reg_set));
	count += ixgbe_read_reg_set(hw, seg3_addr_set, NULL, sizeof(seg3_addr_set)/sizeof(struct rte_dev_reg_set));
	count += ixgbe_read_reg_set(hw, seg4_addr_set, NULL, sizeof(seg4_addr_set)/sizeof(struct rte_dev_reg_set));
	count += sizeof(seg5_addr_set)/sizeof(uint32_t);
	count += ixgbe_read_reg_set(hw, seg6_addr_set, NULL, sizeof(seg6_addr_set)/sizeof(struct rte_dev_reg_set));
	count += ixgbe_read_reg_set(hw, seg7_addr_set, NULL, sizeof(seg7_addr_set)/sizeof(struct rte_dev_reg_set));
	count += sizeof(seg8_addr_set)/sizeof(uint32_t);
	count += ixgbe_read_reg_set(hw, seg9_addr_set, NULL, sizeof(seg9_addr_set)/sizeof(struct rte_dev_reg_set));

	count *= sizeof(int);
	if (mode == READ_REG_COUNT)
		return count;

	/* mac type dependent base address */
	if (hw->mac.type != ixgbe_mac_82598EB) {
		seg1_addr_set[0].base_addr = IXGBE_FCRTL_82599(0);
		seg1_addr_set[1].base_addr = IXGBE_FCRTH_82599(0);
	}

	for(i = 0; i < sizeof(seg0_addr_set)/sizeof(uint32_t); i++)
		regs[total++] = IXGBE_READ_REG(hw, seg0_addr_set[i]);

	total += ixgbe_read_reg_set(hw, seg1_addr_set, &regs[total], sizeof(seg1_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbe_read_reg_set(hw, seg2_addr_set, &regs[total], sizeof(seg2_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbe_read_reg_set(hw, seg3_addr_set, &regs[total], sizeof(seg3_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbe_read_reg_set(hw, seg4_addr_set, &regs[total], sizeof(seg4_addr_set)/sizeof(struct rte_dev_reg_set));

	for(i = 0; i < sizeof(seg5_addr_set)/sizeof(uint32_t); i++)
		regs[total++] = IXGBE_READ_REG(hw, seg5_addr_set[i]);

	total += ixgbe_read_reg_set(hw, seg6_addr_set, &regs[total], sizeof(seg6_addr_set)/sizeof(struct rte_dev_reg_set));
	total += ixgbe_read_reg_set(hw, seg7_addr_set, &regs[total], sizeof(seg7_addr_set)/sizeof(struct rte_dev_reg_set));

	for(i = 0; i < sizeof(seg8_addr_set)/sizeof(uint32_t); i++)
		regs[total++] = IXGBE_READ_REG(hw, seg8_addr_set[i]);

	total += ixgbe_read_reg_set(hw, seg9_addr_set, &regs[total], sizeof(seg9_addr_set)/sizeof(struct rte_dev_reg_set));

	return total;
}

#define ADVERTISED_MASK_10G (SUPPORTED_10000baseT_Full | SUPPORTED_10000baseKX4_Full | SUPPORTED_10000baseKR_Full)
#define ixgbe_isbackplane(type)  ((type == ixgbe_media_type_backplane)? 1 : 0)

static inline void ixgbe_ethtool_cmd_speed_set(
	struct rte_dev_ethtool_cmd *cmd, uint32_t speed)
{
	cmd->speed = speed;
}

static uint32_t ixgbe_backplane_type(struct ixgbe_hw *hw)
{
	uint32_t type = 0x00;
	switch(hw->device_id) {
	case IXGBE_DEV_ID_82598:
	case IXGBE_DEV_ID_82599_KX4:
	case IXGBE_DEV_ID_82599_KX4_MEZZ:
	case IXGBE_DEV_ID_X550EM_X_KX4:
		type = SUPPORTED_10000baseKX4_Full;
		break;
	case IXGBE_DEV_ID_82598_BX:
	case IXGBE_DEV_ID_82599_KR:
	case IXGBE_DEV_ID_X550EM_X_KR:
		type = SUPPORTED_10000baseKR_Full;
		break;
	default:
		type = (SUPPORTED_10000baseKX4_Full | SUPPORTED_10000baseKR_Full);
		break;
	}
	return type;
}

/* ethtool implementation */
static int ixgbe_get_setting(struct rte_eth_dev *dev, struct rte_dev_ethtool_cmd *cmd)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t speed = 0, link_speed;
	int status, autoneg = 0;
	bool link_up;

	status = ixgbe_get_link_capabilities(hw, &speed, &autoneg);
	if (status < 0)
		return status;


	if (autoneg) 
		cmd->autoneg = AUTONEG_ENABLE;
	else
		cmd->autoneg = AUTONEG_DISABLE;

	hw->mac.ops.check_link(hw, &link_speed, &link_up, false);

	if (link_up) {
		switch (link_speed) {
		case IXGBE_LINK_SPEED_10GB_FULL:
			ixgbe_ethtool_cmd_speed_set(cmd, SPEED_10000);
			break;
		case IXGBE_LINK_SPEED_5GB_FULL:
			ixgbe_ethtool_cmd_speed_set(cmd, 5000);
			break;
		case IXGBE_LINK_SPEED_2_5GB_FULL:
			ixgbe_ethtool_cmd_speed_set(cmd, SPEED_2500);
			break;
		case IXGBE_LINK_SPEED_1GB_FULL:
			ixgbe_ethtool_cmd_speed_set(cmd, SPEED_1000);
			break;
		case IXGBE_LINK_SPEED_100_FULL:
			ixgbe_ethtool_cmd_speed_set(cmd, SPEED_100);
			break;
		default:
			break;
		}
		cmd->duplex = DUPLEX_FULL;
	} else {
		ixgbe_ethtool_cmd_speed_set(cmd, SPEED_UNKNOWN);
		cmd->duplex = -1;
	}
	return 0;
}

static int 
ixgbe_set_setting(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_cmd *cmd)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t advertised, old_advertised, speed;
	int status = 0;

	if ((hw->phy.media_type == ixgbe_media_type_copper) ||
	    (hw->phy.multispeed_fiber)) {
		/*
		 * this function does not support duplex forcing, but can
		 * limit the advertising of the adapter to the specified speed
		 */
		if (cmd->advertising & ~cmd->supported)
			return -EINVAL;

		/* only allow one speed at a time if no autoneg */
		if (!cmd->autoneg && hw->phy.multispeed_fiber) {
			if (cmd->advertising ==
			    (ADVERTISED_10000baseT_Full |
			     ADVERTISED_1000baseT_Full))
				return -EINVAL;
		}

		old_advertised = hw->phy.autoneg_advertised;
		advertised = 0;
		if (cmd->advertising & ADVERTISED_10000baseT_Full)
			advertised |= IXGBE_LINK_SPEED_10GB_FULL;

		if (cmd->advertising & ADVERTISED_1000baseT_Full)
			advertised |= IXGBE_LINK_SPEED_1GB_FULL;

		if (cmd->advertising & ADVERTISED_100baseT_Full)
			advertised |= IXGBE_LINK_SPEED_100_FULL;

		if (old_advertised == advertised)
			return status;

		/* stop the device, and try new speed setting */
		dev->dev_ops->dev_stop(dev);
		hw->mac.autotry_restart = true;
		status = hw->mac.ops.setup_link(hw, advertised, true);
		if (status) {
			PMD_INIT_LOG(INFO, "setup link failed with code %d return to old speed\n", status);
			hw->mac.ops.setup_link(hw, old_advertised, true);
		}
		status = dev->dev_ops->dev_start(dev);
	}
	else {
		/* in this case we currently only support 10Gb/FULL */
		speed = cmd->speed;
		if ((cmd->autoneg == AUTONEG_ENABLE) ||
		    (cmd->advertising != ADVERTISED_10000baseT_Full) ||
		    (speed + cmd->duplex != SPEED_10000 + DUPLEX_FULL))
			status = -EINVAL;
	}

	return status;
}

static int
ixgbe_get_reg_length(struct rte_eth_dev *dev)
{
	return ixgbe_registers(dev, READ_REG_COUNT, NULL);
}

static int
ixgbe_get_regs(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_reg *reginfo, void *data)
{
	if ((reginfo == NULL) || (data == NULL))
		return EINVAL;
	reginfo->len = ixgbe_registers(dev, READ_REGISTERS, data);
	reginfo->version = 0x2000000;
	return reginfo->len;
}

static int
ixgbe_get_drvinfo(struct rte_eth_dev *dev, struct rte_dev_ethtool_drvinfo *drvinfo)
{
	struct rte_eth_dev_info dev_info;
	const char driver_name[] = "ixgbe\0";
	int status;

	if (drvinfo == NULL)
		return  -EINVAL;

#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	status = ixgbe_fw_version_get(dev, drvinfo->fw_version, 
		sizeof(drvinfo->fw_version));

	if (status < 0)
		PMD_DRV_LOG(ERR, "Get firmware version error: (%s)\n", strerror(-status));
	else {
		if (status > 0)
			PMD_DRV_LOG(ERR, "Insufficient buffer size of firmware version"
			"minimum size is %d", status);
	}
#else
	snprintf(drvinfo->fw_version, 4, "N/A");
#endif
	memset(&dev_info, 0, sizeof(struct rte_eth_dev_info));
	dev->dev_ops->dev_infos_get(dev, &dev_info);
	if (dev_info.driver_name)
		memcpy(drvinfo->driver, dev_info.driver_name,
		strlen(dev_info.driver_name)+1);
	else
		memcpy(drvinfo->driver, driver_name, strlen(driver_name)+1);

	memcpy(drvinfo->version, rte_version(), strlen(rte_version())+1);

	if (dev_info.pci_dev)
		snprintf(drvinfo->bus_info, sizeof(drvinfo->bus_info),
		"%04x:%02x:%02x.%x",
		dev_info.pci_dev->addr.domain,
		dev_info.pci_dev->addr.bus,
		dev_info.pci_dev->addr.devid,
		dev_info.pci_dev->addr.function);
	else
		snprintf(drvinfo->bus_info, sizeof(drvinfo->bus_info),
		"%04x:%02x:%02x.%x", 0, 0, 0, 0);

	drvinfo->regdump_len = ixgbe_get_reg_length(dev);
	drvinfo->eedump_len = dev->dev_ops->get_eeprom_length(dev);
	drvinfo->n_stats = sizeof(struct rte_eth_stats)/sizeof(uint64_t);
	drvinfo->testinfo_len = 0;
	drvinfo->n_priv_flags = RTE_PRIV_FLAGS_STR_LEN;

	return 0;
}

static int
ixgbe_get_eeprom_len(struct rte_eth_dev *dev)
{
	return dev->dev_ops->get_eeprom_length(dev);
}

static int
ixgbe_get_eeprom(struct rte_eth_dev *dev, 
	struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom)
{
	struct rte_dev_eeprom_info eeprom_info;
	int status;

	if ((info == NULL) || (eeprom == NULL))
		return -EINVAL;

	eeprom_info.offset = info->offset;
	eeprom_info.length = info->len;
	eeprom_info.data = eeprom;

	status = dev->dev_ops->get_eeprom(dev, &eeprom_info);
	if (status)
		return status;
	info->magic = eeprom_info.magic;

	return 0;
}

static int
ixgbe_set_eeprom(struct rte_eth_dev *dev, struct rte_dev_ethtool_eeprom *info,
	uint8_t *eeprom)
{
	struct rte_dev_eeprom_info eeprom_info;
	int status;

	if (info == NULL || eeprom == NULL)
		return -EINVAL;

	eeprom_info.offset = info->offset;
	eeprom_info.length = info->len;
	eeprom_info.data = eeprom;

	status = dev->dev_ops->set_eeprom(dev, &eeprom_info);
	if (status)
		return status;

	info->magic = eeprom_info.magic;
	return 0;
}

static int
ixgbe_get_pauseparam(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_pauseparam *pause_param)
{
	struct rte_eth_fc_conf fc_conf;
	int status;

	if (pause_param == NULL)
		return -EINVAL;

	status = dev->dev_ops->flow_ctrl_get(dev, &fc_conf);
	if (status)
		return status;

	pause_param->tx_pause = 0;
	pause_param->rx_pause = 0;
	switch (fc_conf.mode) {
	case RTE_FC_RX_PAUSE:
		pause_param->rx_pause = 1;
		break;
	case RTE_FC_TX_PAUSE:
		pause_param->tx_pause = 1;
		break;
	case RTE_FC_FULL:
		pause_param->rx_pause = 1;
		pause_param->tx_pause = 1;
	default:
		/* dummy block to avoid compiler warning */
		break;
	}
	pause_param->autoneg = (uint32_t)fc_conf.autoneg;

	return 0;
}

static int
ixgbe_set_pauseparam(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_pauseparam *pause_param)
{
	struct rte_eth_fc_conf fc_conf;
	int status;

	if (pause_param == NULL)
		return -EINVAL;

	/*
	* Read device flow control parameter first since
	* ethtool set_pauseparam op doesn't have all the information.
	* as defined in struct rte_eth_fc_conf.
	* This API requires the device to support both
	* rte_eth_dev_flow_ctrl_get and rte_eth_dev_flow_ctrl_set, otherwise
	* return -ENOTSUP
	*/
	status = dev->dev_ops->flow_ctrl_get(dev, &fc_conf);
	if (status)
		return status;

	fc_conf.autoneg = (uint8_t)pause_param->autoneg;

	if (pause_param->tx_pause) {
		if (pause_param->rx_pause)
			fc_conf.mode = RTE_FC_FULL;
		else
			fc_conf.mode = RTE_FC_TX_PAUSE;
	} else {
		if (pause_param->rx_pause)
			fc_conf.mode = RTE_FC_RX_PAUSE;
		else
			fc_conf.mode = RTE_FC_NONE;
	}

	status = dev->dev_ops->flow_ctrl_set(dev, &fc_conf);
	if (status)
		return status;

	return 0;
}

static int
ixgbe_get_ringparam(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_ringparam *ring_param)
{
	struct rte_eth_dev_info dev_info;
	struct rte_eth_rxq_info rx_qinfo;
	struct rte_eth_txq_info tx_qinfo;

	if (ring_param == NULL)
		return -EINVAL;

	memset(ring_param, 0, sizeof(*ring_param));
	dev->dev_ops->dev_infos_get(dev, &dev_info);
	if (dev->data->nb_rx_queues) {
		dev->dev_ops->rxq_info_get(dev, 0, &rx_qinfo);
		ring_param->rx_pending = rx_qinfo.nb_desc;
		ring_param->rx_max_pending = dev_info.rx_desc_lim.nb_max;
	} else {
		ring_param->rx_pending = 0;
		ring_param->rx_max_pending = 0;
	}

	if (dev->data->nb_tx_queues) {
		dev->dev_ops->txq_info_get(dev, 0, &tx_qinfo);
		ring_param->tx_pending = tx_qinfo.nb_desc;
		ring_param->tx_max_pending = dev_info.tx_desc_lim.nb_max;
	} else {
		ring_param->tx_pending = 0;
		ring_param->tx_max_pending = 0;
	}

	return 0;
}

static int
ixgbe_get_sset_count(struct rte_eth_dev *dev, int val)
{
	switch(val) {
	case ETH_SS_PRIV_FLAGS:
		return RTE_PRIV_FLAGS_STR_LEN;
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS: /* todo */
		return 0;
	default:
		return 0;
	}
}

static int
ixgbe_get_strings(struct rte_eth_dev *dev, uint32_t stringset, uint8_t *ctx)
{
	switch (stringset) {
	case ETH_SS_PRIV_FLAGS:
		memcpy(ctx, rte_priv_flags_strings,
		       RTE_PRIV_FLAGS_STR_LEN * ETH_GSTRING_LEN);
		break;
	default:
		PMD_DRV_LOG(INFO, "Un-supported string type(%d)\n", stringset);
		break;
	}
	return 0;
}


static int
ixgbe_set_ringparam(struct rte_eth_dev *dev,
	struct rte_dev_ethtool_ringparam *ring_param)
{
	struct rte_eth_rxq_info rx_qinfo;
	int status;

	if (ring_param == NULL)
		return -EINVAL;

	dev->dev_ops->rxq_info_get(dev, 0, &rx_qinfo);

	dev->dev_ops->dev_stop(dev);

	status = dev->dev_ops->tx_queue_setup(dev, 0, ring_param->tx_pending,
		rte_socket_id(), NULL);
	if (status != 0)
		return status;

	status = dev->dev_ops->rx_queue_setup(dev, 0, ring_param->rx_pending,
		rte_socket_id(), NULL, rx_qinfo.mp);
	if (status != 0)
		return status;

	return dev->dev_ops->dev_start(dev);
}

static int
ixgbe_get_priv_flags(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	return dev_ex->priv_flag;
}

static int
ixgbe_set_priv_flags(struct rte_eth_dev *dev, u32 flags)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	int i, vf_id, on, mask;

    dev_ex->priv_flag = flags;
	if (!VF_FLAG_IS_VF(flags)) {
		/* PF private flag */
	    /* TODO: review which private flag features are supported in kernel*/
        /* driver, and implement the same feature here                    */
        PMD_DRV_LOG(INFO, "No support of \"ethtool --set-priv-flags\"\n");
		return 0;
    }

	vf_id = VF_FLAG_VF_INDEX(flags);
	if ((dev_ex->vf_flags[vf_id] & VF_FLAG_MASK) ==
		(flags & VF_FLAG_MASK))
		return 0;

	for(i=0; i < VF_FLAG_VF_BIT_MAXI; i++) {
		mask = 1 << i;
		if ((dev_ex->vf_flags[vf_id] & mask) ==
			(flags & mask))
			continue;
		on = (flags & mask)?1:0;
		switch(i) {
		case VF_FLAG_SPLIT_DROP_EN_BIT:
			ixgbe_set_vf_split_drop_en(dev, vf_id, on);
			break;
		case VF_FLAG_MAC_PROMISC_BIT:
			ixgbe_set_vf_promisc(dev, vf_id, on);
			break;
		case VF_FLAG_ALLMULTI_BIT:
			ixgbe_set_vf_allmulticast(dev, vf_id, on);
			break;
		case VF_FLAG_BROADCAST_BIT:
			ixgbe_set_vf_broadcast(dev, vf_id, on);
			break;
		case VF_FLAG_SPOOFCHK_BIT:
			ixgbe_set_vf_spoofchk(dev, vf_id, on);
			break;
		case VF_FLAG_LINK_STATE_BIT:
			ixgbe_set_vf_link_state(dev, vf_id, on);
			break;
		case VF_FLAG_PING_VFS_BIT:
			ixgbe_ping_vfs(dev, vf_id);
			break;
		case VF_FLAG_MAC_ANTISPOOF_BIT:
			ixgbe_set_vf_mac_anti_spoof(dev, vf_id, on);
			break;
		case VF_FLAG_VLAN_ANTISPOOF_BIT:
			ixgbe_set_vf_vlan_anti_spoof(dev, vf_id, on);
			break;
		case VF_FLAG_VLAN_STRIPQ_BIT:
			ixgbe_set_vf_vlan_stripq(dev, vf_id, on);
			break;
		case VF_FLAG_RX_QUEUE_EN_BIT:
			ixgbe_set_vf_rx(dev, vf_id, on);
			break;
		case VF_FLAG_TX_QUEUE_EN_BIT:
			ixgbe_set_vf_tx(dev, vf_id, on);
			break;
		case VF_FLAG_REST_STATISTICS_BIT:
			ixgbe_reset_vf_stats(dev, vf_id);
			break;
		default:
			PMD_DRV_LOG(INFO, "Un-supported VF feature bit(%d)\n", i);
			break;
		}
	}
	return 0;
}

static int
ixgbe_begin(struct rte_eth_dev *dev)
{
	/* DPDK should be always on */
	return 0;
}

static int
ixgbe_complete(struct rte_eth_dev *dev)
{
	/* No close of DPDK, just close the interface */
	return 0;
}

static int 
ixgbe_get_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* update attributes that can be changed run-time */
	memcpy(&dev_ex->netdev_data.perm_addr, hw->mac.perm_addr, ETHER_ADDR_LEN);
	memcpy(&dev_ex->netdev_data.dev_addr, hw->mac.perm_addr, ETHER_ADDR_LEN);
	dev_ex->netdev_data.mtu = dev->data->mtu;
    dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
    dev_ex->netdev_data.type = 1; /* ARPHRD_ETHER */
    dev_ex->netdev_data.flags = ixgbe_dev_get_iff_flag(dev);
	memcpy((void *)netdev_data, &dev_ex->netdev_data, sizeof(struct netdev_priv_data));

	return 0;
}

static int 
ixgbe_set_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
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
		return ixgbe_netdev_flag_sync(dev, change, netdev_data->flags);
		
	return 0;
}

static const struct eth_dev_ethtool_ops ixgbe_ethtool_ops = {
	.get_netdev_data = ixgbe_get_netdev_data,
	.set_netdev_data = ixgbe_set_netdev_data,
	.get_setting = ixgbe_get_setting,
	.set_setting = ixgbe_set_setting,
	.get_strings = ixgbe_get_strings,
	.get_sset_count = ixgbe_get_sset_count,
	.get_regs_len = ixgbe_get_reg_length,
	.get_regs = ixgbe_get_regs,
	.get_drvinfo = ixgbe_get_drvinfo,
	.get_eeprom_len = ixgbe_get_eeprom_len,
	.get_eeprom = ixgbe_get_eeprom,
	.set_eeprom = ixgbe_set_eeprom,
	.get_pauseparam = ixgbe_get_pauseparam,
	.set_pauseparam = ixgbe_set_pauseparam,
	.get_ringparam = ixgbe_get_ringparam,
	.set_ringparam = ixgbe_set_ringparam,
	.get_priv_flags	= ixgbe_get_priv_flags,
	.set_priv_flags	= ixgbe_set_priv_flags,
	/* pseudo function */
	.begin			= ixgbe_begin,
	.complete		= ixgbe_complete,
};

/* netdev op implementation */
static int
ixgbe_dev_start(struct rte_eth_dev *dev)
{
	return dev->dev_ops->dev_start(dev);
}

static int
ixgbe_dev_stop(struct rte_eth_dev *dev)
{
	dev->dev_ops->dev_stop(dev);
	return 0;
}

static int
ixgbe_change_rx_flag(struct rte_eth_dev *dev, int flags)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	if (flags & RTE_IFF_PROMISC) {
		dev->dev_ops->promiscuous_enable(dev);
		dev_ex->dev_iff_flag |= RTE_IFF_PROMISC;
	} else {
		dev->dev_ops->promiscuous_disable(dev);
		dev_ex->dev_iff_flag &= ~RTE_IFF_PROMISC;
	}
		
	if (flags & RTE_IFF_ALLMULTI) {
		dev->dev_ops->allmulticast_enable(dev);
		dev_ex->dev_iff_flag |= RTE_IFF_ALLMULTI;
	} else {
		dev->dev_ops->allmulticast_disable(dev);
		dev_ex->dev_iff_flag &= ~RTE_IFF_ALLMULTI;
	}

	return 0;
}

static int
ixgbe_set_rx_mode(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	struct rte_pci_device *pci_dev = RTE_DEV_TO_PCI(dev->device);
#else
	struct rte_pci_device *pci_dev = dev->pci_dev;
#endif
	uint32_t fctrl, vmolr = IXGBE_VMOLR_BAM | IXGBE_VMOLR_AUPE;
	uint32_t vlnctrl;
	uint8_t *mac_ptr;
	int count, i;
	int pf_pool_index;

	/* Check for Promiscuous and All Multicast modes */
	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);

	/* set all bits that we expect to always be set */
	fctrl |= IXGBE_FCTRL_BAM;
	fctrl |= IXGBE_FCTRL_DPF; /* discard pause frames when FC enabled */
	fctrl |= IXGBE_FCTRL_PMCF;

	/* clear the bits we are changing the status of */
	fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	vlnctrl  &= ~(IXGBE_VLNCTRL_VFE | IXGBE_VLNCTRL_CFIEN);

	/* checking uni-cast promiscuous mode */
	if (dev_ex->dev_iff_flag & RTE_IFF_PROMISC) {
		hw->addr_ctrl.user_set_promisc = true;
		fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
		vmolr |= IXGBE_VMOLR_MPE;
	} else {
		if (dev_ex->dev_iff_flag & RTE_IFF_ALLMULTI) {
			fctrl |= IXGBE_FCTRL_MPE;
			vmolr |= IXGBE_VMOLR_MPE;
		}			
		hw->addr_ctrl.user_set_promisc = false;
	}

	/* disable vlan filtering if vlan promiscuous is enabled */
	/* unless SR-IOV is enabled */
	if (dev_ex->dev_iff_flag & RTE_IFF_VLAN_PROMISC) {
		if (ixgbe_vt_check(hw) >= 0)
			vlnctrl |= (IXGBE_VLNCTRL_VFE | IXGBE_VLNCTRL_CFIEN);
	} else
		vlnctrl |= IXGBE_VLNCTRL_VFE;


	/* Update unicast MAC address filter list */
	/* Set UC promiscuous mode if the # of UC */
	/* is more than hw could support          */
	pf_pool_index = ixgbe_vt_check(hw)>=0?pci_dev->max_vfs:0;
	if (dev_ex->nr_uc_addr > 0) {
		if (dev_ex->nr_uc_addr > hw->mac.num_rar_entries) {
			fctrl |= IXGBE_FCTRL_UPE;
			vmolr |= IXGBE_VMOLR_ROPE;
		} else {
			/* fixme: assume there is no VT enabled */
			mac_ptr = dev_ex->uc_addr_list;
			for(i = 0; i < dev_ex->nr_uc_addr; i++, mac_ptr += ETH_ADDR_LEN)
				ixgbe_set_rar(hw, i, mac_ptr, pf_pool_index, 1);
		}
	}

	/* Update multicast MAC address */
	if (dev_ex->nr_mc_addr > 0) {
		dev->dev_ops->set_mc_addr_list(dev, 
			(struct ether_addr *)dev_ex->mc_addr_list,
			dev_ex->nr_mc_addr);
	}

	if (hw->mac.type != ixgbe_mac_82598EB) {
		pf_pool_index = ixgbe_vt_check(hw)>=0?(pci_dev->max_vfs+1):0;

		vmolr |= IXGBE_READ_REG(hw, IXGBE_VMOLR(pf_pool_index)) &
			~(IXGBE_VMOLR_MPE | IXGBE_VMOLR_ROMPE |
			IXGBE_VMOLR_ROPE);
		IXGBE_WRITE_REG(hw, IXGBE_VMOLR(pf_pool_index), vmolr);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

	return 0;
}

static int
ixgbe_set_mac_addr(struct rte_eth_dev *dev, void *mac_addr)
{
	dev->dev_ops->mac_addr_set(dev, (struct ether_addr *)mac_addr);

	return 0;
}

static int
ixgbe_validate_addr(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (is_valid_assigned_ether_addr((struct ether_addr *)hw->mac.addr))
		return 1;
	return 0;
}

static int
ixgbe_change_mtu(struct rte_eth_dev *dev, int mtu)
{
	return dev->dev_ops->mtu_set(dev, mtu);
}

static int
ixgbe_get_stats64(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
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
ixgbe_get_stats(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
{
	dev->dev_ops->stats_get(dev, out_stats);
	return 0;
}

static int
ixgbe_vlan_rx_add_vid(struct rte_eth_dev *dev, uint16_t vlan_id)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int start_q = 
		ixgbe_vt_check(hw)?RTE_ETH_DEV_SRIOV(dev).def_vmdq_idx:0;
	int i, ret;

	for(i=start_q; i<(start_q+ixgbe_queue_per_pool(hw)); i++){
		ret = hw->mac.ops.set_vfta(hw, vlan_id, i,
			1, false);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int
ixgbe_vlan_rx_kill_vid(struct rte_eth_dev *dev, uint16_t vlan_id)
{
	struct ixgbe_hw *hw =IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int start_q = 
		ixgbe_vt_check(hw)?RTE_ETH_DEV_SRIOV(dev).def_vmdq_idx:0;
	int i, ret;

	for(i=start_q; i<(start_q+ixgbe_queue_per_pool(hw)); i++){
		ret = hw->mac.ops.set_vfta(hw, vlan_id, i,
			0, false);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int
ixgbe_fix_features(struct rte_eth_dev *dev __rte_unused, uint64_t *features)
{
	uint64_t fix_up = *features;
	/* TODO when DCB is on, RXVLAN stripping is also on */
	*features = fix_up;

	return 0;
}

static int
ixgbe_set_features(struct rte_eth_dev *dev, uint64_t features)
{
	if (features & RTE_NETIF_F_HW_VLAN_RX)
		dev->dev_ops->vlan_offload_set(dev, 1);
	else
		dev->dev_ops->vlan_offload_set(dev, 0);

	return 0;
}

static int
ixgbe_set_all_queues_drop_en(struct rte_eth_dev *dev, uint8_t on)
{
	struct ixgbe_hw *hw;
	uint32_t value;
	int i;
	int num_queues = (int)(IXGBE_QDE_IDX_MASK >> IXGBE_QDE_IDX_SHIFT);

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	for (i = 0; i <= num_queues; i++) {
		value = IXGBE_QDE_WRITE |
			(i << IXGBE_QDE_IDX_SHIFT) |
			(on & IXGBE_QDE_ENABLE);
		IXGBE_WRITE_REG(hw, IXGBE_QDE, value);
	}

	return 0;
}

static int
ixgbe_set_tx_loopback(struct rte_eth_dev *dev, uint8_t on)
{
	struct ixgbe_hw *hw;
	uint32_t ctrl;

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	ctrl = IXGBE_READ_REG(hw, IXGBE_PFDTXGSWC);
	/* enable or disable VMDQ loopback */
	if (on)
		ctrl |= IXGBE_PFDTXGSWC_VT_LBEN;
	else
		ctrl &= ~IXGBE_PFDTXGSWC_VT_LBEN;

	IXGBE_WRITE_REG(hw, IXGBE_PFDTXGSWC, ctrl);

	return 0;
}

/* PF manages VF configuration */
static int
ixgbe_is_vf_enabled(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* if Virtualization Technology is enabled */
	int reg_val = ixgbe_vt_check(hw);

	if (reg_val == 0)
		return 1;
	return 0;
}

static int
ixgbe_set_vf_mac_addr(struct rte_eth_dev *dev, int vf_id, uint8_t *addr)
{
	struct ixgbe_hw *hw;
	struct ixgbe_vf_info *vfinfo;
	int rar_entry;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	rar_entry = hw->mac.num_rar_entries - (vf_id + 1);

	if (is_valid_assigned_ether_addr((struct ether_addr *)addr)) {
		memcpy(vfinfo[vf_id].vf_mac_addresses, addr,
			ETHER_ADDR_LEN);
		return hw->mac.ops.set_rar(hw, rar_entry, addr, vf_id,
			IXGBE_RAH_AV);
	}
	PMD_INIT_LOG(ERR, "Invalid ether addr %02x:%02x:%2x:%2x:%2x:%2x", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
	return -EINVAL;
}

static int
ixgbe_set_vf_promisc(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	uint32_t vmolr, fctrl;

	if (hw->mac.type != ixgbe_mac_X550EM_x)
		return -EOPNOTSUPP;

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	if (((fctrl & IXGBE_FCTRL_UPE) == 0) ||
		ixgbe_vlan_strip_enabled(dev)) {
			/* VF promisc requires PF in promisc */
			PMD_INIT_LOG(ERR,
				"Enabling VF promisc requires PF in promisc "
				"and vlan stripping disabled\n");
			return -1;
	}

	vmolr = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id));

	RTE_LOG(INFO, PMD, "VF %u: %s unicast promiscuous\n", vf_id, on?"enabling":"disabling");

	if (on) {
		vmolr |= IXGBE_VMOLR_UPE;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_MAC_PROMISC_BIT);
	} else {
		vmolr &= ~IXGBE_VMOLR_UPE;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_MAC_PROMISC_BIT);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), vmolr);

	return 0;
}

static int
ixgbe_set_vf_allmulticast(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	uint32_t vmolr;

	vmolr = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id));

	if (on) {
		vmolr |= IXGBE_VMOLR_MPE;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_ALLMULTI_BIT);
	} else {
		vmolr &= ~IXGBE_VMOLR_MPE;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_ALLMULTI_BIT);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), vmolr);

	RTE_LOG(INFO, PMD, "VF %u: %s multicast promiscuous\n", vf_id, on?"enabling":"disabling");

	return 0;
}

static int
ixgbe_set_vf_broadcast(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	uint32_t vmolr;

	vmolr = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id));

	if (on) {
		vmolr |= IXGBE_VMOLR_BAM;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_BROADCAST_BIT);
	} else {
		vmolr &= ~IXGBE_VMOLR_BAM;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_BROADCAST_BIT);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), vmolr);

	RTE_LOG(INFO, PMD, "VF %u: %s broadcasting\n", vf_id, on?"enabling":"disabling");

	return 0;
}

static int
ixgbe_set_vf_vlan(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id,
	uint8_t qos)
{
	struct ixgbe_vf_info *vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	int ret = 0;

	if (vlan_id || qos) {
		if (vfinfo[vf_id].default_vf_vlan_id)
			ret = ixgbe_disable_port_vlan(dev, vf_id);
		if (ret)
			return ret;
		ret = ixgbe_enable_port_vlan(dev, vf_id, vlan_id, qos);

	} else {
		ret = ixgbe_disable_port_vlan(dev, vf_id);
	}

	return ret;
}

static int
ixgbe_set_vf_rate(struct rte_eth_dev *dev, int vf_id, int max_tx_rate)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_vf_info *vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	ixgbe_link_speed link_speed = IXGBE_LINK_SPEED_UNKNOWN;
	struct rte_eth_link link;
	int link_up, ret;
	uint32_t q, queues_per_pool;

	memset(&link, 0, sizeof(struct rte_eth_link));
	ixgbe_atomic_read_link_status(dev, &link);
	/* verify link is up */
	if (!link.link_status)
		return -EINVAL;

	/* verify we are linked at 10Gbps */
	if (dev->data->dev_conf.intr_conf.lsc != 0)
		ret = ixgbe_check_link(hw, &link_speed, &link_up, 0);
	else
		ret = ixgbe_check_link(hw, &link_speed, &link_up, 1);

	if (ret || (link_speed != ETH_SPEED_NUM_10G))
		return -EINVAL;

	/* rate limit cannot be less than 10Mbs or greater than link speed */
	if (max_tx_rate && ((max_tx_rate <= 10) || (max_tx_rate > link_speed)))
		return -EINVAL;

	/* store values */
	hw->phy.speeds_supported = link_speed;

	/* update hardware configuration */
	queues_per_pool = ixgbe_queue_per_pool(hw);

	for (q = 0; q < queues_per_pool; q++) {
		vfinfo[vf_id].tx_rate[q] = max_tx_rate;
		dev->dev_ops->set_queue_rate_limit(dev, 
			q+vf_id*queues_per_pool, max_tx_rate);
	}

	return 0;
}

static int
ixgbe_get_vf_config(struct rte_eth_dev *dev, int vf_id,
	struct common_vf_info *ivi)
{
	struct ixgbe_vf_info *vfinfo = 
		*(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	rte_memcpy(ivi->mac, vfinfo[vf_id].vf_mac_addresses, ETHER_ADDR_LEN);
	ivi->vlan = vfinfo[vf_id].default_vf_vlan_id;
	ivi->qos = (vfinfo[vf_id].default_vf_vlan_id >> 12) & 0x7;
	ivi->spoofchk = vfinfo[vf_id].spoofchk_enabled;
	ivi->linkstate = vfinfo[vf_id].clear_to_send?1:0;
	ivi->min_tx_rate = vfinfo[vf_id].tx_rate[0];
	ivi->max_tx_rate = vfinfo[vf_id].tx_rate[0];

	return 0;
}

static int
ixgbe_set_vf_spoofchk(struct rte_eth_dev *dev, int vf_id, int enable)
{
	struct ixgbe_vf_info *vfinfo = 
		*(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	int vf_target_reg = vf_id >> 3;
	int vf_target_shift = vf_id % 8;
	uint32_t value;

	if (vfinfo[vf_id].spoofchk_enabled == enable)
		return 0;

	vfinfo[vf_id].spoofchk_enabled = enable;

	value = IXGBE_READ_REG(hw, IXGBE_PFVFSPOOF(vf_target_reg));
	value &= ~(1 << vf_target_shift);
	value |= (enable << vf_target_shift);
	IXGBE_WRITE_REG(hw, IXGBE_PFVFSPOOF(vf_target_reg), value);

	if (vfinfo[vf_id].vlan_count) {
		vf_target_shift += IXGBE_SPOOF_VLANAS_SHIFT;
		value = IXGBE_READ_REG(hw, IXGBE_PFVFSPOOF(vf_target_reg));
		value &= ~(1 << vf_target_shift);
		value |= (enable << vf_target_shift);
		IXGBE_WRITE_REG(hw, IXGBE_PFVFSPOOF(vf_target_reg), value);
	}

	if (enable)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_SPOOFCHK_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_SPOOFCHK_BIT);

	return 0;
}

static int
ixgbe_set_vf_link_state(struct rte_eth_dev *dev, int vf_id, int link)
{
	return -1;
}

static int
ixgbe_ping_vfs(struct rte_eth_dev *dev, int vf_id)
{
	struct ixgbe_hw *hw;
	struct ixgbe_vf_info *vfinfo;
	uint32_t ctrl;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	vfinfo = *(IXGBE_DEV_PRIVATE_TO_P_VFDATA(dev->data->dev_private));

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	ctrl = IXGBE_PF_CONTROL_MSG;
	if (vfinfo[vf_id].clear_to_send)
		ctrl |= IXGBE_VT_MSGTYPE_CTS;

	ixgbe_write_mbx(hw, &ctrl, 1, vf_id);

	return 0;
}

static int
ixgbe_set_vf_mac_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw;
	struct ixgbe_mac_info *mac;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mac = &hw->mac;
	mac->ops.set_mac_anti_spoofing(hw, on, vf_id);
	if (on)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_MAC_ANTISPOOF_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_MAC_ANTISPOOF_BIT);


	return 0;
}

static int
ixgbe_set_vf_vlan_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw;
	struct ixgbe_mac_info *mac;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mac = &hw->mac;

	mac->ops.set_vlan_anti_spoofing(hw, on, vf_id);
	if (on)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_VLAN_ANTISPOOF_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_VLAN_ANTISPOOF_BIT);

	return 0;
}

static int
ixgbe_set_vf_vlan_filter(struct rte_eth_dev *dev, uint64_t vf_mask, uint16_t vlan,
	uint8_t on)
{
	int ret = 0;
	uint16_t vf_idx;
	struct ixgbe_hw *hw;

	if ((vlan > ETHER_MAX_VLAN_ID) || (vf_mask == 0))
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (ixgbe_vt_check(hw) < 0)
		return -ENOTSUP;

	for (vf_idx = 0; vf_idx < 64; vf_idx++) {
		if (vf_mask & ((uint64_t)(1ULL << vf_idx))) {
			ret = hw->mac.ops.set_vfta(hw, vlan, vf_idx,
				on, false);
			if (ret < 0)
				return ret;
		}
	}

	return ret;
}

static int
ixgbe_set_vf_vlan_insert(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id)
{
	struct ixgbe_hw *hw;
	uint32_t ctrl;

	if (vlan_id > ETHER_MAX_VLAN_ID)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	ctrl = IXGBE_READ_REG(hw, IXGBE_VMVIR(vf_id));
	if (vlan_id) {
		ctrl = vlan_id;
		ctrl |= IXGBE_VMVIR_VLANA_DEFAULT;
	} else {
		ctrl = 0;
	}

	IXGBE_WRITE_REG(hw, IXGBE_VMVIR(vf_id), ctrl);

	return 0;
}

static int
ixgbe_set_vf_rate_limits(struct rte_eth_dev *dev, int vf_id, uint16_t tx_rate,
	uint64_t q_msk)
{
#if RTE_VERSION < RTE_VERSION_NUM(17, 2, 0, 0)
	/* for DPDK older than 17.05 */
	return dev->dev_ops->set_vf_rate_limit(dev, (uint16_t)vf_id, tx_rate, q_msk);
#else
#if RTE_VERSION >= RTE_VERSION_NUM(17, 5, 0, 0)
	/* for DPDK newer than 17.05 */
	return ixgbe_set_vf_rate_limit(dev, (uint16_t) vf_id, tx_rate, q_msk);
#else
	/* for DPDK 17.05 */
	return rte_pmd_ixgbe_set_vf_rate_limit(dev->data->port_id, (uint16_t)vf_id, tx_rate, q_msk);
#endif

#endif
}

static int
ixgbe_set_vf_vlan_stripq(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw;
	uint32_t q, queues_per_pool;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (on > 1)
		return -EINVAL;

	RTE_FUNC_PTR_OR_ERR_RET(*dev->dev_ops->vlan_strip_queue_set, -ENOTSUP);

	queues_per_pool = ixgbe_queue_per_pool(hw);

	for (q=vf_id*queues_per_pool; q<(vf_id+1)*queues_per_pool; q++)
		(*dev->dev_ops->vlan_strip_queue_set)(dev, q, on);

	if (on)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_VLAN_STRIPQ_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_VLAN_STRIPQ_BIT);

	return 0;
}

static int
ixgbe_set_vf_rxmode(struct rte_eth_dev *dev, int vf_id, uint16_t rx_mask, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	int val = 0;
	struct rte_pci_device *pci_dev;
	struct ixgbe_hw *hw;
	uint32_t vmolr;

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	vmolr = IXGBE_READ_REG(hw, IXGBE_VMOLR(vf_id));

	if (hw->mac.type == ixgbe_mac_82598EB) {
		PMD_INIT_LOG(ERR, "setting VF receive mode is not available on 82598");
		return -ENOTSUP;
	}
	if (ixgbe_vt_check(hw) < 0)
		return -ENOTSUP;

	val = ixgbe_convert_vm_rx_mask_to_val(rx_mask, val);

	if (on) {
		vmolr |= val;
		dev_ex->vf_flags[vf_id] |= ((rx_mask << 16) & 0xFFFF0000);
	} else {
		vmolr &= ~val;
		dev_ex->vf_flags[vf_id] &= ((~rx_mask << 16) | 0xFFFF);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vf_id), vmolr);

	return 0;
}

static int
ixgbe_set_vf_rx(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	uint32_t reg, addr;
	uint32_t value;
	const uint8_t bit1 = 0x1;
	struct ixgbe_hw *hw;

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (ixgbe_vt_check(hw) < 0)
		return -ENOTSUP;

	/* for vf >= 32, set bit in PFVFRE[1], otherwise PFVFRE[0] */
	if (vf_id >= 32) {
		addr = IXGBE_VFRE(1);
		value = bit1 << (vf_id - 32);
	} else {
		addr = IXGBE_VFRE(0);
		value = bit1 << vf_id;
	}

	reg = IXGBE_READ_REG(hw, addr);

	if (on) {
		reg |= value;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_RX_QUEUE_EN_BIT);
	} else {
		reg &= ~value;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_RX_QUEUE_EN_BIT);
	}

	IXGBE_WRITE_REG(hw, addr, reg);

	return 0;
}

static int
ixgbe_set_vf_tx(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	uint32_t reg, addr;
	uint32_t value;
	const uint8_t bit1 = 0x1;

	struct ixgbe_hw *hw;

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if (ixgbe_vt_check(hw) < 0)
		return -ENOTSUP;

	/* for vf >= 32, set bit in PFVFTE[1], otherwise PFVFTE[0] */
	if (vf_id >= 32) {
		addr = IXGBE_VFTE(1);
		value = bit1 << (vf_id - 32);
	} else {
		addr = IXGBE_VFTE(0);
		value = bit1 << vf_id;
	}

	reg = IXGBE_READ_REG(hw, addr);

	if (on) {
		reg |= value;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_TX_QUEUE_EN_BIT);
	} else {
		reg &= ~value;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_TX_QUEUE_EN_BIT);
	}

	IXGBE_WRITE_REG(hw, addr, reg);

	return 0;
}

static int
ixgbe_get_vf_stats(struct rte_eth_dev *dev, int vf_id, struct rte_eth_stats *stats)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int diag = 0;
	uint64_t rx_ol, rx_oh, tx_ol, tx_oh;

	stats->ipackets = IXGBE_READ_REG(hw, IXGBE_PVFGPRC(vf_id));
	rx_ol = IXGBE_READ_REG(hw, IXGBE_PVFGORC_LSB(vf_id));
	rx_oh = IXGBE_READ_REG(hw, IXGBE_PVFGORC_MSB(vf_id));
	stats->ibytes = (rx_oh << 32) | rx_ol;  // 36 bit only counter

	stats->opackets = IXGBE_READ_REG(hw, IXGBE_PVFGPTC(vf_id));
	tx_ol = IXGBE_READ_REG(hw, IXGBE_PVFGOTC_LSB(vf_id));
	tx_oh = IXGBE_READ_REG(hw, IXGBE_PVFGOTC_MSB(vf_id));
	stats->obytes = (tx_oh << 32) | tx_ol;  // 36 bit only counter

	return diag;   
}

static int
ixgbe_reset_vf_stats(struct rte_eth_dev *dev, int vf_id)
{
	struct ixgbe_hw_stats *stats =
		IXGBE_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint64_t rx_ol, rx_oh, tx_ol, tx_oh;

	/* HW registers are cleared on read */
	rx_ol = IXGBE_READ_REG(hw, IXGBE_PVFGORC_LSB(vf_id));
	rx_oh = IXGBE_READ_REG(hw, IXGBE_PVFGORC_MSB(vf_id));
	tx_ol = IXGBE_READ_REG(hw, IXGBE_PVFGOTC_LSB(vf_id));
	tx_oh = IXGBE_READ_REG(hw, IXGBE_PVFGOTC_MSB(vf_id));

	/* Reset software totals */
	memset(stats, 0, sizeof(*stats));

	return 0;
}

static int 
ixgbe_set_vf_split_drop_en(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex= rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw;
	uint32_t value;

	if (on > 1)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	value = IXGBE_READ_REG(hw, IXGBE_SRRCTL(vf_id));
	if (on) {
		value |= IXGBE_SRRCTL_DROP_EN;
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_SPLIT_DROP_EN_BIT);
	} else {
		value &= ~IXGBE_SRRCTL_DROP_EN;
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_SPLIT_DROP_EN_BIT);
	}

	IXGBE_WRITE_REG(hw, IXGBE_SRRCTL(vf_id), value);

	return 0;
}

/* MACSec */
#if (RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0))
static int
ixgbe_macsec_enable(struct rte_eth_dev *dev, uint8_t en, uint8_t rp)
{
	struct ixgbe_hw *hw;
	uint32_t value;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Stop the data paths */
	if (ixgbe_disable_sec_rx_path(hw) != IXGBE_SUCCESS)
		return -ENOTSUP;
	/*
	* Workaround:
	* As no ixgbe_disable_sec_rx_path equivalent is
	* implemented for tx in the base code, and we are
	* not allowed to modify the base code in DPDK, so
	* just call the hand-written one directly for now.
	* The hardware support has been checked by
	* ixgbe_disable_sec_rx_path().
	*/
	ixgbe_disable_sec_tx_path_generic(hw);

	/* Enable Ethernet CRC (required by MACsec offload) */
	value = IXGBE_READ_REG(hw, IXGBE_HLREG0);
	value |= IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_RXCRCSTRP;
	IXGBE_WRITE_REG(hw, IXGBE_HLREG0, value);

	/* Enable the TX and RX crypto engines */
	value = IXGBE_READ_REG(hw, IXGBE_SECTXCTRL);
	value &= ~IXGBE_SECTXCTRL_SECTX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, value);

	value = IXGBE_READ_REG(hw, IXGBE_SECRXCTRL);
	value &= ~IXGBE_SECRXCTRL_SECRX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, value);

	value = IXGBE_READ_REG(hw, IXGBE_SECTXMINIFG);
	value &= ~IXGBE_SECTX_MINSECIFG_MASK;
	value |= 0x3;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXMINIFG, value);

	/* Enable SA lookup */
	value = IXGBE_READ_REG(hw, IXGBE_LSECTXCTRL);
	value &= ~IXGBE_LSECTXCTRL_EN_MASK;
	value |= en ? IXGBE_LSECTXCTRL_AUTH_ENCRYPT :
		IXGBE_LSECTXCTRL_AUTH;
	value |= IXGBE_LSECTXCTRL_AISCI;
	value &= ~IXGBE_LSECTXCTRL_PNTHRSH_MASK;
	value |= IXGBE_MACSEC_PNTHRSH & IXGBE_LSECTXCTRL_PNTHRSH_MASK;
	IXGBE_WRITE_REG(hw, IXGBE_LSECTXCTRL, value);

	value = IXGBE_READ_REG(hw, IXGBE_LSECRXCTRL);
	value &= ~IXGBE_LSECRXCTRL_EN_MASK;
	value |= IXGBE_LSECRXCTRL_STRICT << IXGBE_LSECRXCTRL_EN_SHIFT;
	value &= ~IXGBE_LSECRXCTRL_PLSH;
	if (rp)
		value |= IXGBE_LSECRXCTRL_RP;
	else
		value &= ~IXGBE_LSECRXCTRL_RP;
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXCTRL, value);

	/* Start the data paths */
	ixgbe_enable_sec_rx_path(hw);
	/*
	* Workaround:
	* As no ixgbe_enable_sec_rx_path equivalent is
	* implemented for tx in the base code, and we are
	* not allowed to modify the base code in DPDK, so
	* just call the hand-written one directly for now.
	*/
	ixgbe_enable_sec_tx_path_generic(hw);

	return 0;
}

static int
ixgbe_macsec_disable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw;
	uint32_t value;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Stop the data paths */
	if (ixgbe_disable_sec_rx_path(hw) != IXGBE_SUCCESS)
		return -ENOTSUP;
	/*
	* Workaround:
	* As no ixgbe_disable_sec_rx_path equivalent is
	* implemented for tx in the base code, and we are
	* not allowed to modify the base code in DPDK, so
	* just call the hand-written one directly for now.
	* The hardware support has been checked by
	* ixgbe_disable_sec_rx_path().
	*/
	ixgbe_disable_sec_tx_path_generic(hw);

	/* Disable the TX and RX crypto engines */
	value = IXGBE_READ_REG(hw, IXGBE_SECTXCTRL);
	value |= IXGBE_SECTXCTRL_SECTX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, value);

	value = IXGBE_READ_REG(hw, IXGBE_SECRXCTRL);
	value |= IXGBE_SECRXCTRL_SECRX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, value);

	/* Disable SA lookup */
	value = IXGBE_READ_REG(hw, IXGBE_LSECTXCTRL);
	value &= ~IXGBE_LSECTXCTRL_EN_MASK;
	value |= IXGBE_LSECTXCTRL_DISABLE;
	IXGBE_WRITE_REG(hw, IXGBE_LSECTXCTRL, value);

	value = IXGBE_READ_REG(hw, IXGBE_LSECRXCTRL);
	value &= ~IXGBE_LSECRXCTRL_EN_MASK;
	value |= IXGBE_LSECRXCTRL_DISABLE << IXGBE_LSECRXCTRL_EN_SHIFT;
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXCTRL, value);

	/* Start the data paths */
	ixgbe_enable_sec_rx_path(hw);
	/*
	* Workaround:
	* As no ixgbe_enable_sec_rx_path equivalent is
	* implemented for tx in the base code, and we are
	* not allowed to modify the base code in DPDK, so
	* just call the hand-written one directly for now.
	*/
	ixgbe_enable_sec_tx_path_generic(hw);

	return 0;
}

static int
ixgbe_macsec_config_txsc(struct rte_eth_dev *dev, uint8_t *mac)
{
	struct ixgbe_hw *hw;
	uint32_t value;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	value = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	IXGBE_WRITE_REG(hw, IXGBE_LSECTXSCL, value);

	value = mac[4] | (mac[5] << 8);
	IXGBE_WRITE_REG(hw, IXGBE_LSECTXSCH, value);

	return 0;
}

static int
ixgbe_macsec_config_rxsc(struct rte_eth_dev *dev, uint8_t *mac, uint16_t pi)
{
	struct ixgbe_hw *hw;
	uint32_t value;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	value = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXSCL, value);

	pi = rte_cpu_to_be_16(pi);
	value = mac[4] | (mac[5] << 8) | (pi << 16);
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXSCH, value);

	return 0;
}

static int
ixgbe_macsec_select_txsa(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
	uint32_t pn, uint8_t *key)
{
	struct ixgbe_hw *hw;
	uint32_t value, i;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (idx != 0 && idx != 1)
		return -EINVAL;

	if (an >= 4)
		return -EINVAL;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Set the PN and key */
	pn = rte_cpu_to_be_32(pn);
	if (idx == 0) {
		IXGBE_WRITE_REG(hw, IXGBE_LSECTXPN0, pn);

		for (i = 0; i < 4; i++) {
			value = (key[i * 4 + 0] <<  0) |
				(key[i * 4 + 1] <<  8) |
				(key[i * 4 + 2] << 16) |
				(key[i * 4 + 3] << 24);
			IXGBE_WRITE_REG(hw, IXGBE_LSECTXKEY0(i), value);
		}
	} else {
		IXGBE_WRITE_REG(hw, IXGBE_LSECTXPN1, pn);

		for (i = 0; i < 4; i++) {
			value = (key[i * 4 + 0] <<  0) |
				(key[i * 4 + 1] <<  8) |
				(key[i * 4 + 2] << 16) |
				(key[i * 4 + 3] << 24);
			IXGBE_WRITE_REG(hw, IXGBE_LSECTXKEY1(i), value);
		}
	}

	/* Set AN and select the SA */
	value = (an << idx * 2) | (idx << 4);
	IXGBE_WRITE_REG(hw, IXGBE_LSECTXSA, value);

	return 0;
}

static int
ixgbe_macsec_select_rxsa(struct rte_eth_dev *dev, uint8_t idx, uint8_t an,
	uint32_t pn, uint8_t *key)
{
	struct ixgbe_hw *hw;
	uint32_t value, i;

	hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (idx != 0 && idx != 1)
		return -EINVAL;

	if (an >= 4)
		return -EINVAL;

	/* Set the PN */
	pn = rte_cpu_to_be_32(pn);
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXPN(idx), pn);

	/* Set the key */
	for (i = 0; i < 4; i++) {
		value = (key[i * 4 + 0] <<  0) |
			(key[i * 4 + 1] <<  8) |
			(key[i * 4 + 2] << 16) |
			(key[i * 4 + 3] << 24);
		IXGBE_WRITE_REG(hw, IXGBE_LSECRXKEY(idx, i), value);
	}

	/* Set the AN and validate the SA */
	value = an | (1 << 2);
	IXGBE_WRITE_REG(hw, IXGBE_LSECRXSA(idx), value);

	return 0;
}
#endif

static unsigned int
ixgbe_dev_get_iff_flag(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
    unsigned int reg_value;
    unsigned int iff_flag = 0;

	reg_value = IXGBE_READ_REG(hw, IXGBE_FCTRL);
    if (reg_value & IXGBE_FCTRL_BAM)
        iff_flag |= RTE_IFF_BROADCAST;
    else
        iff_flag &= ~RTE_IFF_BROADCAST;

    if (reg_value & IXGBE_FCTRL_MPE)
        iff_flag |= RTE_IFF_ALLMULTI;
    else
        iff_flag &= ~RTE_IFF_ALLMULTI;

    if (reg_value & IXGBE_FCTRL_UPE)
        iff_flag |= RTE_IFF_PROMISC;
    else
        iff_flag &= ~RTE_IFF_PROMISC;

    reg_value = IXGBE_READ_REG(hw, IXGBE_PFDTXGSWC);
	if (reg_value & IXGBE_PFDTXGSWC_VT_LBEN)
        iff_flag |= RTE_IFF_LOOPBACK;
    else
        iff_flag &= ~RTE_IFF_LOOPBACK;

    return iff_flag;      
}

static int
ixgbe_dev_init(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

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
    dev_ex->dev_iff_flag = ixgbe_dev_get_iff_flag(dev);

	return 0;
}

static int
ixgbe_dev_uninit(struct rte_eth_dev *dev)
{
	return 0;
}

static const struct eth_dev_netdev_ops ixgbe_netdev_ops = {
	.init				= ixgbe_dev_init,
	.uninit				= ixgbe_dev_uninit,
	.open				= ixgbe_dev_start,
	.stop				= ixgbe_dev_stop,
	.change_rx_flag		= ixgbe_change_rx_flag,
	.set_rx_mode		= ixgbe_set_rx_mode,
	.validate_addr		= ixgbe_validate_addr,
	.change_mtu			= ixgbe_change_mtu,
	.get_stats64		= ixgbe_get_stats64,
	.get_stats			= ixgbe_get_stats,
	.vlan_rx_add_vid	= ixgbe_vlan_rx_add_vid,
	.vlan_rx_kill_vid	= ixgbe_vlan_rx_kill_vid,
	.set_mac_addr		= ixgbe_set_mac_addr,
	.set_all_queues_drop_en = ixgbe_set_all_queues_drop_en,
	.set_features		= ixgbe_set_features,
	.fix_features		= ixgbe_fix_features,
	.set_tx_loopback	= ixgbe_set_tx_loopback,
	/* PF manage VF functions */
	.is_vf_enabled		= ixgbe_is_vf_enabled,
	.set_vf_mac_addr	= ixgbe_set_vf_mac_addr,
	.set_vf_promisc		= ixgbe_set_vf_promisc,
	.set_vf_allmulticast = ixgbe_set_vf_allmulticast,
	.set_vf_broadcast	= ixgbe_set_vf_broadcast,
	.set_vf_vlan		= ixgbe_set_vf_vlan,
	.set_vf_rate		= ixgbe_set_vf_rate,
	.get_vf_config		= ixgbe_get_vf_config,
	.set_vf_spoofchk	= ixgbe_set_vf_spoofchk,
	.set_vf_link_state	= ixgbe_set_vf_link_state,
	.ping_vfs			= ixgbe_ping_vfs,
	.set_vf_mac_anti_spoof = ixgbe_set_vf_mac_anti_spoof,
	.set_vf_vlan_anti_spoof = ixgbe_set_vf_vlan_anti_spoof,
	.set_vf_vlan_filter	= ixgbe_set_vf_vlan_filter,
	.set_vf_vlan_insert	= ixgbe_set_vf_vlan_insert,
	.set_vf_rate_limit	= ixgbe_set_vf_rate_limits,
	.set_vf_vlan_stripq	= ixgbe_set_vf_vlan_stripq,
	.set_vf_rxmode		= ixgbe_set_vf_rxmode,
	.set_vf_rx			= ixgbe_set_vf_rx,
	.set_vf_tx			= ixgbe_set_vf_tx,
	.get_vf_stats		= ixgbe_get_vf_stats,
	.reset_vf_stats		= ixgbe_reset_vf_stats,
	.set_vf_split_drop_en = ixgbe_set_vf_split_drop_en,
	/* MACSec */
#if (RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0))
	.macsec_enable		= ixgbe_macsec_enable,
	.macsec_disable	= ixgbe_macsec_disable,
	.macsec_config_txsc	= ixgbe_macsec_config_txsc,
	.macsec_config_rxsc	= ixgbe_macsec_config_rxsc,
	.macsec_select_txsa	= ixgbe_macsec_select_txsa,
	.macsec_select_rxsa = ixgbe_macsec_select_rxsa,
#endif
};


static int
eth_ixgbe_dev_init_ex(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	dev_ex->dev_ethtool_ops = &ixgbe_ethtool_ops;
	dev_ex->dev_netdev_ops = &ixgbe_netdev_ops;
	memset(dev_ex->vf_flags, 0, sizeof(unsigned int)*RTE_MAX_VF_COUNT);
	dev_ex->is_vf		= false;

	return 0;
}
RTE_PMD_EX_REGISTER_PCI(net_ixgbe, eth_ixgbe_dev_init_ex);
