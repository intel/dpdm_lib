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
#include <rte_memzone.h>
#include <rte_malloc.h>
#include <rte_dpdm.h>
#include <vni_netdev_flags.h>
#include <rte_pci.h>
#include <rte_tailq.h>
#include <rte_version.h>
#include <vni_share_types.h>
#include <rte_vf_flags.h>

#include "i40e_logs.h"
#include "base/i40e_prototype.h"
#include "base/i40e_adminq_cmd.h"
#include "base/i40e_type.h"
#include "base/i40e_register.h"
#include "base/i40e_dcb.h"
#include "base/i40e_devids.h"
#include "base/i40e_osdep.h"
#ifndef I40E_MAC_X722_VF
#define I40E_MAC_X722_VF (I40E_MAC_VF+2)
#endif
#include "i40e_ethdev.h"
#include "i40e_rxtx.h"
#include "i40e_pf.h"
#include "i40e_common.h"
#include "i40e_ethdev_ex.h"

/* init/unitit */
static int eth_i40e_dev_init_ex(struct rte_eth_dev *dev);

/* ethtool_op APIs */
static int i40e_get_reg_length(struct rte_eth_dev *dev);
static int i40e_get_regs(struct rte_eth_dev *dev, struct rte_dev_ethtool_reg *reg_info, void *data);
static int i40e_get_strings(struct rte_eth_dev *dev, uint32_t stringset, uint8_t *ctx);
static int i40e_get_sset_count(struct rte_eth_dev *dev, int val);

/* netdev_op APIs */
static int i40e_dev_start(struct rte_eth_dev *dev);
static int i40e_dev_stop(struct rte_eth_dev *dev);
static int i40e_set_rx_mode(struct rte_eth_dev *dev);
static int i40e_change_rx_flag(struct rte_eth_dev *dev, int flags);
static int i40e_set_mac_addr(struct rte_eth_dev *dev, void *mac_addr);
static int i40e_validate_addr(struct rte_eth_dev *dev);
static int i40e_change_mtu(struct rte_eth_dev *dev, int mtu);
static int i40e_get_stats64(struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static int i40e_get_stats(struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static int i40e_vlan_rx_add_vid(struct rte_eth_dev *dev, uint16_t vlan_id);
static int i40e_vlan_rx_kill_vid(struct rte_eth_dev *dev, uint16_t vlan_id);

static int i40e_set_features(struct rte_eth_dev *dev, uint64_t features);
static int i40e_set_tx_loopback(struct rte_eth_dev *dev, uint8_t on);

/* PF manage VF routines */
static int i40e_is_vf_enabled(struct rte_eth_dev *dev);
static int i40e_set_vf_mac_addr(struct rte_eth_dev *dev, int vf_id, uint8_t *mac);
static int i40e_set_vf_promisc(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_allmulticast(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_broadcast(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_vlan(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id,
									  uint8_t qos);
static int i40e_set_vf_rate(struct rte_eth_dev *dev, int vf_id, int max_tx_rate);
static int i40e_get_vf_config(struct rte_eth_dev *dev, int vf_id,
									  struct common_vf_info *ivi);
static int i40e_set_vf_spoofchk(struct rte_eth_dev *dev, int vf_id, int enable);
static int i40e_set_vf_link_state(struct rte_eth_dev *dev, int vf_id, int link);
static int i40e_set_vf_mac_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_vlan_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_vlan_filter(struct rte_eth_dev *dev, uint64_t vf_mask, uint16_t vlan,
											 uint8_t on);
static int i40e_set_vf_vlan_insert(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id);
static int i40e_set_vf_rate_limit(struct rte_eth_dev *dev, int vf_id, uint16_t tx_rate,
											  uint64_t q_msk);
static int i40e_ping_vfs(struct rte_eth_dev *dev, int vf_id);
static int i40e_set_vf_vlan_stripq(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_rxmode(struct rte_eth_dev *dev, int vf_id, uint16_t rx_mask, uint8_t on);
static int i40e_set_vf_rx(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_set_vf_tx(struct rte_eth_dev *dev, int vf_id, uint8_t on);
static int i40e_get_vf_stats(struct rte_eth_dev *dev, int vf_id, struct rte_eth_stats *stats);
static int i40e_reset_vf_stats(struct rte_eth_dev *dev, int vf_id);

/* helper routines */
/**
 * Find all vlan options for specific mac addr,
 * return with actual vlan found.
 */
static struct rte_dev_reg_set i40e_reg_list[] = {
	{I40E_QTX_CTL(0),       1, I40E_QTX_CTL(1) - I40E_QTX_CTL(0)},
	{I40E_PFINT_ITR0(0),    3, I40E_PFINT_ITR0(1) - I40E_PFINT_ITR0(0)},
	{I40E_PFINT_ITRN(0, 0), 1, I40E_PFINT_ITRN(0, 1) - I40E_PFINT_ITRN(0, 0)},
	{I40E_PFINT_ITRN(1, 0), 1, I40E_PFINT_ITRN(1, 1) - I40E_PFINT_ITRN(1, 0)},
	{I40E_PFINT_ITRN(2, 0), 1, I40E_PFINT_ITRN(2, 1) - I40E_PFINT_ITRN(2, 0)},
	{I40E_PFINT_STAT_CTL0,  1, 0},
	{I40E_PFINT_LNKLST0,    1, 0},
	{I40E_PFINT_LNKLSTN(0), 1, I40E_PFINT_LNKLSTN(1) - I40E_PFINT_LNKLSTN(0)},
	{I40E_QINT_TQCTL(0),    1, I40E_QINT_TQCTL(1) - I40E_QINT_TQCTL(0)},
	{I40E_QINT_RQCTL(0),    1, I40E_QINT_RQCTL(1) - I40E_QINT_RQCTL(0)},
	{I40E_PFINT_ICR0_ENA,   1, 0}
};

#if RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0)
int
i40e_find_all_vlan_for_mac(struct i40e_vsi *vsi,
			   struct i40e_macvlan_filter *mv_f,
			   int num, ether_addr_t *addr)
{
	int i;
	uint32_t j, k;

	/**
	 * Not to use i40e_find_vlan_filter to decrease the loop time,
	 * although the code looks complex.
	  */
	if (num < vsi->vlan_num)
		return I40E_ERR_PARAM;

	i = 0;
	for (j = 0; j < I40E_VFTA_SIZE; j++) {
		if (vsi->vfta[j]) {
			for (k = 0; k < I40E_UINT32_BIT_SIZE; k++) {
				if (vsi->vfta[j] & (1 << k)) {
					if (i > num - 1) {
						PMD_DRV_LOG(ERR,
							"vlan number doesn't match");
						return I40E_ERR_PARAM;
					}
					(void)rte_memcpy(&mv_f[i].macaddr,
							addr, ETH_ADDR_LEN);
					mv_f[i].vlan_id =
						j * I40E_UINT32_BIT_SIZE + k;
					i++;
				}
			}
		}
	}
	return I40E_SUCCESS;
}

int
i40e_remove_macvlan_filters(struct i40e_vsi *vsi,
			    struct i40e_macvlan_filter *filter,
			    int total)
{
	int ele_num, ele_buff_size;
	int num, actual_num, i;
	uint16_t flags;
	int ret = I40E_SUCCESS;
	struct i40e_hw *hw = I40E_VSI_TO_HW(vsi);
	struct i40e_aqc_remove_macvlan_element_data *req_list;

	if (filter == NULL  || total == 0)
		return I40E_ERR_PARAM;

	ele_num = hw->aq.asq_buf_size / sizeof(*req_list);
	ele_buff_size = hw->aq.asq_buf_size;

	req_list = rte_zmalloc("macvlan_remove", ele_buff_size, 0);
	if (req_list == NULL) {
		PMD_DRV_LOG(ERR, "Fail to allocate memory");
		return I40E_ERR_NO_MEMORY;
	}

	num = 0;
	do {
		actual_num = (num + ele_num > total) ? (total - num) : ele_num;
		memset(req_list, 0, ele_buff_size);

		for (i = 0; i < actual_num; i++) {
			(void)rte_memcpy(req_list[i].mac_addr,
				&filter[num + i].macaddr, ETH_ADDR_LEN);
			req_list[i].vlan_tag =
				rte_cpu_to_le_16(filter[num + i].vlan_id);

			switch (filter[num + i].filter_type) {
			case RTE_MAC_PERFECT_MATCH:
				flags = I40E_AQC_MACVLAN_DEL_PERFECT_MATCH |
					I40E_AQC_MACVLAN_DEL_IGNORE_VLAN;
				break;
			case RTE_MACVLAN_PERFECT_MATCH:
				flags = I40E_AQC_MACVLAN_DEL_PERFECT_MATCH;
				break;
			case RTE_MAC_HASH_MATCH:
				flags = I40E_AQC_MACVLAN_DEL_HASH_MATCH |
					I40E_AQC_MACVLAN_DEL_IGNORE_VLAN;
				break;
			case RTE_MACVLAN_HASH_MATCH:
				flags = I40E_AQC_MACVLAN_DEL_HASH_MATCH;
				break;
			default:
				PMD_DRV_LOG(ERR, "Invalid MAC filter type");
				ret = I40E_ERR_PARAM;
				goto DONE;
			}
			req_list[i].flags = rte_cpu_to_le_16(flags);
		}

		ret = i40e_aq_remove_macvlan(hw, vsi->seid, req_list,
						actual_num, NULL);
		if (ret != I40E_SUCCESS) {
			PMD_DRV_LOG(ERR, "Failed to remove macvlan filter");
			goto DONE;
		}
		num += actual_num;
	} while (num < total);

DONE:
	rte_free(req_list);
	return ret;
}
#endif /* RTE_VERSION*/

int
i40e_vsi_rm_mac_filter(struct i40e_vsi *vsi)
{
	struct i40e_mac_filter *f;
	struct i40e_macvlan_filter *mv_f;
	int i, vlan_num;
	enum rte_mac_filter_type filter_type;
	int ret = I40E_SUCCESS;
	void *temp;

	/* remove all the MACs */
	TAILQ_FOREACH_SAFE(f, &vsi->mac_list, next, temp) {
		vlan_num = vsi->vlan_num;
		filter_type = f->mac_info.filter_type;
		if (filter_type == RTE_MACVLAN_PERFECT_MATCH ||
		    filter_type == RTE_MACVLAN_HASH_MATCH) {
			if (vlan_num == 0) {
				PMD_DRV_LOG(ERR, "VLAN number shouldn't be 0");
				return I40E_ERR_PARAM;
			}
		} else if (filter_type == RTE_MAC_PERFECT_MATCH ||
			   filter_type == RTE_MAC_HASH_MATCH)
			vlan_num = 1;

		mv_f = rte_zmalloc("macvlan_data", vlan_num * sizeof(*mv_f), 0);
		if (!mv_f) {
			PMD_DRV_LOG(ERR, "failed to allocate memory");
			return I40E_ERR_NO_MEMORY;
		}

		for (i = 0; i < vlan_num; i++) {
			mv_f[i].filter_type = filter_type;
			(void)rte_memcpy(&mv_f[i].macaddr,
					 &f->mac_info.mac_addr,
					 ETH_ADDR_LEN);
		}
		if (filter_type == RTE_MACVLAN_PERFECT_MATCH ||
		    filter_type == RTE_MACVLAN_HASH_MATCH) {
			ret = i40e_find_all_vlan_for_mac(vsi, mv_f, vlan_num,
							 &f->mac_info.mac_addr);
			if (ret != I40E_SUCCESS) {
				rte_free(mv_f);
				return ret;
			}
		}

		ret = i40e_remove_macvlan_filters(vsi, mv_f, vlan_num);
		if (ret != I40E_SUCCESS) {
			rte_free(mv_f);
			return ret;
		}

		rte_free(mv_f);
		ret = I40E_SUCCESS;
	}

	return ret;
}

int
i40e_add_rm_all_vlan_filter(struct i40e_vsi *vsi, uint8_t add)
{
	uint32_t j, k;
	uint16_t vlan_id;
	struct i40e_hw *hw = I40E_VSI_TO_HW(vsi);
	struct i40e_aqc_add_remove_vlan_element_data vlan_data = {0};
	int ret;

	for (j = 0; j < I40E_VFTA_SIZE; j++) {
		if (!vsi->vfta[j])
			continue;

		for (k = 0; k < I40E_UINT32_BIT_SIZE; k++) {
			if (!(vsi->vfta[j] & (1 << k)))
				continue;

			vlan_id = j * I40E_UINT32_BIT_SIZE + k;
			if (!vlan_id)
				continue;

			vlan_data.vlan_tag = rte_cpu_to_le_16(vlan_id);
			if (add)
				ret = i40e_aq_add_vlan(hw, vsi->seid,
						       &vlan_data, 1, NULL);
			else
				ret = i40e_aq_remove_vlan(hw, vsi->seid,
							  &vlan_data, 1, NULL);
			if (ret != I40E_SUCCESS) {
				PMD_DRV_LOG(ERR,
					    "Failed to add/rm vlan filter");
				return ret;
			}
		}
	}

	return I40E_SUCCESS;
}

int
i40e_vsi_restore_mac_filter(struct i40e_vsi *vsi, uint16_t vlan_anti_spoof_on)
{
	struct i40e_mac_filter *f;
	struct i40e_macvlan_filter *mv_f;
	int i, vlan_num = 0;
	int ret = I40E_SUCCESS;
	void *temp;

	/* restore all the MACs */
	TAILQ_FOREACH_SAFE(f, &vsi->mac_list, next, temp) {
		if ((f->mac_info.filter_type == RTE_MACVLAN_PERFECT_MATCH) ||
		    (f->mac_info.filter_type == RTE_MACVLAN_HASH_MATCH)) {
			/**
			 * If vlan_num is 0, that's the first time to add mac,
			 * set mask for vlan_id 0.
			 */
			if (vsi->vlan_num == 0) {
#if RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0)
				i40e_set_vlan_filter(vsi, 0, vlan_anti_spoof_on, 1);
#else
				i40e_set_vlan_filter(vsi, 0, 1);
#endif
				vsi->vlan_num = 1;
			}
			vlan_num = vsi->vlan_num;
		} else if ((f->mac_info.filter_type == RTE_MAC_PERFECT_MATCH) ||
			   (f->mac_info.filter_type == RTE_MAC_HASH_MATCH))
			vlan_num = 1;

		mv_f = rte_zmalloc("macvlan_data", vlan_num * sizeof(*mv_f), 0);
		if (!mv_f) {
			PMD_DRV_LOG(ERR, "failed to allocate memory");
			return I40E_ERR_NO_MEMORY;
		}

		for (i = 0; i < vlan_num; i++) {
			mv_f[i].filter_type = f->mac_info.filter_type;
			(void)rte_memcpy(&mv_f[i].macaddr,
					 &f->mac_info.mac_addr,
					 ETH_ADDR_LEN);
		}

		if (f->mac_info.filter_type == RTE_MACVLAN_PERFECT_MATCH ||
		    f->mac_info.filter_type == RTE_MACVLAN_HASH_MATCH) {
			ret = i40e_find_all_vlan_for_mac(vsi, mv_f, vlan_num,
							 &f->mac_info.mac_addr);
			if (ret != I40E_SUCCESS) {
				rte_free(mv_f);
				return ret;
			}
		}

		ret = i40e_add_macvlan_filters(vsi, mv_f, vlan_num);
		if (ret != I40E_SUCCESS) {
			rte_free(mv_f);
			return ret;
		}

		rte_free(mv_f);
		ret = I40E_SUCCESS;
	}

	return ret;
}

int
i40e_vsi_set_tx_loopback(struct i40e_vsi *vsi, uint8_t on, uint8_t vlan_anti_spoof_on)
{
	struct i40e_vsi_context ctxt;
	struct i40e_hw *hw;
	int ret;

	if (!vsi)
		return -EINVAL;

	hw = I40E_VSI_TO_HW(vsi);

	/* Use the FW API if FW >= v5.0 */
	if (hw->aq.fw_maj_ver < 5) {
		PMD_INIT_LOG(ERR, "FW < v5.0, cannot enable loopback");
		return -ENOTSUP;
	}

	/* Check if it has been already on or off */
	if (vsi->info.valid_sections &
		rte_cpu_to_le_16(I40E_AQ_VSI_PROP_SWITCH_VALID)) {
		if (on) {
			if ((vsi->info.switch_id &
			     I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB) ==
			    I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB)
				return 0; /* already on */
		} else {
			if ((vsi->info.switch_id &
			     I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB) == 0)
				return 0; /* already off */
		}
	}

	/* remove all the MAC and VLAN first */
	ret = i40e_vsi_rm_mac_filter(vsi);
	if (ret) {
		PMD_INIT_LOG(ERR, "Failed to remove MAC filters.");
		return ret;
	}
	if (vlan_anti_spoof_on) {
		ret = i40e_add_rm_all_vlan_filter(vsi, 0);
		if (ret) {
			PMD_INIT_LOG(ERR, "Failed to remove VLAN filters.");
			return ret;
		}
	}

	vsi->info.valid_sections = cpu_to_le16(I40E_AQ_VSI_PROP_SWITCH_VALID);
	if (on)
		vsi->info.switch_id |= I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB;
	else
		vsi->info.switch_id &= ~I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB;

	memset(&ctxt, 0, sizeof(ctxt));
	(void)rte_memcpy(&ctxt.info, &vsi->info, sizeof(vsi->info));
	ctxt.seid = vsi->seid;

	ret = i40e_aq_update_vsi_params(hw, &ctxt, NULL);
	if (ret != I40E_SUCCESS) {
		PMD_DRV_LOG(ERR, "Failed to update VSI params");
		return ret;
	}

	/* add all the MAC and VLAN back */
	ret = i40e_vsi_restore_mac_filter(vsi, vlan_anti_spoof_on);
	if (ret)
		return ret;
	if (vlan_anti_spoof_on) {
		ret = i40e_add_rm_all_vlan_filter(vsi, 1);
		if (ret)
			return ret;
	}

	return ret;
}

#if RTE_VERSION < RTE_VERSION_NUM(17, 2, 0, 0)
static void
i40e_notify_vf_link_status(struct rte_eth_dev *dev, struct i40e_pf_vf *vf)
{
	struct i40e_virtchnl_pf_event event;

	event.event = I40E_VIRTCHNL_EVENT_LINK_CHANGE;
	event.event_data.link_event.link_status =
		dev->data->dev_link.link_status;
	event.event_data.link_event.link_speed =
		(enum i40e_aq_link_speed)dev->data->dev_link.link_speed;
	i40e_pf_host_send_msg_to_vf(vf, I40E_VIRTCHNL_OP_EVENT,
		I40E_SUCCESS, (uint8_t *)&event, sizeof(event));
}
#endif

#if RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0)
void
i40e_set_vlan_filter(struct i40e_vsi *vsi,
		     uint16_t vlan_id, uint8_t vlan_anti_spoof_on, bool on)
{
	struct i40e_hw *hw = I40E_VSI_TO_HW(vsi);
	struct i40e_aqc_add_remove_vlan_element_data vlan_data = {0};
	int ret;

	if (vlan_id > ETH_VLAN_ID_MAX)
		return;

	i40e_store_vlan_filter(vsi, vlan_id, on);

	if (!vlan_anti_spoof_on || !vlan_id)
		return;

	vlan_data.vlan_tag = rte_cpu_to_le_16(vlan_id);

	if (on) {
		ret = i40e_aq_add_vlan(hw, vsi->seid,
				       &vlan_data, 1, NULL);
		if (ret != I40E_SUCCESS)
			PMD_DRV_LOG(ERR, "Failed to add vlan filter");
	} else {
		ret = i40e_aq_remove_vlan(hw, vsi->seid,
					  &vlan_data, 1, NULL);
		if (ret != I40E_SUCCESS)
			PMD_DRV_LOG(ERR,
				    "Failed to remove vlan filter");
	}
}

int
i40e_add_macvlan_filters(struct i40e_vsi *vsi,
			 struct i40e_macvlan_filter *filter,
			 int total)
{
	int ele_num, ele_buff_size;
	int num, actual_num, i;
	uint16_t flags;
	int ret = I40E_SUCCESS;
	struct i40e_hw *hw = I40E_VSI_TO_HW(vsi);
	struct i40e_aqc_add_macvlan_element_data *req_list;

	if (filter == NULL  || total == 0)
		return I40E_ERR_PARAM;
	ele_num = hw->aq.asq_buf_size / sizeof(*req_list);
	ele_buff_size = hw->aq.asq_buf_size;

	req_list = rte_zmalloc("macvlan_add", ele_buff_size, 0);
	if (req_list == NULL) {
		PMD_DRV_LOG(ERR, "Fail to allocate memory");
		return I40E_ERR_NO_MEMORY;
	}

	num = 0;
	do {
		actual_num = (num + ele_num > total) ? (total - num) : ele_num;
		memset(req_list, 0, ele_buff_size);

		for (i = 0; i < actual_num; i++) {
			(void)rte_memcpy(req_list[i].mac_addr,
				&filter[num + i].macaddr, ETH_ADDR_LEN);
			req_list[i].vlan_tag =
				rte_cpu_to_le_16(filter[num + i].vlan_id);

			switch (filter[num + i].filter_type) {
			case RTE_MAC_PERFECT_MATCH:
				flags = I40E_AQC_MACVLAN_ADD_PERFECT_MATCH |
					I40E_AQC_MACVLAN_ADD_IGNORE_VLAN;
				break;
			case RTE_MACVLAN_PERFECT_MATCH:
				flags = I40E_AQC_MACVLAN_ADD_PERFECT_MATCH;
				break;
			case RTE_MAC_HASH_MATCH:
				flags = I40E_AQC_MACVLAN_ADD_HASH_MATCH |
					I40E_AQC_MACVLAN_ADD_IGNORE_VLAN;
				break;
			case RTE_MACVLAN_HASH_MATCH:
				flags = I40E_AQC_MACVLAN_ADD_HASH_MATCH;
				break;
			default:
				PMD_DRV_LOG(ERR, "Invalid MAC match type");
				ret = I40E_ERR_PARAM;
				goto DONE;
			}

			req_list[i].queue_number = 0;

			req_list[i].flags = rte_cpu_to_le_16(flags);
		}

		ret = i40e_aq_add_macvlan(hw, vsi->seid, req_list,
						actual_num, NULL);
		if (ret != I40E_SUCCESS) {
			PMD_DRV_LOG(ERR, "Failed to add macvlan filter");
			goto DONE;
		}
		num += actual_num;
	} while (num < total);

DONE:
	rte_free(req_list);
	return ret;
}
#endif

void
i40e_store_vlan_filter(struct i40e_vsi *vsi,
		       uint16_t vlan_id, bool on)
{
	uint32_t vid_idx, vid_bit;

	vid_idx = I40E_VFTA_IDX(vlan_id);
	vid_bit = I40E_VFTA_BIT(vlan_id);

	if (on)
		vsi->vfta[vid_idx] |= vid_bit;
	else
		vsi->vfta[vid_idx] &= ~vid_bit;
}

int
i40e_set_vf_untag_vlan(struct i40e_vsi *vsi, uint8_t drop)
{
	struct i40e_aqc_add_remove_vlan_element_data vlan_data = {0};
	struct i40e_hw *hw;
	int ret;

	hw = I40E_VSI_TO_HW(vsi);
	vlan_data.vlan_tag = 0;
	vlan_data.vlan_flags = 0x1;

	if (drop)
		ret = i40e_aq_remove_vlan(hw, vsi->seid,
					&vlan_data, 1, NULL);
	else
		ret = i40e_aq_add_vlan(hw, vsi->seid, &vlan_data, 1, NULL);

	if (ret)
		PMD_DRV_LOG(ERR, "Fail to %s vlan filter (tag == 0)",
		drop?"remove":"add");

	return ret;
}

#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
static int
i40e_fw_version_get(struct rte_eth_dev *dev, char *fw_version, size_t fw_size)
{
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	u32 full_ver;
	u8 ver, patch;
	u16 build;
	int ret;

	full_ver = hw->nvm.oem_ver;
	ver = (u8)(full_ver >> 24);
	build = (u16)((full_ver >> 8) & 0xffff);
	patch = (u8)(full_ver & 0xff);

	ret = snprintf(fw_version, fw_size,
		 "%d.%d%d 0x%08x %d.%d.%d",
		 ((hw->nvm.version >> 12) & 0xf),
		 ((hw->nvm.version >> 4) & 0xff),
		 (hw->nvm.version & 0xf), hw->nvm.eetrack,
		 ver, build, patch);

	ret += 1; /* add the size of '\0' */
	if (fw_size < (u32)ret)
		return ret;
	else
		return 0;
}
#endif

static void i40e_phy_type_to_ethtool(struct rte_eth_dev *dev, uint32_t *supported,
				     uint32_t *advertising, struct i40e_link_status *hw_link_info)
{
	//struct i40e_link_status *hw_link_info = &pf->hw.phy.link_info;
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	uint64_t phy_types = hw->phy.phy_types;

	*supported = 0x0;
	*advertising = 0x0;

	if (phy_types & I40E_CAP_PHY_TYPE_SGMII) {
		*supported |= SUPPORTED_Autoneg |
			     SUPPORTED_1000baseT_Full;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
			*advertising |= ADVERTISED_1000baseT_Full;
		if (pf->flags & I40E_FLAG_100M_SGMII_CAPABLE) {
			*supported |= SUPPORTED_100baseT_Full;
			*advertising |= ADVERTISED_100baseT_Full;
		}
	}
	if (phy_types & I40E_CAP_PHY_TYPE_XAUI ||
	    phy_types & I40E_CAP_PHY_TYPE_XFI ||
	    phy_types & I40E_CAP_PHY_TYPE_SFI ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_SFPP_CU ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_AOC)
		*supported |= SUPPORTED_10000baseT_Full;
	if (phy_types & I40E_CAP_PHY_TYPE_10GBASE_CR1_CU ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_CR1 ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_T ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_SR ||
	    phy_types & I40E_CAP_PHY_TYPE_10GBASE_LR) {
		*supported |= SUPPORTED_Autoneg |
			     SUPPORTED_10000baseT_Full;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_10GB)
			*advertising |= ADVERTISED_10000baseT_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_XLAUI ||
	    phy_types & I40E_CAP_PHY_TYPE_XLPPI ||
	    phy_types & I40E_CAP_PHY_TYPE_40GBASE_AOC)
		*supported |= SUPPORTED_40000baseCR4_Full;
	if (phy_types & I40E_CAP_PHY_TYPE_40GBASE_CR4_CU ||
	    phy_types & I40E_CAP_PHY_TYPE_40GBASE_CR4) {
		*supported |= SUPPORTED_Autoneg |
			     SUPPORTED_40000baseCR4_Full;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_40GB)
			*advertising |= ADVERTISED_40000baseCR4_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_100BASE_TX) {
		*supported |= SUPPORTED_Autoneg |
			     SUPPORTED_100baseT_Full;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_100MB)
			*advertising |= ADVERTISED_100baseT_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_1000BASE_T ||
	    phy_types & I40E_CAP_PHY_TYPE_1000BASE_SX ||
	    phy_types & I40E_CAP_PHY_TYPE_1000BASE_LX ||
	    phy_types & I40E_CAP_PHY_TYPE_1000BASE_T_OPTICAL) {
		*supported |= SUPPORTED_Autoneg |
			     SUPPORTED_1000baseT_Full;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
			*advertising |= ADVERTISED_1000baseT_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_40GBASE_SR4)
		*supported |= SUPPORTED_40000baseSR4_Full;
	if (phy_types & I40E_CAP_PHY_TYPE_40GBASE_LR4)
		*supported |= SUPPORTED_40000baseLR4_Full;
	if (phy_types & I40E_CAP_PHY_TYPE_40GBASE_KR4) {
		*supported |= SUPPORTED_40000baseKR4_Full |
			     SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_40000baseKR4_Full |
			       ADVERTISED_Autoneg;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_20GBASE_KR2) {
		*supported |= SUPPORTED_20000baseKR2_Full |
			     SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_20GB)
			*advertising |= ADVERTISED_20000baseKR2_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_10GBASE_KR) {
		if (!(pf->flags & I40E_FLAG_HAVE_CRT_RETIMER))
			*supported |= SUPPORTED_10000baseKR_Full |
				      SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_10GB)
			if (!(pf->flags & I40E_FLAG_HAVE_CRT_RETIMER))
				*advertising |= ADVERTISED_10000baseKR_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_10GBASE_KX4) {
		*supported |= SUPPORTED_10000baseKX4_Full |
			     SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_10GB)
			*advertising |= ADVERTISED_10000baseKX4_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_1000BASE_KX) {
		if (!(pf->flags & I40E_FLAG_HAVE_CRT_RETIMER))
			*supported |= SUPPORTED_1000baseKX_Full |
				      SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
			if (!(pf->flags & I40E_FLAG_HAVE_CRT_RETIMER))
				*advertising |= ADVERTISED_1000baseKX_Full;
	}
	if (phy_types & I40E_CAP_PHY_TYPE_25GBASE_KR ||
	    phy_types & I40E_CAP_PHY_TYPE_25GBASE_CR ||
	    phy_types & I40E_CAP_PHY_TYPE_25GBASE_SR ||
	    phy_types & I40E_CAP_PHY_TYPE_25GBASE_LR) {
		*supported |= SUPPORTED_Autoneg;
		*advertising |= ADVERTISED_Autoneg;
	}
}

/**
 * i40e_get_settings_link_up - Get the Link settings for when link is up
 * @hw: hw structure
 * @ecmd: ethtool command to fill in
 * @netdev: network interface device structure
 *
 * Reports link settings that can be determined when link is up
 **/
static void i40e_get_settings_link_up(struct rte_eth_dev *dev,
				      struct rte_dev_ethtool_cmd *cmd,
					  struct i40e_link_status *hw_link_info)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	enum i40e_aq_link_speed link_speed = hw_link_info->link_speed;
	uint32_t e_advertising = 0x0;
	uint32_t e_supported = 0x0;

	/* Initialize supported and advertised settings based on phy settings */
	switch (hw_link_info->phy_type) {
	case I40E_PHY_TYPE_40GBASE_CR4:
	case I40E_PHY_TYPE_40GBASE_CR4_CU:
		cmd->supported = SUPPORTED_Autoneg |
				  SUPPORTED_40000baseCR4_Full;
		cmd->advertising = ADVERTISED_Autoneg |
				    ADVERTISED_40000baseCR4_Full;
		break;
	case I40E_PHY_TYPE_XLAUI:
	case I40E_PHY_TYPE_XLPPI:
	case I40E_PHY_TYPE_40GBASE_AOC:
		cmd->supported = SUPPORTED_40000baseCR4_Full;
		break;
	case I40E_PHY_TYPE_40GBASE_SR4:
		cmd->supported = SUPPORTED_40000baseSR4_Full;
		break;
	case I40E_PHY_TYPE_40GBASE_LR4:
		cmd->supported = SUPPORTED_40000baseLR4_Full;
		break;
	case I40E_PHY_TYPE_10GBASE_SR:
	case I40E_PHY_TYPE_10GBASE_LR:
	case I40E_PHY_TYPE_1000BASE_SX:
	case I40E_PHY_TYPE_1000BASE_LX:
		cmd->supported = SUPPORTED_10000baseT_Full;
		if (hw_link_info->module_type[2] & I40E_MODULE_TYPE_1000BASE_SX ||
		    hw_link_info->module_type[2] & I40E_MODULE_TYPE_1000BASE_LX) {
			cmd->supported |= SUPPORTED_1000baseT_Full;
			if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
				cmd->advertising |= ADVERTISED_1000baseT_Full;
		}
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_10GB)
			cmd->advertising |= ADVERTISED_10000baseT_Full;
		break;
	case I40E_PHY_TYPE_10GBASE_T:
	case I40E_PHY_TYPE_1000BASE_T:
	case I40E_PHY_TYPE_100BASE_TX:
		cmd->supported = SUPPORTED_Autoneg |
				  SUPPORTED_10000baseT_Full |
				  SUPPORTED_1000baseT_Full |
				  SUPPORTED_100baseT_Full;
		cmd->advertising = ADVERTISED_Autoneg;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_10GB)
			cmd->advertising |= ADVERTISED_10000baseT_Full;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
			cmd->advertising |= ADVERTISED_1000baseT_Full;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_100MB)
			cmd->advertising |= ADVERTISED_100baseT_Full;
		break;
	case I40E_PHY_TYPE_1000BASE_T_OPTICAL:
		cmd->supported = SUPPORTED_Autoneg |
				  SUPPORTED_1000baseT_Full;
		cmd->advertising = ADVERTISED_Autoneg |
				    ADVERTISED_1000baseT_Full;
		break;
	case I40E_PHY_TYPE_10GBASE_CR1_CU:
	case I40E_PHY_TYPE_10GBASE_CR1:
		cmd->supported = SUPPORTED_Autoneg |
				  SUPPORTED_10000baseT_Full;
		cmd->advertising = ADVERTISED_Autoneg |
				    ADVERTISED_10000baseT_Full;
		break;
	case I40E_PHY_TYPE_XAUI:
	case I40E_PHY_TYPE_XFI:
	case I40E_PHY_TYPE_SFI:
	case I40E_PHY_TYPE_10GBASE_SFPP_CU:
	case I40E_PHY_TYPE_10GBASE_AOC:
		cmd->supported = SUPPORTED_10000baseT_Full;
		break;
	case I40E_PHY_TYPE_SGMII:
		cmd->supported = SUPPORTED_Autoneg |
				  SUPPORTED_1000baseT_Full;
		if (hw_link_info->requested_speeds & I40E_LINK_SPEED_1GB)
			cmd->advertising |= ADVERTISED_1000baseT_Full;
		if (pf->flags & I40E_FLAG_100M_SGMII_CAPABLE) {
			cmd->supported |= SUPPORTED_100baseT_Full;
			if (hw_link_info->requested_speeds &
			    I40E_LINK_SPEED_100MB)
				cmd->advertising |= ADVERTISED_100baseT_Full;
		}
		break;
	case I40E_PHY_TYPE_40GBASE_KR4:
	case I40E_PHY_TYPE_20GBASE_KR2:
	case I40E_PHY_TYPE_10GBASE_KR:
	case I40E_PHY_TYPE_10GBASE_KX4:
	case I40E_PHY_TYPE_1000BASE_KX:
		cmd->supported |= SUPPORTED_40000baseKR4_Full |
				   SUPPORTED_20000baseKR2_Full |
				   SUPPORTED_10000baseKR_Full |
				   SUPPORTED_10000baseKX4_Full |
				   SUPPORTED_1000baseKX_Full |
				   SUPPORTED_Autoneg;
		cmd->advertising |= ADVERTISED_40000baseKR4_Full |
				     ADVERTISED_20000baseKR2_Full |
				     ADVERTISED_10000baseKR_Full |
				     ADVERTISED_10000baseKX4_Full |
				     ADVERTISED_1000baseKX_Full |
				     ADVERTISED_Autoneg;
		break;
	case I40E_PHY_TYPE_25GBASE_KR:
	case I40E_PHY_TYPE_25GBASE_CR:
	case I40E_PHY_TYPE_25GBASE_SR:
	case I40E_PHY_TYPE_25GBASE_LR:
		cmd->supported = SUPPORTED_Autoneg;
		cmd->advertising = ADVERTISED_Autoneg;
		/* TODO: add speeds when ethtool is ready to support*/
		break;
	default:
		/* if we got here and link is up something bad is afoot */
		PMD_DRV_LOG(INFO, "WARNING: Link is up but PHY type 0x%x is not recognized.\n",
			    hw_link_info->phy_type);
	}

	/* Now that we've worked out everything that could be supported by the
	 * current phy type, get what is supported by the NVM and and them to
	 * get what is truly supported
	 */
	i40e_phy_type_to_ethtool(dev, &e_supported,
				 &e_advertising, hw_link_info);

	cmd->supported = cmd->supported & e_supported;
	cmd->advertising = cmd->advertising & e_advertising;

	/* Set speed and duplex */
	switch (link_speed) {
	case I40E_LINK_SPEED_40GB:
		i40e_link_speed_set(cmd, SPEED_40000);
		break;
	case I40E_LINK_SPEED_25GB:
		i40e_link_speed_set(cmd, SPEED_25000);
		break;
	case I40E_LINK_SPEED_20GB:
		i40e_link_speed_set(cmd, SPEED_10000);
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
}

/**
 * i40e_get_settings_link_down - Get the Link settings for when link is down
 * @hw: hw structure
 * @ecmd: ethtool command to fill in
 *
 * Reports link settings that can be determined when link is down
 **/
static void i40e_get_settings_link_down(struct rte_eth_dev *dev,
					struct rte_dev_ethtool_cmd *cmd, struct i40e_link_status *link_status)
{
	/* link is down and the driver needs to fall back on
	 * supported phy types to figure out what info to display
	 */
	i40e_phy_type_to_ethtool(dev, &cmd->supported,
				 &cmd->advertising, link_status);

	/* With no link speed and duplex are unknown */
	i40e_link_speed_set(cmd, SPEED_UNKNOWN);
	cmd->duplex = DUPLEX_UNKNOWN;
}

/* ethtool implementation */
static int i40e_get_setting(struct rte_eth_dev *dev,
			     struct rte_dev_ethtool_cmd *cmd)
{
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i40e_link_status link_status;
	bool enable_lse = dev->data->dev_conf.intr_conf.lsc ? true : false;
	bool link_up;
	int status;

	status = i40e_aq_get_link_info(hw, enable_lse, &link_status, NULL);
	if (status != I40E_SUCCESS) {
		PMD_DRV_LOG(ERR, "Failed to get link info");
		return -1;
	}

	link_up = link_status.link_info & I40E_AQ_LINK_UP;
	if (link_up)
		i40e_get_settings_link_up(dev, cmd, &link_status);
	else
		i40e_get_settings_link_down(dev, cmd, &link_status);

	/* Now set the settings that don't rely on link being up/down */
	/* Set autoneg settings */
	cmd->autoneg = (link_status.an_info & I40E_AQ_AN_COMPLETED ?
			  AUTONEG_ENABLE : AUTONEG_DISABLE);

	/* Set media type settings */
	switch (hw->phy.media_type) {
	case I40E_MEDIA_TYPE_BACKPLANE:
		cmd->supported |= SUPPORTED_Autoneg |
				   SUPPORTED_Backplane;
		cmd->advertising |= ADVERTISED_Autoneg |
				     ADVERTISED_Backplane;
		cmd->port = PORT_NONE;
		break;
	case I40E_MEDIA_TYPE_BASET:
		cmd->supported |= SUPPORTED_TP;
		cmd->advertising |= ADVERTISED_TP;
		cmd->port = PORT_TP;
		break;
	case I40E_MEDIA_TYPE_DA:
	case I40E_MEDIA_TYPE_CX4:
		cmd->supported |= SUPPORTED_FIBRE;
		cmd->advertising |= ADVERTISED_FIBRE;
		cmd->port = PORT_DA;
		break;
	case I40E_MEDIA_TYPE_FIBER:
		cmd->supported |= SUPPORTED_FIBRE;
		cmd->port = PORT_FIBRE;
		break;
	case I40E_MEDIA_TYPE_UNKNOWN:
	default:
		cmd->port = PORT_OTHER;
		break;
	}

	/* Set transceiver */
	cmd->transceiver = XCVR_EXTERNAL;

	/* Set flow control settings */
	cmd->supported |= SUPPORTED_Pause;

	switch (hw->fc.requested_mode) {
	case I40E_FC_FULL:
		cmd->advertising |= ADVERTISED_Pause;
		break;
	case I40E_FC_TX_PAUSE:
		cmd->advertising |= ADVERTISED_Asym_Pause;
		break;
	case I40E_FC_RX_PAUSE:
		cmd->advertising |= (ADVERTISED_Pause |
				      ADVERTISED_Asym_Pause);
		break;
	default:
		cmd->advertising &= ~(ADVERTISED_Pause |
				       ADVERTISED_Asym_Pause);
		break;
	}

	return 0;
}

static int
i40e_set_setting(struct rte_eth_dev *dev,
			     struct rte_dev_ethtool_cmd *cmd)
{
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_aq_get_phy_abilities_resp abilities;
	struct i40e_aq_set_phy_config config;
	struct rte_dev_ethtool_cmd old_cmd;
	i40e_status status = 0;
	bool change = FALSE;
	int err = 0;
	u8 autoneg;
	u32 advertise;
	u32 old_ethtool_advertising = 0;

	/* Changing port settings is not supported if this isn't the
	 * port's controlling PF
	 */

	if (hw->phy.media_type != I40E_MEDIA_TYPE_BASET &&
	    hw->phy.media_type != I40E_MEDIA_TYPE_FIBER &&
	    hw->phy.media_type != I40E_MEDIA_TYPE_BACKPLANE &&
	    hw->phy.media_type != I40E_MEDIA_TYPE_DA &&
	    hw->phy.link_info.link_info & I40E_AQ_LINK_UP)
		return -EOPNOTSUPP;

	if (hw->device_id == I40E_DEV_ID_KX_B ||
	    hw->device_id == I40E_DEV_ID_KX_C ||
	    hw->device_id == I40E_DEV_ID_20G_KR2 ||
	    hw->device_id == I40E_DEV_ID_20G_KR2_A) {
		PMD_DRV_LOG(INFO, "Changing settings is not supported on backplane.\n");
		return -EOPNOTSUPP;
	}

	/* get our own copy of the bits to check against */
	memset(&old_cmd, 0, sizeof(struct rte_dev_ethtool_cmd));
	i40e_get_setting(dev, &old_cmd);

	/* save autoneg and speed out of cmd */
	autoneg = cmd->autoneg;
	advertise = cmd->advertising;

	/* set autoneg and speed back to what they currently are */
	cmd->autoneg = old_cmd.autoneg;
	cmd->advertising = old_cmd.advertising;

	/* Due to a bug in ethtool versions < 3.6 this check is necessary */
	old_ethtool_advertising = cmd->supported &
				  (ADVERTISED_10baseT_Half |
				   ADVERTISED_10baseT_Full |
				   ADVERTISED_100baseT_Half |
				   ADVERTISED_100baseT_Full |
				   ADVERTISED_1000baseT_Half |
				   ADVERTISED_1000baseT_Full |
				   ADVERTISED_2500baseX_Full |
				   ADVERTISED_10000baseT_Full);
	old_ethtool_advertising |= (old_ethtool_advertising |
				   ADVERTISED_20000baseMLD2_Full |
				   ADVERTISED_20000baseKR2_Full);

	if (advertise == old_ethtool_advertising)
		PMD_DRV_LOG(INFO, "If you are not setting advertising to %x then you may have an old version of ethtool. Please update.\n",
			    advertise);
	cmd->cmd = old_cmd.cmd;
	/* If cmd and old_cmd are not the same now, then they are
	 * trying to set something that we do not support
	 */
	if (memcmp(cmd, &old_cmd, sizeof(struct ethtool_cmd)))
		return -EOPNOTSUPP;

	/* TODO: make sure device is not busy */

	/* Get the current phy config */
	status = i40e_aq_get_phy_capabilities(hw, false, false, &abilities,
					      NULL);
	if (status)
		return -EAGAIN;

	/* Copy abilities to config in case autoneg is not
	 * set below
	 */
	memset(&config, 0, sizeof(struct i40e_aq_set_phy_config));
	config.abilities = abilities.abilities;

	/* Check autoneg */
	if (autoneg == AUTONEG_ENABLE) {
		/* If autoneg was not already enabled */
		if (!(hw->phy.link_info.an_info & I40E_AQ_AN_COMPLETED)) {
			/* If autoneg is not supported, return error */
			if (!(old_cmd.supported & SUPPORTED_Autoneg)) {
				PMD_DRV_LOG(INFO, "Autoneg not supported on this phy\n");
				return -EINVAL;
			}
			/* Autoneg is allowed to change */
			config.abilities = abilities.abilities |
					   I40E_AQ_PHY_ENABLE_AN;
			change = true;
		}
	} else {
		/* If autoneg is currently enabled */
		if (hw->phy.link_info.an_info & I40E_AQ_AN_COMPLETED) {
			/* If autoneg is supported 10GBASE_T is the only phy
			 * that can disable it, so otherwise return error
			 */
			if (old_cmd.supported & SUPPORTED_Autoneg &&
			    hw->phy.link_info.phy_type !=
			    I40E_PHY_TYPE_10GBASE_T) {
				PMD_DRV_LOG(INFO, "Autoneg cannot be disabled on this phy\n");
				return -EINVAL;
			}
			/* Autoneg is allowed to change */
			config.abilities = abilities.abilities &
					   ~I40E_AQ_PHY_ENABLE_AN;
			change = true;
		}
	}

	if (advertise & ~old_cmd.supported)
		return -EINVAL;

	if (advertise & ADVERTISED_100baseT_Full)
		config.link_speed |= I40E_LINK_SPEED_100MB;
	if (advertise & ADVERTISED_1000baseT_Full ||
	    advertise & ADVERTISED_1000baseKX_Full)
		config.link_speed |= I40E_LINK_SPEED_1GB;
	if (advertise & ADVERTISED_10000baseT_Full ||
	    advertise & ADVERTISED_10000baseKX4_Full ||
	    advertise & ADVERTISED_10000baseKR_Full)
		config.link_speed |= I40E_LINK_SPEED_10GB;
	if (advertise & ADVERTISED_20000baseKR2_Full)
		config.link_speed |= I40E_LINK_SPEED_20GB;
	if (advertise & ADVERTISED_40000baseKR4_Full ||
	    advertise & ADVERTISED_40000baseCR4_Full ||
	    advertise & ADVERTISED_40000baseSR4_Full ||
	    advertise & ADVERTISED_40000baseLR4_Full)
		config.link_speed |= I40E_LINK_SPEED_40GB;

	/* If speed didn't get set, set it to what it currently is.
	 * This is needed because if advertise is 0 (as it is when autoneg
	 * is disabled) then speed won't get set.
	 */
	if (!config.link_speed)
		config.link_speed = abilities.link_speed;

	if (change || (abilities.link_speed != config.link_speed)) {
		/* copy over the rest of the abilities */
		config.phy_type = abilities.phy_type;
		config.phy_type_ext = abilities.phy_type_ext;
		config.eee_capability = abilities.eee_capability;
		config.eeer = abilities.eeer_val;
		config.low_power_ctrl = abilities.d3_lpan;
#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
		config.fec_config = abilities.fec_cfg_curr_mod_ext_info &
				    I40E_AQ_PHY_FEC_CONFIG_MASK;
#endif
		/* save the requested speeds */
		hw->phy.link_info.requested_speeds = config.link_speed;
		/* set link and auto negotiation so changes take effect */
		config.abilities |= I40E_AQ_PHY_ENABLE_ATOMIC_LINK;
		/* If link is up put link down */
		if (hw->phy.link_info.link_info & I40E_AQ_LINK_UP) {
#if RTE_VERSION >= RTE_VERSION_NUM(20, 5, 0, 0)
			int i;

			for (i = 0; i < dev->data->nb_tx_queues; i++)
				i40e_dev_tx_queue_stop(dev, i);

			for (i = 0; i < dev->data->nb_rx_queues; i++)
				i40e_dev_rx_queue_stop(dev, i);
#else
			i40e_dev_switch_queues(pf, FALSE);
#endif
		}

		/* make the aq call */
		status = i40e_aq_set_phy_config(hw, &config, NULL);
		if (status) {
			PMD_DRV_LOG(INFO, "Set phy config failed, err %s aq_err %s\n",
				    i40e_stat_str(hw, status),
				    i40e_aq_str(hw, hw->aq.asq_last_status));
			return -EAGAIN;
		}

		status = i40e_update_link_info(hw);
		if (status)
			PMD_DRV_LOG(INFO, "Updating link info failed with err %s aq_err %s\n",
				   i40e_stat_str(hw, status),
				   i40e_aq_str(hw, hw->aq.asq_last_status));

	} else {
		PMD_DRV_LOG(INFO, "Nothing changed, exiting without setting anything.\n");
	}

	return err;
}

static int
i40e_set_broadcast(struct rte_eth_dev *dev, uint8_t on)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct i40e_vsi *vsi = pf->main_vsi;

	if (i40e_aq_set_vsi_broadcast(hw, vsi->seid, on, NULL) != I40E_SUCCESS) {
		PMD_DRV_LOG(ERR, "Failed to set VSI broadcast");
		return -ENOTSUP;
	}

	return 0;
}

static int
i40e_netdev_flag_sync(struct rte_eth_dev *dev, unsigned int change,
	unsigned int new_flag)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	unsigned int supported[] = {RTE_IFF_BROADCAST, RTE_IFF_LOOPBACK,
		RTE_IFF_PROMISC, RTE_IFF_ALLMULTI, RTE_IFF_MULTICAST};
	int i;

	for (i=0; i < sizeof(supported)/sizeof(unsigned int); i++){
		if (change & supported[i]){
			switch(supported[i]){
			case RTE_IFF_BROADCAST:
				if (new_flag & RTE_IFF_BROADCAST) {
					i40e_set_broadcast(dev, 1);
					dev_ex->dev_iff_flag |= RTE_IFF_BROADCAST;
				} else {
					i40e_set_broadcast(dev, 0);
					dev_ex->dev_iff_flag &= ~RTE_IFF_BROADCAST;
				}
				break;
			case RTE_IFF_LOOPBACK:
				if ((new_flag & RTE_IFF_LOOPBACK) != 0) {
					i40e_set_tx_loopback(dev, 1);
					dev_ex->dev_iff_flag |= RTE_IFF_LOOPBACK;
				} else {
					i40e_set_tx_loopback(dev, 0);
					dev_ex->dev_iff_flag &= ~RTE_IFF_LOOPBACK;
				}
				break;
			case RTE_IFF_PROMISC:
				if (new_flag & RTE_IFF_PROMISC) {
					dev->dev_ops->promiscuous_enable(dev);
					dev_ex->dev_iff_flag |= RTE_IFF_PROMISC;
				} else {
					dev->dev_ops->promiscuous_disable(dev);
					dev_ex->dev_iff_flag &= ~RTE_IFF_PROMISC;
				}
				break;
			case RTE_IFF_ALLMULTI:
				if (new_flag & RTE_IFF_ALLMULTI) {
					dev->dev_ops->allmulticast_enable(dev);
					dev_ex->dev_iff_flag |= RTE_IFF_ALLMULTI;
				} else {
					dev->dev_ops->allmulticast_disable(dev);
					dev_ex->dev_iff_flag &= ~RTE_IFF_ALLMULTI;
				}
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

/* Ethtool Ops */
static int
i40e_get_reg_length(struct rte_eth_dev *dev)
{
	int total = 0;
	int i;

	for(i = 0; i < sizeof(i40e_reg_list)/sizeof(struct rte_dev_reg_set); i++)
		total += i40e_reg_list[i].count;

	return total*sizeof(int);
}

static int
i40e_get_regs(struct rte_eth_dev *dev,
	      struct rte_dev_ethtool_reg *regs, void *data)
{
	struct i40e_hw *hw = I40E_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t *reg = data;
	int total = 0, count;
	int i, j;

	if ((regs == NULL) || (data == NULL))
		return -EINVAL;

	for(i = 0; i < sizeof(i40e_reg_list)/sizeof(struct rte_dev_reg_set); i++) {
		count = i40e_reg_list[i].count;
		if (count > 1){
			for(j = 0; j < count; j++)
				reg[total++] = I40E_READ_REG(hw, i40e_reg_list[i].base_addr +
					i40e_reg_list[i].stride*j);
		} else
			reg[total++] = I40E_READ_REG(hw, i40e_reg_list[i].base_addr);

	}
	
	regs->len = total *sizeof(int);
	regs->version = 0x2000000;
	return regs->len;
}

static int
i40e_get_drvinfo(struct rte_eth_dev *dev, struct rte_dev_ethtool_drvinfo *drvinfo)
{
	struct rte_eth_dev_info dev_info;
	const char driver_name[] = "i40e\0";
	struct rte_pci_device *pci_dev = NULL;
#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	int status;
#endif
#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	const struct rte_bus *bus = NULL;
#endif

	if (drvinfo == NULL)
		return  -EINVAL;

#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
	status = i40e_fw_version_get(dev, drvinfo->fw_version, 
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
#if RTE_VERSION >= RTE_VERSION_NUM(18, 5, 0, 0)
	dev_info.device = dev->device;
#endif
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

	drvinfo->regdump_len = i40e_get_reg_length(dev);
	drvinfo->eedump_len = dev->dev_ops->get_eeprom_length(dev);
	drvinfo->n_stats = sizeof(struct rte_eth_stats)/sizeof(uint64_t);
	drvinfo->n_priv_flags = RTE_PRIV_FLAGS_STR_LEN;
	drvinfo->testinfo_len = 0;

	return 0;
}

static int
i40e_get_eeprom_len(struct rte_eth_dev *dev)
{
	return dev->dev_ops->get_eeprom_length(dev);
}

static int
i40e_get_eeprom(struct rte_eth_dev *dev, 
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
i40e_set_eeprom(struct rte_eth_dev *dev, struct rte_dev_ethtool_eeprom *info,
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
i40e_get_pauseparam(struct rte_eth_dev *dev,
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
i40e_set_pauseparam(struct rte_eth_dev *dev,
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
i40e_get_ringparam(struct rte_eth_dev *dev,
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
i40e_get_sset_count(struct rte_eth_dev *dev, int val)
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
i40e_get_strings(struct rte_eth_dev *dev, uint32_t stringset, uint8_t *ctx)
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
i40e_set_ringparam(struct rte_eth_dev *dev,
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
i40e_get_priv_flags(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	return dev_ex->priv_flag;
}

static int
i40e_set_priv_flags(struct rte_eth_dev *dev, u32 flags)
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
			PMD_DRV_LOG(INFO, "queue split drop enable is not supported over i40e\n");
			break;
		case VF_FLAG_MAC_PROMISC_BIT:
			i40e_set_vf_promisc(dev, vf_id, on);
			break;
		case VF_FLAG_ALLMULTI_BIT:
			i40e_set_vf_allmulticast(dev, vf_id, on);
			break;
		case VF_FLAG_BROADCAST_BIT:
			i40e_set_vf_broadcast(dev, vf_id, on);
			break;
		case VF_FLAG_SPOOFCHK_BIT:
			i40e_set_vf_spoofchk(dev, vf_id, on);
			break;
		case VF_FLAG_LINK_STATE_BIT:
			i40e_set_vf_link_state(dev, vf_id, on);
			break;
		case VF_FLAG_PING_VFS_BIT:
			i40e_ping_vfs(dev, vf_id);
			break;
		case VF_FLAG_MAC_ANTISPOOF_BIT:
			i40e_set_vf_mac_anti_spoof(dev, vf_id, on);
			break;
		case VF_FLAG_VLAN_ANTISPOOF_BIT:
			i40e_set_vf_vlan_anti_spoof(dev, vf_id, on);
			break;
		case VF_FLAG_VLAN_STRIPQ_BIT:
			i40e_set_vf_vlan_stripq(dev, vf_id, on);
			break;
		case VF_FLAG_RX_QUEUE_EN_BIT:
			PMD_DRV_LOG(INFO, "vf rx queue enable is not supported over i40e\n");
			break;
		case VF_FLAG_TX_QUEUE_EN_BIT:
			PMD_DRV_LOG(INFO, "vf tx queue enable is not supported over i40e\n");
			break;
		case VF_FLAG_REST_STATISTICS_BIT:
			if (on)
				i40e_reset_vf_stats(dev, vf_id);
			break;
		default:
			PMD_DRV_LOG(INFO, "Un-supported VF feature bit(%d)\n", i);
			break;
		}
	}
	return 0;
}

static int
i40e_begin(struct rte_eth_dev *dev)
{
	/* DPDK should be always on */
	return 0;
}

static int
i40e_complete(struct rte_eth_dev *dev)
{
	/* No close of DPDK, just close the interface */
	return 0;
}

static int 
i40e_get_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	/* update attributes that can be changed run-time */
	memcpy(&dev_ex->netdev_data.perm_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN);
	memcpy(&dev_ex->netdev_data.dev_addr, &dev->data->mac_addrs[0], ETHER_ADDR_LEN);
	dev_ex->netdev_data.mtu = dev->data->mtu;
    dev_ex->netdev_data.addr_len = ETHER_ADDR_LEN;
    dev_ex->netdev_data.type = 1; /* ARPHRD_ETHER */
	/*
	 * i40e doesn't have register to track IFF flag; instead, using
	 * internal register to track IFF flag change. Implication is that
	 * the initial value might not follow what's set by system.
	*/
	dev_ex->netdev_data.flags = dev_ex->dev_iff_flag;
    memcpy((void *)netdev_data, &dev_ex->netdev_data, sizeof(struct netdev_priv_data));

	return 0;
}

static int 
i40e_set_netdev_data(struct rte_eth_dev *dev, struct netdev_priv_data *netdev_data)
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
		return i40e_netdev_flag_sync(dev, change, netdev_data->flags);
		
	return 0;
}

static int
i40e_get_channels(struct rte_eth_dev *dev, struct rte_dev_ethtool_channels *ch)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

    memset(ch, 0, sizeof(struct rte_dev_ethtool_channels));
	if (i40e_is_vf_enabled(dev)) {
		ch->max_combined = I40E_MAX_VF_QUEUES*2;
        ch->combined_count = pf->vf_nb_qps;
    } else {
        ch->max_combined = I40E_MAX_PF_QUEUES*2;
        /* i40e support equal # rx/tx queue allocation */
        ch->combined_count = dev->data->nb_rx_queues*2;
    }

	return 0;
}

static const struct eth_dev_ethtool_ops i40e_ethtool_ops = {
	.get_netdev_data = i40e_get_netdev_data,
	.set_netdev_data = i40e_set_netdev_data,
	.get_setting = i40e_get_setting,
	.set_setting = i40e_set_setting,
	.get_regs_len = i40e_get_reg_length,
	.get_regs = i40e_get_regs,
	.get_drvinfo = i40e_get_drvinfo,
	.get_eeprom_len = i40e_get_eeprom_len,
	.get_eeprom = i40e_get_eeprom,
	.set_eeprom = i40e_set_eeprom,
	.get_pauseparam = i40e_get_pauseparam,
	.set_pauseparam = i40e_set_pauseparam,
	.get_ringparam = i40e_get_ringparam,
	.set_ringparam = i40e_set_ringparam,
	.get_priv_flags	= i40e_get_priv_flags,
	.set_priv_flags	= i40e_set_priv_flags,
	.get_strings = i40e_get_strings,
	.get_sset_count = i40e_get_sset_count,
	.get_channels = i40e_get_channels,
	/* pseudo function */
	.begin	= i40e_begin,
	.complete		= i40e_complete,
};

/* netdev op implementation */
static int
i40e_dev_start(struct rte_eth_dev *dev)
{
	return dev->dev_ops->dev_start(dev);
}

static int
i40e_dev_stop(struct rte_eth_dev *dev)
{
	dev->dev_ops->dev_stop(dev);
	return 0;
}

static int
i40e_change_rx_flag(struct rte_eth_dev *dev, int flags)
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
i40e_set_rx_mode(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = 
		rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_vsi *vsi = pf->main_vsi;
	uint8_t *mac_ptr;
	int i;

	if (dev_ex->dev_iff_flag & RTE_IFF_PROMISC)
		dev->dev_ops->promiscuous_enable(dev);
	else
		dev->dev_ops->promiscuous_disable(dev);
		
	if (dev_ex->dev_iff_flag & RTE_IFF_ALLMULTI)
		dev->dev_ops->allmulticast_enable(dev);
	else
		dev->dev_ops->allmulticast_disable(dev);

	/* set uc MAC address */
	if (dev_ex->nr_uc_addr > vsi->max_macaddrs) {
		dev->dev_ops->promiscuous_enable(dev);
	} else {
		if (dev_ex->nr_uc_addr) {
			mac_ptr = dev_ex->uc_addr_list;
			rte_memcpy(&dev->data->mac_addrs[0], mac_ptr,
				ETH_ADDR_LEN);
			dev->dev_ops->mac_addr_set(dev, (ether_addr_t *)mac_ptr);
			if (dev_ex->nr_uc_addr > 1)
				for (i = 1; i < dev_ex->nr_uc_addr; i++, 
					mac_ptr += ETH_ADDR_LEN) {
					dev->dev_ops->mac_addr_add(dev, (ether_addr_t *)mac_ptr,
						i, 0);
					rte_memcpy(&dev->data->mac_addrs[i], mac_ptr, 
						ETH_ADDR_LEN);
				}
		}
	}
	/* TO: add mc address list when i40e support dev_ops::mc_addr_list*/
	return 0;
}

static int
i40e_set_mac_addr(struct rte_eth_dev *dev, void *mac_addr)
{
	dev->dev_ops->mac_addr_set(dev, (ether_addr_t *)mac_addr);

	return 0;
}

static int 
i40e_validate_addr(struct rte_eth_dev *dev)
{
	if (is_valid_assigned_ether_addr((ether_addr_t *)&dev->data->mac_addrs[0]))
		return 1;

	return 0;
}

static int
i40e_change_mtu(struct rte_eth_dev *dev, int mtu)
{
	return dev->dev_ops->mtu_set(dev, mtu);
}

static int
i40e_get_stats64(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
{
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
i40e_get_stats(struct rte_eth_dev *dev, struct rte_eth_stats *out_stats)
{
	dev->dev_ops->stats_get(dev, out_stats);
	return 0;
}

static int
i40e_vlan_rx_add_vid(struct rte_eth_dev *dev, uint16_t vlan_id)
{
	return dev->dev_ops->vlan_pvid_set(dev, vlan_id, 1);
}

static int
i40e_vlan_rx_kill_vid(struct rte_eth_dev *dev, uint16_t vlan_id)
{
	return dev->dev_ops->vlan_pvid_set(dev, vlan_id, 0);
}

static int
i40e_is_vf_enabled(struct rte_eth_dev *dev)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_hw *hw = I40E_PF_TO_HW(pf);

	if (!hw->func_caps.sr_iov_1_1 || pf->vf_num == 0 ||
	    pf->vf_nb_qps == 0) {
		return 0;
	}

	return 1;
}
static int
i40e_set_vf_mac_addr(struct rte_eth_dev *dev, int vf_id, uint8_t *mac_addr)
{
	struct i40e_mac_filter *f;
	struct i40e_pf_vf *vf;
	struct i40e_vsi *vsi;
	struct i40e_pf *pf;
	void *temp;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	vf = &pf->vfs[vf_id];

	vsi = vf->vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	ether_addr_copy((ether_addr_t *)mac_addr, &vf->mac_addr);

	/* Remove all existing mac */
	TAILQ_FOREACH_SAFE(f, &vsi->mac_list, next, temp)
		i40e_vsi_delete_mac(vsi, &f->mac_info.mac_addr);

	return 0;
}

static int
i40e_set_vf_promisc(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	struct i40e_hw *hw;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	hw = I40E_VSI_TO_HW(vsi);

	ret = i40e_aq_set_vsi_unicast_promiscuous(hw, vsi->seid,
						  on, NULL, true);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to set unicast promiscuous mode");
	} else {
		if (on)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_MAC_PROMISC_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_MAC_PROMISC_BIT);
	}

	return ret;
}

static int
i40e_set_vf_allmulticast(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	struct i40e_hw *hw;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	hw = I40E_VSI_TO_HW(vsi);

	ret = i40e_aq_set_vsi_multicast_promiscuous(hw, vsi->seid,
						    on, NULL);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to set multicast promiscuous mode");
	} else {
		if (on)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_ALLMULTI_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_ALLMULTI_BIT);
	}

	return ret;
}

static int
i40e_set_vf_broadcast(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	struct i40e_hw *hw;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	hw = I40E_PF_TO_HW(pf);

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	ret = i40e_aq_set_vsi_broadcast(hw, vsi->seid, on, NULL);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to set VSI broadcast");
	} else {
		if (on)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_BROADCAST_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_BROADCAST_BIT);
	}


	return ret;
}

/*
 * This is an insertion of 16-bit vlan-tag (12-bit vlan-tag +
 * 3-bit qos + 1-bit vlan_proto
 */
static int
i40e_set_vf_vlan(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id,
	uint8_t qos)
{
	uint16_t vlanprio = vlan_id | ((uint16_t)qos << 12);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	int ret = 0;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	if (vlanprio) {
		if ((ret = dev->dev_ops->vlan_pvid_set(dev, vlanprio, 1)))
			return ret;
	} else {
		i40e_set_vf_vlan_stripq(dev, vf_id, 0); /* disable vlan-strip on the vsi */
		vsi->info.pvid = 0;
	}

	if (vlan_id) {
		RTE_LOG(INFO, PMD, "Setting VF %d with vlan-id %x, qos 0x%x\n",
			vf_id, vlan_id, qos);

		ret = i40e_vsi_add_vlan(vsi, vlan_id);
	}

	return ret;
}

static int
i40e_set_vf_rate(struct rte_eth_dev *dev, int vf_id, int max_tx_rate)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_hw *hw = I40E_PF_TO_HW(pf);
	struct i40e_pf_vf *pf_vf = &pf->vfs[vf_id];
	struct i40e_vf *vf = container_of(pf_vf->vsi, struct i40e_vf, vsi);
	struct i40e_vsi *vsi;
	int speed = 0;
	int ret = 0;

	vsi = pf->vfs[vf_id].vsi;;

	switch (vf->link_speed) {
	case I40E_LINK_SPEED_40GB:
		speed = 40000;
		break;
	case I40E_LINK_SPEED_25GB:
		speed = 25000;
		break;
	case I40E_LINK_SPEED_20GB:
		speed = 20000;
		break;
	case I40E_LINK_SPEED_10GB:
		speed = 10000;
		break;
	case I40E_LINK_SPEED_1GB:
		speed = 1000;
		break;
	default:
		break;
	}

	if (max_tx_rate > speed) {
		PMD_DRV_LOG(ERR, "Invalid tx rate %d specified for VF %d.",
			max_tx_rate, vf_id);
		return -EINVAL;
	}
	if ((max_tx_rate < 50) && (max_tx_rate > 0)) {
		PMD_DRV_LOG(WARNING, "Setting tx rate to minimum usable value of 50Mbps.\n");
		max_tx_rate = 50;
	}

	/* Tx rate credits are in values of 50Mbps, 0 is disabled */
	/* device can accumulate 4 credits max */
	ret = i40e_aq_config_vsi_bw_limit(hw, vsi->seid,
					  max_tx_rate / 50, 4, NULL);
	if (ret) {
		PMD_DRV_LOG(ERR, "Unable to set tx rate, error code %d.\n", ret);
		return ret;
	}
	
	/* Fixme: not-support-yte, vf->tx_rate = max_tx_rate;*/

	return 0;
}

static int
i40e_get_vf_config(struct rte_eth_dev *dev, int vf_id,
	struct common_vf_info *ivi)
{
	struct i40e_pf_vf *vf;
	struct i40e_vsi *vsi;
	struct i40e_pf *pf;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	vf = &pf->vfs[vf_id];

	vsi = vf->vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	rte_memcpy(ivi->mac, &vf->mac_addr, ETHER_ADDR_LEN);
	ivi->vlan = le16_to_cpu(vsi->info.pvid) & 0xFFF;
	ivi->qos = (le16_to_cpu(vsi->info.pvid) >> 12) & 0x7;
	
	/* Check if it has been already on or off */
	if (vsi->info.valid_sections &
		rte_cpu_to_le_16(I40E_AQ_VSI_PROP_SECURITY_VALID)) {		
		if (check_flag(vsi->info.sec_flags, I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK) ||
			check_flag(vsi->info.sec_flags, I40E_AQ_VSI_SEC_FLAG_ENABLE_VLAN_CHK))
			ivi->spoofchk = 1;
		else
			ivi->spoofchk = 0;
	}

	return 0;
}

static int
i40e_set_vf_spoofchk(struct rte_eth_dev *dev, int vf_id, int enable)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_hw *hw = I40E_PF_TO_HW(pf);
	struct i40e_vsi_context ctxt;
	struct i40e_vsi *vsi;
	int ret = 0;


	vsi = pf->vfs[vf_id].vsi;;
	
	/* need to add this variable to i40e_vf */
	/*if (enable == vf->spoofchk)
		goto out;

	vf->spoofchk = enable;*/
	memset(&ctxt, 0, sizeof(ctxt));
	ctxt.seid = vsi->seid;
	ctxt.pf_num = hw->pf_id;
	ctxt.info.valid_sections = cpu_to_le16(I40E_AQ_VSI_PROP_SECURITY_VALID);
	if (enable)
		ctxt.info.sec_flags |= (I40E_AQ_VSI_SEC_FLAG_ENABLE_VLAN_CHK |
					I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK);
	ret = i40e_aq_update_vsi_params(hw, &ctxt, NULL);
	if (ret)
		PMD_DRV_LOG(ERR, "Error %d updating VSI parameters\n", ret);
	else{
		if (enable)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_SPOOFCHK_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_SPOOFCHK_BIT);
	}

	return ret;
}

static int
i40e_set_vf_link_state(struct rte_eth_dev *dev, int vf_id, int link)
{
	return -1;
}

static int
i40e_ping_vfs(struct rte_eth_dev *dev, int vf_id)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	i40e_notify_vf_link_status(dev, &pf->vfs[vf_id]);

	return 0;
}

static int
i40e_set_vf_mac_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	struct i40e_hw *hw;
	struct i40e_vsi_context ctxt;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	vsi = pf->vfs[vf_id].vsi;
	/* Check if it has been already on or off */
	if (vsi->info.valid_sections &
		rte_cpu_to_le_16(I40E_AQ_VSI_PROP_SECURITY_VALID)) {
		if (on) {
			if ((vsi->info.sec_flags &
			     I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK) ==
			    I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK)
				return 0; /* already on */
		} else {
			if ((vsi->info.sec_flags &
			     I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK) == 0)
				return 0; /* already off */
		}
	}

	vsi->info.valid_sections = cpu_to_le16(I40E_AQ_VSI_PROP_SECURITY_VALID);
	if (on)
		vsi->info.sec_flags |= I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK;
	else
		vsi->info.sec_flags &= ~I40E_AQ_VSI_SEC_FLAG_ENABLE_MAC_CHK;

	memset(&ctxt, 0, sizeof(ctxt));
	(void)rte_memcpy(&ctxt.info, &vsi->info, sizeof(vsi->info));
	ctxt.seid = vsi->seid;

	hw = I40E_VSI_TO_HW(vsi);
	ret = i40e_aq_update_vsi_params(hw, &ctxt, NULL);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to update VSI params");
	} else {
		if (on)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_MAC_ANTISPOOF_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_MAC_ANTISPOOF_BIT);
	}
	return 0;
}

static int
i40e_set_vf_vlan_anti_spoof(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	struct i40e_hw *hw;
	struct i40e_vsi_context ctxt;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	if (vf_id >= pf->vf_num || !pf->vfs) {
		PMD_DRV_LOG(ERR, "Invalid argument.");
		return -EINVAL;
	}

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	/* Check if it has been already on or off */
	if (!!(dev_ex->vf_flags[vf_id]&VF_FLAG_VLAN_ANTISPOOF_BIT) == on)
		return 0; /* already on or off */

	if (on)
		dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_VLAN_ANTISPOOF_BIT);
	else
		dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_VLAN_ANTISPOOF_BIT);

	ret = i40e_add_rm_all_vlan_filter(vsi, on);
	if (ret) {
		PMD_DRV_LOG(ERR, "Failed to remove VLAN filters.");
		return -ENOTSUP;
	}

	vsi->info.valid_sections = cpu_to_le16(I40E_AQ_VSI_PROP_SECURITY_VALID);
	if (on)
		vsi->info.sec_flags |= I40E_AQ_VSI_SEC_FLAG_ENABLE_VLAN_CHK;
	else
		vsi->info.sec_flags &= ~I40E_AQ_VSI_SEC_FLAG_ENABLE_VLAN_CHK;

	memset(&ctxt, 0, sizeof(ctxt));
	(void)rte_memcpy(&ctxt.info, &vsi->info, sizeof(vsi->info));
	ctxt.seid = vsi->seid;

	hw = I40E_VSI_TO_HW(vsi);
	ret = i40e_aq_update_vsi_params(hw, &ctxt, NULL);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to update VSI params");
	}

	return ret;
}

/*
 * Inser/delete VLAN-ID on multiple VFs based upon vf_mask
 * @vf_mask: bit 1: the specific VF is selected for
 * vlan-id insertion/deletion
 * @vlan: vlan tag to insert/delete
 * @on: 1: insert 0: delete
 *
 */
static int
i40e_set_vf_vlan_filter(struct rte_eth_dev *dev, uint64_t vf_mask,
	uint16_t vlan_id, uint8_t on)
{
	struct i40e_pf *pf;
	uint16_t vf_idx;
	int ret = I40E_SUCCESS;

	if (vf_mask == 0)
		return -EINVAL;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	for (vf_idx = 0; vf_idx < 64 && ret == I40E_SUCCESS; vf_idx++) {
		if (vf_mask & ((uint64_t)(1ULL << vf_idx))) {
			if (on)
				ret = i40e_vsi_add_vlan(pf->vfs[vf_idx].vsi, vlan_id);
			else
				ret = i40e_vsi_delete_vlan(pf->vfs[vf_idx].vsi, vlan_id);
			if (ret != I40E_SUCCESS) {
				ret = -ENOTSUP;
				PMD_DRV_LOG(ERR, "Failed to set VF VLAN filter, on = %d"
					", on vf #%d", on, vf_idx);
			}
		}
	}
	return ret;
}

static int
i40e_set_vf_vlan_insert(struct rte_eth_dev *dev, int vf_id, uint16_t vlan_id)
{
	struct i40e_pf *pf;
	struct i40e_hw *hw;
	struct i40e_vsi *vsi;
	struct i40e_vsi_context ctxt;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	hw = I40E_PF_TO_HW(pf);

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	vsi->info.valid_sections = cpu_to_le16(I40E_AQ_VSI_PROP_VLAN_VALID);
	vsi->info.pvid = vlan_id;
	if (vlan_id > 0)
		vsi->info.port_vlan_flags |= I40E_AQ_VSI_PVLAN_INSERT_PVID;
	else
		vsi->info.port_vlan_flags &= ~I40E_AQ_VSI_PVLAN_INSERT_PVID;

	memset(&ctxt, 0, sizeof(ctxt));
	(void)rte_memcpy(&ctxt.info, &vsi->info, sizeof(vsi->info));
	ctxt.seid = vsi->seid;

	hw = I40E_VSI_TO_HW(vsi);
	ret = i40e_aq_update_vsi_params(hw, &ctxt, NULL);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to update VSI params");
	}

	return ret;
}

static int
i40e_set_vf_rate_limit(struct rte_eth_dev *dev, int vf_id, uint16_t tx_rate,
	uint64_t q_msk)
{
	/* ? i40e doesn't support per-queue rate limit */
	return -1;
}

static int
i40e_set_vf_vlan_stripq(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	if (vf_id >= pf->vf_num || !pf->vfs) {
		PMD_DRV_LOG(ERR, "Invalid argument.");
		return -EINVAL;
	}

	vsi = pf->vfs[vf_id].vsi;

	if (!vsi)
		return -EINVAL;

	ret = i40e_vsi_config_vlan_stripping(vsi, !!on);
	if (ret != I40E_SUCCESS) {
		ret = -ENOTSUP;
		PMD_DRV_LOG(ERR, "Failed to set VLAN stripping!");
	} else {
		if (on)
			dev_ex->vf_flags[vf_id] |= (1 << VF_FLAG_VLAN_STRIPQ_BIT);
		else
			dev_ex->vf_flags[vf_id] &= ~(1 << VF_FLAG_VLAN_STRIPQ_BIT);
	}

	return 0;
}

static int
i40e_set_vf_rxmode(struct rte_eth_dev *dev, int vf_id, uint16_t rx_mask, uint8_t on)
{
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;
	int mask_bits[]={ETH_VMDQ_ACCEPT_UNTAG, ETH_VMDQ_ACCEPT_HASH_MC,
		ETH_VMDQ_ACCEPT_HASH_UC, ETH_VMDQ_ACCEPT_BROADCAST, ETH_VMDQ_ACCEPT_MULTICAST};
	int i, ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	if (vf_id >= pf->vf_num || !pf->vfs) {
		PMD_DRV_LOG(ERR, "Invalid argument.");
		return -EINVAL;
	}

	vsi = pf->vfs[vf_id].vsi;

	if (!vsi)
		return -EINVAL;

	for(i = 0; i < sizeof(mask_bits)/sizeof(int); i++)
		if (rx_mask & mask_bits[i]) {
			ret = 0;
			switch(mask_bits[i]) {
			case ETH_VMDQ_ACCEPT_UNTAG:
				i40e_set_vf_untag_vlan(vsi, on==0);
				break;
			case ETH_VMDQ_ACCEPT_HASH_MC:
				break;
			case ETH_VMDQ_ACCEPT_HASH_UC:
				break;
			case ETH_VMDQ_ACCEPT_BROADCAST:
				ret = i40e_set_vf_broadcast(dev, vf_id, on);
				break;
			case ETH_VMDQ_ACCEPT_MULTICAST:
				ret = i40e_set_vf_allmulticast(dev, vf_id, on);
				break;
			}
			if (ret) {
				PMD_DRV_LOG(ERR, "Fail to set %s on vf%d\n",
					mask_bits[i]==ETH_VMDQ_ACCEPT_UNTAG?"ETH_VMDQ_ACCEPT_UNTAG":
					mask_bits[i]==ETH_VMDQ_ACCEPT_HASH_MC?"ETH_VMDQ_ACCEPT_HASH_MC":
					mask_bits[i]==ETH_VMDQ_ACCEPT_HASH_UC?"ETH_VMDQ_ACCEPT_HASH_UC":
					mask_bits[i]==ETH_VMDQ_ACCEPT_BROADCAST?"ETH_VMDQ_ACCEPT_BROADCAST":
					mask_bits[i]==ETH_VMDQ_ACCEPT_MULTICAST?"ETH_VMDQ_ACCEPT_MULTICAST":"unknown",
					vf_id);
				return ret;
			}
		}
		
	return 0;
}

static int
i40e_set_vf_rx(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	return -1;
}

static int
i40e_set_vf_tx(struct rte_eth_dev *dev, int vf_id, uint8_t on)
{
	return -1;
}

static int
i40e_get_vf_stats(struct rte_eth_dev *dev, int vf_id, struct rte_eth_stats *stats)
{
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	if (vf_id >= pf->vf_num || !pf->vfs) {
		PMD_DRV_LOG(ERR, "Invalid VF ID.");
		return -EINVAL;
	}

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	i40e_update_vsi_stats(vsi);

	stats->ipackets = vsi->eth_stats.rx_unicast +
			vsi->eth_stats.rx_multicast +
			vsi->eth_stats.rx_broadcast;
	stats->opackets = vsi->eth_stats.tx_unicast +
			vsi->eth_stats.tx_multicast +
			vsi->eth_stats.tx_broadcast;
	stats->ibytes   = vsi->eth_stats.rx_bytes;
	stats->obytes   = vsi->eth_stats.tx_bytes;
	stats->ierrors  = vsi->eth_stats.rx_discards;
	stats->oerrors  = vsi->eth_stats.tx_errors + vsi->eth_stats.tx_discards;

	return 0;
}

static int
i40e_reset_vf_stats(struct rte_eth_dev *dev, int vf_id)
{
	struct i40e_pf *pf;
	struct i40e_vsi *vsi;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	if (vf_id >= pf->vf_num || !pf->vfs) {
		PMD_DRV_LOG(ERR, "Invalid VF ID.");
		return -EINVAL;
	}

	vsi = pf->vfs[vf_id].vsi;
	if (!vsi) {
		PMD_DRV_LOG(ERR, "Invalid VSI.");
		return -EINVAL;
	}

	vsi->offset_loaded = false;
	i40e_update_vsi_stats(vsi);

	return 0;
}

static int
i40e_set_tx_loopback(struct rte_eth_dev *dev, uint8_t on)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);
	struct i40e_pf *pf;
	struct i40e_pf_vf *vf;
	struct i40e_vsi *vsi;
	uint16_t vf_id;
	uint8_t vlan_anti_spoof_on;
	int ret;

	pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	/* setup PF TX loopback */
	vsi = pf->main_vsi;
	/* PF anti-spoof is always off */
	ret = i40e_vsi_set_tx_loopback(vsi, on, 0);
	if (ret)
		return -ENOTSUP;

	/* setup TX loopback for all the VFs */
	if (!pf->vfs) {
		/* if no VF, do nothing. */
		return 0;
	}

	for (vf_id = 0; vf_id < pf->vf_num; vf_id++) {
		vf = &pf->vfs[vf_id];
		vsi = vf->vsi;

		vlan_anti_spoof_on = (dev_ex->vf_flags[vf_id] & VF_FLAG_SPOOFCHK_BIT)?
			1:0;
		ret = i40e_vsi_set_tx_loopback(vsi, on, vlan_anti_spoof_on);
		if (ret)
			return -ENOTSUP;
	}

	return ret;
}

static int
i40e_fix_features(struct rte_eth_dev *dev __rte_unused, uint64_t *features)
{
	uint64_t fix_up = *features;

	*features = fix_up;

	return 0;
}

static int
i40e_set_features(struct rte_eth_dev *dev, uint64_t features)
{
	if (features & RTE_NETIF_F_HW_VLAN_RX)
		dev->dev_ops->vlan_offload_set(dev, 1);
	else
		dev->dev_ops->vlan_offload_set(dev, 0);

	return 0;
}

static int
i40e_dev_init(struct rte_eth_dev *dev)
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
i40e_dev_uninit(struct rte_eth_dev *dev __rte_unused)
{
	return 0;
}

static const struct eth_dev_netdev_ops i40e_netdev_ops = {
	.init				= i40e_dev_init,
	.uninit				= i40e_dev_uninit,
	.open				= i40e_dev_start,
	.stop				= i40e_dev_stop,
	.set_rx_mode		= i40e_set_rx_mode,
	.change_rx_flag		= i40e_change_rx_flag,
	.validate_addr		= i40e_validate_addr,
	.change_mtu			= i40e_change_mtu,
	.get_stats64		= i40e_get_stats64,
	.get_stats			= i40e_get_stats,
	.vlan_rx_add_vid	= i40e_vlan_rx_add_vid,
	.vlan_rx_kill_vid	= i40e_vlan_rx_kill_vid,
	.set_mac_addr		= i40e_set_mac_addr,
	.set_tx_loopback	= i40e_set_tx_loopback,
	/* i40e doesn't support fix_features */
	.fix_features		= i40e_fix_features,
	.set_features		= i40e_set_features,
/* PF manage VF functions */
	.is_vf_enabled		= i40e_is_vf_enabled,
	.set_vf_mac_addr	= i40e_set_vf_mac_addr,
	.set_vf_promisc		= i40e_set_vf_promisc,
	.set_vf_allmulticast = i40e_set_vf_allmulticast,
	.set_vf_broadcast	= i40e_set_vf_broadcast,
	.set_vf_vlan		= i40e_set_vf_vlan,
	.set_vf_rate		= i40e_set_vf_rate,
	.get_vf_config		= i40e_get_vf_config,
	.set_vf_spoofchk	= i40e_set_vf_spoofchk,
	.set_vf_link_state	= i40e_set_vf_link_state,
	.ping_vfs			= i40e_ping_vfs,
	.set_vf_mac_anti_spoof = i40e_set_vf_mac_anti_spoof,
	.set_vf_vlan_anti_spoof = i40e_set_vf_vlan_anti_spoof,
	.set_vf_vlan_filter	= i40e_set_vf_vlan_filter,
	.set_vf_vlan_insert	= i40e_set_vf_vlan_insert,
	.set_vf_rate_limit	= i40e_set_vf_rate_limit,
	.set_vf_vlan_stripq = i40e_set_vf_vlan_stripq,
	.set_vf_rxmode		= i40e_set_vf_rxmode,
	.set_vf_rx			= i40e_set_vf_rx,
	.set_vf_tx			= i40e_set_vf_tx,
	.get_vf_stats		= i40e_get_vf_stats,
	.reset_vf_stats		= i40e_reset_vf_stats,
};

static int
eth_i40e_dev_init_ex(struct rte_eth_dev *dev)
{
	struct rte_eth_dev_ex *dev_ex = rte_eth_get_devex_by_dev(dev);

	dev_ex->dev_ethtool_ops = &i40e_ethtool_ops;
	dev_ex->dev_netdev_ops = &i40e_netdev_ops;
	memset(dev_ex->vf_flags, 0, sizeof(unsigned int)*RTE_MAX_VF_COUNT);
	dev_ex->is_vf		= false;

	return 0;
}
RTE_PMD_EX_REGISTER_PCI(net_i40e, eth_i40e_dev_init_ex);
