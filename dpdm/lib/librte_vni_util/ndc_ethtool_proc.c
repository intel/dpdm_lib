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
#include <linux/ethtool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <vni_share_types.h>
#include "rte_ethdev.h"
#include "rte_manage.h"
#include "rte_ethtool_ops.h"
#include "rte_netdev_ops.h"
#include "ethdev_ethtool_types.h"
#include "rte_vf_flags.h"

#include "ndc_types.h"
#include "ndc_ethtool_proc.h"


inline int is_ethtool_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_ethtool_set_setting) &&
		(cmd <= vni_ethtool_set_per_queue_coalesce))
		return 1;
	return 0;
}

void vni_upstream_vf_stats(struct rte_eth_stats *stats)
{
	char buff[1024];
	int fd = open("/sys/kernel/vni_manage/client_cache", O_WRONLY);

	memset(buff, 0, 1024);
	snprintf(buff, 1024,
		"ipackets %llu\n"
		"opackets %llu\n"
		"ibytes %llu\n"
		"obytes %llu\n"
		"ierrors %llu\n"
		"oerrors %llu\n",
		(unsigned long long)stats->ipackets,
		(unsigned long long)stats->opackets,
		(unsigned long long)stats->ibytes,
		(unsigned long long)stats->obytes,
		(unsigned long long)stats->ierrors,
		(unsigned long long)stats->oerrors);
	if (write(fd, buff, strlen(buff)) < strlen(buff))
		RTE_VF_FLAG_TRACE("fail to write all the buff\n");
	close(fd);
}

int ethtool_set_vf_priv_flat(port_t port_id, uint32_t priv_flag)
{
	struct rte_eth_stats vf_stats;
	uint8_t seq;
	uint16_t feature_bits, vlan_id;
	int vf_id, mode, status=0;
	uint64_t mask;

	seq = VF_FLAG_OP_SEQ(priv_flag);
	vf_id = VF_FLAG_VF_INDEX(priv_flag);
	feature_bits = VF_FLAG_FEATURES(priv_flag);
	mode = (VF_FLAG_MODE(priv_flag) > 0)?1:0;
	RTE_VF_FLAG_TRACE("vf-id is %d, feature-bits is %x, op-code is %d\n",
		vf_id, feature_bits, VF_FLAG_OP_CODE(priv_flag));

	switch(VF_FLAG_OP_CODE(priv_flag)) {
	case VF_FLAG_CODE_VF_FEATURE:
		RTE_VF_FLAG_TRACE("set_priv_flags with %x\n", priv_flag);
		status = rte_ethtool_set_priv_flags(port_id, priv_flag);
		break;
	case VF_FLAG_CODE_RXMODE_SET:
		RTE_VF_FLAG_TRACE("rxmode_set with %x\n", priv_flag);
		status = rte_netdev_set_vf_rxmode(port_id, vf_id, feature_bits,
			mode);
		break;
	case VF_FLAG_CODE_GET_STATS:
		RTE_VF_FLAG_TRACE("get_stats\n");
		status = rte_netdev_get_vf_stats(port_id, vf_id, &vf_stats);
		vni_upstream_vf_stats(&vf_stats);
		break;
	case VF_FLAG_CODE_VLAN_FILT:
		switch(seq) {
		case 0:
			mask = rte_get_qmask(port_id);
			vlan_id = feature_bits & ETHER_MAX_VLAN_ID;
			RTE_VF_FLAG_TRACE("vlan_filter with mask=%llx, vlan-id=%x mode=%x\n",
				(unsigned long long)mask, vlan_id, mode);
			status = rte_netdev_set_vf_vlan_filter(port_id,
				mask, vlan_id, mode);
			RTE_VF_FLAG_TRACE("status is %d from rte_netdev_set_vf_vlan_filter\n", status);
			break;
		case 1:
		case 2:
		case 3:
		case 4:
			rte_set_qmask(port_id, seq-1, feature_bits);
			status = 0;
			break;
		default:
			RTE_VF_FLAG_TRACE("Incorrect seq(%d), ignored ...\n", seq);
			status = 0;
			break;
		}
		break;
	case VF_FLAG_CODE_VLAN_INSERT:
		RTE_VF_FLAG_TRACE("set vf (%d) with vlan (%x) insertion\n", vf_id,
			feature_bits);
		status = rte_netdev_set_vf_vlan_insert(port_id, vf_id, feature_bits);
		break;
	case VF_FLAG_CODE_VF_RATE:
		RTE_VF_FLAG_TRACE("set vf rate %x on vf %d\n", feature_bits, vf_id);
		status = rte_netdev_set_vf_rate(port_id, vf_id, feature_bits);
		break;
	case VF_FLAG_CODE_VF_Q_RATE:
		switch(seq) {
		case 0:
			mask = rte_get_mask(port_id, vf_id);
			RTE_VF_FLAG_TRACE("rte_netdev_set_vf_rate_limit with mask=%llx,"
				" vf-id=%d rate=%x\n", (unsigned long long)mask, vf_id, feature_bits);
			status = rte_netdev_set_vf_rate_limit(port_id,
				vf_id, feature_bits, mask);
			break;
		case 1:
		case 2:
		case 3:
		case 4:
			rte_set_mask(port_id, seq-1, vf_id, feature_bits);
			status = 0;
			break;
		default:
			RTE_VF_FLAG_TRACE("Incorrect seq(%d), ignored ...\n", seq);
			status = 0;
			break;
		}
		break;
	default:
		RTE_VF_FLAG_TRACE("unsupported VF private flag code %d\n",
			VF_FLAG_OP_CODE(priv_flag));
		status = -1;
		break;
	}
	return status;
}

void ethtool_cmd_proc(netdev_cmd_info *cmd_info, netdev_cmd_info *send_cmd_info)
{
	struct rte_dev_ethtool_reg *reg_info;
	struct rte_dev_ethtool_eeprom *eeprom_info;
	struct rte_dev_ethtool_dump *dump;
	struct rte_dev_ethtool_tunable *tun;
	struct netdev_priv_data *netdev_data;
	int sset_count;
	uint32_t priv_flag;
	unsigned char *buf;

	switch (cmd_info->cmd) {
	case vni_ethtool_set_setting:
		send_cmd_info->status = rte_ethtool_set_settings(cmd_info->port_id,
			(struct rte_dev_ethtool_cmd *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_setting:
		INIT_DATA(send_cmd_info->data, 0, sizeof(struct rte_dev_ethtool_cmd));
		send_cmd_info->status = rte_ethtool_get_settings(cmd_info->port_id,
			(struct rte_dev_ethtool_cmd *)send_cmd_info->data);
		{
			struct rte_dev_ethtool_cmd *cmd = (struct rte_dev_ethtool_cmd *)send_cmd_info->data;
			RTE_VNI_DEBUG_TRACE("\ncmd=%x supported=%d advertising=%d speed=%d\n"
				"port=%d phy_address=%d transceiver=%d autoneg=%d\n"
				"autoneg=%d mdio_support=%d maxtxpkt=%d maxrxpkt=%d\n"
				"speed_hi=%d eth_tp_mdix=%d eth_tp_mdix_ctrl=%d\n"
				"lp_advertising=%d\n",
				cmd->cmd, cmd->supported, cmd->advertising, cmd->speed,
				cmd->duplex, cmd->port, cmd->phy_address, cmd->transceiver,
				cmd->autoneg, cmd->mdio_support, cmd->maxtxpkt, cmd->maxrxpkt,
				cmd->speed_hi, cmd->eth_tp_mdix, cmd->eth_tp_mdix_ctrl,
				cmd->lp_advertising
				);
		}

		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_cmd);
		break;
	case vni_ethtool_get_drvinfo:
		INIT_DATA(send_cmd_info->data, 0, sizeof(struct rte_dev_ethtool_drvinfo));
		send_cmd_info->status = rte_ethtool_get_drvinfo(cmd_info->port_id,
			(struct rte_dev_ethtool_drvinfo *)send_cmd_info->data);
		RTE_VNI_DEBUG_TRACE("drvinfo: name=%s\n", ((struct rte_dev_ethtool_drvinfo *)send_cmd_info->data)->driver);
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_drvinfo);
		break;
	case vni_ethtool_get_reg_len:
		send_cmd_info->status = rte_ethtool_get_reg_len(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_reg:
		reg_info = (struct rte_dev_ethtool_reg *)send_cmd_info->data;
		COPY_DATA(send_cmd_info->data, cmd_info->data, sizeof(struct rte_dev_ethtool_reg));
		send_cmd_info->status = rte_ethtool_get_reg(cmd_info->port_id,
			(struct rte_dev_ethtool_reg *)send_cmd_info->data,
			GET_PTR(send_cmd_info, sizeof(struct rte_dev_ethtool_reg), void));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_reg)+reg_info->len;
		break;
	case vni_ethtool_get_wol:
		COPY_DATA(send_cmd_info->data, cmd_info->data, sizeof(struct rte_dev_ethtool_wolinfo));
		send_cmd_info->status = rte_ethtool_get_wol(cmd_info->port_id,
			(struct rte_dev_ethtool_wolinfo *)send_cmd_info->data);
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_wolinfo);
		break;
	case vni_ethtool_set_wol:
		send_cmd_info->status = rte_ethtool_set_wol(cmd_info->port_id,
			(struct rte_dev_ethtool_wolinfo *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_msglevel:
		send_cmd_info->status = rte_ethtool_get_msglevel(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_set_msglevel:
		send_cmd_info->status = rte_ethtool_set_msglevel(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, uint32_t, cmd_info->data_length));
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_nway_reset:
		send_cmd_info->status = rte_ethtool_nway_reset(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_link:
		send_cmd_info->status = rte_ethtool_get_link(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_eeprom_len:
		send_cmd_info->status = rte_ethtool_get_eeprom_len(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_eeprom:
		COPY_DATA(send_cmd_info->data, cmd_info->data, sizeof(struct rte_dev_ethtool_eeprom));
		eeprom_info = (struct rte_dev_ethtool_eeprom *)send_cmd_info->data;
		send_cmd_info->status = rte_ethtool_get_eeprom(cmd_info->port_id,
			(struct rte_dev_ethtool_eeprom *)send_cmd_info->data,
			GET_PTR(send_cmd_info->data, sizeof(struct rte_dev_ethtool_eeprom), uint8_t));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_eeprom)+eeprom_info->len;
		break;
	case vni_ethtool_set_eeprom:
		buf = (unsigned char *)send_cmd_info->data;
		eeprom_info = (struct rte_dev_ethtool_eeprom *)cmd_info->data;
		send_cmd_info->status = rte_ethtool_set_eeprom(cmd_info->port_id,
			eeprom_info, (uint8_t *)&buf[sizeof(struct rte_dev_ethtool_eeprom)]);
		break;
	case vni_ethtool_get_coalesce:
		COPY_DATA(send_cmd_info->data, cmd_info->data, sizeof(struct rte_dev_ethtool_coalesce));
		send_cmd_info->status = rte_ethtool_get_coalesce(cmd_info->port_id,
			(struct rte_dev_ethtool_coalesce *)send_cmd_info->data);
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_coalesce);
		break;
	case vni_ethtool_set_coalesce:
		send_cmd_info->status = rte_ethtool_set_coalesce(cmd_info->port_id,
			(struct rte_dev_ethtool_coalesce *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_ringparam:
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_ringparam));
		send_cmd_info->status = rte_ethtool_get_ringparam(cmd_info->port_id,
			(struct rte_dev_ethtool_ringparam *)send_cmd_info->data);
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_ringparam);
		break;
	case vni_ethtool_set_ringparam:
		send_cmd_info->status = rte_ethtool_set_ringparam(cmd_info->port_id,
			(struct rte_dev_ethtool_ringparam *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_pauseparam:
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_pauseparam));
		send_cmd_info->status = rte_ethtool_get_pauseparam(cmd_info->port_id,
			(struct rte_dev_ethtool_pauseparam *)send_cmd_info->data);
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_pauseparam);
		break;
	case vni_ethtool_set_pauseparam:
		send_cmd_info->status = rte_ethtool_set_pauseparam(cmd_info->port_id,
			(struct rte_dev_ethtool_pauseparam *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_self_test:
		buf = (unsigned char *)send_cmd_info->data;
		send_cmd_info->status = rte_ethtool_self_test(cmd_info->port_id,
			(struct rte_dev_ethtool_test *)cmd_info->data,
			(uint64_t *)&buf[sizeof(struct rte_dev_ethtool_test)]);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_test));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_test);
		break;
	case vni_ethtool_get_strings:
		sset_count = rte_ethtool_get_sset_count(cmd_info->port_id, ETH_SS_PRIV_FLAGS);
		send_cmd_info->status = rte_ethtool_get_strings(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, uint32_t, cmd_info->data_length),
			GET_PTR(send_cmd_info->data, 0, uint8_t));
		send_cmd_info->data_length = sset_count*ETH_GSTRING_LEN;
		break;
	case vni_ethtool_set_phys_id:
		send_cmd_info->status = rte_ethtool_set_phys_id(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, enum rte_dev_ethtool_phys_id_state, cmd_info->data_length));
		send_cmd_info->data_length = 0;
		break;
	/* TODO: instead of passing string pointer, passing the entire stat string */
	case vni_ethtool_get_stats: 
		buf = (unsigned char *)send_cmd_info->data;
		send_cmd_info->status = rte_ethtool_get_ethtool_stats(cmd_info->port_id,
			(struct rte_dev_ethtool_stats *)buf,
			(uint64_t *)(&buf[sizeof(struct rte_dev_ethtool_stats)]));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_stats);
		break;
	case vni_ethtool_begin:
		rte_ethtool_get_netdev_data(cmd_info->port_id, (void *)send_cmd_info->data);
		send_cmd_info->status = rte_ethtool_begin(cmd_info->port_id);
		send_cmd_info->data_length = sizeof(struct netdev_priv_data);
		break;
	case vni_ethtool_complete:
		send_cmd_info->status = rte_ethtool_complete(cmd_info->port_id);
		netdev_data = (struct netdev_priv_data *)cmd_info->data;
		rte_ethtool_set_netdev_data(cmd_info->port_id, (void *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_priv_flags:
		send_cmd_info->status = rte_ethtool_get_priv_flags(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_set_priv_flags:
		priv_flag = GET_DATA(cmd_info->data, 0, uint32_t, cmd_info->data_length);
		if (VF_FLAG_IS_VF(priv_flag))
			send_cmd_info->status = ethtool_set_vf_priv_flat(cmd_info->port_id, priv_flag);
		else {
			/* PF priv flag */
			send_cmd_info->status = rte_ethtool_set_priv_flags(cmd_info->port_id,
				priv_flag);
		}
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_sset_count:
		send_cmd_info->status = rte_ethtool_get_sset_count(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, int, 
			sizeof(cmd_info)-sizeof(struct netdev_cmd_info)));
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_rxnfc:
		send_cmd_info->status = rte_ethtool_get_rxnfc(cmd_info->port_id,
			(struct rte_dev_ethtool_rxnfc *)cmd_info->data,
			(uint32_t *)((void *)((uint8_t *)cmd_info->data + sizeof(struct rte_dev_ethtool_rxnfc))));
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_set_rxnfc:
		send_cmd_info->status = rte_ethtool_set_rxnfc(cmd_info->port_id,
			(struct rte_dev_ethtool_rxnfc *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_flash_device:
		send_cmd_info->status = rte_ethtool_flash_device(cmd_info->port_id,
			(struct rte_dev_ethtool_flash *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_reset:
		send_cmd_info->status = rte_ethtool_reset(cmd_info->port_id,
			(uint32_t *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_rxfh_key_size:
		send_cmd_info->status = rte_ethtool_get_rxfh_key_size(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_rxfh_indir_size:
		send_cmd_info->status = rte_ethtool_get_rxfh_indir_size(cmd_info->port_id);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_rxfh:
		send_cmd_info->status = rte_ethtool_get_rxfh(cmd_info->port_id,
			(uint32_t *)cmd_info->data,
			(uint8_t *)((uint8_t *)cmd_info->data + sizeof(uint32_t)),
			(uint8_t *)send_cmd_info->data);
		send_cmd_info->data_length = 1;
		break;
	case vni_ethtool_set_rxfh:
		send_cmd_info->status = rte_ethtool_set_rxfh(cmd_info->port_id,
			(uint32_t *)cmd_info->data,
			(uint8_t *)((uint8_t *)cmd_info->data + sizeof(uint32_t)),
			(uint8_t *)((uint8_t *)cmd_info->data + sizeof(uint32_t)+sizeof(uint8_t)));
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_channels:
		send_cmd_info->status = rte_ethtool_get_channels(cmd_info->port_id,
			(struct rte_dev_ethtool_channels *)cmd_info->data);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_channels));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_channels);
		break;
	case vni_ethtool_set_channels:
		send_cmd_info->status = rte_ethtool_set_channels(cmd_info->port_id,
			(struct rte_dev_ethtool_channels *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_dump_flag:
		send_cmd_info->status = rte_ethtool_get_dump_flag(cmd_info->port_id,
			(struct rte_dev_ethtool_dump *)cmd_info->data);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_dump));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_dump);
		break;
	case vni_ethtool_get_dump_data:
		buf = (unsigned char *)send_cmd_info->data;
		dump = (struct rte_dev_ethtool_dump *)cmd_info->data;
		send_cmd_info->status = rte_ethtool_get_dump_data(cmd_info->port_id,
			dump,
			(void *)(&buf[sizeof(struct rte_dev_ethtool_dump)]));
		COPY_DATA(send_cmd_info->data, dump,
			sizeof(struct rte_dev_ethtool_dump));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_dump)+
			dump->len;
		break;
	case vni_ethtool_set_dump:
		send_cmd_info->status = rte_ethtool_set_dump(cmd_info->port_id,
			(struct rte_dev_ethtool_dump *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_ts_info:
		send_cmd_info->status = rte_ethtool_get_ts_info(cmd_info->port_id,
			(struct rte_dev_ethtool_ts_info *)cmd_info->data);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_ts_info));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_ts_info);
		break;
	case vni_ethtool_get_module_info:
		send_cmd_info->status = rte_ethtool_get_module_info(cmd_info->port_id,
			(struct rte_dev_ethtool_modinfo *)cmd_info->data);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_modinfo));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_modinfo);
		break;
	case vni_ethtool_get_module_eeprom:
		buf = (unsigned char *)send_cmd_info->data;
		eeprom_info = (struct rte_dev_ethtool_eeprom *)cmd_info->data;
		send_cmd_info->status = rte_ethtool_get_module_eeprom(cmd_info->port_id,
			eeprom_info, GET_PTR(send_cmd_info->data, sizeof(struct rte_dev_ethtool_eeprom),
			uint8_t));
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_eeprom));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_eeprom) +
			eeprom_info->len;
		break;
	case vni_ethtool_get_eee:
		send_cmd_info->status = rte_ethtool_get_eee(cmd_info->port_id,
			(struct rte_dev_ethtool_eee *)cmd_info->data);
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_eee));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_eee);
		break;
	case vni_ethtool_set_eee:
		send_cmd_info->status = rte_ethtool_set_eee(cmd_info->port_id,
			(struct rte_dev_ethtool_eee *)cmd_info->data);
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_tunable:
		buf = (unsigned char *)cmd_info->data;
		tun = (struct rte_dev_ethtool_tunable *)cmd_info->data;
		send_cmd_info->status = rte_ethtool_get_tunable(cmd_info->port_id,
			tun, GET_PTR(cmd_info, 0, void));
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_tunable));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_tunable) +
			tun->len;
		break;
	case vni_ethtool_set_tunable:
		send_cmd_info->status = rte_ethtool_set_tunable(cmd_info->port_id,
			(struct rte_dev_ethtool_tunable *)cmd_info->data,
			(const void *)((uint8_t *)cmd_info->data + sizeof(struct rte_dev_ethtool_tunable)));
		send_cmd_info->data_length = 0;
		break;
	case vni_ethtool_get_per_queue_coalesce:
		send_cmd_info->status = rte_ethtool_get_per_queue_coalesce(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, uint32_t, cmd_info->data_length), 
			(struct rte_dev_ethtool_coalesce *)((uint8_t *)cmd_info->data + sizeof(uint32_t)));
		COPY_DATA(send_cmd_info->data, cmd_info->data,
			sizeof(struct rte_dev_ethtool_coalesce));
		send_cmd_info->data_length = sizeof(struct rte_dev_ethtool_coalesce);
		break;
	case vni_ethtool_set_per_queue_coalesce:
		send_cmd_info->status = rte_ethtool_set_per_queue_coalesce(cmd_info->port_id,
			GET_DATA(cmd_info->data, 0, uint32_t, cmd_info->data_length),
			(struct rte_dev_ethtool_coalesce *)((uint8_t *)cmd_info->data + sizeof(uint32_t)));
		send_cmd_info->data_length = 0;
		break;
	default:
		send_cmd_info->status = -ENOTSUP;
		send_cmd_info->data_length = 0;
		break;
	}
}

