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
#ifndef _RTE_ETHTOOL_DATA_H_
#define _RTE_ETHTOOL_DATA_H_

#include <inttypes.h>
#include "ethdev_ethtool_types.h"

struct netdev_priv_data;

typedef int (*eth_ethtool_get_settings_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_cmd *cmd);
typedef int (*eth_ethtool_set_settings_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_cmd *cmd);
typedef int (*eth_ethtool_get_drvinfo_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_drvinfo *info);
typedef int (*eth_ethtool_get_regs_len_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_regs_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_reg *info, void *data);
typedef int (*eth_ethtool_get_wol_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_wolinfo *info);
typedef int (*eth_ethtool_set_wol_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_wolinfo *info);
typedef int (*eth_ethtool_get_msglevel_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_set_msglevel_t)(struct rte_eth_dev *dev, uint32_t level);
typedef int (*eth_ethtool_nway_reset_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_link_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_eeprom_len_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_eeprom_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom);
typedef int (*eth_ethtool_set_eeprom_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom);
typedef int (*eth_ethtool_get_coalesce_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_coalesce *param);
typedef int (*eth_ethtool_set_coalesce_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_coalesce *param);
typedef int (*eth_ethtool_get_ringparam_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_ringparam *param);
typedef int (*eth_ethtool_set_ringparam_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_ringparam *param);
typedef int (*eth_ethtool_get_pauseparam_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_pauseparam *param);
typedef int (*eth_ethtool_set_pauseparam_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_pauseparam *param);
typedef int (*eth_ethtool_self_test_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_test *test, uint64_t *ctx);
typedef int (*eth_ethtool_get_strings_t)(struct rte_eth_dev *dev, uint32_t stringset, uint8_t *ctx);
typedef int (*eth_ethtool_set_phys_id_t)(struct rte_eth_dev *dev, enum rte_dev_ethtool_phys_id_state state);
typedef int (*eth_ethtool_get_ethtool_stats_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_stats *stat, uint64_t *ctx);
typedef int (*eth_ethtool_begin_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_complete_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_priv_flags_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_set_priv_flags_t)(struct rte_eth_dev *dev, uint32_t ctx);
typedef int (*eth_ethtool_get_sset_count_t)(struct rte_eth_dev *dev, int val);
typedef int (*eth_ethtool_get_rxnfc_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_rxnfc *rxnfc, uint32_t *rule_locs);
typedef int (*eth_ethtool_set_rxnfc_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_rxnfc *rxnfc);
typedef int (*eth_ethtool_flash_device_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_flash *flash);
typedef int (*eth_ethtool_reset_t)(struct rte_eth_dev *dev, uint32_t *ctx);
typedef int (*eth_ethtool_get_rxfh_key_size_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_rxfh_indir_size_t)(struct rte_eth_dev *dev);
typedef int (*eth_ethtool_get_rxfh_t)(struct rte_eth_dev *dev, uint32_t *indir, uint8_t *key, uint8_t *hfunc);
typedef int (*eth_ethtool_set_rxfh_t)(struct rte_eth_dev *dev, uint32_t *indir, uint8_t *key, uint8_t *hfunc);
typedef int (*eth_ethtool_get_channels_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_channels *chan);
typedef int (*eth_ethtool_set_channels_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_channels *chan);
typedef int (*eth_ethtool_get_dump_flag_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_dump *data);
typedef int (*eth_ethtool_get_dump_data_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_dump *data, void *ctx);
typedef int (*eth_ethtool_set_dump_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_dump *data);
typedef int (*eth_ethtool_get_ts_info_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_ts_info *data);
typedef int (*eth_ethtool_get_module_info_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_modinfo *data);
typedef int (*eth_ethtool_get_module_eeprom_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_eeprom *data, uint8_t *ctx);
typedef int (*eth_ethtool_get_eee_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_eee *data);
typedef int (*eth_ethtool_set_eee_t)(struct rte_eth_dev *dev, struct rte_dev_ethtool_eee *data);
typedef int (*eth_ethtool_get_tunable_t)(struct rte_eth_dev *dev, const struct rte_dev_ethtool_tunable *data, void *ctx);
typedef int (*eth_ethtool_set_tunable_t)(struct rte_eth_dev *dev, const struct rte_dev_ethtool_tunable *data, const void *ctx);
typedef int (*eth_ethtool_get_per_queue_coalesce_t)(struct rte_eth_dev *dev, uint32_t ctx, struct rte_dev_ethtool_coalesce *data);
typedef int (*eth_ethtool_set_per_queue_coalesce_t)(struct rte_eth_dev *dev, uint32_t ctx, struct rte_dev_ethtool_coalesce *data);
typedef int (*eth_ethtool_get_netdev_data_t)(struct rte_eth_dev *dev, struct netdev_priv_data *data);
typedef int (*eth_ethtool_set_netdev_data_t)(struct rte_eth_dev *dev, struct netdev_priv_data *data);

struct eth_dev_ethtool_ops {
	eth_ethtool_get_settings_t		get_setting;
	eth_ethtool_set_settings_t		set_setting;
	eth_ethtool_get_drvinfo_t		get_drvinfo;
	eth_ethtool_get_regs_len_t		get_regs_len;
	eth_ethtool_get_regs_t			get_regs;
	eth_ethtool_get_wol_t			get_wol;
	eth_ethtool_set_wol_t			set_wol;
	eth_ethtool_get_msglevel_t		get_msglevel;
	eth_ethtool_set_msglevel_t		set_msglevel;
	eth_ethtool_nway_reset_t		nway_reset;
	eth_ethtool_get_link_t			get_link;
	eth_ethtool_get_eeprom_len_t	get_eeprom_len;
	eth_ethtool_get_eeprom_t		get_eeprom;
	eth_ethtool_set_eeprom_t		set_eeprom;
	eth_ethtool_get_coalesce_t		get_coalesce;
	eth_ethtool_set_coalesce_t		set_coalesce;
	eth_ethtool_get_ringparam_t		get_ringparam;
	eth_ethtool_set_ringparam_t		set_ringparam;
	eth_ethtool_get_pauseparam_t	get_pauseparam;
	eth_ethtool_set_pauseparam_t	set_pauseparam;
	eth_ethtool_self_test_t			self_test;
	eth_ethtool_get_strings_t		get_strings;
	eth_ethtool_set_phys_id_t		set_phys_id;
	eth_ethtool_get_ethtool_stats_t	get_ethtool_stats;
	eth_ethtool_begin_t				begin;
	eth_ethtool_complete_t			complete;
	eth_ethtool_get_priv_flags_t	get_priv_flags;
	eth_ethtool_set_priv_flags_t	set_priv_flags;
	eth_ethtool_get_sset_count_t	get_sset_count;
	eth_ethtool_get_rxnfc_t			get_rxnfc;
	eth_ethtool_set_rxnfc_t			set_rxnfc;
	eth_ethtool_flash_device_t		flash_device;
	eth_ethtool_reset_t				reset;
	eth_ethtool_get_rxfh_key_size_t	get_rxfh_key_size;
	eth_ethtool_get_rxfh_indir_size_t	get_rxfh_indir_size;
	eth_ethtool_get_rxfh_t			get_rxfh;
	eth_ethtool_set_rxfh_t			set_rxfh;
	eth_ethtool_get_channels_t		get_channels;
	eth_ethtool_set_channels_t		set_channels;
	eth_ethtool_get_dump_flag_t		get_dump_flag;
	eth_ethtool_get_dump_data_t		get_dump_data;
	eth_ethtool_set_dump_t			set_dump;
	eth_ethtool_get_ts_info_t		get_ts_info;
	eth_ethtool_get_module_info_t	get_module_info;
	eth_ethtool_get_module_eeprom_t	get_module_eeprom;
	eth_ethtool_get_eee_t			get_eee;
	eth_ethtool_set_eee_t			set_eee;
	eth_ethtool_get_tunable_t		get_tunable;
	eth_ethtool_set_tunable_t		set_tunable;
	eth_ethtool_get_per_queue_coalesce_t	get_per_queue_coalesce;
	eth_ethtool_set_per_queue_coalesce_t	set_per_queue_coalesce;

	eth_ethtool_get_netdev_data_t get_netdev_data;
	eth_ethtool_set_netdev_data_t set_netdev_data;
	unsigned char capability[16];
};

#endif /* _RTE_ETHTOOL_DATA_H_ */
