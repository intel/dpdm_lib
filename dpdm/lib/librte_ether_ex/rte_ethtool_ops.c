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
#include <rte_ethdev.h>
#include <rte_ethdev_ex.h>
#include <rte_manage.h>
#include <rte_ethtool_ops.h>
#include <vni_share_types.h>

extern int rte_eth_dev_is_valid_port(port_t);

int
rte_ethtool_get_netdev_data(port_t port_id, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;
    struct rte_eth_txq_info tx_qinfo;
    int status;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_netdev_data, -ENOTSUP);
    RTE_FUNC_PTR_OR_ERR_RET(*dev->dev_ops->txq_info_get, -ENOTSUP);

	status = (*dev_ex->dev_ethtool_ops->get_netdev_data)(dev, netdev_data);
    if (!status) {
        /* get tx queue info */
        (*dev->dev_ops->txq_info_get)(dev, 0, &tx_qinfo);
        netdev_data->nb_tx_desc = tx_qinfo.nb_desc;
    }
    return status;
}

int
rte_ethtool_set_netdev_data(port_t port_id, struct netdev_priv_data *netdev_data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_netdev_data, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_netdev_data)(dev, netdev_data);
}

int
rte_ethtool_get_settings(port_t port_id, struct rte_dev_ethtool_cmd *cmd)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);

	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_setting, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_setting)(dev, cmd);
}

int
rte_ethtool_set_settings(port_t port_id, struct rte_dev_ethtool_cmd *cmd)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_setting, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_setting)(dev, cmd);
}

int
rte_ethtool_get_drvinfo(port_t port_id, struct rte_dev_ethtool_drvinfo *info)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_drvinfo, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_drvinfo)(dev, info);
}

int
rte_ethtool_get_reg_len(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_regs_len, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_regs_len)(dev);
}

int
rte_ethtool_get_reg(port_t port_id, struct rte_dev_ethtool_reg *reginfo, void *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;

	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_regs, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_regs)(dev, reginfo, data);
}

int
rte_ethtool_get_wol(port_t port_id, struct rte_dev_ethtool_wolinfo *info)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_wol, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_wol)(dev, info);
}

int
rte_ethtool_set_wol(port_t port_id, struct rte_dev_ethtool_wolinfo *info)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_wol, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_wol)(dev, info);
}

int
rte_ethtool_get_msglevel(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_msglevel, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_msglevel)(dev);
}

int
rte_ethtool_set_msglevel(port_t port_id, uint32_t level)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_msglevel, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_msglevel)(dev, level);
}

int
rte_ethtool_nway_reset(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->nway_reset, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->nway_reset)(dev);
}

int
rte_ethtool_get_link(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_link, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_link)(dev);
}

int
rte_ethtool_get_eeprom_len(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_eeprom_len, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_eeprom_len)(dev);
}

int
rte_ethtool_get_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_eeprom, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_eeprom)(dev, info, eeprom);
}

int
rte_ethtool_set_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_eeprom, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_eeprom)(dev, info, eeprom);
}

int
rte_ethtool_get_coalesce(port_t port_id, struct rte_dev_ethtool_coalesce *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_coalesce, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_coalesce)(dev, param);
}

int
rte_ethtool_set_coalesce(port_t port_id, struct rte_dev_ethtool_coalesce *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_coalesce, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_coalesce)(dev, param);
}

int
rte_ethtool_get_ringparam(port_t port_id, struct rte_dev_ethtool_ringparam *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_ringparam, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_ringparam)(dev, param);
}

int
rte_ethtool_set_ringparam(port_t port_id, struct rte_dev_ethtool_ringparam *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_ringparam, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_ringparam)(dev, param);
}

int
rte_ethtool_get_pauseparam(port_t port_id, struct rte_dev_ethtool_pauseparam *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_pauseparam, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_pauseparam)(dev, param);
}

int
rte_ethtool_set_pauseparam(port_t port_id, struct rte_dev_ethtool_pauseparam *param)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_pauseparam, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_pauseparam)(dev, param);
}

int
rte_ethtool_self_test(port_t port_id, struct rte_dev_ethtool_test *test, uint64_t *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->self_test, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->self_test)(dev, test, ctx);
}

int
rte_ethtool_get_strings(port_t port_id, uint32_t stringset, uint8_t *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_strings, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_strings)(dev, stringset, ctx);
}

int
rte_ethtool_set_phys_id(port_t port_id, enum rte_dev_ethtool_phys_id_state state)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_phys_id, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_phys_id)(dev, state);
}

int
rte_ethtool_get_ethtool_stats(port_t port_id, struct rte_dev_ethtool_stats *stat, uint64_t *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_ethtool_stats, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_ethtool_stats)(dev, stat, ctx);
}

int
rte_ethtool_begin(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->begin, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->begin)(dev);
}

int
rte_ethtool_complete(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->complete, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->complete)(dev);
}

int
rte_ethtool_get_priv_flags(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_priv_flags, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_priv_flags)(dev);
}

int
rte_ethtool_set_priv_flags(port_t port_id, uint32_t ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_priv_flags, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_priv_flags)(dev, ctx);
}

int
rte_ethtool_get_sset_count(port_t port_id, int val)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_sset_count, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_sset_count)(dev, val);
}

int
rte_ethtool_get_rxnfc(port_t port_id, struct rte_dev_ethtool_rxnfc *rxnfc, uint32_t *rule_locs)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_rxnfc, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_rxnfc)(dev, rxnfc, rule_locs);
}

int
rte_ethtool_set_rxnfc(port_t port_id, struct rte_dev_ethtool_rxnfc *rxnfc)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_rxnfc, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_rxnfc)(dev, rxnfc);
}

int
rte_ethtool_flash_device(port_t port_id, struct rte_dev_ethtool_flash *flash)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->flash_device, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->flash_device)(dev, flash);
}

int
rte_ethtool_reset(port_t port_id, uint32_t *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->reset, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->reset)(dev, ctx);
}

int
rte_ethtool_get_rxfh_key_size(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_rxfh_key_size, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_rxfh_key_size)(dev);
}

int
rte_ethtool_get_rxfh_indir_size(port_t port_id)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_rxfh_indir_size, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_rxfh_indir_size)(dev);
}

int
rte_ethtool_get_rxfh(port_t port_id, uint32_t *indir, uint8_t *key, uint8_t *hfunc)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_rxfh, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_rxfh)(dev, indir, key, hfunc);
}

int
rte_ethtool_set_rxfh(port_t port_id, uint32_t *indir, uint8_t *key, uint8_t *hfunc)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_rxfh, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_rxfh)(dev, indir, key, hfunc);
}

int
rte_ethtool_get_channels(port_t port_id, struct rte_dev_ethtool_channels *chan)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_channels, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_channels)(dev, chan);
}

int
rte_ethtool_set_channels(port_t port_id, struct rte_dev_ethtool_channels *chan)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_channels, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_channels)(dev, chan);
}

int
rte_ethtool_get_dump_flag(port_t port_id, struct rte_dev_ethtool_dump *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_dump_flag, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_dump_flag)(dev, data);
}

int
rte_ethtool_get_dump_data(port_t port_id, struct rte_dev_ethtool_dump *data, void *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_dump_data, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_dump_data)(dev, data, ctx);
}

int
rte_ethtool_set_dump(port_t port_id, struct rte_dev_ethtool_dump *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_dump, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_dump)(dev, data);
}

int
rte_ethtool_get_ts_info(port_t port_id, struct rte_dev_ethtool_ts_info *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_ts_info, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_ts_info)(dev, data);
}

int
rte_ethtool_get_module_info(port_t port_id, struct rte_dev_ethtool_modinfo *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_module_info, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_module_info)(dev, data);
}

int
rte_ethtool_get_module_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *data, uint8_t *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_module_eeprom, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_module_eeprom)(dev, data, ctx);
}

int
rte_ethtool_get_eee(port_t port_id, struct rte_dev_ethtool_eee *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_eee, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_eee)(dev, data);
}

int
rte_ethtool_set_eee(port_t port_id, struct rte_dev_ethtool_eee *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_eee, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_eee)(dev, data);
}

int
rte_ethtool_get_tunable(port_t port_id, const struct rte_dev_ethtool_tunable *data, void *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_tunable, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_tunable)(dev, data, ctx);
}

int
rte_ethtool_set_tunable(port_t port_id, const struct rte_dev_ethtool_tunable *data, const void *ctx)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_tunable, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_tunable)(dev, data, ctx);
}

int
rte_ethtool_get_per_queue_coalesce(port_t port_id, uint32_t ctx, struct rte_dev_ethtool_coalesce *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->get_per_queue_coalesce, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->get_per_queue_coalesce)(dev, ctx, data);
}

int
rte_ethtool_set_per_queue_coalesce(port_t port_id, uint32_t ctx, struct rte_dev_ethtool_coalesce *data)
{
	struct rte_eth_dev_ex *dev_ex;
	struct rte_eth_dev *dev;

	RTE_ETH_VALID_PORTID_OR_ERR_RET(port_id, -ENODEV);
	rte_eth_get_devs_by_port(port_id, &dev_ex, &dev);
	
	if (!dev || !dev_ex || !dev_ex->dev_ethtool_ops)
		return -ENODEV;
	RTE_FUNC_PTR_OR_ERR_RET(*dev_ex->dev_ethtool_ops->set_per_queue_coalesce, -ENOTSUP);
	return (*dev_ex->dev_ethtool_ops->set_per_queue_coalesce)(dev, ctx, data);
}
