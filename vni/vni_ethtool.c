/*_
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
 *   Foundation, Inc., 51 Franklin St _ Fifth Floor, Boston, MA 02110_1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */
#include <linux/ethtool.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include "vni.h"

static int vni_set_setting(struct net_device *dev, struct ethtool_cmd *cmd)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_setting, cmd,
		sizeof(struct ethtool_cmd));
}

static int vni_get_setting(struct net_device *dev, struct ethtool_cmd *cmd)
{
	int status = k2u_link_1var(dev, vni_ethtool_get_setting, cmd,
		sizeof(struct ethtool_cmd));
	{
		vni_log("\ncmd=%x supported=%d advertising=%d speed=%d\n",
			cmd->cmd, cmd->supported, cmd->advertising, cmd->speed);
		vni_log("duplex=%d port=%d phy_address=%d transceiver=%d\n",
			cmd->duplex, cmd->port, cmd->phy_address, cmd->transceiver);
		vni_log("autoneg=%d mdio_support=%d maxtxpkt=%d maxrxpkt=%d\n",
			cmd->autoneg, cmd->mdio_support, cmd->maxtxpkt, cmd->maxrxpkt);
		vni_log("speed_hi=%d eth_tp_mdix=%d eth_tp_mdix_ctrl=%d\n",
			cmd->speed_hi, cmd->eth_tp_mdix, cmd->eth_tp_mdix_ctrl);
		vni_log("lp_advertising=%d\n", cmd->lp_advertising);
	}
	return status;
}

static void vni_get_drvinfo(struct net_device *dev,
	struct ethtool_drvinfo *drvinfo)
{
	int status = k2u_link_1var(dev, vni_ethtool_get_drvinfo, drvinfo,
		sizeof(struct ethtool_drvinfo));

	if (status < 0)
		vni_elog("vni: get_drvinfo (inf: %s)failed with error"
		" code: %d\n", dev->name, status);
	printk(KERN_INFO "vni: vni_get_drvinfo: driver name=%s\n", drvinfo->driver);
	return;
}

static int vni_get_reg_len(struct net_device *dev)
{
	return k2u_link(dev, vni_ethtool_get_reg_len);
}

static void vni_get_reg(struct net_device *dev, struct ethtool_regs *reg_info,
						void *regs)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_get_reg, reg_info, sizeof(struct ethtool_regs));
	if(!u2k_cmd_info)
		return;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: get_regs (inf: %s)failed with error"
		" code: %d\n", dev->name, u2k_cmd_info->status);
	} else {
		memcpy(reg_info, u2k_cmd_info->data, sizeof(struct ethtool_regs));
		memcpy(regs, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_regs), reg_info->len);
	}
	return;
}

static void vni_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	int status =  k2u_link_1var(dev, vni_ethtool_get_wol, wol, sizeof(struct ethtool_wolinfo));

	if (status < 0)
		vni_elog("vni: get_wol (inf: %s)failed with error code: %d\n",
		dev->name, status);
	return;
}

static int vni_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_wol, wol,
		sizeof(struct ethtool_wolinfo));
}

static u32 vni_get_msglevel(struct net_device *dev)
{
	return k2u_link(dev, vni_ethtool_get_msglevel);
}

static void vni_set_msglevel(struct net_device *dev, u32 level)
{
	int status = k2u_link_1var_noupdate(dev, vni_ethtool_set_msglevel,
		&level, sizeof(u32));

	if (status < 0)
		vni_elog("vni: set_msglevel (inf: %s)failed with error code: %d\n",
		dev->name, status);
	return;
}

static int vni_nway_reset(struct net_device *dev)
{
	return k2u_link(dev, vni_ethtool_nway_reset);
}
	
static u32 vni_get_link(struct net_device *dev)
{
	return (u32)k2u_link(dev, vni_ethtool_get_link);
}

static int vni_get_eeprom_len(struct net_device *dev)
{
	return k2u_link(dev, vni_ethtool_get_eeprom_len);
}

static int vni_get_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom_info,
						  u8 *data)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_get_eeprom, eeprom_info, sizeof(struct ethtool_eeprom));
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: get_eeprom (inf: %s)failed with error"
		" code: %d\n", dev->name, u2k_cmd_info->status);
	} else {
		memcpy(eeprom_info, u2k_cmd_info->data, sizeof(struct ethtool_eeprom));
		memcpy(data, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_eeprom),
		eeprom_info->len);
	}

	return u2k_cmd_info->status;
}

static int vni_set_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom_info,
						  u8 *data)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_set_eeprom,
		sizeof(struct ethtool_eeprom) + eeprom_info->len);
		
	if(!k2u_cmd_info)
		return -1;

	memcpy(k2u_cmd_info->data, eeprom_info, sizeof(struct ethtool_eeprom));
	memcpy((u8 *)k2u_cmd_info->data+sizeof(struct ethtool_eeprom), data,
		eeprom_info->len);

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
static int vni_get_coalesce(struct net_device *dev, struct ethtool_coalesce *coalesce_info)
{
	return k2u_link_1var(dev, vni_ethtool_get_coalesce, coalesce_info,
		sizeof(struct ethtool_coalesce));
}

static int vni_set_coalesce(struct net_device *dev, struct ethtool_coalesce *coalesce_info)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_coalesce, coalesce_info,
		sizeof(struct ethtool_coalesce));
}
	
static void vni_get_ringparam(struct net_device *dev, struct ethtool_ringparam *ring_info)
{
	int status = k2u_link_1var(dev, vni_ethtool_get_ringparam, ring_info,
		sizeof(struct ethtool_ringparam));
	if (status < 0)
		vni_elog("vni: get_ringparam (inf: %s)failed with error code: %d\n",
		dev->name, status);
	return;
}
	
static int vni_set_ringparam(struct net_device *dev, struct ethtool_ringparam *ring_info)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_ringparam, ring_info,
		sizeof(struct ethtool_ringparam));
}
	
static void vni_get_pauseparam(struct net_device *dev, struct ethtool_pauseparam *pause_info)
{
	int status = k2u_link_1var(dev, vni_ethtool_get_pauseparam, pause_info,
		sizeof(struct ethtool_pauseparam));
	if (status < 0) {
		vni_elog("vni: get_pauseparam (inf: %s)failed with error code: %d\n",
		dev->name, status);
	}
	else 
	/* debug */
		vni_dlog("vni: received pause frame info: autoneg: %x, rx_pause: %x tx_pause: %x\n",
			pause_info->autoneg, pause_info->rx_pause, pause_info->tx_pause);
	return;
}
	
static int vni_set_pauseparam(struct net_device *dev, struct ethtool_pauseparam *ring_info)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_pauseparam, ring_info,
		sizeof(struct ethtool_pauseparam));
}
	
static void vni_self_test(struct net_device *dev, struct ethtool_test *test_info, u64 *ctx)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_self_test, test_info, sizeof(struct ethtool_test));

	memcpy(ctx, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_test), sizeof(u64));

	if (u2k_cmd_info->status < 0)
		vni_elog("vni: self_test (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	return;
}

static void vni_get_strings(struct net_device *dev, u32 stringset, u8 *strings)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev, vni_ethtool_get_strings,
											&stringset, sizeof(u32));
	if(!u2k_cmd_info)
		return;
	memcpy(strings, u2k_cmd_info->data, u2k_cmd_info->data_length);

	if (u2k_cmd_info->status < 0)
		vni_elog("vni: get_strings (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);

	return;
}
	
static int vni_set_phys_id(struct net_device *dev, enum ethtool_phys_id_state state)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_phys_id, &state,
		sizeof(enum ethtool_phys_id_state));
}
	
static void vni_get_stats(struct net_device *dev, struct ethtool_stats *stats, u64 *ctx)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev, vni_ethtool_get_stats,
											&stats, sizeof(struct ethtool_stats));
	if(!u2k_cmd_info)
		return;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: get_stats (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else
		memcpy(ctx, u2k_cmd_info->data, u2k_cmd_info->data_length);

	return;
}

static int vni_begin(struct net_device *dev)
{
	struct netdev_priv_data remote_netdev_data;
	struct netdev_priv_data *netdev_data = netdev_priv(dev);
	int status = k2u_link_1var(dev, vni_ethtool_begin, &remote_netdev_data,
		sizeof(struct netdev_priv_data));
	memcpy(netdev_data, &remote_netdev_data, sizeof(struct netdev_priv_data));
	set_netdevice(dev, netdev_data);
	vni_log("begin: features:%llx hw_features:%llx\n",
		dev->features, dev->hw_features);	
	return status;
}

static void vni_complete(struct net_device *dev)
{
	struct netdev_priv_data *netdev_data = netdev_priv(dev);
	int status;
	
	get_netdevice(dev, netdev_data);
	vni_log("complete: features:%llx dw_features=%llx\n",
		netdev_data->features, netdev_data->hw_features);
	status = k2u_link_1var_noupdate(dev, vni_ethtool_complete, netdev_data,
		sizeof(struct netdev_priv_data));

	if (status < 0)
		vni_elog("vni: complete (inf: %s)failed with error code: %d\n",
		dev->name, status);

	return;
}

static u32 vni_get_priv_flags(struct net_device *dev)
{
	return (u32)k2u_link(dev, vni_ethtool_get_priv_flags);
}
	
static int vni_set_priv_flags(struct net_device *dev, u32 flags)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_priv_flags,
								  &flags, sizeof(u32));
}
	
static int vni_get_sset_count(struct net_device *dev, int sset)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_get_sset_count,
								  &sset, sizeof(int));
}
	
static int vni_get_rxnfc(struct net_device *dev, struct ethtool_rxnfc *cmd, u32 *rule_locs)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_get_rxnfc,
		sizeof(struct ethtool_rxnfc) + cmd->rule_cnt *sizeof(u32));
		
	if(!k2u_cmd_info)
		return -1;
	memcpy(k2u_cmd_info->data, cmd, sizeof(struct ethtool_rxnfc));
	memcpy((u8 *)k2u_cmd_info->data+sizeof(struct ethtool_rxnfc), rule_locs,
		cmd->rule_cnt *sizeof(u32));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
	
static int vni_set_rxnfc(struct net_device *dev, struct ethtool_rxnfc *cmd)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_rxnfc,
								  cmd, sizeof(struct ethtool_rxnfc));
}
	
static int vni_flash_device(struct net_device *dev, struct ethtool_flash *flash)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_flash_device,
								  flash, sizeof(struct ethtool_flash));
}
	
static int vni_reset(struct net_device *dev, u32 *mask)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_reset,
								  mask, sizeof(u32));
}

#ifdef RXFH_ALL	
static u32 vni_get_rxfh_key_size(struct net_device *dev)
{
	return (u32)k2u_link(dev, vni_ethtool_get_rxfh_key_size);
}
	
static u32 vni_get_rxfh_indir_size(struct net_device *dev)
{
	return (u32)k2u_link(dev, vni_ethtool_get_rxfh_indir_size);
}
	
static int vni_get_rxfh(struct net_device *dev, u32 *indir,
						u8 *key, u8* hfunc)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_get_rxfh,
		sizeof(indir) + sizeof(key));
		
	if(!k2u_cmd_info)
		return -1;
	memcpy(k2u_cmd_info->data, indir, sizeof(indir));
	memcpy((u8 *)k2u_cmd_info->data+sizeof(indir), key,
		sizeof(key));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0)
		vni_elog("vni: get_get_rxfh (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	else
		memcpy(hfunc, u2k_cmd_info->data, u2k_cmd_info->data_length);

	return u2k_cmd_info->status;
}
	
static int vni_set_rxfh(struct net_device *dev, u32 *indir,
						u8 *key, u8* hfunc)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_set_rxfh,
		sizeof(indir) + sizeof(key) + sizeof(hfunc));
		
	if(!k2u_cmd_info)
		return -1;
	memcpy(k2u_cmd_info->data, indir, sizeof(indir));
	memcpy((u8 *)k2u_cmd_info->data+sizeof(indir), key,
		sizeof(key));
	memcpy((u8 *)k2u_cmd_info->data+sizeof(indir)+sizeof(key), hfunc,
		sizeof(hfunc));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#endif	

static void vni_get_channels(struct net_device *dev, struct ethtool_channels *chan)
{
	int status = k2u_link_1var(dev, vni_ethtool_get_channels,
								  chan, sizeof(struct ethtool_channels));
	if (status < 0)
		vni_elog("vni: get_get_channels (inf: %s)failed with error code: %d\n",
		dev->name, status);

	return;
}
	
static int vni_set_channels(struct net_device *dev, struct ethtool_channels *chan)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_channels,
								  chan, sizeof(struct ethtool_channels));
}
	
static int vni_get_dump_flag(struct net_device *dev,
	struct ethtool_dump *dump)
{
	return k2u_link_1var(dev, vni_ethtool_get_dump_flag,
								  dump, sizeof(struct ethtool_dump));
}
	
static int vni_get_dump_data(struct net_device *dev,
	struct ethtool_dump *dump, void *data)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_get_dump_data, dump, sizeof(struct ethtool_dump)+dump->len);
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: get_get_dump_data (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else
		memcpy(data, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_dump),
		dump->len);

	return u2k_cmd_info->status;
}
	
static int vni_set_dump(struct net_device *dev,  struct ethtool_dump *dump)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_dump,
								  dump, sizeof(struct ethtool_dump));
}
	
static int vni_get_ts_info(struct net_device *dev, struct ethtool_ts_info *ts_info)
{
	return k2u_link_1var(dev, vni_ethtool_get_ts_info,
								  ts_info, sizeof(struct ethtool_ts_info));
}
	
static int vni_get_module_info(struct net_device *dev,
	struct ethtool_modinfo *mod_info)
{
	return k2u_link_1var(dev, vni_ethtool_get_module_info,
								  mod_info, sizeof(struct ethtool_modinfo));
}

static int vni_get_module_eeprom(struct net_device *dev,
	struct ethtool_eeprom *eeprom_info, u8 *ctx)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_get_module_eeprom, eeprom_info,
		sizeof(struct ethtool_eeprom));
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0) {
		vni_elog("vni: get_module_eeprom (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	} else
		memcpy(ctx, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_eeprom), eeprom_info->len);

	return u2k_cmd_info->status;
}
	
static int vni_get_eee(struct net_device *dev, struct ethtool_eee *data)
{
	return k2u_link_1var(dev, vni_ethtool_get_eee,
								  data, sizeof(struct ethtool_eee));
}
	
static int vni_set_eee(struct net_device *dev, struct ethtool_eee *data)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_eee,
								  data, sizeof(struct ethtool_eee));
}

#ifdef TUNABLE	
static int vni_get_tunable(struct net_device *dev,
	const struct ethtool_tunable *data, void *ctx)
{
	netdev_cmd_info *u2k_cmd_info = k2u_link_1var_other(dev,
		vni_ethtool_get_tunable, data, sizeof(struct ethtool_tunable));
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0)
		vni_elog("vni: get_tunable (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	else
		memcpy(ctx, (u8 *)u2k_cmd_info->data + sizeof(struct ethtool_tunable), data->len);

	return u2k_cmd_info->status;
}
	
static int vni_set_tunable(struct net_device *dev,
	const struct ethtool_tunable *data, const void *ctx)
{
	return k2u_link_1var_noupdate(dev, vni_ethtool_set_tunable,
								  data, sizeof(struct ethtool_tunable));
}
#endif

#ifdef PER_QUEU_COALESCE
static int vni_get_per_queue_coalesce(struct net_device *dev, u32 queue,
	struct ethtool_coalesce *ec)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_get_per_queue_coalesce,
		sizeof(u32) + sizeof(struct ethtool_coalesce));
		
	if(!k2u_cmd_info)
		return -1;
	memcpy((u8*)k2u_cmd_info->data, &queue, sizeof(u32));
	memcpy((u8*)k2u_cmd_info->data+sizeof(u32), ec, sizeof(struct ethtool_coalesce));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	if (u2k_cmd_info->status < 0)
		vni_elog("vni: get_per_queue_coalesce (inf: %s)failed with error code: %d\n",
		dev->name, u2k_cmd_info->status);
	else
		memcpy(ec, (u8 *)u2k_cmd_info->data, u2k_cmd_info->data_length);

	return u2k_cmd_info->status;
}
	
static int vni_set_per_queue_coalesce(struct net_device *dev, u32 queue,
	struct ethtool_coalesce *ec)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	
	k2u_cmd_info = k2u_downlink(dev, vni_ethtool_set_per_queue_coalesce,
		sizeof(u32) + sizeof(struct ethtool_coalesce));
		
	if(!k2u_cmd_info)
		return -1;
	memcpy((u8*)k2u_cmd_info->data, &queue, sizeof(u32));
	memcpy((u8*)k2u_cmd_info->data+sizeof(u32), ec, sizeof(struct ethtool_coalesce));

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info)
		return -1;

	return u2k_cmd_info->status;
}
#endif

static struct ethtool_ops vni_ethtool_ops =
{
	.set_settings = vni_set_setting,
	.get_settings = vni_get_setting,
	.get_drvinfo = vni_get_drvinfo,
	.get_regs_len = vni_get_reg_len,
	.get_regs = vni_get_reg,
	.get_wol = vni_get_wol,
	.set_wol = vni_set_wol,
	.get_msglevel = vni_get_msglevel,
	.set_msglevel = vni_set_msglevel,
	.nway_reset = vni_nway_reset,
	.get_link = vni_get_link,
	.get_eeprom_len = vni_get_eeprom_len,
	.get_eeprom = vni_get_eeprom,
	.set_eeprom = vni_set_eeprom,
	.get_coalesce = vni_get_coalesce,
	.set_coalesce = vni_set_coalesce,
	.get_ringparam = vni_get_ringparam,
	.set_ringparam = vni_set_ringparam,
	.get_pauseparam = vni_get_pauseparam,
	.set_pauseparam = vni_set_pauseparam,
	.self_test = vni_self_test,
	.get_strings = vni_get_strings,
	.set_phys_id = vni_set_phys_id,
	.get_ethtool_stats = vni_get_stats,
	.begin = vni_begin,
	.complete = vni_complete,
	.get_priv_flags = vni_get_priv_flags,
	.set_priv_flags = vni_set_priv_flags,
	.get_sset_count = vni_get_sset_count,
	.get_rxnfc = vni_get_rxnfc,
	.set_rxnfc = vni_set_rxnfc,
	.flash_device = vni_flash_device,
	.reset = vni_reset,
#ifdef RXFH_ALL
	.get_rxfh_key_size = vni_get_rxfh_key_size,
	.get_rxfh_indir_size = vni_get_rxfh_indir_size,

	.get_rxfh = vni_get_rxfh,
	.set_rxfh = vni_set_rxfh,
#endif
	.get_channels = vni_get_channels,
	.set_channels = vni_set_channels,
	.get_dump_flag = vni_get_dump_flag,
	.get_dump_data = vni_get_dump_data,
	.set_dump = vni_set_dump,
	.get_ts_info = vni_get_ts_info,
	.get_module_info = vni_get_module_info,
	.get_module_eeprom = vni_get_module_eeprom,
	.get_eee = vni_get_eee,
	.set_eee = vni_set_eee,
#ifdef TUNABLE
	.get_tunable = vni_get_tunable,
	.set_tunable = vni_set_tunable,
#endif
#ifdef PER_QUEU_COALESCE
	.get_per_queue_coalesce = vni_get_per_queue_coalesce,
	.set_per_queue_coalesce = vni_set_per_queue_coalesce,
#endif
};

void
dev_add_ethtool_ops(struct net_device *dev)
{
	dev->ethtool_ops = &vni_ethtool_ops;
}


