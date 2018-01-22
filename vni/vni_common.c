/*-
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
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include "vni.h"
#include "vni_share_types.h"

#include "vni_netdev_flags.h"

#define LINK_STATE_START_MASK (1 << __LINK_STATE_START)

#define rte_feature(netdev, rte, feature) \
	if (netdev & feature) \
		rte |= RTE_##feature

#define netdev_feature(netdev, rte, feature) \
	if (rte & RTE_##feature) \
		netdev |= feature

int get_stats64_enable;
struct netdev_table netdev_tbl[MAXI_MANAGEMENT_DEV];
static int defer_netdev_client = INVALID_NETDEV_INDEX;
struct netdev_cmd_info pending_inf_cmd;
struct mutex vni_netlink_mutex, vni_netdev_mutex;
static vni_msg_status vni_msg_state;
static void netdev_setup(struct net_device *dev);
static void connect_netdev_op(struct net_device *dev,
	struct netdev_priv_data *priv_data);

int is_inf_closing(struct net_device *net)
{
	int i;
	struct net_device *exiting_net;
	int num_of_if;
	
	if (defer_netdev_client == INVALID_NETDEV_INDEX)
		return 0;

	if ((num_of_if = netdev_tbl[defer_netdev_client].num_of_if)){
		for(i=0; i < num_of_if; i++){
			exiting_net = netdev_tbl[defer_netdev_client].dev_ptr_table[i];
			if (strcmp(exiting_net->name, net->name) == 0)
				return 1;
		}
	}
	return 0;
}

void vni_init_netdev_tbl(void)
{
	int i;

	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++)
		netdev_tbl[i].num_of_if = 0;
}

int get_netdev_count(void)
{
	int i, count = 0;

	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++)
		if (netdev_tbl[i].num_of_if)
			count ++;

	return count;
}

int get_get_stats64_enable(void)
{
	return get_stats64_enable;
}

void set_get_stats64_enable(int enable)
{
	get_stats64_enable = enable;
}

const char *cmd_name(netdev_cmd_type cmd)
{
	const char* invalid_str = "invalid netdev cmd";
	const char* manage_str[] = {
		"vni_manage",
		"vni_manage_clean_inf",
		"vni_manage_add_inf",
		"vni_manage_del_inf",
		"vni_manage_ack_kernel",
		"vni_manage_ack_us",
		"vni_manage_test"
	};

	const char* netdev_str[] = {
		"vni_netdev_init",
		"vni_netdev_uninit",
		"vni_netdev_open",
		"vni_netdev_stop",
		"vni_netdev_start_xmit",
		"vni_netdev_change_rx_flags",
		"vni_netdev_set_rx_mode",
		"vni_netdev_set_mac_addr",
		"vni_netdev_validate_addr",
		"vni_netdev_do_ioctl",
		"vni_netdev_change_mtu",
		"vni_netdev_tx_timeout",
		"vni_netdev_get_stats64",
		"vni_netdev_get_stats",
		"vni_netdev_vlan_rx_add_vid",
		"vni_netdev_vlan_rx_kill_vid",
		"vni_netdev_set_vf_mac",
		"vni_netdev_set_vf_vlan",
		"vni_netdev_set_vf_rate",
		"vni_netdev_set_vf_spoofchk",
		"vni_netdev_get_vf_config",
        "vni_netdev_set_vf_link_state",
        "vin_netdev_get_vf_stat",
		"vni_netdev_set_vf_trust",
		"vni_netdev_fix_features",
		"vni_netdev_set_features"
	};

	const char* ethtool_str[] = {
		"vni_ethtool_set_setting",
		"vni_ethtool_get_setting",
		"vni_ethtool_get_drvinfo",
		"vni_ethtool_get_reg_len",
		"vni_ethtool_get_reg",
		"vni_ethtool_get_wol",
		"vni_ethtool_set_wol",
		"vni_ethtool_get_msglevel",
		"vni_ethtool_set_msglevel",
		"vni_ethtool_nway_reset",
		"vni_ethtool_get_link",
		"vni_ethtool_get_eeprom_len",
		"vni_ethtool_get_eeprom",
		"vni_ethtool_set_eeprom",
		"vni_ethtool_get_coalesce",
		"vni_ethtool_set_coalesce",
		"vni_ethtool_get_ringparam",
		"vni_ethtool_set_ringparam",
		"vni_ethtool_get_pauseparam",
		"vni_ethtool_set_pauseparam",
		"vni_ethtool_self_test",
		"vni_ethtool_get_strings",
		"vni_ethtool_set_phys_id",
		"vni_ethtool_get_stats",
		"vni_ethtool_begin",
		"vni_ethtool_complete",
		"vni_ethtool_get_priv_flags",
		"vni_ethtool_set_priv_flags",
		"vni_ethtool_get_sset_count",
		"vni_ethtool_get_rxnfc",
		"vni_ethtool_set_rxnfc",
		"vni_ethtool_flash_device",
		"vni_ethtool_reset",
		"vni_ethtool_get_rxfh_key_size",
		"vni_ethtool_get_rxfh_indir_size",
		"vni_ethtool_get_rxfh",
		"vni_ethtool_set_rxfh",
		"vni_ethtool_get_channels",
		"vni_ethtool_set_channels",
		"vni_ethtool_get_dump_flag",
		"vni_ethtool_get_dump_data",
		"vni_ethtool_set_dump",
		"vni_ethtool_get_ts_info",
		"vni_ethtool_get_module_info",
		"vni_ethtool_get_module_eeprom",
		"vni_ethtool_get_eee",
		"vni_ethtool_set_eee",
		"vni_ethtool_get_tunable",
		"vni_ethtool_set_tunable",
		"vni_ethtool_get_per_queue_coalesce",
		"vni_ethtool_set_per_queue_coalesce"
	};

	if (is_vni_manage_cmd(cmd))
		return manage_str[cmd];
	else {
		if (is_vni_netdev_cmd(cmd))
			return netdev_str[cmd - vni_netdev_init];
		else {
			if (is_vni_ethtool_cmd(cmd))
				return ethtool_str[cmd - vni_ethtool_set_setting];
			else
				return invalid_str;
		}
	}
	return invalid_str;
}

int is_vni_manage_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_manage) &&
		(cmd <= vni_manage_test))
		return 1;
	return 0;
}

int is_vni_netdev_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_netdev_init) &&
		(cmd <= vni_netdev_set_features))
		return 1;
	return 0;
}

int is_vni_ethtool_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_ethtool_set_setting) &&
		(cmd <= vni_ethtool_set_per_queue_coalesce))
		return 1;
	return 0;
}

void set_msg_state(vni_msg_status new_state)
{
	vni_msg_state = new_state;
}

inline vni_msg_status get_msg_state(void)
{
	return vni_msg_state;
}

char *mac_addr_string(void *in_mac_addr)
{
	static unsigned char addr_string[MAX_ADDR_LEN*3];
	int i, char_ind = 0;
	unsigned char *mac_addr = in_mac_addr;

	snprintf(&addr_string[char_ind], MAX_ADDR_LEN*3 - char_ind, "%2x",
		mac_addr[0]);
	char_ind += 2;
	for(i = 1; i < MAX_ADDR_LEN; i++, char_ind += 3)
		snprintf(&addr_string[char_ind], MAX_ADDR_LEN*3 - char_ind, ":%2x",
		mac_addr[i]);

	return addr_string;
}

void vni_lock(void)
{
	mutex_lock(&vni_netlink_mutex);
}

void vni_unlock(void)
{
	mutex_unlock(&vni_netlink_mutex);
}

void vni_trylock(void)
{
	mutex_trylock(&vni_netlink_mutex);
}

void vni_mutex_init(void)
{
	mutex_init(&vni_netlink_mutex);
	mutex_init(&vni_netdev_mutex);
}

int vni_is_locked(void)
{
	return mutex_is_locked(&vni_netlink_mutex);
}

void vni_release_lock(void)
{
	if (vni_is_locked())
		vni_unlock();
}

void clean_pending_inf_cmd(void)
{
	int i;
	struct net_device *net;

	if (defer_netdev_client != INVALID_NETDEV_INDEX) {
		for( i = 0; i < netdev_tbl[defer_netdev_client].num_of_if; i++) {
			net = netdev_tbl[defer_netdev_client].dev_ptr_table[i];
			net->dev.parent = 0; /* disconnect with proxy driver */
			unregister_netdev(net);
			vni_log("Unregister netdev %s\n", net->name);
			free_netdev(net);
			vni_log("Free un-registered netdev\n");
		}
		netdev_tbl[defer_netdev_client].num_of_if = 0;
	}

	defer_netdev_client = INVALID_NETDEV_INDEX;
	if ((pending_inf_cmd.cmd != vni_manage_del_inf) &&
		(pending_inf_cmd.cmd != vni_invalid))
		vni_log("Unexpected pending cmd(%s)\n", cmd_name(pending_inf_cmd.cmd));
	else
		pending_inf_cmd.cmd = vni_invalid;
}

/* helper routine for send/recv nl packets to/from user-space */
netdev_cmd_info *k2u_downlink(struct net_device *dev, netdev_cmd_type cmd,
									 size_t data_size)
{
	netdev_cmd_info *k2u_cmd_info;
	k2u_cmd_info = new_netlink_skbbuf(sizeof(netdev_cmd_info)+data_size);
	if(k2u_cmd_info == NULL) {
		vni_release_lock();
		return NULL;
	}

	k2u_cmd_info->app_pid = get_info_from_inf(dev->name, &k2u_cmd_info->port_id);
	if (k2u_cmd_info->app_pid == 0) {
		vni_elog("Could not find the interface name"
			" from register inface table with cmd(%s)\n", cmd_name(cmd));
		vni_release_lock();
		return NULL;
	}
	k2u_cmd_info->cmd = cmd;
	k2u_cmd_info->data_length = data_size;

	return k2u_cmd_info;
}

netdev_cmd_info *k2u_uplink(struct net_device *dev, netdev_cmd_info *k2u_cmd_info)
{
	int status;

	status = vni_nl_xmit_msg(k2u_cmd_info);
	if (status < 0) {
		vni_elog("Fail to send cmd(%s) from kernel to user-space"
		"(pid=%d) fails from device %s\n", cmd_name(k2u_cmd_info->cmd),
		k2u_cmd_info->app_pid, dev->name);
		vni_release_lock();
		return NULL;
	}

	return get_u2k_netdev_cmd();
}

int k2u_link(struct net_device *dev, netdev_cmd_type cmd)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;

	k2u_cmd_info = k2u_downlink(dev, cmd, 0);
	if(!k2u_cmd_info) {
		return -1;
	}

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if (!u2k_cmd_info) {
		return -1;
	}

	return u2k_cmd_info->status;
}

netdev_cmd_info *k2u_link_0var(struct net_device *dev, netdev_cmd_type cmd)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;

	k2u_cmd_info = k2u_downlink(dev, cmd, 0);
	if(!k2u_cmd_info) {
		return NULL;
	}

	return k2u_uplink(dev, k2u_cmd_info);
}

netdev_cmd_info *k2u_link_1var_other(struct net_device *dev,
                                     netdev_cmd_type cmd,
                                     void *var,
                                     size_t data_size)
{
	netdev_cmd_info *k2u_cmd_info;

	k2u_cmd_info = k2u_downlink(dev, cmd, data_size);
	if(!k2u_cmd_info) {
		return NULL;
	}

	memcpy(k2u_cmd_info->data, var, data_size);
	
	return k2u_uplink(dev, k2u_cmd_info);
}

int k2u_link_1var_noupdate(struct net_device *dev, netdev_cmd_type cmd,
								  void *var, size_t data_size)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;
	k2u_cmd_info = k2u_downlink(dev, cmd, data_size);
	if(!k2u_cmd_info) {
		return -1;
	}

	memcpy(k2u_cmd_info->data, var, data_size);

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info) {
		return -1;
	}

	return u2k_cmd_info->status;
}

int k2u_link_1var(struct net_device *dev, netdev_cmd_type cmd,
						 void *var, size_t data_size)
{
	netdev_cmd_info *k2u_cmd_info, *u2k_cmd_info;

	k2u_cmd_info = k2u_downlink(dev, cmd, 0);
	if(!k2u_cmd_info) {
		return -1;
	}

	memcpy(k2u_cmd_info->data, var, data_size);

	u2k_cmd_info = k2u_uplink(dev, k2u_cmd_info);
	if(!u2k_cmd_info) {
		return -1;
	}

	if(u2k_cmd_info->status < 0){
		vni_elog("Uplink cmd(%s) (inf: %s)failed with error code: %d\n",
		cmd_name(u2k_cmd_info->cmd), dev->name, u2k_cmd_info->status);
	} else 
		memcpy(var, u2k_cmd_info->data, data_size);

	return u2k_cmd_info->status;
}

void get_netdevice(struct net_device *dev,
	struct netdev_priv_data *netdev_data)
{
	netdev_data->features = dev->features;
	netdev_data->hw_features = dev->hw_features;
	netdev_data->addr_len = dev->addr_len;
	netdev_data->mtu = dev->mtu;
    netdev_data->flags = dev->flags;
    if (dev->state & LINK_STATE_START_MASK)
        netdev_data->link = 1;
    else
        netdev_data->link = 0;
	memcpy(netdev_data->perm_addr, dev->perm_addr, netdev_data->addr_len);
	memcpy(netdev_data->dev_addr, dev->dev_addr, netdev_data->addr_len);
	if(!memcmp(dev->dev_addr, dev->perm_addr, netdev_data->addr_len))
		vni_log("Interface address change\n");
}

void set_netdevice(struct net_device *dev,
	struct netdev_priv_data *netdev_data)
{
    dev->features = netdev_data->features;
    dev->hw_features = netdev_data->hw_features;
    dev->addr_len = netdev_data->addr_len;
    dev->mtu = netdev_data->mtu;
    dev->type = netdev_data->type;
    dev->flags = netdev_data->flags;
    if (netdev_data->link) {
        dev->state |= LINK_STATE_START_MASK;
        dev->operstate = IF_OPER_UP;
    } else {
        dev->state &= ~LINK_STATE_START_MASK;
        dev->operstate = IF_OPER_DOWN;
    }

    memcpy(dev->perm_addr, netdev_data->perm_addr, netdev_data->addr_len);
}


/* end of helper routine */

static void netdev_setup(struct net_device *dev)
{
	dev_add_dummy_netdev(dev);
	vni_log("Defer netdev op registration!\n");
}

static void connect_netdev_op(struct net_device *dev,
	struct netdev_priv_data *priv_data)
{
	dev_add_netdev_ops(dev);
	dev_add_ethtool_ops(dev);
	/* update net-device with user-space private data */
	/* initial device interface address is set with */
	/* permanent addr								*/
	set_netdevice(dev, priv_data);
	dev->dev_addr = dev->perm_addr;
}

pid_t get_info_from_inf(char *name, unsigned short *port_id)
{
	int i, j;

	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++) {
		if (netdev_tbl[i].num_of_if) {
			for(j = 0; j < netdev_tbl[i].num_of_if; j++) {
				if (!strncmp(name, netdev_tbl[i].inf_set[j].inf_name,
					strlen(name))) {
					
					*port_id = j;
					return netdev_tbl[i].app_pid;
				}
			}
		}
	}

	return 0;
}

struct completion *get_completion_flag(pid_t app_pid, unsigned char port_id)
{
	int i;

	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++)
		if (netdev_tbl[i].num_of_if) {
			if (netdev_tbl[i].app_pid == app_pid) {
				if (port_id < netdev_tbl[i].num_of_if)
					return &netdev_tbl[i].done[port_id];
				else {
					vni_elog("Netdev_tbl entry only support"
						" %d interface requested port is %d\n",
						netdev_tbl[i].num_of_if, port_id);
					return NULL;
				}
			}
		}

	return NULL;
}
/*
 * Release netdev devices registered for a particular
 * management (share command device name prefix)
 */
int release_netdev(int ind)
{

	clean_pending_inf_cmd();
	defer_netdev_client = ind;

	return  get_netdev_count();
}

void release_netdev_all(void)
{
	int i;

	/* remove all outstanding netdev interface */
	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++)
		if (netdev_tbl[i].num_of_if)
			release_netdev(i);
	
	usleep_range(1000, 5000); /* give time to all pending call to fails */

	clean_pending_inf_cmd();
}

int vni_del_netdev_devices(struct netdev_cmd_info *req_info) {
	int i, ind = INVALID_NETDEV_INDEX;

	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++) {
		if (netdev_tbl[i].num_of_if &&
			(netdev_tbl[i].app_pid == req_info->app_pid)){
			ind = i;
			break;
		}
	}

	if (ind == INVALID_NETDEV_INDEX) {
		vni_elog("Could not find matched app(%d) process id from %d devices\n",
			(int)req_info->app_pid, get_netdev_count());
		return -1;
	}

	return release_netdev(ind);

}


/*
 * Connect netdev-op and ethtool-op with newly create net_devices, and
 * fill in necessary kernel proxy driver for kernel module to share data
 *
 * pid: process id of user-space application
 * 
 */
int vni_netdev_connect(pid_t pid)
{
	int i, j;
	struct common_pci_addr *pci_addr;
	struct pci_dev *pci_dev;
	struct net_device *net;

	vni_log("Register netdev/ethtool op\n");
	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++) {
		if (netdev_tbl[i].num_of_if) {
			if (netdev_tbl[i].app_pid == pid) {
				for(j = 0; j < netdev_tbl[i].num_of_if; j++) {
					net = netdev_tbl[i].dev_ptr_table[j];
					connect_netdev_op(net, &netdev_tbl[i].inf_set[j].data);
					/* connect net_device::dev::parent for "ip" command */
					pci_addr = &netdev_tbl[i].inf_set[j].pci_addr;
					pci_dev = pci_get_domain_bus_and_slot(
						pci_addr->domain,
						pci_addr->bus, 
						PCI_DEVFN(pci_addr->devid, pci_addr->function));

					if (pci_dev) {
						net->dev.parent = &pci_dev->dev;
						if (pci_dev->is_physfn)
							vni_log("There are %d VFs created, and"
							" %d assigned from proxy driver\n",
							pci_num_vf(pci_dev), pci_vfs_assigned(pci_dev));
					} else
						vni_elog("Fail to find pci_dev for pci_addr:%4d:%2d:%2d.%d\n",
							pci_addr->domain, pci_addr->bus, pci_addr->devid,
							pci_addr->function);
				}
				break;
			}
		}
	}
	if (i == MAXI_MANAGEMENT_DEV) {
		vni_elog("Fail to find a netdev table entry for"
			" pid = %d\n",(int)pid);
		return -1;
	}
	return 0;
}

int vni_find_next_netdev(int num_of_if)
{
	int i;
	mutex_lock(&vni_netdev_mutex);
	for(i = 0; i < MAXI_MANAGEMENT_DEV; i++)
		if (netdev_tbl[i].num_of_if == 0){
			netdev_tbl[i].num_of_if = num_of_if;
			mutex_unlock(&vni_netdev_mutex);
			return i;
		}
	mutex_unlock(&vni_netdev_mutex);
	return INVALID_NETDEV_INDEX;
}
/*
 * data: int: num_of_port; char* name_string
 */
int vni_add_netdev_devices(struct netdev_cmd_info *req_info)
{
	char *ifname;
	struct net_device *netdev_ptr;
	void *inf_set = VNI_GET_PTR(req_info->data, sizeof(int));
	int num_of_ports =  VNI_GET_DATA(req_info->data, 0, int, req_info->data_length);
	int i, status;
	int netdev_index = vni_find_next_netdev(num_of_ports);

	vni_log( "Add %d netdev interface\n", num_of_ports);
	if (netdev_index == INVALID_NETDEV_INDEX) {
		vni_elog("Too many netdev managers\n");
		return -1;
	}

	if (req_info->data_length < INF_REQ_SIZE(num_of_ports)) {
		vni_log("Insufficent data in request packet"
			"expect %d and recv %d\n", INF_REQ_SIZE(num_of_ports),
			req_info->data_length);
		return -1;
	}

	if (num_of_ports > MAXI_NETDEV_PER_CLIENT) {
		vni_log("Too many netdev interface request (%d), truncated to %d\n",
			num_of_ports, MAXI_NETDEV_PER_CLIENT);
		num_of_ports = MAXI_NETDEV_PER_CLIENT;
	}

	if (pending_inf_cmd.cmd == vni_manage_del_inf){
		clean_pending_inf_cmd();
		pending_inf_cmd.cmd = vni_invalid;
	}

	netdev_tbl[netdev_index].app_pid = req_info->app_pid;
	memcpy(netdev_tbl[netdev_index].inf_set, inf_set,
		num_of_ports*sizeof(struct inf_info));

	for(i = 0; i < num_of_ports; i++) {
		ifname = netdev_tbl[netdev_index].inf_set[i].inf_name;
		netdev_ptr = alloc_netdev(sizeof(struct net_device)+
			sizeof(struct netdev_priv_data), ifname, 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,4)
			NET_NAME_UNKNOWN,
#endif
			netdev_setup);
		if (netdev_ptr == NULL) {
			vni_elog("Fail to alloc netdev %s\n", ifname);
			return -ENOMEM;
		}
		if ((status = register_netdev(netdev_ptr)) < 0) {
			vni_elog("Fail to register netdev with name=%s ret=status\n", ifname, status);
			return 0;
		}
		vni_log("Succeed register netdev with inf %s\n", netdev_ptr->name);
		netdev_tbl[netdev_index].dev_ptr_table[i] = netdev_ptr;
	}
		
	for(i = 0; i < num_of_ports; i++)
		init_completion(&netdev_tbl[netdev_index].done[i]);

	vni_log("Succeed register %d inf with name %s ...\n", netdev_tbl[netdev_index].num_of_if,
			netdev_tbl[netdev_index].inf_set[0].inf_name);
	vni_log("%d netdev device\n", get_netdev_count());
	return get_netdev_count();
}

void vni_save_inf_cmd(netdev_cmd_info *inf_cmd)
{
	if (pending_inf_cmd.cmd == vni_manage_del_inf) {
		clean_pending_inf_cmd();
		pending_inf_cmd.cmd = vni_invalid;
	}
	else {
		if (pending_inf_cmd.cmd == vni_manage_add_inf)
			vni_elog("Imcomplete vni_manage_ad_inf transactions: add_inf->ack_kernel->ack_us\n");
	}
	memcpy(&pending_inf_cmd, inf_cmd, sizeof(netdev_cmd_info));
}

int vni_proc_inf_cmd(void)
{
	/* netdev/ethtool op registration */
	switch(pending_inf_cmd.cmd){
	case vni_manage_add_inf:
		vni_netdev_connect(pending_inf_cmd.app_pid);
		/* this is the last step of deferred add_inf process */
		pending_inf_cmd.cmd = vni_invalid;
		break;
	case vni_manage_del_inf:
		vni_del_netdev_devices(&pending_inf_cmd);
		break;
	case vni_manage_clean_inf:
		break;
	default:
		vni_elog("Unexpected inf cmd (%d: %s)\n",
			pending_inf_cmd.cmd, cmd_name(pending_inf_cmd.cmd));
	}
	return 0;
}

static struct sock *vni_socket(int mode, struct sock *new_socket)
{
	static struct sock *g_socket = 0;

	if (mode == GET_SOCKET)
		return g_socket;

	g_socket = new_socket;
		return NULL;
}

void vni_set_socket(struct sock *sock)
{
	vni_socket(SET_SOCKET, sock);
}

struct sock *vni_get_socket(void)
{
	return vni_socket(GET_SOCKET, NULL);
}
/*
 * Kernel version dependent feature definition conversion
 */
netdev_features_t rte_features(netdev_features_t netdev_flags)
{
	netdev_features_t rte_flags = 0;

	rte_feature(netdev_flags, rte_flags, NETIF_F_SG);
	rte_feature(netdev_flags, rte_flags, NETIF_F_IP_CSUM);
	rte_feature(netdev_flags, rte_flags, NETIF_F_HW_CSUM);
	rte_feature(netdev_flags, rte_flags, NETIF_F_IPV6_CSUM);
	rte_feature(netdev_flags, rte_flags, NETIF_F_HIGHDMA);
	rte_feature(netdev_flags, rte_flags, NETIF_F_FRAGLIST);

#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if (netdev_flags & NETIF_F_HW_VLAN_CTAG_RX)
#else
	if (netdev_flags & NETIF_F_HW_VLAN_RX)
#endif
		rte_flags |= RTE_NETIF_F_HW_VLAN_RX;

#ifdef NETIF_F_HW_VLAN_CTAG_TX
	if (netdev_flags & NETIF_F_HW_VLAN_CTAG_TX)
#else
	if (netdev_flags & NETIF_F_HW_VLAN_TX)
#endif
		rte_flags |= RTE_NETIF_F_HW_VLAN_TX;

#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	if (netdev_flags & NETIF_F_HW_VLAN_CTAG_FILTER)
#else
	if (netdev_flags & NETIF_F_HW_VLAN_FILTER)
#endif
		rte_flags |= RTE_NETIF_F_HW_VLAN_FILTER;

	rte_feature(netdev_flags, rte_flags, NETIF_F_VLAN_CHALLENGED);
	rte_feature(netdev_flags, rte_flags, NETIF_F_GSO);
	rte_feature(netdev_flags, rte_flags, NETIF_F_LLTX);
	rte_feature(netdev_flags, rte_flags, NETIF_F_NETNS_LOCAL);

	rte_feature(netdev_flags, rte_flags, NETIF_F_GRO);
	rte_feature(netdev_flags, rte_flags, NETIF_F_LRO);
	rte_feature(netdev_flags, rte_flags, NETIF_F_SCTP_CSUM);
	rte_feature(netdev_flags, rte_flags, NETIF_F_TSO);
	rte_feature(netdev_flags, rte_flags, NETIF_F_UFO);
	rte_feature(netdev_flags, rte_flags, NETIF_F_TSO6);

	return rte_flags;
}

netdev_features_t netdev_features(netdev_features_t rte_flags)
{
	netdev_features_t netdev_flags = 0;

	netdev_feature(netdev_flags, rte_flags, NETIF_F_SG);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_IP_CSUM);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_HW_CSUM);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_IPV6_CSUM);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_HIGHDMA);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_FRAGLIST);

	if (rte_flags & RTE_NETIF_F_HW_VLAN_RX)
#ifdef NETIF_F_HW_VLAN_CTAG_RX
		netdev_flags |= NETIF_F_HW_VLAN_CTAG_RX;
#else
		netdev_flags |= NETIF_F_HW_VLAN_RX;
#endif

	if (rte_flags & RTE_NETIF_F_HW_VLAN_TX)
#ifdef NETIF_F_HW_VLAN_CTAG_TX
		netdev_flags |= NETIF_F_HW_VLAN_CTAG_TX;
#else
		netdev_flags |= NETIF_F_HW_VLAN_TX;
#endif

	if (rte_flags & RTE_NETIF_F_HW_VLAN_FILTER)
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
		netdev_flags |= NETIF_F_HW_VLAN_CTAG_FILTER;
#else
		netdev_flags |= NETIF_F_HW_VLAN_FILTER;
#endif

	netdev_feature(netdev_flags, rte_flags, NETIF_F_VLAN_CHALLENGED);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_GSO);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_LLTX);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_NETNS_LOCAL);

	netdev_feature(netdev_flags, rte_flags, NETIF_F_GRO);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_LRO);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_SCTP_CSUM);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_TSO);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_UFO);
	netdev_feature(netdev_flags, rte_flags, NETIF_F_TSO6);

	return netdev_flags;
}

void vni_init_netdev(void)
{
	vni_mutex_init();
	vni_init_netdev_tbl();
	
}

