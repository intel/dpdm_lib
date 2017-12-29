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
#ifndef _VNI_COMMON_H_
#define _VNI_COMMON_H_

#include <net/sock.h>
#include "vni_types.h"

#define GET_SOCKET 0
#define SET_SOCKET 1

#ifdef DEBUG
#define VNI_TRACE(...) printk(KERN_INFO "vni: " __VA_ARGS__)
#else
#define VNI_TRACE(...)
#endif

#define VNI_ERROR(...) printk(KERN_ERR "vni: " __VA_ARGS__)

char *mac_addr_string(void *in_mac_addr);
pid_t get_info_from_inf(char *name, unsigned short *port_id);
int release_netdev(int ind);
int vni_add_netdev_devices(netdev_cmd_info *req_info);
int vni_del_netdev_devices(netdev_cmd_info *req_info);
int vni_netdev_op_registration(pid_t pid);
void release_netdev_all(void);
void vni_set_socket(struct sock *socket);
struct sock *vni_get_socket(void);
struct completion *get_completion_flag(pid_t app_pid, unsigned char port_id);
void set_msg_state(vni_msg_status new_state);
vni_msg_status get_msg_state(void);
void vni_save_inf_cmd(netdev_cmd_info *inf_cmd);
int vni_proc_inf_cmd(void);
netdev_features_t rte_features(netdev_features_t netdev_flags);
netdev_features_t netdev_features(netdev_features_t rte_flags);
int get_get_stats64_enable(void);
void set_get_stats64_enable(int enable);
int get_netdev_count(void);

void clean_pending_inf_cmd(void);
int is_inf_closing(struct net_device *net);

int vni_find_next_netdev(int num_of_if);
void vni_init_netdev(void);

/* helper routine for send/recv nl packets to/from user-space */
netdev_cmd_info *k2u_downlink(struct net_device *dev,
	netdev_cmd_type cmd, size_t data_size);

netdev_cmd_info *k2u_uplink(struct net_device *dev,
	netdev_cmd_info *k2u_cmd_info);

int k2u_link(struct net_device *dev, netdev_cmd_type cmd);
netdev_cmd_info *k2u_link_1var_other(struct net_device *dev,
	netdev_cmd_type cmd, void *var, size_t data_size);

int k2u_link_1var_noupdate(struct net_device *dev,
	netdev_cmd_type cmd, void *var, size_t data_size);

int k2u_link_1var(struct net_device *dev,
	netdev_cmd_type cmd, void *var, size_t data_size);

const char *cmd_name(netdev_cmd_type cmd);
int is_vni_manage_cmd(netdev_cmd_type cmd);
int is_vni_netdev_cmd(netdev_cmd_type cmd);
int is_vni_ethtool_cmd(netdev_cmd_type cmd);

void vni_lock(void);
void vni_unlock(void);
void vni_mutex_init(void);
void vni_trylock(void);
int vni_is_locked(void);
void vni_release_lock(void);
/* end of helper routine */

#endif /* _VNI_COMMON_H_ */
