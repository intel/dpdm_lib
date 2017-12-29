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
#ifndef _VNI_NETLINK_H_H
#define _VNI_NETLINK_H_H

#include "vni_types.h"

struct sock;

netdev_cmd_info *new_netlink_skbbuf(size_t msg_size);
struct sock *vni_create_netlink(void);
int vni_nl_xmit_msg(netdev_cmd_info *cmd);
netdev_cmd_info *get_u2k_netdev_cmd(void);

#endif /* _VNI_NETLINK_H_H */