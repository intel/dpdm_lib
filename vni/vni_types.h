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
#ifndef _VNI_TYPES_H_
#define _VNI_TYPES_H_

#include <linux/completion.h>
#include "vni_share_types.h"

#define MAXI_NETDEV_PER_CLIENT	64
#define INVALID_NETDEV_INDEX	-1

struct netdev_table {
	pid_t app_pid;
	int num_of_if;
	struct net_device *dev_ptr_table[MAXI_NETDEV_PER_CLIENT];
	struct completion done[MAXI_NETDEV_PER_CLIENT];
	struct inf_info inf_set[MAXI_NETDEV_PER_CLIENT];
};

struct netdev_info_ext{
	pid_t app_pid;
	unsigned char port_id;
};

#endif /* _VNI_TYPES_H_ */
