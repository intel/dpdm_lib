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
#ifndef _VNI_NETDEV_H_
#define _VNI_NETDEV_H_

struct net_device;
void dev_add_netdev_ops(struct net_device *dev);
void dev_add_dummy_netdev(struct net_device *dev);

#endif /* _VNI_NETDEV_H_ */
