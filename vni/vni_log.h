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
#ifndef _VNI_LOG_H_
#define _VNI_LOG_H_

#define MAXI_MSG_LENGTH	(1024*4-512)
#define MAXI_LINE		200

#define DBG_MSG "vni-dbg: "
#define INFO_MSG "vni-info: "
#define ERR_MSG "vni-error: "

#ifdef DEBUG
#define vni_dlog(format, ...)		printk(KERN_DEBUG DBG_MSG format, ## __VA_ARGS__)
#define vni_log(format, ...)		printk(KERN_INFO INFO_MSG format, ## __VA_ARGS__)
#define vni_elog(format, ...)		printk(KERN_ERROR ERR_MSG format, ## __VA_ARGS__)
#else
#define vni_dlog(format, ...)		vlog(DBG_MSG format, ## __VA_ARGS__)
#define vni_log(format, ...)		vlog(INFO_MSG format, ## __VA_ARGS__)
#define vni_elog(format, ...)		vlog(ERR_MSG format, ## __VA_ARGS__)
#endif

struct msg_info {
	char buf[MAXI_MSG_LENGTH+1];
	int next;
	int overlap;
};

struct msg_info *log_info(void);
void vlog(char *format, ...);

#endif	/* _VNI_LOG_H_ */
