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

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/netdevice.h> 
#include <linux/ethtool.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include "vni.h"

#define VNI_CACHE_SIZE 1024
char client_cache[VNI_CACHE_SIZE];
const char man_msg1[] = "VF_FLAG encoding\n"
	"\t31: PF/VF (0/1)\n"
	"\t30:24: vf-index\n"
	"\t23:21: op-code\n"
	"\t\t0: VF-feature bit set\t(bit[0:15]: feature bits)\n"
	"\t\t1: VF Rx mode set\t(bit[0:15]: rxmode feature bits)\n"
	"\t\t2: VF get statistics\t(output: /sys/kernel/vni_manage/client_cache)\n"
	"\t\t3: VF vlan filter\t(multi-step op; bit[0:15]: vlan-id)\n"
	"\t\t4: VF vlan insert\t(bit[0:15]: vlan-id)\n"
	"\t\t5: VF rate\t\t(bit[0:15]: rx_rate)\n"
	"\t\t6: VF queue rate limit	(multi-step op; bit[0:15]: rx_rate)\n"
	"\n"
	"\t20:18: code sequence (only for multi-step op)\n"
	"\t17:16: reserved\n"
	"\t15:0: feature bits/rx_rate/16-bit-vlan-id\n"
	"\n";

const char man_msg2[] = "Position string\n"
	"\tsplit-drop.untag.b0\n"
	"\tpromisc.hash-mc.b1\n"
	"\tallmulti.hash-uc.b2\n"
	"\tbroadcast.b3\n"
	"\tspoofchk.mc.b4\n"
	"\tlink-state.b5\n"
	"\tpf-ping-vf.b6\n"
	"\tmac-antispoof.b7\n"
	"\tvlan-antispoof.b8\n"
	"\tvlan-stripq.b9\n"
	"\trx-queue-en.b10\n"
	"\ttx-queue-en.b11\n"
	"\treset-stats.b12\n"
	"\tb13\n"
	"\tb14\n"
	"\ttrusted.mode.b15\n"
	"\tb16\n"
	"\tb17\n"
	"\tseq0\n"
	"\tseq1\n"
	"\tseq2\n"
	"\top-code0\n"
	"\top-code1\n"
	"\top-code2\n"
	"\tvf-id0\n"
	"\tvf-id1\n"
	"\tvf-id2\n"
	"\tvf-id3\n"
	"\tvf-id4\n"
	"\tvf-id5\n"
	"\tvf-id6\n"
	"\tvf-flag\n"
	"\n";

const char man_msg3[] = "Examples:\n"
	"1. Enable VF #1 with promisc and spoof-check\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on vf-id0 on "
	"spoofchk.mc.b4 on promisc.hash-mc.b1 on\n\n"
	"2. Set VF #2 rxmode with accepting untagged packets\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on vf-id1 on "
	"op-code0 on trusted.mode.b15 on split-drop.untag.b0 on\n\n"
	"3. Add vlan-id 1 to VF #0 and #1 (multi-step op, step 1..4 "
	"to specify 4x16-bit VF mask)\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on op-code1 "
	"on op-code0 on seq2 on\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on op-code1 "
	"on op-code0 on seq1 on seq0 on\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on op-code1 "
	"on op-code0 on seq1 on\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on op-code1 "
	"on op-code0 on seq0 on promisc.hash-mc.b1 on"
	" split-drop.untag.b0 on\n"
	"\tethtool --set-priv-flags <dev-name> vf-flag on op-code1 "
	"on op-code0 on trusted.mode.b15 on split-drop.untag.b0 on\n\n"
	;

static ssize_t get_stats64_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{

	buf[0] = get_get_stats64_enable() > 0 ?'1':'0';
	buf[1] = '\n';

	return 2;
}

static ssize_t get_stats64_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int enable = buf[0] - '0';
	set_get_stats64_enable(enable);
	return count;
}

static ssize_t trace_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	ssize_t length, remain;
	struct msg_info *log = log_info();

	if (log->overlap == 0) {
		length = snprintf(buf, log->next, "%s", log->buf);
	} else {
		remain = MAXI_MSG_LENGTH - log->next;
		log->buf[MAXI_MSG_LENGTH] ='\0';
		snprintf(buf, remain, "%s", &log->buf[log->next]);

		snprintf(&buf[remain], log->next, "%s", log->buf);
		length = MAXI_MSG_LENGTH;
	}
	buf[length] = '\n';
	printk(KERN_INFO "trace with length of %d\n", (int)length+1);
	return length+1;
}

static ssize_t trace_flush(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct msg_info *log = log_info();

	log->buf[0] = '\0';
	log->buf[MAXI_MSG_LENGTH] = '\0';
	log->next = 0;
	log->overlap = 0;

	return count;
}

static ssize_t client_cache_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	ssize_t length = strlen(client_cache);

	memcpy(buf, client_cache, length+1);
	client_cache[0] = '\0';

	return length+1;
}

static ssize_t client_cache_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	size_t save_count = count >= VNI_CACHE_SIZE?(VNI_CACHE_SIZE-1):count;

	memcpy(client_cache, buf, save_count);
	client_cache[save_count] = '\0';

	return save_count;
}

static ssize_t man_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	memcpy(buf, man_msg1, strlen(man_msg1));
	memcpy(&buf[strlen(man_msg1)], man_msg2, strlen(man_msg2));
	memcpy(&buf[strlen(man_msg1)+strlen(man_msg2)], man_msg3, strlen(man_msg3));

	return strlen(man_msg1)+strlen(man_msg2)+strlen(man_msg3);
}
/* Sysfs attributes cannot be world-writable. */
static struct kobj_attribute trace_attribute =
	__ATTR(trace, 0664, trace_show, trace_flush);

static struct kobj_attribute get_stats_en_attribute =
	__ATTR(get_stats_en, 0664, get_stats64_show, get_stats64_store);

static struct kobj_attribute client_cache_attribute =
	__ATTR(client_cache, 0664, client_cache_show, client_cache_store);

static struct kobj_attribute man_attribute =
	__ATTR_RO(man);

static struct attribute *attrs[] = {
	&trace_attribute.attr,
	&get_stats_en_attribute.attr,
	&client_cache_attribute.attr,
	&man_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *vni_trace_kobj;

static int __init
vni_init(void)
{
	struct sock *socket;
	struct msg_info *log_data = log_info();
	int status;

	/* create sys kobject for trace file */
	vni_trace_kobj = kobject_create_and_add("vni_manage", kernel_kobj);
	if (!vni_trace_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	status = sysfs_create_group(vni_trace_kobj, &attr_group);
	if (status)
		kobject_put(vni_trace_kobj);

	memset(log_data, 0, sizeof(struct msg_info));

	socket = vni_create_netlink();
	if (!socket) {
		vni_log("vni: Error creating socket for netlink\n");
		return -1;
	}
	vni_set_socket(socket);
	vni_init_netdev();

	vni_log("vni: module init\n");
	return 0;
}

static void __exit
vni_exit(void)
{
	release_netdev_all();
	netlink_kernel_release(vni_get_socket());
	printk(KERN_INFO "vni: module exit\n");
	kobject_put(vni_trace_kobj);
}

module_init(vni_init);
module_exit(vni_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Liang-Min Wang, Intel Corp");
