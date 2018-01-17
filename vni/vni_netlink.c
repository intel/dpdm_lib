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
#include <linux/completion.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/ethtool.h> /* for debugging */
#include <linux/skbuff.h>
#include <linux/string.h>
#include "vni_common.h"
#include "vni_types.h"
#include "vni_log.h"

netdev_cmd_info *u2k_netdev_cmd; /* user-space to kernel netdev cmd */

static void vni_nl_manage_inf(netdev_cmd_info *inf_cmd);
static void vni_nl_recv_msg(struct sk_buff *skb);

netdev_cmd_info *get_u2k_netdev_cmd(void)
{
	return u2k_netdev_cmd;
}

netdev_cmd_info *new_netlink_skbbuf(size_t msg_size)
{
	struct sk_buff *skb_out;
	struct nlmsghdr *nlh;
	netdev_cmd_info *new_netdev_cmd;

	vni_lock();
	skb_out = nlmsg_new(NLMSG_ALIGN(msg_size), GFP_ATOMIC);

	if(!skb_out)
	{
		vni_elog("Failed to allocate new sk_buff\n");
		vni_release_lock();
		return NULL;
	}
	set_msg_state( msg_created);
	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);  
	NETLINK_CB(skb_out).dst_group = 0; /* unicast */
	new_netdev_cmd = (netdev_cmd_info *)nlmsg_data(nlh);
	
	new_netdev_cmd->host = skb_out;
	return new_netdev_cmd;
}

int vni_nl_xmit_msg(netdev_cmd_info *netdev_cmd)
{
	struct sk_buff *skb_out = netdev_cmd->host;
	struct completion *done;
	struct sock *vni_sock = vni_get_socket();
	int status;

	if (!vni_sock) {
		vni_elog("no socket for vni_nl_xmit_msg\n");
		vni_release_lock();
		return -1;
	}

	vni_log("send cmd(%d: %s) to user-space daemon (pid = %d)\n", netdev_cmd->cmd,
		cmd_name(netdev_cmd->cmd), netdev_cmd->app_pid);
	status = nlmsg_unicast(vni_sock, skb_out, netdev_cmd->app_pid);
	if(status < 0) {
		vni_elog("Error while sending a unicast nlmsg to application(pid=%d)"
			" with error code(%d)\n", netdev_cmd->app_pid, status);
		vni_release_lock();
		return status;
	}
	set_msg_state(msg_sent);
	/* vni_manage_add_inf and vni_manage_del_inf are initiated from user-space */
	/* applicaiton, therefore there is no waiting kernel command packet */
	if ((netdev_cmd->cmd == vni_manage_add_inf) ||
		(netdev_cmd->cmd == vni_manage_del_inf) ||
		(netdev_cmd->cmd == vni_manage_clean_inf)) {
		vni_elog("Kernel should not initiate this cmd(%s)\n", cmd_name(netdev_cmd->cmd));
		vni_release_lock();
		return 0;
	}

	/*
	 * vni_manage_ack_kernel message is initiated inside received messag callback, therefore it blocks
	 * further receive msg. To avoid unwanted blocking because of wait-for-completion api, skip wait-for-
	 * complet intead using received vni_manage_ack_user to un-lock mutex to enforce atomicy of send-and-receive
	 * message
	 */
	if (netdev_cmd->cmd != vni_manage_ack_kernel) {
		done = get_completion_flag(netdev_cmd->app_pid, netdev_cmd->port_id);
		if (!done) {
			vni_elog("fail to get completion flag from app/port-id(%d/%d) with cmd(%d, %s)\n",
				netdev_cmd->app_pid, netdev_cmd->port_id, netdev_cmd->cmd, cmd_name(netdev_cmd->cmd));
			vni_release_lock();
			return -1;
		}

		status = (int)wait_for_completion_interruptible_timeout(done,
			msecs_to_jiffies(500));
		if (status <= 0){
			vni_elog("wait_for_completion_interruptible_timeout fails"
				" caused by %s (error-code=%d) msg_state=%d", (status == 0)?"time out":"interrupt", status, get_msg_state());
			vni_release_lock();
			return status;
		}
		vni_log("cmd(%d, %s) msg status(%d)\n", netdev_cmd->cmd, cmd_name(netdev_cmd->cmd), get_msg_state());
		vni_release_lock();
	}
	return 0;
}

static void vni_nl_manage_inf(netdev_cmd_info *inf_cmd)
{
	netdev_cmd_info *send_cmd;

	if (inf_cmd->cmd == vni_manage_clean_inf) {
		clean_pending_inf_cmd();
		return;
	}

	send_cmd = new_netlink_skbbuf(sizeof(netdev_cmd_info));
	if (!send_cmd) {
		vni_elog("fail to alloc skb for inf cmd (%s)\n", cmd_name(inf_cmd->cmd));
		vni_release_lock();
		return;
	}
	vni_log("received inf cmd(%d, %s)\n", inf_cmd->cmd, cmd_name(inf_cmd->cmd));
	if (inf_cmd->cmd == vni_manage_add_inf)
		send_cmd->status = vni_add_netdev_devices(inf_cmd);

	send_cmd->cmd = vni_manage_ack_kernel;
	send_cmd->data_length = 0;
	send_cmd->app_pid = inf_cmd->app_pid;
	send_cmd->port_id = inf_cmd->port_id;

	if (vni_nl_xmit_msg(send_cmd) < 0) {
		vni_elog("fail to send cmd(%s) to user-space daemon (pid=%d)"
			"with port-id(%d)\n",
			cmd_name(send_cmd->cmd), send_cmd->app_pid, send_cmd->port_id);
		vni_release_lock();
		return;
	}
	vni_save_inf_cmd(inf_cmd);
}

static void vni_nl_recv_msg(struct sk_buff *skb)
{

	struct nlmsghdr *nlh;
	struct completion *done;
	netdev_cmd_info *netdev_cmd;

	set_msg_state(msg_recv);
	nlh = (struct nlmsghdr *)skb->data;
	netdev_cmd = (netdev_cmd_info *)nlmsg_data(nlh);
	
	u2k_netdev_cmd = netdev_cmd;
	/* vni_manage_add_inf and vni_manage_del_inf are initiated from user-space */
	/* applicaiton, therefore there is no waiting kernel command packet */
	if ((netdev_cmd->cmd == vni_manage_add_inf) ||
		(netdev_cmd->cmd == vni_manage_del_inf) ||
		(netdev_cmd->cmd == vni_manage_clean_inf)) {
		vni_log("process inf cmd (%d, %s)\n", netdev_cmd->cmd, cmd_name(netdev_cmd->cmd));
		vni_nl_manage_inf(netdev_cmd);
		vni_log("done with inf cmd (%d, %s)\n", netdev_cmd->cmd, cmd_name(netdev_cmd->cmd));
		return;
	}

	if (netdev_cmd->cmd != vni_manage_ack_us) {
		done = get_completion_flag(netdev_cmd->app_pid, netdev_cmd->port_id);
		if(!done) {
			vni_elog("fail to find synchronization flag for port_id %d cmd=%s\n",
				netdev_cmd->port_id, cmd_name(netdev_cmd->cmd));
			vni_release_lock();
			return;
		}
		complete(done);
	} else {
		vni_proc_inf_cmd();
		vni_log("received inf-manage-cmd(%s) with status(%x)\n",
			cmd_name(netdev_cmd->cmd), netdev_cmd->status);
		vni_release_lock();
	}
	vni_log("complete recv cmd(%s) from port(%d) with status=%d\n",
		cmd_name(netdev_cmd->cmd), netdev_cmd->port_id, netdev_cmd->status);
}

//This is for 3.6 kernels and above.
static struct netlink_kernel_cfg cfg = {
	.input = vni_nl_recv_msg,
};

struct sock *vni_create_netlink(void)
{
	return netlink_kernel_create(&init_net, NETLINK_VNI, &cfg);
}

