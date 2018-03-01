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
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include <fcntl.h>
#include <errno.h>

#include <rte_atomic.h>
#include <rte_ethdev.h>
#include "rte_netdev_ops.h"
#include "ndc_types.h"
#include "ndc.h"

#define MAXI_REPLY_SIZE	(4*1024*2)
#define MAXI_RECV_SIZE 1024
#define MAXI_NETLINK_CHANNEL 128
#define MAXI_REP_COUNT 5
#define NAME_DELIMITER ','

struct netlink_data {
	int sock_fd;
	int num_of_ports;
	struct sockaddr_nl dest_addr;
	struct msghdr send_msg, recv_msg;
	unsigned char send_data[MAXI_REPLY_SIZE];
	unsigned char recv_data[MAXI_RECV_SIZE];
	struct iovec send_iov, recv_iov;
	struct inf_info inf_info[0];
};

static int nl_chan_dep = 0;
static struct netlink_data *netlink_chans[MAXI_NETLINK_CHANNEL];

/* local helper routines */
static struct netlink_data* ndc_nl_data_alloc(char *inf_name, int num_of_ports, pid_t dest_pid);
static void ndc_nl_data_release(struct netlink_data *nl_data);
static void ndc_recv_msgbuf_init(struct netlink_data *nl_data);
static int ndc_inf_proc(struct netlink_data *nl_data, int num_of_ports, int add);
static int ndc_send_cmd(struct netlink_data *nl_data, netdev_cmd_info *cmd_info);
static ssize_t ndc_receive(struct netlink_data *nl_data);
static void ndc_recv_to_send(netdev_cmd_info *dst_cmd, netdev_cmd_info *src_cmd);
static void ndc_manage_cmd_proc(netdev_cmd_info *send_netdev_cmd);
static int is_manage_cmd(netdev_cmd_type cmd);

const char *cmd_name(netdev_cmd_type cmd)
{
	const char *manage[] = {
	"vni_manage",
	"vni_manage_clean_inf",
	"vni_manage_add_inf",
	"vni_manage_del_inf",
	"vni_manage_ack_kernel",
	"vni_manage_ack_us",
	"vni_manage_test"
	};

	const char *netdev[] = {
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

	const char *ethtool[] = {
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
	"vni_ethtool_set_per_queue_coalesce",
	"vni_invald"};

	if ((cmd >= vni_manage) &&
		(cmd <= vni_manage_test))
		return manage[cmd];
	else {
		if ((cmd >= vni_netdev_init) &&
			(cmd <= vni_netdev_set_features))
			return netdev[cmd-vni_netdev_init];
		else
			return ethtool[cmd - vni_ethtool_set_setting];
	}

}

struct netlink_data*
find_nl_with_inf_name(char *inf_name)
{
	int i, j;

	for(i = 0; i < nl_chan_dep; i++)
		for (j = 0; j < netlink_chans[i]->num_of_ports; j++)
			if (strncmp(netlink_chans[i]->inf_info[j].inf_name,
				inf_name, strlen(inf_name)) == 0)
				return netlink_chans[i];

	return (struct netlink_data *)0;
}

static void ndc_nl_data_release(struct netlink_data *nl_data)
{
	int i, j;
	
	for(i = 0; i < nl_chan_dep; i++) 
		if (!strcmp(netlink_chans[i]->inf_info[0].inf_name, nl_data->inf_info[0].inf_name))
			break;

	if (i == nl_chan_dep) {
		RTE_VNI_DEBUG_TRACE("missing nl_data link pointer with inf_name=%s nl_chap_dep=%d\n",
			nl_data->inf_info[0].inf_name);
		return;
	}
	free(nl_data);
	
	if (i == (nl_chan_dep -1))
		return;

	i++;
	for(;i < nl_chan_dep; i++)
		netlink_chans[i-1] = netlink_chans[i];
	nl_chan_dep--;
}

static ssize_t ndc_receive(struct netlink_data *nl_data)
{
	struct nlmsghdr *recv_nlh = RECV_NLH(nl_data);

	memset(recv_nlh, 0, sizeof(nl_data->recv_data));
	recv_nlh->nlmsg_len = sizeof(nl_data->recv_data);
	recv_nlh->nlmsg_pid = getpid();
	recv_nlh->nlmsg_flags = 0;
	nl_data->recv_iov.iov_base = (void *)recv_nlh;
	nl_data->recv_iov.iov_len = recv_nlh->nlmsg_len;
	nl_data->recv_msg.msg_iov = &(nl_data->recv_iov);
	nl_data->recv_msg.msg_iovlen = 1;

	return recvmsg(nl_data->sock_fd, &(nl_data->recv_msg), 0);
}

static int ndc_send_cmd(struct netlink_data *nl_data, netdev_cmd_info *cmd_info)
{
	int status;
	struct nlmsghdr *nlh = SEND_NLH(nl_data);

	nlh->nlmsg_len = NLMSG_SPACE(sizeof(netdev_cmd_info)+cmd_info->data_length);
	nlh->nlmsg_pid = getpid();
	nlh->nlmsg_flags = 0;

	nl_data->send_iov.iov_base = (void *)nlh;
	nl_data->send_iov.iov_len = nlh->nlmsg_len;
	nl_data->send_msg.msg_name = (void *)&(nl_data->dest_addr);
	nl_data->send_msg.msg_namelen = sizeof(nl_data->dest_addr);
	nl_data->send_msg.msg_iov = &(nl_data->send_iov);
	nl_data->send_msg.msg_iovlen = 1;

	status = sendmsg(nl_data->sock_fd,&nl_data->send_msg,0);
	RTE_VNI_DEBUG_TRACE("sendmsg(cmd=%d: %s) ret=%d; Waiting for message from kernel\n",
		cmd_info->cmd, cmd_name(cmd_info->cmd), status);

	return status;
}

static void parse_inf_name(char *name_string, int count,
	struct inf_info inf_info[]) {
	char *curr_str, *next_str;
	char *inf_name;
	int i;

	curr_str = name_string;
	for(i = 0; i < count; i++) {
		inf_name = inf_info[i].inf_name;
		memset(inf_name, 0, sizeof(inf_info[i].inf_name));
		next_str = strchr(curr_str, (int)NAME_DELIMITER);
		if (!next_str) {
			if (curr_str == name_string)
				snprintf(inf_name, MAXI_INTERFACE_NAME, "%s%d", curr_str, i);
			else
				snprintf(inf_name, MAXI_INTERFACE_NAME, "%s", curr_str);
		} else {
			*next_str = '\0';
			snprintf(inf_name, MAXI_INTERFACE_NAME, "%s", curr_str);
			curr_str = &next_str[1];
		}
	}
}

static struct netlink_data* 
ndc_nl_data_alloc(char *inf_name, int num_of_ports, pid_t dest_pid)
{
	struct netlink_data *nl_data;
	char *next_name;
	struct sockaddr_nl src_addr;
	int sock_fd, status, i;

	if (nl_chan_dep == MAXI_NETLINK_CHANNEL) {
		RTE_VNI_DEBUG_TRACE("too many netlink channels are created\n"); /* TODO log */
		return NULL;
	}

	nl_data = malloc(sizeof(struct netlink_data) + 
		num_of_ports*sizeof(struct inf_info));
	if (nl_data == NULL) {
		RTE_VNI_DEBUG_TRACE("run out of memory for netlink_data\n"); /* TODO log */
		return NULL;
	}
	netlink_chans[nl_chan_dep] = nl_data;

	/* initialize the netlink info */
	/* create socket for listening netdev api call */
	sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_VNI);
	if(sock_fd < 0) {
		RTE_VNI_DEBUG_TRACE("Fail to create a socket for PF_NETLINK with"
			" socket-type=%d, errno=%d\n", NETLINK_VNI, errno); /*TODO log */
		close(sock_fd);
		free(nl_data);
		return NULL;
	}
	nl_chan_dep++;
	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid(); /* self pid */

	status = bind(sock_fd, (struct sockaddr*)&src_addr, sizeof(src_addr));
	if (status == -1) {
		RTE_VNI_DEBUG_TRACE("Fail to bind socket %d with errno %d\n",
			sock_fd, errno);
		close(sock_fd);
		free(nl_data);
		return NULL;
	}
	memset(&(nl_data->dest_addr), 0, sizeof(nl_data->dest_addr));
	nl_data->dest_addr.nl_family = AF_NETLINK;
	nl_data->dest_addr.nl_pid = dest_pid; /* For Linux Kernel */
	nl_data->dest_addr.nl_groups = 0; /* unicast */

	nl_data->sock_fd = sock_fd;
	nl_data->num_of_ports = num_of_ports;
	/* find port pci_addr and parsing interface name for each port */
	parse_inf_name(inf_name, num_of_ports, nl_data->inf_info);
	{
		struct rte_pci_addr addr;
        struct rte_eth_txq_info qinfo;

		for(i = 0; i < num_of_ports; i++) {
			rte_eth_get_pci_addr(i, &addr);
			nl_data->inf_info[i].pci_addr.domain = addr.domain;
			nl_data->inf_info[i].pci_addr.bus = addr.bus;
			nl_data->inf_info[i].pci_addr.devid = addr.devid;
			nl_data->inf_info[i].pci_addr.function = addr.function;
			rte_ethtool_get_netdev_data(i, (void *)&nl_data->inf_info[i].data);
            rte_eth_tx_queue_info_get(i, 0, &qinfo);
            nl_data->inf_info[i].data.nb_tx_desc = qinfo.nb_desc;
		}
	}
	ndc_recv_msgbuf_init(nl_data);

	return nl_data;
}


static void ndc_recv_msgbuf_init(struct netlink_data *nl_data)
{
	struct iovec *iov = &nl_data->recv_iov;
	struct nlmsghdr *recv_nlh = RECV_NLH(nl_data);

	memset(recv_nlh, 0, sizeof(nl_data->recv_data));
	recv_nlh->nlmsg_flags = 0;
	recv_nlh->nlmsg_pid = getpid();
	recv_nlh->nlmsg_len = sizeof(nl_data->recv_data);

	iov->iov_base = (void *)recv_nlh;
	iov->iov_len = recv_nlh->nlmsg_len;

	nl_data->recv_msg.msg_iov = iov;
	nl_data->recv_msg.msg_iovlen = 1;
	nl_data->recv_msg.msg_namelen = sizeof(struct sockaddr_nl);
}

static void ndc_recv_to_send(netdev_cmd_info *dst_cmd, netdev_cmd_info *src_cmd)
{
	dst_cmd->app_pid = src_cmd->app_pid;
	dst_cmd->port_id = src_cmd->port_id;
	dst_cmd->cmd = src_cmd->cmd;
}

static void ndc_manage_cmd_proc(netdev_cmd_info *send_cmd_info)
{
	send_cmd_info->port_id = 0;
	send_cmd_info->app_pid = getpid();
	send_cmd_info->cmd = vni_manage_ack_us;
	send_cmd_info->status = 0;
	send_cmd_info->data_length = 0;
	RTE_VNI_DEBUG_TRACE("Don't expect manage msg while interface is open and running\n");
}

static int is_manage_cmd(netdev_cmd_type cmd)
{
	if ((cmd >= vni_manage) &&
		(cmd <= vni_manage_test))
		return 1;
	return 0;
}

/*
 * Sequence for INF management calls
 * A. Adding a new interface
 *		1. ndc (send "vni_manage_add_inf")
 *		2. kernel (reply "vni_manage_ack_kernel")
 *		3. ndc (send "vni_manage_ack_us") close transaction
 *
 * B. Delete a existing interface
 *		1. ndc (send "vni_manage_del_inf")
 *		2. kernel (reply "vni_manage_ack_kernel")
 *		3. ndc (send "vni_manage_ack_us") close transaction
 *		4. ndc (send "vni_manage_clean_inf")
 *
 */
static int ndc_inf_proc(struct netlink_data *nl_data, int num_of_ports, int add)
{
	int status = 0, i;
	netdev_cmd_info *send_cmd_info, *recv_cmd_info;

	/* sending vni_manage_add(del)_inf cmd */
	send_cmd_info = SEND_CMD(nl_data);
	send_cmd_info->host = &(nl_data->send_msg);
	send_cmd_info->app_pid = getpid();
	if (add)
		send_cmd_info->cmd = vni_manage_add_inf;
	else
		send_cmd_info->cmd = vni_manage_del_inf;

	send_cmd_info->data_length = sizeof(int)+num_of_ports*sizeof(struct inf_info);
	*(int *)&send_cmd_info->data[0] = nl_data->num_of_ports;
	memcpy((char *)(&send_cmd_info->data[sizeof(int)]), (void *)nl_data->inf_info,
		num_of_ports*sizeof(struct inf_info));

	RTE_VNI_DEBUG_TRACE("send inf cmd(%s) to kernel\n", cmd_name(send_cmd_info->cmd));
	status = ndc_send_cmd(nl_data, send_cmd_info);
	if (status < 0) {
		RTE_VNI_DEBUG_TRACE("send cmd(%s) fail with error code %d\n", cmd_name(send_cmd_info->cmd), status);
		return status;
	}

	/* receive confirmation from netlink partner */
	ndc_receive(nl_data);
	recv_cmd_info = RECV_CMD(nl_data);
	RTE_VNI_DEBUG_TRACE("Received netdev cmd (%d %s) with data_length=%d\n", recv_cmd_info->cmd,
		cmd_name(recv_cmd_info->cmd), recv_cmd_info->data_length);

	/* send vni_manage_ack_us */
	send_cmd_info->port_id = 0;
	send_cmd_info->app_pid = getpid();
	send_cmd_info->status = 0;
	send_cmd_info->data_length = 0;
	send_cmd_info->cmd = vni_manage_ack_us;
	if (add) {
		/* pass netdev private data back to kernel proxy net_device */
		for(i = 0; i < num_of_ports; i++) {
			status = rte_netdev_init((uint8_t)i);
			if (status < 0)
				send_cmd_info->status = status;
		}
	}

	status = ndc_send_cmd(nl_data, send_cmd_info);
	if (status < 0) {
		RTE_VNI_DEBUG_TRACE("send cmd(%s) fail with error code %d\n",
			cmd_name(send_cmd_info->cmd), status);
		return status;
	} 

	if (!add) {
		send_cmd_info->port_id = 0;
		send_cmd_info->app_pid = getpid();
		send_cmd_info->status = 0;
		send_cmd_info->data_length = 0;
		send_cmd_info->cmd = vni_manage_clean_inf;
		status = ndc_send_cmd(nl_data, send_cmd_info);
		if (status < 0) {
			RTE_VNI_DEBUG_TRACE("send cmd(%s) fail with error code %d\n",
					cmd_name(send_cmd_info->cmd), status);
			return status;
		}
	}
	return status;
}

int set_socket_nonblocking(int sock_fd)
{
	struct timeval tv;

	tv.tv_sec = 1;
	tv.tv_usec = 0;

	return setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

int set_socket_blocking(int sock_fd)
{
	struct timeval tv;

	tv.tv_sec = 0;
	tv.tv_usec = 0;

	return setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

int ndc_receive_loop(struct netlink_data *nl_data, rte_atomic32_t *flag)
{
	ssize_t recv_status;

	recv_status = ndc_receive(nl_data);

	while ( (recv_status < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) {
		if (rte_atomic32_read(flag) != NDC_STATE_INIT) {
			RTE_VNI_DEBUG_TRACE("received stop request for attaching netdev interface\n");
			return 0;
		}

		recv_status = recvmsg(nl_data->sock_fd, &(nl_data->recv_msg), 0);
	}
	if (recv_status < 0)
		return (int)recv_status;

	return 1;
}

int rte_ndc_close_confirm(rte_atomic32_t *state)
{
	return rte_atomic32_read(state) == NDC_STATE_CLOSE;
}

void rte_ndc_session_init(rte_atomic32_t *state)
{
	rte_atomic32_set(state, NDC_STATE_INIT);
}

void rte_ndc_session_stop(rte_atomic32_t *state)
{
	rte_atomic32_set(state, NDC_STATE_STOP);
}

/*
 * Create a netlink device client thread
 *
 * inf_name: interface name for netdev. for a multiple port
 *	configuraiton, two naming choices: 
 *	1. base-name with port-id as suffix
 *	2. each port has its own interface name; use token "," to
 *		separate inteface name
 *
 * num_of_ports: number of netdev interface to be created for
 *	port #0 to port #(num_of_ports -1)
 *
 * dest_pid: application pid
 *
 * done: synchronous between this thread and calling thread
 *	{ NDC_STATE_NONE, NDC_STATE_INIT, NDC_STATE_STOP
 *		NDC_STATE_CLOSE }
 *
 */
rte_ndc_status rte_ndc_client(char *inf_name, int num_of_ports,
	pid_t dest_pid, rte_atomic32_t *done)
{
	int status;
	struct netlink_data *nl_data;
	int sock_fd;
	netdev_cmd_info *recv_netdev_cmd, *send_netdev_cmd;

	if (num_of_ports > MAXI_MANAGEMENT_DEV) {
		RTE_VNI_DEBUG_TRACE("Too many NDC interface request (req: %d maxi:%d)\n",
			num_of_ports, MAXI_MANAGEMENT_DEV);
		return ndc_no_inf;
	}

	nl_data = ndc_nl_data_alloc(inf_name, num_of_ports, dest_pid);
	if (nl_data == 0)
		return ndc_no_inf;

	send_netdev_cmd = SEND_CMD(nl_data);
	recv_netdev_cmd = RECV_CMD(nl_data);

	status = ndc_inf_proc(nl_data, num_of_ports, 1); /* open new inf */
	if (status < 0)
		RTE_VNI_DEBUG_TRACE("Fail to open an INF from application %d\n", dest_pid);

	/* set socket as non-blocking to listen any netdev callback from kernel */
	if (set_socket_nonblocking(nl_data->sock_fd) < 0) {
		RTE_VNI_DEBUG_TRACE("Unable to put socket in non-blocking mode (errno=%d)\n", errno);
		close(nl_data->sock_fd);
		return ndc_error;
	}

	rte_atomic32_set(done, NDC_STATE_INIT);
	while ((status = ndc_receive_loop(nl_data, done))) {
		if (status < 0) {
			RTE_VNI_DEBUG_TRACE("recvmsg fails with error: %d\n", errno);
			break;
		}
		RTE_VNI_DEBUG_TRACE("received cmd(%d %s)\n", recv_netdev_cmd->cmd,
			cmd_name(recv_netdev_cmd->cmd));

		/* dispatch user-space API according to msg */
		ndc_recv_to_send(send_netdev_cmd, recv_netdev_cmd);
		if (is_ethtool_cmd(recv_netdev_cmd->cmd))
			ethtool_cmd_proc(recv_netdev_cmd, send_netdev_cmd);
		else {
			if (is_netdev_cmd(recv_netdev_cmd->cmd))
				netdev_cmd_proc(recv_netdev_cmd, send_netdev_cmd);
			else {
				if (is_manage_cmd(recv_netdev_cmd->cmd)) {
					ndc_manage_cmd_proc(send_netdev_cmd);
				}
				else
					RTE_VNI_DEBUG_TRACE("Invalid netdev cmd (%d) \n", recv_netdev_cmd->cmd); /*TODO log */
			}
		}
		
		status = ndc_send_cmd(nl_data, send_netdev_cmd);
		RTE_VNI_DEBUG_TRACE("Sending result back to kernel\n");
	}
	/* send del_inf commdn to close out open netdev interface  */
	status = ndc_inf_proc(nl_data, num_of_ports, 0); /* close inf */
	if (status < 0)
		RTE_VNI_DEBUG_TRACE("Fail to close an INF from application %d\n", dest_pid);

	RTE_VNI_DEBUG_TRACE("close out netlink with %s (pid=%d)\n", dest_pid==0?"kernel":"user-space", dest_pid);
	close(nl_data->sock_fd);
	ndc_nl_data_release(nl_data);
	rte_atomic32_set(done, NDC_STATE_CLOSE);
	return ndc_sucess;
}
