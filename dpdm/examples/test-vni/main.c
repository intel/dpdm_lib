/*
 *
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <pthread.h>

#include <rte_atomic.h>
#include <rte_ethdev.h>
#include <rte_dpdm.h>

#include "ndc.h"

typedef struct _ndc_thread_info {
	pthread_t thd_id;
	int num_of_ports;
	char inf_name[MAXI_INTERFACE_NAME];
	pid_t dest_pid;
	rte_ndc_status status;
	rte_atomic32_t *done;
} ndc_thread_info;

static void*
recv_thread(void* arg)
{
	ndc_thread_info *thd_info = arg;
	pid_t dest_pid = 0; /* 0: for kernel */

	rte_ndc_session_init(thd_info->done);	
	thd_info->status = rte_ndc_client(thd_info->inf_name,
		thd_info->num_of_ports,
		dest_pid,
		thd_info->done);

	return NULL;
}

/*
 */
static void vni_usage(const char *prgname)
{
	printf("%s [EAL options] -- [-k] [-u app-pid] -f interface-name-base\n"
	       "  -k creating Netlink between this application and kernel\n"
		   "  -u app-id: creating Netlink between this application and another user-space app\n"
		   "  -o lcore-id: core assign to vni thread\n"
		   "  -f interface-base-name: base-name string for proxy netdev interface names\n", prgname);
}

static char short_options[] =
	"k" "f:" "o:" "u:";

static struct option lgopts [] = {
	{NULL, 0, NULL, 0}};

static int nl_kernel = 1;
static int vni_core = 4;
static pid_t app_pid = 0;
static char *inf_name = NULL;

static int
vni_parse_args(int argc, char **argv)
{
	int opt, ret;
	char **argvopt;
	int option_index;
	char *dummy = NULL;
	char *prgname = argv[0];

	argvopt = argv;

	while ((opt = getopt_long(argc, argvopt, short_options,
				  lgopts, &option_index)) != EOF) {

		switch (opt) {
		/* setup netlink between application and kernel */
		case 'k':
			nl_kernel = 1;
			app_pid = 0;
			break;

		/* setup netlink between application and another user-space app */
		case 'u':
			app_pid = atoi(optarg);
			nl_kernel = 0;
			break;

		/* interface name for netdev */
		case 'f':
			inf_name = optarg;
			break;

		/* interface name for netdev */
		case 'o':
			vni_core = strtol((const char*)optarg, &dummy, 10);
			break;

		/* long options */
		default:
			vni_usage(prgname);
			return -1;
		}
	}

	if (optind >= 0)
		argv[optind-1] = prgname;

	ret = optind-1;
	optind = 0; /* reset getopt lib */
	return ret;
}

int main(int argc, char** argv) {
	int ret;
	ndc_thread_info thread_info;
	pthread_attr_t attr;
	cpu_set_t cpuset;
	rte_atomic32_t done;
	struct rte_dev_ethtool_drvinfo drv_info;
	const char *def_inf_name= "dpdk";
	int status;

	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_panic("Cannot init EAL\n");

	argc -= ret;
	argv += ret;

	if (rte_dev_probe_extension(rte_eth_dev_count()))
		printf("Fail to connect extended drivers \n");

	vni_parse_args(argc, argv);
	if (inf_name == NULL) {
		printf("No interface name using \"dpdk\" instead\n");
		strncpy(thread_info.inf_name, def_inf_name, strlen(def_inf_name)+1);
	} else {
		printf("Interface name is %s\n", inf_name);
		strncpy(thread_info.inf_name, inf_name, strlen(inf_name)+1);
	}
	
	status = rte_ethtool_get_drvinfo(0, &drv_info);
	if (status < 0) {
		printf(" fail on ethtool_get_drvinfo with errro code %d\n", status);
		return 0;
	}

	thread_info.num_of_ports = rte_eth_dev_count();
	if (nl_kernel)
		thread_info.dest_pid = 0;
	else
		thread_info.dest_pid = app_pid;

	thread_info.done = &done;
	thread_info.status = 0;

	/* Setup CPU affinity */
	CPU_ZERO(&cpuset);
	CPU_SET(vni_core, &cpuset);
	pthread_attr_init(&attr);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
	
	status = pthread_create(&thread_info.thd_id,
		 &attr, &recv_thread, &thread_info);
	if (status) {
		printf("Fail to create thread with error %d\n", status);
		return 0;
	}

	printf("type c to stop the process ...\n");
	while (getc(stdin) != 'c') {
		usleep(100);
	}
	
	usleep(100);
	rte_ndc_session_stop(&done);

	printf("waiting for ndc to close ...\n");
	while (!rte_ndc_close_confirm(&done))
		usleep(100);

	if (thread_info.status != ndc_sucess) {
		printf("Netlink fail with error code = %d\n", thread_info.status);
	}
	return 0;
}
