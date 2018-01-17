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
#include <rte_version.h>
#if RTE_VERSION < RTE_VERSION_NUM(16, 11, 0, 0)
#include <rte_config.h>
#endif
#include <rte_pci.h>
#include <rte_ethdev.h>
#include <rte_dpdm.h>

struct rte_eth_dev_ex rte_eth_devices_ex[RTE_MAX_ETHPORTS];
struct rte_dev_drv_info rte_drv_set[RTE_MAXI_EXT_DRV_SET];
static int rte_drv_pair = 0;

void
rte_eth_get_devs_by_port(port_t port_id, struct rte_eth_dev_ex **dev_ex, struct rte_eth_dev **dev)
{
	*dev_ex = &rte_eth_devices_ex[port_id];
	*dev = &rte_eth_devices[port_id];
}

void rte_eth_get_pci_addr(port_t port_id, struct rte_pci_addr *addr)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	struct rte_pci_device *pci_dev = DPDM_DEV_TO_PCI(dev);

	memcpy(addr, &pci_dev->addr, sizeof(struct rte_pci_addr));
}
struct rte_eth_dev_ex*
rte_eth_get_devex_by_dev(struct rte_eth_dev *dev)
{
	return &rte_eth_devices_ex[dev - rte_eth_devices];
}

void
rte_dev_add_ext_drv(const char *base_name, drv_init init)
{
	int i;

	if (rte_drv_pair == 0) {
		for(i = 0; i < RTE_MAXI_EXT_DRV_SET; i++)
			memset(&rte_drv_set[i], 0, sizeof(struct rte_dev_drv_info));
	}
	rte_drv_set[rte_drv_pair].base_name = base_name;
	rte_drv_set[rte_drv_pair].init_op = init;
	RTE_LOG(INFO, EAL,"add extended driver: %s \n", base_name);
	rte_drv_pair += 1;
}

int rte_dev_probe_extension(port_t num_of_ports)
{
	struct rte_driver *rte_drv;
	int i, match = 0;

	uint8_t j;
	const char *base_name;

	/* 
	 * From base name find the owning rte_eth_dev,
	 * then run init to bind the extension operators
	 */
	for (i = 0; i < rte_drv_pair; i++)
		RTE_LOG(DEBUG, EAL,"ext %d: name = %s\n", i, rte_drv_set[i].base_name);

	for (i = 0; i < num_of_ports; i++) {
#if RTE_VERSION >= RTE_VERSION_NUM(17, 5, 0, 0)
		base_name = rte_eth_devices[i].device->driver->name;
#else
		base_name =  rte_eth_devices[i].driver->pci_drv.driver.name;
#endif
		for(j = 0; j < rte_drv_pair; j++) {
			if (strcmp(base_name, rte_drv_set[j].base_name) == 0) {
				(*rte_drv_set[j].init_op)(&rte_eth_devices[i]);
				match ++;
				break;
			}
		}
	}
	RTE_LOG(DEBUG, EAL, "%d ports assigned with extended driver\n", match);

	return 0;
}

void rte_set_mask(port_t port_id, uint8_t seq, uint16_t vf_id, unsigned short s_mask)
{
	rte_eth_devices_ex[port_id].vf_mask[vf_id].s_mask[seq] = s_mask;
}

void rte_set_qmask(port_t port_id, uint8_t seq, unsigned short s_mask)
{
	rte_eth_devices_ex[port_id].queue_mask.s_mask[seq] = s_mask;
}

uint64_t rte_get_mask(port_t port_id, uint16_t vf_id)
{
	return rte_eth_devices_ex[port_id].vf_mask[vf_id].q_mask;
}

uint64_t rte_get_qmask(port_t port_id)
{
	return rte_eth_devices_ex[port_id].queue_mask.q_mask;
}

