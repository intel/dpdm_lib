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
#ifndef _RTE_MANAGE_H_
#define _RTE_MANAGE_H_

#include <rte_version.h>

#if RTE_VERSION >= RTE_VERSION_NUM(17, 11, 0, 0)
#include <rte_bus_pci.h>
#else
#include <rte_pci.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RTE_MAXI_EXT_DRV_SET				16

#define SRIOV_FIRST_QUEUE(dev, vf_id)		(vf_id*(dev)->data->sriov.nb_q_per_pool)
#define SRIOV_LAST_QUEUE(dev, vf_id)		((vf_id+1)*(dev)->data->sriov.nb_q_per_pool - 1)
#define PCI_DRV_TO_ETH_DRV(dr)				container_of(dr, struct eth_driver, pci_drv)

#ifndef RTE_DEV_TO_PCI
#define RTE_DEV_TO_PCI(ptr)	container_of(ptr, struct rte_pci_device, device)
#endif

#if RTE_VERSION >= RTE_VERSION_NUM(17, 2, 0, 0)
#define DPDM_DEV_TO_PCI(dev) RTE_DEV_TO_PCI(dev->device)
#else
#define DPDM_DEV_TO_PCI(dev) dev->pci_dev
#endif

#define RET_ERR_ON_VF_ID_CHECK(dev, pci_dev, vf_id) \
		pci_dev = DPDM_DEV_TO_PCI(dev); \
		if (vf_id >= (int)pci_dev->max_vfs) \
			return -EINVAL

#define RET_ERR_ON_VF_ENABLE(port_id) \
		if (rte_netdev_is_vf_enabled(port_id) <= 0 ) \
			return -EINVAL

/* macro for registering a extended driver */
#define RTE_PMD_EX_REGISTER_PCI(nm, dev_init) \
static void __attribute__((constructor, used)) pciinitfn_ ##nm (void); \
static void pciinitfn_ ##nm(void) \
{\
	rte_dev_add_ext_drv(RTE_STR(nm), dev_init);\
} \
RTE_PMD_EXPORT_NAME(nm, __COUNTER__)

#if RTE_VERSION < RTE_VERSION_NUM(17, 11, 0, 0)
typedef uint8_t port_t;
#else
typedef uint16_t port_t;
#endif

struct rte_eth_dev_ex;
struct rte_eth_dev;
struct rte_pci_driver;

typedef int (*drv_init)(struct rte_eth_dev*);

struct rte_dev_drv_info {
	const char *base_name;
	drv_init	init_op;
};

/*
 * Fetch base driver and extended driver through port-id
 */
void rte_eth_get_devs_by_port(port_t port_id, struct rte_eth_dev_ex **dev_ex, struct rte_eth_dev **dev);

/*
 * Fetch pci device address from port-id
 */
void rte_eth_get_pci_addr(port_t port_id, struct rte_pci_addr *addr);

/*
 * Fetch the extended driver data structure from base driver data structure
 */
struct rte_eth_dev_ex *rte_eth_get_devex_by_dev(struct rte_eth_dev *dev);

/*
 * Record necessary information for extended driver to be associated
 * with the base driver
 */
void rte_dev_add_ext_drv(const char *base_name, drv_init init);

/*
 * Probing all registered extended drivers, and run init routine
 */
int rte_dev_probe_extension(port_t num_of_ports);

/*
 * Set vf_mask of extended driver data structure
 */
void rte_set_mask(port_t port_id, uint8_t seq, uint16_t vf_id, unsigned short s_mask);

/*
 * Set vf_mask of extended driver data structure
 */
void rte_set_qmask(port_t port_id, uint8_t seq, unsigned short s_mask);

/*
 * Retrieve vf_mask of extended driver data structure
 */
uint64_t rte_get_mask(port_t port_id, uint16_t vf_id);

/*
 * Retrieve vf_mask of extended driver data structure
 */
uint64_t rte_get_qmask(port_t port_id);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_MANAGE_H_ */
