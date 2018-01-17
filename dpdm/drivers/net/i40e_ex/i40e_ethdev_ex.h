/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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
#ifndef _I40E_ETHDEV_EX_H_
#define _I40E_ETHDEV_EX_H_
#include <rte_version.h>

#define check_flag(flag, bit) ((flag & bit) == bit)

#ifdef __cplusplus
extern "C" {
#endif
		
int i40e_add_rm_all_vlan_filter(struct i40e_vsi *vsi, uint8_t add);
int i40e_vsi_set_tx_loopback(struct i40e_vsi *vsi, uint8_t on, uint8_t vlan_anti_spoof_on);
int i40e_vsi_rm_mac_filter(struct i40e_vsi *vsi);
int i40e_vsi_restore_mac_filter(struct i40e_vsi *vsi, uint16_t vlan_anti_spoof_on);

#if RTE_VERSION < RTE_VERSION_NUM(17, 5, 0, 0)
int i40e_find_all_vlan_for_mac(struct i40e_vsi *vsi,
			   struct i40e_macvlan_filter *mv_f,
			   int num, struct ether_addr *addr);
int i40e_remove_macvlan_filters(struct i40e_vsi *vsi,
			    struct i40e_macvlan_filter *filter,
			    int total);
void i40e_set_vlan_filter(struct i40e_vsi *vsi,
		     uint16_t vlan_id, uint8_t vlan_anti_spoof_on, bool on);
int i40e_add_macvlan_filters(struct i40e_vsi *vsi,
			 struct i40e_macvlan_filter *filter,
			 int total);
#endif

void i40e_store_vlan_filter(struct i40e_vsi *vsi, uint16_t vlan_id, bool on);
int i40e_set_vf_untag_vlan(struct i40e_vsi *vsi, uint8_t drop);

#ifdef __cplusplus
}
#endif


#endif /* _I40E_ETHDEV_EX_H_ */