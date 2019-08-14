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
#ifndef _RTE_ETHTOOL_OPS_H_
#define _RTE_ETHTOOL_OPS_H_

#ifdef __cplusplus
extern "C" {
#endif

struct netdev_priv_data;
#include <inttypes.h>
#include <rte_ethdev.h>
#include <rte_dev_info.h>
#include <rte_manage.h>
#include <rte_ethtool_types.h>
#if RTE_VERSION >= RTE_VERSION_NUM(19, 8, 0, 0) 
#undef ETHER_MAX_VLAN_ID
#define ETHER_MAX_VLAN_ID RTE_ETHER_MAX_VLAN_ID
#undef ETHER_ADDR_LEN
#define ETHER_ADDR_LEN RTE_ETHER_ADDR_LEN
#define is_valid_assigned_ether_addr rte_is_valid_assigned_ether_addr
#define ether_addr_copy rte_ether_addr_copy
typedef struct rte_ether_addr ether_addr_t;
#else
typedef struct ether_addr ether_addr_t;
#endif
// RTE Ethtool APIs
/**
 * Retrieve netdev private data from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param netdev_data
 *    The pointer for retrieve netdev data
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_netdev_data(port_t port_id, struct netdev_priv_data *netdev_data);

/**
 * Set netdev private data for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param netdev_data
 *    The pointer for the input netdev data
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_netdev_data(port_t port_id, struct netdev_priv_data *netdev_data);

/**
 * Retrieve ethtool setting from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param cmd
 *    The pointer for ethtool cmd data structure
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_settings(port_t port_id, struct rte_dev_ethtool_cmd *cmd);

/**
 * Set ethtool parameters for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param cmd
 *    The pointer for ethtool cmd data structure
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_settings(port_t port_id, struct rte_dev_ethtool_cmd *cmd);

/**
 * Retrieve device driver information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer for driver infomation data structure
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_drvinfo(port_t port_id, struct rte_dev_ethtool_drvinfo* info);

/**
 * Retrieve device register length from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_reg_len(port_t port_id);

/**
 * Retrieve device register value and other information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer for register infomation data structure
 *	
 * @param data
 *    The pointer for register value
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_reg(port_t port_id, struct rte_dev_ethtool_reg *info, void *data);

/**
 * Retrieve device wake-on-lan setting from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer for wake-on-lan parameters
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_wol(port_t port_id, struct rte_dev_ethtool_wolinfo *info);

/**
 * Set device wake-on-lan parameter for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer wake-on-lan parameters
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_wol(port_t port_id, struct rte_dev_ethtool_wolinfo *info);

/**
 * Retrieve device driver message level from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_msglevel(port_t port_id);

/**
 * Set device driver message level from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param level
 *    New driver output message level (range device-dependent)
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_msglevel(port_t port_id, uint32_t level);

/**
 * Apply nway resete on the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 *	
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_nway_reset(port_t port_id);

/**
 * Retrieve link status from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (1) link-up and sucessful.
 *   - (0) link-down and successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_link(port_t port_id);

/**
 * Retrieve device eeprom length (in bytes) from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_eeprom_len(port_t port_id);

/**
 * Retrieve device eeprom info and eeprom content from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer for eeprom info.
 *
 * @param eeprom
 *    The port for return eeprom data.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom);

/**
 * Set device eeprom data with input eeprom configuraiton information for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param info
 *    The pointer for eeprom info.
 *
 * @param eeprom
 *    The port for new eeprom data.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *info, uint8_t *eeprom);

/**
 * Retrieve device interrupt coelesce parameters from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for interrupt coelesce parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_coalesce(port_t port_id, struct rte_dev_ethtool_coalesce *param);

/**
 * Set device interrupt coelesce parameters for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for interrupt coelesce parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_coalesce(port_t port_id, struct rte_dev_ethtool_coalesce *param);

/**
 * Retrieve device descriptor rings (both Rx and Tx) configuration from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for descriptor ring parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_ringparam(port_t port_id, struct rte_dev_ethtool_ringparam *param);

/**
 * Set device descriptor rings (both Rx and Tx) configuration for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for descriptor ring parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_ringparam(port_t port_id, struct rte_dev_ethtool_ringparam *param);

/**
 * Retrieve device pause frame configuration from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for pause frame configuration parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_pauseparam(port_t port_id, struct rte_dev_ethtool_pauseparam *param);

/**
 * Set device pause frame configuration for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param param
 *    The pointer for pause frame configuration parameters.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_pauseparam(port_t port_id, struct rte_dev_ethtool_pauseparam *param);

/**
 * Retrieve device driver self-test information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param test
 *    The pointer for self-test configuration parameters.
 *
 * @param ctx
 *    The 64-bit on/off test bits.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_self_test(port_t port_id, struct rte_dev_ethtool_test *test, uint64_t*ctx);

/**
 * Retrieve device driver string information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param stringset
 *    Type of strings (statistic, private flag etc) to be retrieved
 *
 * @param ctx
 *    The pointer for the return string.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_strings(port_t port_id, uint32_t stringset, uint8_t *ctx);

/**
 * Set device physical id for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param stringset
 *    Type of strings (statistic, private flag etc) to be retrieved
 *
 * @param ctx
 *    The pointer for the return string.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_phys_id(port_t port_id, enum rte_dev_ethtool_phys_id_state state);

/**
 * Retrieve device statistic information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param stat
 *    The pointer to statistics information
 *
 * @param ctx
 *    The pointer for return statistics value.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_ethtool_stats(port_t port_id, struct rte_dev_ethtool_stats *stat, uint64_t*ctx);

/**
 * Begin of an ethtool session
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_begin(port_t port_id);

/**
 * End of an ethtool session
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_complete(port_t port_id);

/**
 * Retrieve device-specific private flag from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_priv_flags(port_t port_id);

/**
 * Set device-specific private flag for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param ctx
 *    A 32-bit represents 32 private features (1:on , 0: off).
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_priv_flags(port_t port_id, uint32_t ctx);

/**
 * Retrieve specific string count from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param val
 *    The string type.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_sset_count(port_t port_id, int val);

/**
 * Retrieve Rx flow classification rules. from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param rxnfc
 *    The pointer for Rx flow classification data structure.
 *
 * @param rule_locs
 *    The pointer for rule buffer.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_rxnfc(port_t port_id, struct rte_dev_ethtool_rxnfc *rxnfc, uint32_t *rule_locs);

/**
 * Set Rx flow classification rules. from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param rxnfc
 *    The pointer for Rx flow classification data structure.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_rxnfc(port_t port_id, struct rte_dev_ethtool_rxnfc *rxnfc);

/**
 * Write a firmware image to device's flash memory for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param flash
 *    The pointer for the flash image information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_flash_device(port_t port_id, struct rte_dev_ethtool_flash *flash);

/**
 * Reset the device, as specified by a bitmask of flags
 * from &enum ethtool_reset_flags, from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param ctx
 *    The bitmask flag.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_reset(port_t port_id, uint32_t *ctx);

/**
 * Retrieve Rx flow hash key from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_rxfh_key_size(port_t port_id);

/**
 * Retrieve RX flow hash indirection table from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_rxfh_indir_size(port_t port_id);

/**
 * Retrieve the contents of the RX flow hash indirection table, hash key
 *	and/or hash function from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param indir
 *    The address pointer for returning configuration data structure.
 *
 * @param key
 *    The address pointer for the key.
 *
 * @param hfunc
 *    The address pointer for the hash type.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_rxfh(port_t port_id, uint32_t *indir, uint8_t *key, uint8_t *hfunc);

/**
 * Set the contents of the RX flow hash indirection table, hash key
 *	and/or hash function from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param indir
 *    The address pointer for returning configuration data structure.
 *
 * @param key
 *    The address pointer for the key.
 *
 * @param hfunc
 *    The address pointer for the hash type.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_rxfh(port_t port_id, uint32_t *indir, uint8_t *key, uint8_t *hfunc);

/**
 * Retrieve number of channels from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for ethtool dump information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_channels(port_t port_id, struct rte_dev_ethtool_channels *chan);

/**
 * Set number of channels for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for ethtool dump information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_channels(port_t port_id, struct rte_dev_ethtool_channels *chan);

/**
 * Retrieve dump flag indicating dump information from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for ethtool dump information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_dump_flag(port_t port_id, struct rte_dev_ethtool_dump *data);

/**
 * Retrieve dump data from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for ethtool dump information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_dump_data(port_t port_id, struct rte_dev_ethtool_dump *data, void *ctx);

/**
 * Set dump data for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for ethtool dump information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_dump(port_t port_id, struct rte_dev_ethtool_dump *data);

/**
 * Retrieve the time stamping and PTP hardware clock capabilities from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for time steamp and PTP information.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_ts_info(port_t port_id, struct rte_dev_ethtool_ts_info *data);

int rte_ethtool_get_module_info(port_t port_id, struct rte_dev_ethtool_modinfo *data);
/**
 * Retrieve module eeprom from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for module eeprom.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_module_eeprom(port_t port_id, struct rte_dev_ethtool_eeprom *data, uint8_t *ctx);

/**
 * Retrieve energy-efficiency configuration from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for the port energy-efficiency configuration.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_eee(port_t port_id, struct rte_dev_ethtool_eee *data);

/**
 * Set energy-efficiency configuration for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param data
 *    The pointer for the port energy-efficiency configuration.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_eee(port_t port_id, struct rte_dev_ethtool_eee *data);

int rte_ethtool_get_tunable(port_t port_id, const struct rte_dev_ethtool_tunable *data, void *ctx);
int rte_ethtool_set_tunable(port_t port_id, const struct rte_dev_ethtool_tunable *data, const void *ctx);
/**
 * Retrieve queue-specific interrupt coalesce configuraiton from the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param ctx
 *    The queue index.
 *
 * @param data
 *    The pointer to the coalesce configuration.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_get_per_queue_coalesce(port_t port_id, uint32_t ctx, struct rte_dev_ethtool_coalesce *data);

/**
 * Se queue-specific interrupt coalesce configuraiton for the specified port
 *
 * @param port
 *    The port identifier of the Ethernet device.
 *
 * @param ctx
 *    The queue index.
 *
 * @param data
 *    The pointer to the coalesce configuration.
 *
 * @return
 *   - (0) if successful.
 *   - (-ENODEV) if *port* invalid.
 *   - (-EINVAL) if bad parameter.
 */
int rte_ethtool_set_per_queue_coalesce(port_t port_id, uint32_t ctx, struct rte_dev_ethtool_coalesce *data);

#ifdef __cplusplus
}
#endif

#endif // _RTE_ETHTOOL_OPS_H_
