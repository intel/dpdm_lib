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
#ifndef _ETHDEV_ETHTOOL_TYPES_H_
#define _ETHDEV_ETHTOOL_TYPES_H_

#ifndef __be16
#define __be16 uint16_t
#endif

#ifndef __be32
#define __be32 uint32_t
#endif
/* User-space ethtool data structure */

/*
 * Driver and device information
 */
struct rte_dev_ethtool_drvinfo {
	uint32_t	cmd;
	char	driver[32];
	char	version[32];
	char	fw_version[32];
	char	bus_info[32];
	char	erom_version[32];
	char	reserved2[12];
	uint32_t	n_priv_flags;
	uint32_t	n_stats;
	uint32_t	testinfo_len;
	uint32_t	eedump_len;
	uint32_t	regdump_len;
};

/*
 * Link control and status
 */
struct rte_dev_ethtool_cmd {
	uint32_t	cmd; /* cmd # */
	uint32_t	supported; /* link mode support flag */
	uint32_t	advertising; /* link mode advertized flag */
	uint16_t	speed; /* link speed low-16 bits*/
	uint8_t		duplex; /* duplex mode */
	uint8_t		port; /* connect type */
	uint8_t		phy_address; /* MDIO address */
	uint8_t		transceiver;
	uint8_t		autoneg; /* autonegotiation */
	uint8_t		mdio_support; /* MDIO protocols supported */
	uint32_t	maxtxpkt;
	uint32_t	maxrxpkt;
	uint16_t	speed_hi;
	uint8_t		eth_tp_mdix;
	uint8_t		eth_tp_mdix_ctrl;
	uint32_t	lp_advertising; /* link-mode and link-feature advertised */
	uint32_t	reserved[2];
};

/*
 * Wake-On-Lan control parameter
 */
struct rte_dev_ethtool_wolinfo {
	uint32_t	cmd; /* cmd # */
	uint32_t	supported;	/* mode (wol) support mask */
	uint32_t	wolopts;	/* mode (wol) enable mask */
	uint8_t		sopass[6];	/* SecureOn(tm) password; meaningful only if %WAKE_MAGICSECURE*/
};

/*
 * Device register access
 */
struct rte_dev_ethtool_reg {
	uint32_t	cmd;
	uint32_t	version;
	uint32_t	len;
	uint8_t	data[0];
};

/*
 * EEPROM get/set parameters
 */
struct rte_dev_ethtool_eeprom {
	uint32_t	cmd;
	uint32_t	magic;
	uint32_t	offset;
	uint32_t	len;
	uint8_t	data[0];
};

/**
 * Rx/Tx interrupt coalesce control parameter
 */
struct rte_dev_ethtool_coalesce {
	uint32_t	cmd;
	uint32_t	rx_coalesce_usecs;
	uint32_t	rx_max_coalesced_frames;
	uint32_t	rx_coalesce_usecs_irq;
	uint32_t	rx_max_coalesced_frames_irq;
	uint32_t	tx_coalesce_usecs;
	uint32_t	tx_max_coalesced_frames;
	uint32_t	tx_coalesce_usecs_irq;
	uint32_t	tx_max_coalesced_frames_irq;
	uint32_t	stats_block_coalesce_usecs;
	uint32_t	use_adaptive_rx_coalesce;
	uint32_t	use_adaptive_tx_coalesce;
	uint32_t	pkt_rate_low;
	uint32_t	rx_coalesce_usecs_low;
	uint32_t	rx_max_coalesced_frames_low;
	uint32_t	tx_coalesce_usecs_low;
	uint32_t	tx_max_coalesced_frames_low;
	uint32_t	pkt_rate_high;
	uint32_t	rx_coalesce_usecs_high;
	uint32_t	rx_max_coalesced_frames_high;
	uint32_t	tx_coalesce_usecs_high;
	uint32_t	tx_max_coalesced_frames_high;
	uint32_t	rate_sample_interval;
};

/*
 * Rx/Tx ring (# of descriptors) control 
 */
struct rte_dev_ethtool_ringparam {
	uint32_t	cmd;
	uint32_t	rx_max_pending;
	uint32_t	rx_mini_max_pending;
	uint32_t	rx_jumbo_max_pending;
	uint32_t	tx_max_pending;
	uint32_t	rx_pending;
	uint32_t	rx_mini_pending;
	uint32_t	rx_jumbo_pending;
	uint32_t	tx_pending;
};

/*
 * Device pause frame (flow control) parameters
 */
struct rte_dev_ethtool_pauseparam {
	uint32_t	cmd;
	uint32_t	autoneg;
	uint32_t	rx_pause;
	uint32_t	tx_pause;
};

/*
 * Ethtool test definition
 */
struct rte_dev_ethtool_test {
	uint32_t	cmd;
	uint32_t	flags;
	uint32_t	reserved;
	uint32_t	len;
	uint64_t	data[0];
};

enum rte_dev_ethtool_phys_id_state {
	ETHTOOL_ID_INACTIVE,
	ETHTOOL_ID_ACTIVE,
	ETHTOOL_ID_ON,
	ETHTOOL_ID_OFF
};

/*
 * Ethtool statistics: using data[0] as a flexible
 * statistics data structure (n_stats *uint32_t items)
 */
struct rte_dev_ethtool_stats {
	uint32_t	cmd;
	uint32_t	n_stats;
	uint64_t	data[0];
};

/*
 * Ethnet packets header
 */
struct rte_dev_ethhdr {
	unsigned char   h_dest[6];       /* destination mac addr */
	unsigned char   h_source[6];     /* source mac addr    */
	__be16          h_proto;         /* packet type ID field */
};

union rte_dev_ethtool_flow_union {
	struct rte_dev_ethhdr	ether_spec;
	uint8_t					hdata[52];
};

struct rte_dev_ethtool_flow_ext {
	uint8_t		padding[2];
	unsigned char	h_dest[6];		/* dest mac addr */
	__be16		vlan_etype;
	__be16		vlan_tci;
	__be32		data[2];
};

struct rte_dev_ethtool_rx_flow_spec {
	uint32_t		flow_type;
	union rte_dev_ethtool_flow_union h_u;
	struct rte_dev_ethtool_flow_ext h_ext;
	union rte_dev_ethtool_flow_union m_u;
	struct rte_dev_ethtool_flow_ext m_ext;
	uint64_t		ring_cookie;
	uint32_t		location;
};

/*
 * Flow classification rules configuration
 */
struct rte_dev_ethtool_rxnfc {
	uint32_t				cmd; /* get/set flow classification */
	uint32_t				flow_type;
	uint64_t				data;
	struct rte_dev_ethtool_rx_flow_spec	fs; /* flow classification rule */
	uint32_t				rule_cnt;
	uint32_t				rule_locs[0];
};

/*
 * Firmware access parameters
 */
struct rte_dev_ethtool_flash {
	uint32_t	cmd;
	uint32_t	region;
	char	data[128];	/* maximu flash file name */
};

/*
 * Configure # of network channels
 */
struct rte_dev_ethtool_channels {
	uint32_t	cmd;
	uint32_t	max_rx;
	uint32_t	max_tx;
	uint32_t	max_other;
	uint32_t	max_combined;
	uint32_t	rx_count;
	uint32_t	tx_count;
	uint32_t	other_count;
	uint32_t	combined_count;
};

/*
 * Data structure for device dump.
 */
struct rte_dev_ethtool_dump {
	uint32_t	cmd;
	uint32_t	version;
	uint32_t	flag;
	uint32_t	len;
	uint8_t	data[0];
};

/*
 * Time-stamp information
 */
struct rte_dev_ethtool_ts_info {
	uint32_t	cmd;
	uint32_t	so_timestamping;
	int32_t		phc_index;
	uint32_t	tx_types;
	uint32_t	tx_reserved[3];
	uint32_t	rx_filters;
	uint32_t	rx_reserved[3];
};

/*
 * EEPROM information for plug-in modules
 */
struct rte_dev_ethtool_modinfo {
	uint32_t   cmd;
	uint32_t   type;
	uint32_t   eeprom_len;
	uint32_t   reserved[8];
};

/*
 * EEE (Energy Efficient Ethernet) information
 */
struct rte_dev_ethtool_eee {
	uint32_t	cmd;
	uint32_t	supported;
	uint32_t	advertised;
	uint32_t	lp_advertised;
	uint32_t	eee_active;
	uint32_t	eee_enabled;
	uint32_t	tx_lpi_enabled;
	uint32_t	tx_lpi_timer;
	uint32_t	reserved[2];
};

struct rte_dev_ethtool_tunable {
	uint32_t	cmd;
	uint32_t	id;
	uint32_t	type_id;
	uint32_t	len;
	void	*data[0];
};

struct rte_dev_reg_set {
	uint32_t base_addr;
	int	count;
	int stride;
};

#endif /* _ETHDEV_ETHTOOL_TYPES_H_ */
