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
#ifndef _RTE_IF_FLAGS_H_
#define _RTE_IF_FLAGS_H_

#define ETH_ADDR_LEN 6
/* RTE_NETIF re-use defintion in netdev.h */
#define RTE_NETIF_F_SG              1       /* Scatter/gather IO. */
#define RTE_NETIF_F_IP_CSUM         2       /* Can checksum TCP/UDP over IPv4. */
#define RTE_NETIF_F_NO_CSUM         4       /* Does not require checksum. F.e. loopack. */
#define RTE_NETIF_F_HW_CSUM         8       /* Can checksum all the packets. */

#define RTE_NETIF_F_IPV6_CSUM       0x10      /* Can checksum TCP/UDP over IPV6 */
#define RTE_NETIF_F_HIGHDMA         0x20      /* Can DMA to high memory. */
#define RTE_NETIF_F_FRAGLIST        0x40      /* Scatter/gather IO. */
#define RTE_NETIF_F_HW_VLAN_TX      0x80     /* Transmit VLAN hw acceleration */

#define RTE_NETIF_F_HW_VLAN_RX      0x100     /* Receive VLAN hw acceleration */
#define RTE_NETIF_F_HW_VLAN_FILTER  0x200     /* Receive filtering on VLAN */
#define RTE_NETIF_F_VLAN_CHALLENGED 0x400    /* Device cannot handle VLAN packets */
#define RTE_NETIF_F_GSO             0x800    /* Enable software GSO. */



#define RTE_NETIF_F_NETNS_LOCAL     0x2000    /* Does not change network namespaces */
#define RTE_NETIF_F_GRO             0x4000   /* Generic receive offload */
#define RTE_NETIF_F_LRO             0x8000   /* large receive offload */

/* the GSO_MASK reserves bits 16 through 23 */
#define RTE_RTE_NETIF_F_SCTP_CSUM	0x2000000       /* SCTP checksum offload */

/* Segmentation offload features */
#define RTE_NETIF_F_GSO_SHIFT       16
#define RTE_NETIF_F_GSO_MASK        0x00ff0000
#define RTE_NETIF_F_TSO             0x10000
#define RTE_NETIF_F_UFO             0x20000
#define RTE_NETIF_F_GSO_ROBUST      0x40000
#define RTE_NETIF_F_TSO_ECN         0x80000
#define RTE_NETIF_F_TSO6            0x200000

/* List of features with software fallbacks. */
#define RTE_NETIF_F_GSO_SOFTWARE    (RTE_NETIF_F_TSO | RTE_NETIF_F_TSO_ECN | RTE_NETIF_F_TSO6)
 
#define RTE_NETIF_F_GEN_CSUM        (RTE_NETIF_F_NO_CSUM | RTE_NETIF_F_HW_CSUM)
#define RTE_NETIF_F_V4_CSUM         (RTE_NETIF_F_GEN_CSUM | RTE_NETIF_F_IP_CSUM)
#define RTE_NETIF_F_V6_CSUM         (RTE_NETIF_F_GEN_CSUM | RTE_NETIF_F_IPV6_CSUM)
#define RTE_NETIF_F_ALL_CSUM        (RTE_NETIF_F_V4_CSUM | RTE_NETIF_F_V6_CSUM)


/* IFF flag defintion from netdevice->flags */
#define RTE_IFF_UP          0x1             /* interface is up              */
#define RTE_IFF_BROADCAST   0x2             /* broadcast address valid      */
#define RTE_IFF_DEBUG       0x4             /* turn on debugging            */
#define RTE_IFF_LOOPBACK    0x8             /* is a loopback net            */

#define RTE_IFF_POINTOPOINT 0x10            /* interface is has p-p link    */
#define RTE_IFF_NOTRAILERS  0x20            /* avoid use of trailers        */
#define RTE_IFF_RUNNING     0x40            /* interface RFC2863 OPER_UP    */
#define RTE_IFF_NOARP       0x80            /* no ARP protocol              */

#define RTE_IFF_PROMISC     0x100           /* mac promiscuous				*/
#define RTE_IFF_ALLMULTI    0x200           /* receive all multicast packets*/
#define RTE_IFF_VLAN_PROMISC	0x400		/* vlan promiscuous             */

#define RTE_IFF_MASTER      0x400           /* master of a load balancer    */
#define RTE_IFF_SLAVE       0x800           /* slave of a load balancer     */

#define RTE_IFF_MULTICAST   0x1000          /* Supports multicast           */

#define RTE_IFF_PORTSEL     0x2000          /* can set media type           */
#define RTE_IFF_AUTOMEDIA   0x4000          /* auto media select active     */
#define RTE_IFF_DYNAMIC     0x8000          /* dialup device with changing addresses*/

#define RTE_IFF_LOWER_UP    0x10000         /* driver signals L1 up         */
#define RTE_IFF_DORMANT     0x20000         /* driver signals dormant       */

#define RTE_IFF_ECHO        0x40000         /* echo sent packets            */

#define RTE_IFF_VOLATILE    (RTE_IFF_LOOPBACK|RTE_IFF_POINTOPOINT|RTE_IFF_BROADCAST|RTE_IFF_ECHO| \
                 RTE_IFF_MASTER|RTE_IFF_SLAVE|RTE_IFF_RUNNING|RTE_IFF_LOWER_UP|RTE_IFF_DORMAN

#define RTE_IFF_UNICAST_FLT		0x1000000
#define RTE_IFF_SUPP_NOFCS		0x2000000

#endif /* _RTE_IF_FLAGS_H_ */
