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
#ifndef _I40E_COMMON_H_
#define _I40E_COMMON_H_

#define I40E_MAX_VF_QUEUES		16
#define I40E_MAX_PF_QUEUES      1536

#ifndef ADVERTISED_40000baseKR4_Full
#define SUPPORTED_40000baseKR4_Full	(1 << 23)
#define SUPPORTED_40000baseCR4_Full	(1 << 24)
#define SUPPORTED_40000baseSR4_Full	(1 << 25)
#define SUPPORTED_40000baseLR4_Full	(1 << 26)
#define ADVERTISED_40000baseKR4_Full	(1 << 23)
#define ADVERTISED_40000baseCR4_Full	(1 << 24)
#define ADVERTISED_40000baseSR4_Full	(1 << 25)
#define ADVERTISED_40000baseLR4_Full	(1 << 26)
#endif

#ifndef SPEED_20000
#define SPEED_20000		20000
#endif

#ifndef SPEED_25000
#define SPEED_25000		25000
#endif

#ifndef SPEED_40000
#define SPEED_40000		40000
#endif

#ifndef SPEED_UNKNOWN
#define SPEED_UNKNOWN -1
#endif

#ifndef I40E_FLAG_100M_SGMII_CAPABLE
#define I40E_FLAG_100M_SGMII_CAPABLE		BIT_ULL(43)
#endif

#ifndef I40E_FLAG_HAVE_CRT_RETIMER
#define I40E_FLAG_HAVE_CRT_RETIMER		BIT_ULL(51)
#endif

#define i40e_link_speed_set(cmd, new_speed) \
	{ \
		if ((unsigned int)new_speed > 0xFFFF) { \
			cmd->speed_hi = (new_speed >> 16) & 0xFFFF; \
			cmd->speed = new_speed & 0xFFFF; \
		} else { \
			cmd->speed_hi = 0; \
			cmd->speed = new_speed; \
		} \
	}

#endif /* _I40E_COMMON_H_ */
