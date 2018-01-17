/*_
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
 *   Foundation, Inc., 51 Franklin St _ Fifth Floor, Boston, MA 02110_1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */
#include <linux/string.h>
#include <linux/kernel.h>
#include <stdarg.h>
#include "vni_log.h"

struct msg_info log_data;

void vlog(char *format, ...)
{
	char line_buf[MAXI_LINE];
	va_list var_list;
	int length, remain;

	line_buf[0]= '\0';
	log_data.buf[MAXI_MSG_LENGTH] = '\0';

	va_start(var_list, format);
	length = vsnprintf(line_buf, MAXI_LINE, format, var_list);
	va_end(var_list);
	
	length = length > MAXI_LINE? MAXI_LINE:length;

	/* update log-data */
	if ((log_data.next + length) >= MAXI_MSG_LENGTH) {
		remain = MAXI_MSG_LENGTH - log_data.next;
		memcpy(&log_data.buf[log_data.next], line_buf, remain);
		memcpy(log_data.buf, &line_buf[remain], length - remain);

		log_data.next = length - remain;
		log_data.overlap = 1;
	} else {
		memcpy(&log_data.buf[log_data.next], line_buf, length);
		log_data.next += length;
	}
}

struct msg_info *log_info(void)
{
	return &log_data;
}
