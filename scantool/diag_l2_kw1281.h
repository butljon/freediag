#ifndef _DIAG_L2_KW1281_H_
#define _DIAG_L2_KW1281_H_
/*
 * !!! INCOMPLETE !!!!
 *
 *	freediag - Vehicle Diagnostic Utility
 *
 * CVSID $Id: diag_l2_vag.h,v 1.1.1.1 2004/06/05 01:56:41 sjbaker Exp $
 *
 *
 * Copyright (C) 2001 Richard Almeida & Ibex Ltd (rpa@ibex.co.uk)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *************************************************************************
 *
 * Diag
 *
 * L2 driver for Volkswagen Audi Group protocol (Keyword 0x01 0x8a)
 *
 */

#if defined(__cplusplus)
extern "C" {
#endif


struct diag_l2_kw1281
{
	uint8_t srcaddr;	// Src address used, normally 0xF1 (tester)
	uint8_t target;	// Target address used, normally 0x33 (ISO9141)
	uint8_t seq_nr;	/* Sequence number */
	uint8_t master;	/* Master flag, 1 = us, 0 = ECU */


	uint8_t rxbuf[MAXRBUF];	/* Receive buffer, for building message in */
	int rxoffset;		/* Offset to write into buffer */
	uint8_t state;
	struct monitor_type *monitor;
};

int diag_l2_kw1281_add(void);

#if defined(__cplusplus)
}
#endif
#endif /* _DIAG_L2_KW1281_H_ */
