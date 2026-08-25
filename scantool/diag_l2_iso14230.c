/*
 *	freediag - Vehicle Diagnostic Utility
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
 * L2 driver for ISO14230-2 layer 2
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "diag.h"
#include "diag_tty.h"
#include "diag_l1.h"
#include "diag_l2.h"
#include "diag_err.h"
#include "diag_iso14230.h"
#include "diag_l2_iso9141.h"
#include "diag_vag.h"

#include "diag_l2_iso14230.h" /* prototypes for this file */

/*
 * ISO 14230 specific data
 */
struct diag_l2_14230 {
	uint8_t type;		/* FAST/SLOW/CARB */
	
	uint8_t srcaddr;	/* Src address used */
	uint8_t dstaddr;	/* Dest address used (for connect) */
	uint16_t modeflags;	/* Flags */

	uint8_t state;

	uint8_t first_frame;	/* First frame flag, used mainly for
					monitor mode when we need to find
					out whether we see a CARB or normal
					init */

	uint8_t rxbuf[MAXRBUF];	/* Receive buffer, for building message in */
	int rxoffset;		/* Offset to write into buffer */
};

#define STATE_CLOSED	  0	/* Established comms */
#define STATE_CONNECTING  1	/* Connecting */
#define STATE_ESTABLISHED 2	/* Established */

/*
 * Useful internal routines
 */
void decode_value(struct diag_msg *tmsg, int i) {

    int j;
    uint8_t displayBit, value;
    double temp =0;
  /* decoding according to https://github.com/ibanezgomez/FISBlocks/blob/master/KWP.cpp */
    switch(tmsg->data[i]) {
        case 0x01:
            printf("%f rpm\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2] *0.2));
            break;
        case 0x02:
            printf("% (throttle) position: %f \%\n",((double)tmsg->data[i+1] * (double)tmsg->data[i+2] *0.002));
            break;
        case 0x03:
            printf("Angle: %f degrees\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2] *0.002));
            break;
        case 0x04:
            printf("ATDC: %f\n", (abs((double)tmsg->data[i+2] - 127) * (double)tmsg->data[i+1] *0.01));
            break;
        case 0x05:
            printf("Temperature: %f C\n", ((double)tmsg->data[i+1] * ((double)tmsg->data[i+2]-100) *0.1));
            break;
        case 0x06:
            printf("Voltage: %f V\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2]*0.001));
            break;
        case 0x07:
            printf("Speed: %f km/h\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2]*0.01));
            break;
        case 0x08:
            printf("Activated flushing rate(?): %f\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2]*0.1));
            break;
        case 0x09:
            printf("Angle %f degrees\n", (((double)tmsg->data[i+2] - 127) * (double)tmsg->data[i+1] *0.02));
            break;
        case 0x0a:
            printf("COLD/WARM: %s\n", tmsg->data[i+2] == 0 ? "COLD" : "WARM");
            break;
        case 0x0B:
            printf("Adaptation value(?): %f\n", (((double)tmsg->data[i+1] * ((double)tmsg->data[i+2]-128)*0.0001)+1));
            break;
        case 0x0c:
            printf("Resistance: %f ohm\n", ((double)tmsg->data[i+2] * (double)tmsg->data[i+1] *0.001));
            break;
        case 0x0d:
            printf("Length: %f mm\n", (((double)tmsg->data[i+2] - 127) * (double)tmsg->data[i+1] *0.001));
            break;
        case 0x0e:
            printf("Pressure: %f bar\n", ((double)tmsg->data[i+2] * (double)tmsg->data[i+1] *0.005));
            break;
        case 0x0F:
            printf("Time: %f ms\n", ((double)tmsg->data[i+1] * (double)tmsg->data[i+2]*0.01));
            break;
        case 0x10:
            printf("8 bit block: <");
            value = tmsg->data[i+2];
            for(j=0; j<8; j++) {
	            printf("%d", displayBit = (value & 0x80) ? 1 : 0);
	            value = value << 1;
            }
            printf(">\n");
            break;
        case 0x12:
            printf("Pressure: %f mbar\n", ((double)tmsg->data[i+2] * (double)tmsg->data[i+1] *0.04));
            break;
        case 0x13:
            printf("Volume: %f L\n", ((double)tmsg->data[i+2] * (double)tmsg->data[i+1] *0.01));
            break;
        case 0x14:
            printf("\%: %f\n", ((double)tmsg->data[i+1] * ((double)tmsg->data[i+2]-128)/128));
            break;
        case 0x15:
            printf("Volts: %f V\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1] *0.001));
            break;
        case 0x16:
            printf("Time: %f ms\n", ((double)tmsg->data[i+1]*(double)tmsg->data[i+2]*0.001));
            break;
        case 0x17:
            printf("Valve duty cycle(?): %f \%\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/256));
            break;
        case 0x18:
            printf("Current: %f A\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]*0.001));
            break;
        case 0x19:
            printf("Acceleration: %f g/s\n", ((double)tmsg->data[i+2]*1.421) + ((double)tmsg->data[i+1]/182));
            break;
        case 0x1A:
            printf("Temperature: %f C\n", ((double)tmsg->data[i+2] - (double)tmsg->data[i+1]));
            break;
        case 0x1B:
            printf("Angle: %f degrees\n", (abs((double)tmsg->data[i+2]-128) *(double)tmsg->data[i+1] * 0.01));
            break;
        case 0x1C:
            printf("Unknown (b-a): %f C\n", ((double)tmsg->data[i+2] - (double)tmsg->data[i+1]));
            break;
        case 0x1E:
            printf("Deg k/w: %f C\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/12));
            break;
        case 0x1F:
            printf("Temperature: %f C\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/2560));
            break;
        case 0x21:
            printf("\%: %f\n", tmsg->data[i+1] ? (100*(double)tmsg->data[i+2]/(double)tmsg->data[i+1]) : (100 * (double)tmsg->data[i+1]));
            break;
        case 0x22:
            printf("Power: %f kW\n", (((double)tmsg->data[i+2]-128)*(double)tmsg->data[i+1]*0.01));
            break;
        case 0x23:
            printf("Flow: %f L/h\n", tmsg->data[i+1] ? (100 * (double)tmsg->data[i+2] / (double)tmsg->data[i+1]) : (100 * (double)tmsg->data[i+2]));
            break;
        case 0x24:
            printf("Distance: %f km\n", ((((unsigned long) tmsg->data[i+1]*2560)+((unsigned long) tmsg->data[i+2]*10))));
            break;
        case 0x25:
            printf("Unknown (a, b): %f, %f\n", (double)tmsg->data[i+1], (double)tmsg->data[i+2]);
            break;
        case 0x26:
            printf("Deg k/w: %f\n", (((double)tmsg->data[i+2]-128)*(double)tmsg->data[i+1]*0.001));
            break;
        case 0x27:
            printf("Flow: %f mg/h\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/256));
            break;
        case 0x28:
            printf("Current: %f A\n", (((double)tmsg->data[i+2]*0.1)+((double)tmsg->data[i+1]*25.5)-400));
            break;
        case 0x29:
            printf("Charge: %f Ah\n", ((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*255)));
            break;
        case 0x2A:
            printf("Kw: %f\n", (((double)tmsg->data[i+2]*0.1)+((double)tmsg->data[i+1]*25.5)-400));
            break;
        case 0x2B:
            printf("Voltage: %f V\n", (((double)tmsg->data[i+2]*0.1)+((double)tmsg->data[i+1]*25.5)));
            break;
        case 0x2C:
            printf("%2d:%2d\n", (double)tmsg->data[i+1], (double)tmsg->data[i+2]);
            break;
        case 0x2D:
            printf("Unknown 0.1*a*b/100: %f\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]*0.1/100));
            break;
        case 0x2E:
        	printf("Deg k/w: %f\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]-3200)*0.0027);
            break;
        case 0x2F:
            printf("Time: %f ms\n", ((double)tmsg->data[i+2]-128)*(double)tmsg->data[i+1]);
            break;
        case 0x30:
            printf("Unknown b+a*255: %f\n", ((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*255)));
            break;
        case 0x31:
            printf("Flow: %f mg/h\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]*0.1/4));
            break;
        case 0x32:
            printf("Pressure: %f mbar\n", (((double)tmsg->data[i+2]-128)/((double)tmsg->data[i+1]*0.01)));
            break;
        case 0x33:
            printf("Flow: %f kW\n", (((double)tmsg->data[i+2]-128)*(double)tmsg->data[i+1]/255));
            break;
        case 0x34:
            printf("Torque: %f Nm\n", (((double)tmsg->data[i+2]*(double)tmsg->data[i+1]*0.02)-(double)tmsg->data[i+1]));
            break;
        case 0x35:
            printf("Acceleration: %f g/s\n", ((((double)tmsg->data[i+2]-128)*1.4222)+((double)tmsg->data[i+1]*0.006)));
            break;
        case 0x36:
            printf("Count: %f\n", (((double)tmsg->data[i+1]*256) + (double)tmsg->data[i+2]));
            break;
        case 0x37:
            printf("Time: %f s\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/200));
            break;
        case 0x38:
            printf("WSC: %f\n", ((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256)));
            break;
        case 0x39:
            printf("WSC: %f\n", ((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256)+65536));
            break;
        case 0x3B:
            printf("Acceleration: %f g/s\n", (((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256))/32768));
            break;
        case 0x3C:
            printf("Time: %f s\n", (((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256))*0.01));
            break;
        case 0x3E:
            printf("Siemens?: %f S\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]*0.256));
            break;
        case 0x40:
            printf("Resistance: %f ohm\n", ((double)tmsg->data[i+2]+(double)tmsg->data[i+1]));
            break;
        case 0x41:
            printf("Length: %f mm\n", (((double)tmsg->data[i+2]-127)*(double)tmsg->data[i+1]*0.01));
            break;
        case 0x42:
            printf("Voltage: %f V\n", ((double)tmsg->data[i+2]*(double)tmsg->data[i+1]/511.12));
            break;
        case 0x43:
            printf("Angle: %f degrees\n", (((double)tmsg->data[i+1]*640)+((double)tmsg->data[i+2]*2.5)));
            break;
        case 0x44:
            printf("Rotation: %f degree/s\n", ((((double)tmsg->data[i+1]*256)+(double)tmsg->data[i+2])/7.365));
            break;
        case 0x45:
            printf("Pressure: %f bar\n", (((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256))*0.3254));
            break;
        case 0x46:
            printf("Acceleration: %f m/s^2\n", (((double)tmsg->data[i+2]+((double)tmsg->data[i+1]*256))*0.192));
            break;
        default:
            printf("Don't know that unit, please add here, (a, b): %f, %f\n", (double)tmsg->data[i+1], (double)tmsg->data[i+2]);
            break;
    }

    return;

}

/*
 * Decode the message header, returning the length
 * of the message if a whole message has been received.
 * Note that this may be called with more than one message
 * but it only worries about the first message
 */
static int diag_l2_proto_14230_decode(uint8_t *data, int len,
		 int *hdrlen, int *datalen, int *source, int *dest,
		int first_frame, uint8_t reqId) {
	int dl;
	uint8_t respId;

	if (diag_l2_debug & DIAG_DEBUG_PROTO) {
		int i;
		printf("Inbound message check, len: %d, ", len);
		for (i = 0; i < len ; i++) {
			printf("<%x>", data[i]&0xff);
		}
		printf("\n");
	}

	dl = data[0] & 0x3f;
	if (dl == 0) {
		/* Additional length field present */
		switch (data[0] & 0xC0)
		{
		case 0x80:
		case 0xC0:
			/* Addresses supplied, additional len byte */
			if (len < 4) {
				if (diag_l2_debug & DIAG_DEBUG_PROTO)
					fprintf(stderr, FLFMT "decode len short \n", FL);
				return diag_iseterr(DIAG_ERR_INCDATA);
			}
			*hdrlen = 4;
			*datalen = data[3];
			if (dest)
				*dest = data[1];
			if (source)
				*source = data[2];
			respId = data[4];
			break;
		case 0x00:
			/* Addresses not supplied, additional len byte */
			if (first_frame)
				return diag_iseterr(DIAG_ERR_BADDATA);
			if (len < 2)
				return diag_iseterr(DIAG_ERR_INCDATA);
			*hdrlen = 2;
			*datalen = data[1];
			if (dest)
				*dest = 0;
			if (source)
				*source = 0;
			respId = data[2];
			break;
		case 0X40:
			/* CARB MODE */
			return diag_iseterr(DIAG_ERR_BADDATA);
		}
	} else {
		/* Additional length field not present */
		switch (data[0] & 0xC0) {
		case 0x80:
		case 0xC0:
			/* Addresses supplied, NO additional len byte */
			if (len < 3)
				return diag_iseterr(DIAG_ERR_INCDATA);
			*hdrlen = 3;
			*datalen = dl;
			if (dest)
				*dest = data[1];
			if (source)
				*source = data[2];
			respId = data[3];
			break;
		case 0x00:
			/* Addresses not supplied, No additional len byte */
			if (first_frame)
				return diag_iseterr(DIAG_ERR_BADDATA);
			*hdrlen = 1;
			*datalen = dl;
			if (dest)
				*dest = 0;
			if (source)
				*source = 0;
			respId = data[1];
			break;
		case 0X40:
			/* CARB MODE */
			return diag_iseterr(DIAG_ERR_BADDATA);
		}
	}
	/*
	 * If len is silly [i.e 0] we've got this mid stream
	 */
	if (*datalen == 0)
		return diag_iseterr(DIAG_ERR_BADDATA);

	/*
	 * And confirm data is long enough, incl cksum
	 * If not, return saying data is incomplete so far
	 */
	if(len < (*hdrlen + *datalen + 1))
		return diag_iseterr(DIAG_ERR_INCDATA);

	if(diag_l2_debug & DIAG_DEBUG_PROTO)
	{
		fprintf(stderr, FLFMT "decode hdrlen = %d, datalen = %d, cksum = 1\n",
			FL, *hdrlen, *datalen);
	}
	if(NULL != reqId && (0x40 & reqId) != respId)
		fprintf(stderr, FLFMT "Error: from reqId %x, expected respId %x but received %x",
				reqId, (0x40 & reqId), respId);

	return (*hdrlen + *datalen + 1);
}

/* basic ISO14230 callback routine */
void l2_iso14230_data_rcv(void *handle __attribute__((unused)), struct diag_msg *msg) {

	int i;
	struct diag_msg *tmsg;
	/*
	 * Layer 2 call back, just print the data, this is used if we
	 * do a "read" and we haven't yet added a L3 protocol
	 */

	tmsg = msg;
	while(tmsg) {

	  switch (tmsg->data[0]) {

	  case (DIAG_KW2K_SI_STADS):
	  case (DIAG_KW2K_SI_REID):
		  break;

		case DIAG_KW2K_RC_RDDBLI:
		  for(i=2; i< tmsg->len; i+=3) {
//		    if(tmsg->data[i] != 0x25) {
		    printf("(Sensor) block %d, block id: <%x>, ", ((i-2)/3 +1), tmsg->data[i]);
		    printf("sensor bytes: <%x>", tmsg->data[i+1]);
		    printf("<%x>\n", tmsg->data[i+2]);
		    decode_value(tmsg, i);
//		    }
		  }
		  printf("\n");
		  break;
		default:
//	          printf("fmt <%x> type <%x> dest <%x> src <%x> len <%x> data ", tmsg->fmt, tmsg->type, tmsg->dest, tmsg->src, tmsg->len);
	          for(i=1; i<tmsg->len; i++)
	            printf("<%x>", tmsg->data[i]);
	          printf("\n");
	          for(i=1; i<tmsg->len; i++)
	            printf("%c", tmsg->data[i]);
	          printf("\n");
	  }
	  tmsg = tmsg->next;
	}

	return;	
}

/*
 * Internal receive function does all the message building, but doesn't
 * do call back, returns the complete message, hasn't removed checksum
 * and header info
 *
 * Data from the first message is put into *data, and len into *datalen
 *
 * If the L1 interface is clever (DOESL2FRAME), then each read will give
 * us a complete message, and we will wait a little bit longer than the normal
 * timeout to detect "end of all responses"
 */
static int diag_l2_proto_14230_int_recv(struct diag_l2_conn *d_l2_conn, int timeout,
	uint8_t *data, int *pDatalen) {
	struct diag_l2_14230 *dp;
	int rv, l1_doesl2frame, l1flags;
	int tout;
	int state;
	struct diag_msg	*tmsg, *lastmsg;

#define ST_STATE1	1	/* Start */
#define ST_STATE2	2	/* Interbyte */
#define ST_STATE3	3	/* Inter message */

	dp = (struct diag_l2_14230 *)d_l2_conn->diag_l2_proto_data;

	if (diag_l2_debug & DIAG_DEBUG_READ)
		fprintf(stderr,
			FLFMT "diag_l2_14230_intrecv offset %x\n",
				FL, dp->rxoffset);

	state = ST_STATE1;
	tout = timeout;

	/* Clear out last received message if not done already */
	if (d_l2_conn->diag_msg) {
		diag_freemsg(d_l2_conn->diag_msg);
		d_l2_conn->diag_msg = NULL;
	}

	l1flags = d_l2_conn->diag_link->diag_l2_l1flags;
	if (l1flags & (DIAG_L1_DOESL2FRAME|DIAG_L1_DOESP4WAIT)) {
		if (timeout < 100)	/* Extend timeouts */
			timeout = 100;
	}
	if (l1flags & DIAG_L1_DOESL2FRAME)
		l1_doesl2frame = 1;
	else
		l1_doesl2frame = 0;

	while (1) {
		switch (state) {
		case ST_STATE1:
			tout = timeout;
			break;
		case ST_STATE2:
			tout = d_l2_conn->diag_l2_p2min - 2;
			if (tout < d_l2_conn->diag_l2_p1max)
				tout = d_l2_conn->diag_l2_p1max;
			break;
		case ST_STATE3:
			if (l1_doesl2frame)
				tout = 150;	/* Arbitrary, short, value ... */
			else
				tout = d_l2_conn->diag_l2_p2max;
		}

		/* Receive data into the buffer */
		/*
		 * In l1_doesl2frame mode, we get full frames, so we don't
		 * do the read in state2
		 */
		if((state == ST_STATE2) && l1_doesl2frame)
			rv = DIAG_ERR_TIMEOUT;
		else
			rv = diag_l1_recv(d_l2_conn->diag_link->diag_l2_dl0d, 0,
				&dp->rxbuf[dp->rxoffset], sizeof(dp->rxbuf) - dp->rxoffset,
				tout);

		if(rv == DIAG_ERR_TIMEOUT) {
			/* Timeout, end of message, or end of responses */
			switch (state) {
			case ST_STATE1:
				/*
				 * 1st read, if we got 0 bytes, just return
				 * the timeout error
				 */
				if(dp->rxoffset == 0)
					break;
				/*
				 * Otherwise see if there are more bytes in
				 * this message,
				 */
				state = ST_STATE2;
				continue;
			case ST_STATE2:
				/*
				 * End of that message, maybe more to come
				 * Copy data into a message
				 */
				tmsg = diag_allocmsg((size_t)dp->rxoffset);
				tmsg->len = dp->rxoffset;
				memcpy(tmsg->data, dp->rxbuf, (size_t)dp->rxoffset);
				(void)gettimeofday(&tmsg->rxtime, NULL);
				dp->rxoffset = 0;
				/*
				 * ADD message to list
				 */
				diag_l2_addmsg(d_l2_conn, tmsg);
				if (d_l2_conn->diag_msg == tmsg) {
					/* 1st one */
					if (data) {
						memcpy(data, tmsg->data, (size_t)tmsg->len);
						*pDatalen = tmsg->len;
					}
				}
				state = ST_STATE3;
				continue;
			case ST_STATE3:
				/*
				 * No more messages, but we did get one
				 */
				rv = d_l2_conn->diag_msg->len;
				break;
			}
			if (state == ST_STATE3)
				break;
		}
		
		if (rv<0)
			break;

		/* Data received OK */
		dp->rxoffset += rv;

		if (dp->rxoffset && (dp->rxbuf[0] == '\0')) {
			/*
			 * We get this when in
			 * monitor mode and there is
			 * a fastinit, pretend it didn't exist
			 */
			dp->rxoffset--;
			if (dp->rxoffset)
				memcpy(&dp->rxbuf[0], &dp->rxbuf[1],
					(size_t)dp->rxoffset);
			continue;
		}
		if ( (state == ST_STATE1) || (state == ST_STATE3) ) {
			/*
			 * Got some data in state1/3, now we're in a message
			 */
			state = ST_STATE2;
		}
	}

	/*
	 * Now check the messages that we have checksum etc, stripping
	 * off headers etc
	 */
	if (rv >= 0) {
		tmsg = d_l2_conn->diag_msg;
		lastmsg = NULL;

		while (tmsg) {
			int hdrlen, datalen, source, dest;

			/*
			 * We have the message with the header etc, we
			 * need to strip the header and checksum
			 */
			dp = (struct diag_l2_14230 *)d_l2_conn->diag_l2_proto_data;
			rv = diag_l2_proto_14230_decode(tmsg->data, tmsg->len,
				&hdrlen, &datalen, &source, &dest, dp->first_frame, d_l2_conn->diag_l2_request_id);

			if (rv < 0)		/* decode failure */
				return diag_iseterr(rv);

			/*
			 * If L1 isnt doing L2 framing then it is possible
			 * we have misframed this message and it is infact
			 * more than one message, so see if we can decode it
			 */
			if ((l1_doesl2frame == 0) && (rv < tmsg->len)) {
				/*
				 * This message contains more than one	
				 * data frame (because it arrived with
				 * odd timing), this means we have to
				 * do horrible copy about the data
				 * things ....
				 */
				struct diag_msg	*amsg;
				amsg = diag_dupsinglemsg(tmsg);
				amsg->len = rv;
				tmsg->len -=rv;
				tmsg->data += rv;

				/*  Insert new amsg before old msg */
				amsg->next = tmsg;
				if (lastmsg == NULL)
					d_l2_conn->diag_msg = amsg;
				else
					lastmsg->next = amsg;

				tmsg = amsg; /* Finish processing this one */
			}

			if ((tmsg->data[0] & 0xC0) == 0xC0) {
				tmsg->fmt = DIAG_FMT_ISO_FUNCADDR;
			} else {
				tmsg->fmt = 0;
			}
			tmsg->fmt |= DIAG_FMT_FRAMED | DIAG_FMT_DATAONLY;
			tmsg->fmt |= DIAG_FMT_CKSUMMED;

			if ((l1flags & DIAG_L1_STRIPSL2CKSUM) == 0) {
				/* XXX check checksum */
			}

			tmsg->src = source;
			tmsg->dest = dest;
			tmsg->data += hdrlen;	/* Skip past header */
			tmsg->len -= hdrlen; /* remove header */

			/* remove checksum byte if needed */
			if ((l1flags & DIAG_L1_STRIPSL2CKSUM) == 0)
				tmsg->len--;

			dp->first_frame = 0;

			lastmsg = tmsg;
			tmsg = tmsg->next;
		}
	}
	return rv;
}

static int diag_l2_proto_14230_stopcomms(struct diag_l2_conn* pX) {
	struct diag_l2_14230 *dp;
	struct diag_msg	msg, *tmsg;
	uint8_t buff;
	int rv = 0;
	uint8_t *data;

	dp = (struct diag_l2_14230 *)pX->diag_l2_proto_data;

	if(dp->state > STATE_CLOSED) {

 	  if (diag_calloc(&msg.data, 1)) {
 	    fprintf(stderr,
 		 	FLFMT "diag_calloc failed for StopDiagnosticSession service request\n", FL);
 	    return(DIAG_ERR_NOMEM);
 	  }
 	  msg.len = 1;
 	  msg.src = pX->diag_l2_srcaddr;
 	  msg.dest = pX->diag_l2_destaddr;
 	  buff = DIAG_KW2K_SI_STODS;
      memcpy(msg.data, &buff, msg.len*sizeof(uint8_t));
      pX->diag_l2_request_id = buff;

	  rv = diag_l2_send(pX, &msg);
	  if (rv < 0) {
		fprintf(stderr, FLFMT "failed to send StopDiagnosticSession service request\n", FL);
		return rv;
	  }
	  free(msg.data);

	  diag_os_millisleep(25);
	  rv = diag_l2_recv(pX, pX->diag_l2_p3min, l2_iso14230_data_rcv, NULL);

// would like to check return code, but guess need do that in the callback by passing a HANDLE
//	  tmsg = pX->diag_msg;
//	  if((tmsg->data[0] != (DIAG_KW2K_SI_STODS+0x40))
//	    && (tmsg->len != 1)) {
	  if(rv <0) {
		  fprintf(stderr, FLFMT "StopDiagnosticSession service failed\n", FL);
		  return -1;
	  }	  
	  dp->state = STATE_CLOSED;
	}

	return 0;

}

/*
 * Just send the data
 *
 * We add the header and checksums here as appropriate, based on the keybytes
 *
 * We take the source and dest from the internal data NOT from the msg fields
 *
 * We also wait p3 ms
 */
static int diag_l2_proto_14230_send(struct diag_l2_conn *d_l2_conn, struct diag_msg *msg) {
	int rv, csum;
	unsigned int i;
	size_t len;
	uint8_t buf[MAXRBUF];
	int offset;
	struct diag_l2_14230 *dp;

	if (diag_l2_debug & DIAG_DEBUG_WRITE)
		fprintf(stderr,
			FLFMT "diag_l2_14230_send %p msg %p len %d called\n",
				FL, d_l2_conn, msg, msg->len);

	dp = (struct diag_l2_14230 *)d_l2_conn->diag_l2_proto_data;

	/* Build the new message */
	if (dp->modeflags & DIAG_L2_TYPE_FUNCADDR)
		buf[0] = 0xC0;
	else
		buf[0] = 0x80;

	/* If user supplied addresses, use them, else use the originals */
	if (msg->dest)
		buf[1] = msg->dest;
	else
		buf[1] = dp->dstaddr;
	if (msg->src)
		buf[2] = msg->src;
	else
		buf[2] = dp->srcaddr;

/* XXX, check mode flag that specifies always use 4 byte hdr ,
	or mode flag showing never use extended header, and
		received keybytes */
	if (msg->len < 64) {
		if (msg->len < 1)
			return diag_iseterr(DIAG_ERR_BADLEN);
		buf[0] |= msg->len;
		offset = 3;
	} else {
//#if NOWARNINGS==0
//	/* silly hack to get around compiler warning */ XXX what ?
//	/* msg->len is an unsigned byte, this is a NOP */
//#endif
		len = msg->len;

		/* Extended length */
		if (len > 255)
			return diag_iseterr(DIAG_ERR_BADLEN);
		buf[3] = msg->len;
		offset = 4;
	}
	memcpy(&buf[offset], msg->data, msg->len * sizeof(uint8_t));
	len = msg->len + offset;	/* data + hdr */

	if ((d_l2_conn->diag_link->diag_l2_l1flags & DIAG_L1_DOESL2CKSUM) == 0) {
		/* We must add checksum, which is sum of bytes */
		for (i = 0, csum = 0; i < len; i++)
			csum += buf[i];
		buf[len] = csum;
		len++;				/* + checksum */
	}

	/* Wait p3min milliseconds, but not if doing fast/slow init */
	if (dp->state == STATE_ESTABLISHED)
		diag_os_millisleep(d_l2_conn->diag_l2_p3min);

//xxx	if(buf[3] != DIAG_KW2K_SI_TP) {
		printf("Outbound message check: ");
			for (i=0; i< len; i++)
				printf("<%x>", buf[i]);
			printf("\n");
//	}
	rv = diag_l1_send (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		buf, len, d_l2_conn->diag_l2_p4min);

	if (diag_l2_debug & DIAG_DEBUG_WRITE)
		fprintf(stderr, FLFMT "send about to return %d\n", FL, rv);

	return rv;

}

/*
 * Protocol receive routine
 *
 * Will sleep until a complete set of responses has been received, or fail
 * with a timeout error
 *
 * The interbyte type in data from an ECU is between P1Min and P1Max
 * The intermessage time for part of one response is P2Min and P2Max
 *
 * If we are running with an intelligent L1 interface, then we will be
 * getting one message per frame, and we will wait a bit longer
 * for extra messages
 */
static int diag_l2_proto_14230_recv(struct diag_l2_conn *d_l2_conn, int timeout,
	void (*callback)(void *handle, struct diag_msg *msg),
	void *handle) {
	uint8_t data[256];
	int rv;
	int datalen;

	/* Call internal routine */
	rv = diag_l2_proto_14230_int_recv(d_l2_conn, timeout, data, &datalen);

	if (rv < 0)	/* Failure */
		return rv;

	if (diag_l2_debug & DIAG_DEBUG_READ)
		fprintf(stderr, FLFMT "calling rcv callback %p handle %p\n", FL,
			callback, handle);

	/*
	 * Call user callback routine
	 */
	if (callback)
		callback(handle, d_l2_conn->diag_msg);

	/* No longer needed */
	diag_freemsg(d_l2_conn->diag_msg);
	d_l2_conn->diag_msg = NULL;

	if (diag_l2_debug & DIAG_DEBUG_READ)
		fprintf(stderr, FLFMT "rcv callback completed\n", FL);

	return 0;

}

static struct diag_msg *
diag_l2_proto_14230_request(struct diag_l2_conn *d_l2_conn, struct diag_msg *msg,
		int *errval)
{
	int rv;
	struct diag_msg *rmsg = NULL;

	rv = diag_l2_send(d_l2_conn, msg);
	if (rv < 0) {
		*errval = rv;
		return (struct diag_msg *)diag_pseterr(rv);
	}

#if 1
	rv = diag_l2_recv(d_l2_conn,
		d_l2_conn->diag_l2_p2max + 10, l2_iso14230_data_rcv, NULL);

	if (rv < 0) {
		*errval = DIAG_ERR_TIMEOUT;
		return (struct diag_msg *)diag_pseterr(rv);
	}
	
	return NULL;
#else	
	while (1) {
		rv = diag_l2_proto_14230_int_recv(d_l2_conn,
			d_l2_conn->diag_l2_p2max + 10, NULL, NULL);

		if (rv < 0) {
			*errval = DIAG_ERR_TIMEOUT;
			return (struct diag_msg *)diag_pseterr(rv);
		}

		/*
		 * The connection now has the received message data
		 * stored, remove it and deal with it
		 */
		rmsg = d_l2_conn->diag_msg;
		d_l2_conn->diag_msg = NULL;

		/* Got a Error message */
		if (rmsg->data[0] == DIAG_KW2K_RC_NR) {
			if (rmsg->data[2] == DIAG_KW2K_RC_B_RR) {
				/*
				 * Msg is busyRepeatRequest
				 * So do a send again
				 */
				rv = diag_l2_send(d_l2_conn, msg);
				if (rv < 0) {
					*errval = rv;
					return (struct diag_msg *)diag_pseterr(rv);
				}
				diag_freemsg(rmsg);
				continue;
			}

			if (rmsg->data[2] == DIAG_KW2K_RC_RCR_RP) {
				/*
				 * Msg is a requestCorrectlyRcvd-RspPending
				 * so do read again
				 */
				diag_freemsg(rmsg);
				continue;
			}
			/* Some other type of error */
		} else {
			/* Success */
			break;
		}
	}
	/* Return the message to user, who is responsible for freeing it */
	return rmsg;
#endif
}

/*
 * Timeout, - if we don't send something to the ECU it will timeout
 * soon, so send it a keepalive message now.
 */
static void diag_l2_proto_14230_timeout(struct diag_l2_conn *d_l2_conn) {
	struct diag_l2_14230 *dp;
	struct diag_msg	msg;
	uint8_t data[256];
	int timeout;

	dp = (struct diag_l2_14230 *)d_l2_conn->diag_l2_proto_data;

	if(dp->state < STATE_ESTABLISHED)
		return;

	// XXX fprintf not async-signal-safe
	if (diag_l2_debug & DIAG_DEBUG_TIMER)
		fprintf(stderr, FLFMT "timeout impending for %p type %d\n", FL, d_l2_conn, dp->type);

	msg.data = data;

	// Prepare the "keepalive" message
	// Idle using ISO "Tester Present" message
	msg.len = 2;
	msg.src = d_l2_conn->diag_l2_srcaddr;
	msg.dest = d_l2_conn->diag_l2_destaddr;
	data[0] = DIAG_KW2K_SI_TP;
	data[1] = 0x01;
	d_l2_conn->diag_l2_request_id = data[0];

	/*
	 * There is no point in checking for errors, or checking
	 * the received response as we can't pass an error back
	 * from here
	 */

	/* Send it, important to use l2_send as it updates the timers */
	(void)diag_l2_send(d_l2_conn, &msg);

	/*
	 * Get the response in p2max, we allow longer, and even
	 * longer on "smart" L2 interfaces
	 */
	timeout = d_l2_conn->diag_l2_p3min;
	if (d_l2_conn->diag_link->diag_l2_l1flags &
			(DIAG_L1_DOESL2FRAME|DIAG_L1_DOESP4WAIT)) {
		if (timeout < 100)
			timeout = 100;
	}
	(void)diag_l2_recv(d_l2_conn, timeout, NULL, NULL);
	
	return;

}

static const struct diag_l2_proto diag_l2_proto_14230 = {
	DIAG_L2_PROT_ISO14230, DIAG_L2_FLAG_FRAMED | DIAG_L2_FLAG_DATA_ONLY
	| DIAG_L2_FLAG_KEEPALIVE | DIAG_L2_FLAG_DOESCKSUM,
	diag_l2_proto_14230_stopcomms,
	diag_l2_proto_14230_send,
	diag_l2_proto_14230_recv,
	diag_l2_proto_14230_request,
	diag_l2_proto_14230_timeout
};

int diag_l2_14230_add(void) {
	return diag_l2_add_protocol(&diag_l2_proto_14230);
}
