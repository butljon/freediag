/*
 * !!! INCOMPLETE !!!!
 *
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
 * L2 driver for Volkswagen Audi Group protocol (Keyword 0x01 0x8a)
 *
 * !!! INCOMPLETE !!!!
 *
 */

#include <stdlib.h>
#include <string.h>

#include "diag.h"
#include "diag_err.h"
#include "diag_tty.h"
#include "diag_l1.h"
#include "diag_l2.h"
#include "diag_l2_raw.h"
#include "diag_vag.h"
#include "diag_l2_iso9141.h"

#include "diag_l2_kw1281.h" /* prototypes for this file */

CVSID("$Id: diag_l2_kw1281.c,v 1.3 2004/07/03 02:04:18 vnevoa Exp $");

/*
 * ISO KW1281 specific data
 */
struct monitor_type
{
	uint8_t value;
	struct monitor_type *next;
};

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

#define STATE_CLOSED	  0	/* Established comms */
#define STATE_CONNECTING  1	/* Connecting */
#define STATE_ESTABLISHED 2	/* Established */


static int
diag_l2_proto_kw1281_recv(struct diag_l2_conn *d_l2_conn, int timeout,
	void (*callback)(void *handle, struct diag_msg *msg),
	void *handle);

static int
diag_l2_proto_kw1281_send(struct diag_l2_conn *d_l2_conn, struct diag_msg *msg);

/*
 * Useful internal routines
 */

#if notyet
/*
 * Decode the message header
 */
static int
diag_l2_proto_kw1281_decode(char *data, int len,
		 int *hdrlen, int *datalen, int *source, int *dest,
		int first_frame)
{
	int dl;

	if (diag_l2_debug & DIAG_DEBUG_PROTO)
	{
		int i;
		fprintf(stderr, FLFMT "decode len %d", FL, len);
		for (i = 0; i < len ; i++)
			fprintf(stderr, " 0x%x", data[i]&0xff);

		fprintf(stderr, "\n");
	}
	
	dl = data[0] & 0x3f;
	if (dl == 0)
	{
		/* Additional length field present */
		switch (data[0] & 0xC0)
		{
		case 0x80:
		case 0xC0:
			/* Addresses supplied, additional len byte */
			if (len < 4)
			{
				if (diag_l2_debug & DIAG_DEBUG_PROTO)
					fprintf(stderr, FLFMT "decode len short \n", FL);
				return(diag_iseterr(DIAG_ERR_INCDATA));
			}
			*hdrlen = 4;
			*datalen = data[3];
			if (dest)
				*dest = data[1];
			if (source)
				*source = data[2];
			break;
		case 0x00:
			/* Addresses not supplied, additional len byte */
			if (first_frame)
				return(diag_iseterr(DIAG_ERR_BADDATA));
			if (len < 2)
				return(diag_iseterr(DIAG_ERR_INCDATA));
			*hdrlen = 2;
			*datalen = data[1];
			if (dest)
				*dest = 0;
			if (source)
				*source = 0;
			break;
		case 0X40:
			/* CARB MODE */
			return(diag_iseterr(DIAG_ERR_BADDATA));
		}
	}
	else
	{
		/* Additional length field not present */
		switch (data[0] & 0xC0)
		{
		case 0x80:
		case 0xC0:
			/* Addresses supplied, NO additional len byte */
			if (len < 3)
				return(diag_iseterr(DIAG_ERR_INCDATA));
			*hdrlen = 3;
			*datalen = dl;
			if (dest)
				*dest = data[1];
			if (source)
				*source = data[2];
			break;
		case 0x00:
			/* Addresses not supplied, No additional len byte */
			if (first_frame)
				return(diag_iseterr(DIAG_ERR_BADDATA));
			*hdrlen = 1;
			*datalen = dl;
			if (dest)
				*dest = 0;
			if (source)
				*source = 0;
			break;
		case 0X40:
			/* CARB MODE */
			return(diag_iseterr(DIAG_ERR_BADDATA));
		}
	}
	/*
	 * If len is silly [i.e 0] we've got this mid stream
	 */
	if (*datalen == 0)
		return(diag_iseterr(DIAG_ERR_BADDATA));

	/*
	 * And confirm data is long enough, incl cksum
	 * If not, return saying data is incomplete so far
	 */

	if (len < (*hdrlen + *datalen + 1))
		return(diag_iseterr(DIAG_ERR_INCDATA));

	return(0);
}
#endif


struct monitor_type *
push_monitor(struct monitor_type *node, uint8_t monitor) {
   
  struct monitor_type *ptr;
  
  if(node == NULL) {
    node = (struct monitor_type *) malloc(sizeof(struct monitor_type));
    node->value = monitor;
    node->next = NULL;
    return node;
  }
    
  ptr = node;
  while(ptr) {
    if(ptr->value == monitor) {
      printf("Group %d already on monitor watch list\n", monitor);
      return node;
    }
    if(ptr->next == NULL) {
      ptr->next = (struct monitor_type *) malloc(sizeof(struct monitor_type));
      ptr = ptr->next;
      ptr->value = monitor;
      ptr->next = NULL;
      return node;
    }
    ptr = ptr->next;
  }

  /* should never get here, hence: error */
  fprintf(stderr, FLFMT "Something strange: no able to push monitor %d\n",
				FL, monitor);
  return node;
}


struct monitor_type *
pop_monitor(struct monitor_type *node, uint8_t monitor) {
  
  struct monitor_type *ptr, *found;

  if(node == NULL) {
    printf("Monitor list empty, group %d cannot be removed\n", monitor);
    return node;
  }

  if(node->value == monitor) {
    ptr=node->next;
    free(node);
    return ptr;
  }

  ptr=node;
  while(ptr->next) {
    if(ptr->next->value == monitor) {
      found = ptr->next;
      ptr->next = found->next;
      free(found);
      return node;
    }
    ptr = ptr->next;
  }
  printf("Group %d not on monitor watch list\n", monitor);
  return node;

}

void
show_monitor(struct monitor_type *node) {
  
  struct monitor_type *ptr;

  if(node == NULL) {
    printf("Monitor watch list empty\n");
    return;
  }

  ptr=node;
  while(ptr) {
    printf("Group %d on monitor watch list\n", ptr->value);
    ptr = ptr->next;
  }

  return;
}


void decode_value(struct diag_msg *tmsg, int i)
{
  int j;
  uint8_t displayBit, value;
  double temp =0;
  /* decoding according to http://www.blafusel.de/obd/obd2_kw1281.html */
  switch(tmsg->data[i]) {
    case 0x01:
      printf("%f rpm\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2] *0.2));
      break;
    case 0x02:
      printf("Absoulte (throttle) position: %f \%\n",((double) tmsg->data[i+1] * (double) tmsg->data[i+2] *0.002));
      break;
    case 0x03:
      printf("Angle: %f degrees\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2] *0.002));
      break;
    case 0x05:
      printf("Temperature: %f C\n", ((double) tmsg->data[i+1] * ((double) tmsg->data[i+2]-100) *0.1));
      break;
    case 0x06:
    case 0x08:
    case 0x15:
      printf("Voltage: %f V\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2]*0.001));
      break;
    case 0x07:
      printf("Speed: %f km/h\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2]*0.01));
      break;
//    case 0x08:
//      printf("Activated flushing rate(?): %f\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2]*0.1));
//      break;
    case 0x10:
      printf("8 bit block: <");
      value = tmsg->data[i+2];
      for (j=0; j<8; j++) {
	printf("%d", displayBit = (value & 0x80) ? 1 : 0);
	value = value << 1;
      }
      printf(">\n");
      break;
    case 0x0B:
      printf("Adaptation value(?): %f\n", ((double) tmsg->data[i+1] * ((double) tmsg->data[i+2]-128)*0.0001+1));
      break;
    case 0x0F:
      printf("Time: %f ms\n", ((double) tmsg->data[i+1] * (double) tmsg->data[i+2]*0.01));
      break;
    case 0x17:
      printf("Valve duty cycle(?): %f \%\n", ((double) tmsg->data[i+2]/((double) tmsg->data[i+1] * 256)));
      break;
    case 0x21:
      temp = (tmsg->data[i+1]) ? (100 * (double) tmsg->data[i+2] / (double) tmsg->data[i+1]) : (100 * (double) tmsg->data[i+2]);
      printf("Gaspedal angle(?): %f \%\n", temp);
      break;
    default:
      printf("Don't know that unit, please add here!\n");
      break;
  }
  return;
}

/* basic VAG KW1281 callback routine */
#ifdef WIN32
void
l2_kw1281_data_rcv(void *handle,
struct diag_msg *msg)
#else
void
l2_kw1281_data_rcv(void *handle __attribute__((unused)),
struct diag_msg *msg)
#endif
{
	/*
	 * Layer 2 call back, just print the data, this is used if we
	 * do a "read" and we haven't yet added a L3 protocol
	 */
	int i;
	struct diag_msg *tmsg;

	tmsg = msg;
	while(tmsg) {
	  switch (tmsg->type) {

		case DIAG_VAG_RSP_ASCII:
		  for(i=0; i< tmsg->len; i++)
		      printf("%c", tmsg->data[i]);
		  printf("\n");
		  for(i=0; i< tmsg->len; i++)
		      printf("<%x>", tmsg->data[i]);
		  printf("\n");
		  break;
		case DIAG_VAG_RSP_DTC_RQST:
		  for(i=0; i< tmsg->len; i+=3) {
		    printf("Error code %d, code: <%x>", (i/3 +1), tmsg->data[i]);
		    printf("<%x>", tmsg->data[i+1]);
		    printf(" status: <%x>\n", tmsg->data[i+2]);
		  }
		  printf("\n");
		  break;
		case DIAG_VAG_RSP_DATA_OTHER:
		  for(i=0; i< tmsg->len; i+=3) {
		    printf("(Sensor) block %d, block id: <%x>, ", (i/3 +1), tmsg->data[i]);
		    printf("sensor bytes: <%x>", tmsg->data[i+1]);
		    printf("<%x>\n", tmsg->data[i+2]);
		    decode_value(tmsg, i);
		  }
		  printf("\n");
		  break;
		case DIAG_VAG_RSP_CHAN_READ:
		  for(i=0; i< tmsg->len; i+=3) {
		    printf("Channel %d, channel id: <%x>, ", (i/3 +1), tmsg->data[i]);
		    printf("channel bytes: <%x>", tmsg->data[i+1]);
		    printf("<%x>\n", tmsg->data[i+2]);
		  }
		  printf("\n");
		  break;
		case DIAG_VAG_CMD_ACK:
		/* not going to print ACKs */
		  break;
		case 0x0a:
		/* seems to just be an empty Group or Channel address ? */
		  break;
		default:
		  fprintf(stderr,
			FLFMT "message type: <%x> not yet known, add code here please!\n",
				FL, tmsg->type);
		  for(i=0; i< tmsg->len; i++)
		      printf("<%x>", tmsg->data[i]);
		  printf("\n");

	  }
	  tmsg = tmsg->next;
	}

	return;	
}

/* receives a byte and if end = 1 echos back the compliment */
static int
diag_l2_proto_kw1281_recv_byte(struct diag_l2_conn *d_l2_conn, int timeout,
			                        databyte_type *databyte, int end)
{
        int rv;
	uint8_t db;

	diag_os_millisleep(5);

     /* receive 1 byte */
        rv = diag_l1_recv (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		databyte, 1, timeout);
	if (rv < 0) {
		fprintf(stderr,
			FLFMT "failed to receive a byte\n", FL);
		return rv;
	}
	
	if(end) {
	  diag_os_millisleep(5);

	  db = *databyte;
	  db = ~db;

	  rv = diag_l1_send (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		&db, 1, d_l2_conn->diag_l2_p4min);
	  if (rv < 0) {
		fprintf(stderr,
			FLFMT "failed to send back byte %x, i.e. the compliment of %x\n", FL, db, *databyte);
		return rv;
	  }
	}
	return 0;
}


#ifdef WIN32
static int
diag_l2_proto_kw1281_recv_msg(struct diag_l2_conn *d_l2_conn, 
int timeout)
#else
static int
diag_l2_proto_kw1281_recv_msg(struct diag_l2_conn *d_l2_conn, 
int timeout __attribute__((unused)))
#endif
{
	struct diag_l2_kw1281 *dp;
	int rv = 0, i;
	uint8_t block_len, databyte;
	struct diag_msg *tmsg;

	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	/*
	 * And receive the new message
	 */
        rv = diag_l2_proto_kw1281_recv_byte(d_l2_conn, timeout, &databyte, 1);
	if (rv < 0) {
 	    fprintf(stderr,
		FLFMT "Could not receive the length byte\n", FL);
	     return rv;
   	}

      /* note msg->len is uint8_t, hence message length restriction of 256 bytes */
	block_len = databyte-3;

	rv = diag_l2_proto_kw1281_recv_byte(d_l2_conn, timeout, &databyte, 1);
	if (rv < 0) {
 	    fprintf(stderr,
		FLFMT "Could not receive the sequence byte\n", FL);
	     return rv;
   	}

	if (databyte != ++dp->seq_nr) {
     /* wrong message number */
	    fprintf(stderr,
		FLFMT "Sequence numbers out of sync\n", FL);
	     return -1;
   	}     
        
     /* now store message */
	tmsg = diag_allocmsg((size_t)block_len);
	tmsg->len = block_len;

 	rv = diag_l2_proto_kw1281_recv_byte(d_l2_conn, timeout, &tmsg->type, 1);
	if (rv < 0) {
 	    fprintf(stderr,
		FLFMT "Could not receive the type byte\n", FL);
	     return rv;
   	}

	if(tmsg->len) {
	  for (i=0; i < tmsg->len; i++) {
	      rv = diag_l2_proto_kw1281_recv_byte(d_l2_conn, timeout, &dp->rxbuf[i], 1);
	      if (rv < 0) {
		fprintf(stderr,
		  FLFMT "Could not receive %d th data byte\n", FL, i);
		return rv;
	      }
	  }
	}

 	rv = diag_l2_proto_kw1281_recv_byte(d_l2_conn, timeout, &databyte, 0);
	if (rv < 0) {
 	    fprintf(stderr,
		FLFMT "Could not receive end of frame byte\n", FL);
	     return rv;
   	}

	if(databyte != DIAG_VAG_END_FRAME) {
 	    fprintf(stderr,
		FLFMT "Should have been end of frame byte, instead we got: %x\n", FL, databyte);
	     return -1;
   	}

      /* store data */
	memcpy(tmsg->data, &dp->rxbuf[0], tmsg->len);
	(void)gettimeofday(&tmsg->rxtime, NULL);
	tmsg->next = NULL;
	
    /* add message to list */
	diag_l2_addmsg(d_l2_conn, tmsg);
	
	return (tmsg->len+4);

}


/*
 * Internal receive function (does all the message building, but doesn't
 * do call back, receives messages until an ACK appears.  Builds all messages
 * including ACKs onto the list.
 */
#ifdef WIN32
static int
diag_l2_proto_kw1281_int_recv(struct diag_l2_conn *d_l2_conn, 
int timeout)
#else
static int
diag_l2_proto_kw1281_int_recv(struct diag_l2_conn *d_l2_conn, 
int timeout __attribute__((unused)))
#endif
{
	struct diag_l2_kw1281 *dp;
	int rv = 0, i, bytes=0;
	uint8_t block_len, databyte;
	struct diag_msg *tmsg, msg;

	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	if (diag_l2_debug & DIAG_DEBUG_READ)
		fprintf(stderr,
			FLFMT "diag_l2_kw1281_int_recv offset %x\n",
				FL, dp->rxoffset);

	/* Clear out last received message if not done already  */
	if (d_l2_conn->diag_msg)
	{
		diag_freemsg(d_l2_conn->diag_msg);
		d_l2_conn->diag_msg = NULL;
	}

	/*
	 * And receive the new message
	 */
	rv = diag_l2_proto_kw1281_recv_msg(d_l2_conn, timeout);
	if (rv < 0) {
 	    fprintf(stderr,
		FLFMT "Could not receive message\n", FL);
	     return rv;
   	}
	bytes +=rv;

 	diag_os_millisleep(5);
	
	tmsg = d_l2_conn->diag_msg;
    /* get the rest */
	while(tmsg->type != DIAG_VAG_CMD_ACK) {

 	    if (diag_calloc(&msg.data, 1)) {
 	      fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KW1281\n", FL);
 	      return(DIAG_ERR_NOMEM);
 	    }
 	    msg.len = 1;
 	    databyte = (0xff & (uint8_t) DIAG_VAG_CMD_ACK);
    	    memcpy(msg.data, &databyte, sizeof(uint8_t));

	    rv = diag_l2_send(d_l2_conn, &msg);
	    if(rv < 0) {
	    	fprintf(stderr,
			FLFMT "Could not send an ACK block properly\n", FL);
	   	return rv;
	    }
  	    free(msg.data);

	   /*
	    * And receive the new message
	    */
	    rv = diag_l2_proto_kw1281_recv_msg(d_l2_conn, timeout);
	    if (rv < 0) {
	      fprintf(stderr,
		  FLFMT "Could not receive further messages\n", FL);
	      return rv;
	    }
	    bytes += rv;
 	    tmsg = tmsg->next;

 	    diag_os_millisleep(5);

	}

	return bytes;
}


/*
 * Send a byte, and ensure we get the inverted ack back
 */
static int
diag_l2_proto_kw1281_send_byte(struct diag_l2_conn *d_l2_conn, databyte_type databyte, int end)
{
	uint8_t db = (uint8_t)databyte;
	int rv = 0;

  	diag_os_millisleep(5);

	/* Send the data byte */
	rv = diag_l1_send (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		&db, 1, d_l2_conn->diag_l2_p4min);
	if (rv < 0) {
	  fprintf(stderr,
		FLFMT "Failed to send byte %x\n", FL, db);
	  return rv;
	}

	diag_l2_sendstamp(d_l2_conn); /* update the last sent timer */

	/* Receive the ack ... but only if it is not the end byte! */	
 
	/* receive 1 byte */
        if(end)
	{
// 		diag_os_millisleep(W4min);
	    diag_os_millisleep(5);

	    rv = diag_l1_recv (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		&db, 1, KW_1281_TIM_MIN_P3);
	    if (rv < 0) {
	      fprintf(stderr,
		FLFMT "Failed to receive back the compliment of %x (i.e. %x)\n", FL, databyte, (~databyte & 0xff));
	      return rv;
	    }
	
	    if (db != ((~databyte) & 0xff) ) {
	      fprintf(stderr,
		FLFMT "Received byte: %x is not compliment of sent byte: %x\n", FL, db, databyte);
	      return -1;
	    }
	}
	return rv;
}


/* External interface */

/*
 * The complex initialisation routine for ISOvag, which should support
 * 2 types of initialisation 5-BAUD and FAST (only 5-BAUD implemented here)
 * and functional and physical addressing. The ISOvag spec describes
 * CARB initialisation which is done in the ISO9141 code
 */
#ifdef WIN32
static int
diag_l2_proto_kw1281_startcomms( struct diag_l2_conn *d_l2_conn,
flag_type flags,
int bitrate, target_type target, source_type source)
#else
static int
diag_l2_proto_kw1281_startcomms( struct diag_l2_conn *d_l2_conn,
flag_type flags __attribute__((unused)),
int bitrate, target_type target, source_type source __attribute__((unused)))
#endif
{
	struct diag_serial_settings set;
	struct diag_l2_node *node;
	uint8_t cbuf[MAXRBUF];
	int rv = 0, i, wait_time;
/*	int hdrlen;*/
/*	int datalen;*/
/*	int datasrc;*/

	struct diag_l1_initbus_args in;
	struct diag_l2_kw1281 *dp;

	if (diag_calloc(&dp, 1)) {
	   fprintf(stderr,
	 	FLFMT "diag_calloc failed for KW1281\n", FL);
	   return(DIAG_ERR_NOMEM);
	}

	d_l2_conn->diag_l2_proto_data = dp;
	dp->monitor = NULL;

	/*
	 * diag_l2_p3max is used later for firing stay-alive timeouts, see diag_l2.c, and
	 * for monitoring in KW1281. For KW1281, the full diag_l2_p3max * 2/3
	 * (= 10/3 secs) appears too long between stay-alive pings, so
	 * increase the ping frequency:
	 */
	d_l2_conn->diag_l2_p3max = KW_1281_TIM_MAX_P3;

	/*
	 * The ISO_14230_TIM_MIN_P3 (=55) value for diag_l2_p3max appeared to short for
	 * my (presumably slow!) KW1281 door lock 0x46 ECU, so increase this to 100.
	 * This seemed to have no side effects on interaction with any other ECUs.
	 */
	
	d_l2_conn->diag_l2_p3min = KW_1281_TIM_MIN_P3;
	/*
	 * going to use dp->master later with timeout:
	 * dp->master = 1 means "we are busy with the line", thus
	 * only do a timeout thingy if (!dp->master)
	 */
	dp->master = 1;

	/*
	 * Override "set speed" value; we will probe with 9600 baud
	 */
	set.speed = 9600;
	set.databits = diag_databits_8;
	set.stopbits = diag_stopbits_1;
	set.parflag = diag_par_n;

	/* Set the speed as shown */
	rv = diag_l1_setspeed( d_l2_conn->diag_link->diag_l2_dl0d, &set);
	if (rv < 0) {
	  fprintf(stderr,
		FLFMT "Could not set speed %d\n", FL, set.speed);
	  return rv;
	}

	/* Flush unread input, then wait for idle bus. */
	(void)diag_tty_iflush(d_l2_conn->diag_link->diag_l2_dl0d);
	diag_os_millisleep(300);


	/* Now do 5 baud init of supplied address */
	in.type = DIAG_L1_INITBUS_5BAUD;
	in.addr = target;
	rv = diag_l2_ioctl(d_l2_conn, DIAG_IOCTL_INITBUS, &in);
	if (rv < 0) {
	  fprintf(stderr,
		FLFMT "ioctl INITBUS failed\n", FL);
	  return rv;
	}

	/* Key bytes are in 7-Odd-1, read as 8N1 and ignore parity */

	rv = diag_l1_recv (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		&cbuf[0], 1, d_l2_conn->diag_l2_p3min);
	if (rv < 0) {
	  fprintf(stderr,
		FLFMT "Failed to receive key mode byte\n", FL);
	  return rv;
	}

	rv = diag_l1_recv (d_l2_conn->diag_link->diag_l2_dl0d, 0,
		&cbuf[1], 1, d_l2_conn->diag_l2_p3min);
	if (rv < 0) {
	  fprintf(stderr,
		FLFMT "Failed to receive key mode byte\n", FL);
	  return rv;
	}

	/* Note down the key bytes */
	d_l2_conn->diag_l2_kb1 = cbuf[0] & 0x7f;
	d_l2_conn->diag_l2_kb2 = cbuf[1] & 0x7f;

	if ( (d_l2_conn->diag_link->diag_l2_l1flags
		& DIAG_L1_DOESSLOWINIT) == 0)
	{
		diag_os_millisleep(W4min);
	  	/*
		 * Now transmit KB2 inverted
		 */
		cbuf[0] = ~ cbuf[1];
 
		rv = diag_l1_send (d_l2_conn->diag_link->diag_l2_dl0d, 0,
			&cbuf, 1, d_l2_conn->diag_l2_p4min);

		if (rv < 0) {
	  	   fprintf(stderr,
		 	FLFMT "Failed to send inverse %x of second key byte %x\n", FL, cbuf[0], cbuf[1]);
	  	   return rv;
		}
	}

        /*
	 * Protocol specific things: this now a hybrid of ISO9141'ish stuff
	 * for KW1281 and switching to ISO14230 for KWP20k.  We play a
	 * similar trick to what we did with the speed in diag_l0_ftdi.c
	 * where we switch the d_l2_conn->l2proto to ISO14230 if necessary.
	 */
	
        if (d_l2_conn->diag_l2_kb1 == 0x01 && d_l2_conn->diag_l2_kb2 == 0x0a) {
        /* (VAG) KW1281 */
		printf("VAG KW1281 protocol\n");

	/*
	 * Now receive the first 3 messages
	 * which show ECU versions etc
	 */

		diag_os_millisleep(W4min);

	        /* obtain first messages from ECU */
		rv = diag_l2_recv(d_l2_conn, d_l2_conn->diag_l2_p3min, l2_kw1281_data_rcv, NULL);
// 		rv = diag_l2_proto_kw1281_recv(d_l2_conn, d_l2_conn->diag_l2_p3min, l2_kw1281_data_rcv, NULL);
		if (rv < 0 ) {
	  	   fprintf(stderr,
		 	FLFMT "Receipt of initial blocks from ECU %d failed\n", FL, target);
	  	   return rv;
		}

		dp->state = STATE_ESTABLISHED;

	} else {
	  if (d_l2_conn->diag_l2_kb2 == 0x0f) {
	/* (VAG) KWP20k */
		/* perform some KWP20k consistency checks, see ISO 14230 */
		if(!(d_l2_conn->diag_l2_kb1 & (1<<6)) || (d_l2_conn->diag_l2_kb1 & (1<<5)) == (d_l2_conn->diag_l2_kb1 & (1<<4))) {
			fprintf(stderr, FLFMT "Wierd KWP20k protocol with keywords KB1, KB2: <%x><%x> which is not really allowed according to ISO-14230\n",
				FL, d_l2_conn->diag_l2_kb1, d_l2_conn->diag_l2_kb2);
			return -1;
		}
		
		fprintf(stderr,
		 	FLFMT "Looks like (VAG) KWP%d protocol\n", FL, 1920+(uint8_t)d_l2_conn->diag_l2_kb1);

	  }
	  fprintf(stderr,
		 FLFMT "Failed to connect with KW1281\n", FL);
	  return -1;
	}
	dp->master = 0;

	return 0;
}

/*
 * Send the data
 *
 * Send one single diag_msg and do NOT wait for any response
 *
 */
static int
diag_l2_proto_kw1281_send(struct diag_l2_conn *d_l2_conn, struct diag_msg *msg)
{
	struct diag_l2_kw1281 *dp;
	int i, rv = 0;
	uint8_t databyte, cbuf[256];

	if (diag_l2_debug & DIAG_DEBUG_WRITE)
		fprintf(stderr,
			FLFMT "diag_l2_kw1281_send %p msg %p len %d called\n",
				FL, d_l2_conn, msg, msg->len);

	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	/* Send the length field, which is len + seq + data */
	databyte = msg->len + 2;

	if ( (rv = diag_l2_proto_kw1281_send_byte(d_l2_conn, databyte, 1)) < 0) {
		fprintf(stderr, FLFMT "failed to send length %d\n", FL,
			databyte);
		return rv;
	}
	
	/* Now send the sequence nr */
	if ( (rv = diag_l2_proto_kw1281_send_byte(d_l2_conn, ++dp->seq_nr, 1)) < 0) {
		fprintf(stderr, FLFMT "failed to send seq_nr %d\n", FL,
			dp->seq_nr-1);
		return rv;
	}

	/* Now send the data */
	memcpy(&cbuf[0], msg->data, msg->len);

	for (i=0; i < msg->len; i++)
	{
	  if ( (rv = diag_l2_proto_kw1281_send_byte(d_l2_conn, cbuf[i], 1)) < 0) {
		fprintf(stderr, FLFMT "failed to send %dth byte %d\n", FL,
			i, cbuf[i]);
		return rv;
	  }
	}

	/* And send 0x03 as end of frame */
	databyte = (uint8_t) DIAG_VAG_END_FRAME;
       	if ( (rv = diag_l2_proto_kw1281_send_byte(d_l2_conn, databyte, 0)) < 0) {
		fprintf(stderr, FLFMT "failed to send end of block %x\n", FL,
			databyte);
		return rv;
	}
	
	diag_os_millisleep(5);	

	return (3+msg->len);
}


/*
 * Protocol receive routine
 *
 * Will sleep until a complete set of responses has been received, or fail
 * with a timeout error
 *
 * The interbyte type in data from an ecu is between P1Min and P1Max
 * The intermessage time for part of one response is P2Min and P2Max
 *
 * If we are running with an intelligent L1 interface, then we will be
 * getting one message per frame, and we will wait a bit longer
 * for extra messages
 */
static int
diag_l2_proto_kw1281_recv(struct diag_l2_conn *d_l2_conn, int timeout,
	void (*callback)(void *handle, struct diag_msg *msg),
	void *handle)
{
	int rv = 0;
	struct diag_msg *tmsg;

	/* Call internal routine */
	rv = diag_l2_proto_kw1281_int_recv(d_l2_conn, timeout);

	if (rv < 0) {
		fprintf(stderr, FLFMT "failed to receive messages\n", FL);
		return rv;
	}

	if (diag_l2_debug & DIAG_DEBUG_READ)
	{
		fprintf(stderr, FLFMT "calling rcv callback %p handle %p\n", FL,
			callback, handle);
	}

	if (callback)
	  callback(handle, d_l2_conn->diag_msg);
	
	/* Message no longer needed */
  	diag_freemsg(d_l2_conn->diag_msg);
 	d_l2_conn->diag_msg = NULL;

	if (diag_l2_debug & DIAG_DEBUG_READ)
	{
		fprintf(stderr, FLFMT "rcv callback completed\n", FL);
	}

	return rv;
}

static struct diag_msg *
diag_l2_proto_kw1281_request(struct diag_l2_conn *d_l2_conn, struct diag_msg *msg,
		int *errval)
{
	struct diag_l2_kw1281 *dp;
	int rv = 0;
	
	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	switch((msg->data[0] & 0xff)) {
	  case DIAG_VAG_MONITOR:
	  /* push groups to monitor */
	    dp->monitor = push_monitor(dp->monitor, msg->data[1]);
	    break;
	  case DIAG_VAG_STOP_MONITOR:
	  /* pop groups to monitor */
	    dp->monitor = pop_monitor(dp->monitor, msg->data[1]);
	    break;
	  case DIAG_VAG_SHOW_MONITOR:
	  /* shows groups on monitor list */
	    show_monitor(dp->monitor);
	    break;
	  default:
	    dp->master = 1;
	    rv = diag_l2_send(d_l2_conn, msg);
// 	    rv = diag_l2_proto_kw1281_send(d_l2_conn, msg);
	    if (rv < 0)
	    {
		fprintf(stderr, FLFMT "failed to send message\n", FL);
		*errval = rv;
		return NULL;
	    }

	    /* And wait for response */

	    rv = diag_l2_recv(d_l2_conn, d_l2_conn->diag_l2_p3min, l2_kw1281_data_rcv, NULL);
// 	    rv = diag_l2_proto_kw1281_recv(d_l2_conn, d_l2_conn->diag_l2_p3min, l2_kw1281_data_rcv, NULL);
	    if (rv < 0) {
		/* Error */
		fprintf(stderr, FLFMT "failed to receive messages\n", FL);
		*errval = rv;
		return NULL;
	    }
	    dp->master = 0;
	}
	
	*errval = rv;
	return NULL;
}

/*
 * VAG Stopcomms
 */
#ifdef WIN32
int
diag_l2_proto_kw1281_stopcomms(struct diag_l2_conn* d_l2_conn)
#else
int
diag_l2_proto_kw1281_stopcomms(struct diag_l2_conn* d_l2_conn __attribute__((unused)))
#endif
{
	struct diag_l2_kw1281 *dp;
	struct diag_msg	msg;
	uint8_t databyte;
	int rv = 0;

	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	if(dp->state > STATE_CLOSED) {

 	  if (diag_calloc(&msg.data, 1)) {
 	    fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KW1281\n", FL);
 	    return(DIAG_ERR_NOMEM);
 	  }
 	  msg.len = 1;
 	  databyte = (0xff & (uint8_t) DIAG_VAG_CMD_END_COMMS);
    	  memcpy(msg.data, &databyte, sizeof(uint8_t));

	  dp->master = 1;
	  rv = diag_l2_send(d_l2_conn, &msg);
	  if (rv < 0)
	  {
		fprintf(stderr, FLFMT "failed to send stopcomms message\n", FL);
		return rv;
	  }
	  dp->master = 0;

	  free(msg.data);

	  dp->state = STATE_CLOSED;

	}

	return 0;
}

/*
 * Timeout, - if we don't send something to the ECU it will timeout
 * soon, so send it a keepalive message now.
 */
static void
diag_l2_proto_kw1281_timeout(struct diag_l2_conn *d_l2_conn)
{
	struct diag_l2_kw1281 *dp;
	struct diag_msg	msg;
	struct monitor_type *ptr;
	uint8_t databyte, buff[2];
	int rv = 0;

	dp = (struct diag_l2_kw1281 *)d_l2_conn->diag_l2_proto_data;

	if (diag_l2_debug & DIAG_DEBUG_TIMER)
	{
		fprintf(stderr, FLFMT "timeout impending for %p\n",
				FL, d_l2_conn);
	}
	if(dp->state < STATE_ESTABLISHED || dp->master)
	  return;

	if(dp->monitor) {

	      if (diag_calloc(&msg.data, 2)) {
		fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KW1281\n", FL);
		return;
	      }
	      msg.len = 2;
	      buff[0] = (0xff & (uint8_t) DIAG_VAG_CMD_DATA_OTHER);

	      ptr = dp->monitor;

	      while(ptr) {
		buff[1] = ptr->value;
		memcpy(msg.data, &buff[0], 2*sizeof(uint8_t));

		printf("Group read, group %d\n", ptr->value);
		(void) diag_l2_proto_kw1281_request(d_l2_conn, &msg, &rv);
		if (rv < 0) {
		  free(msg.data);
		  fprintf(stderr, FLFMT "failed to monitor group %d\n", FL, ptr->value);
		  return;
		}
		ptr = ptr->next;
	      }
	} else {
	      if (diag_calloc(&msg.data, 1)) {
		fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KW1281\n", FL);
		return;
	      }
	      msg.len = 1;
	      databyte = (0xff & (uint8_t) DIAG_VAG_CMD_ACK);
	      memcpy(msg.data, &databyte, sizeof(uint8_t));

	      (void) diag_l2_proto_kw1281_request(d_l2_conn, &msg, &rv);

	      if (rv < 0) {
		free(msg.data);
		fprintf(stderr, FLFMT "failed to send a stay-alive ACK on the timeout\n", FL);
		return;
	      }
	}
	free(msg.data);
	    
	return;
}

static const struct diag_l2_proto diag_l2_proto_kw1281 = {
	DIAG_L2_PROT_KW1281, DIAG_L2_FLAG_FRAMED | DIAG_L2_FLAG_DOESCKSUM,
	diag_l2_proto_kw1281_startcomms,
	diag_l2_proto_kw1281_stopcomms,
	diag_l2_proto_kw1281_send,
	diag_l2_proto_kw1281_recv,
	diag_l2_proto_kw1281_request,
	diag_l2_proto_kw1281_timeout
};

int diag_l2_kw1281_add(void) {
	return diag_l2_add_protocol(&diag_l2_proto_kw1281);
}
