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
 *
 * Mostly ODBII Compliant Scan Tool (as defined in SAE J1978)
 *
 * CLI routines - vag subcommand
 */

#include "diag.h"
#include "diag_l1.h"
#include "diag_l2.h"

#include "diag_vag.h"
#include "diag_iso14230.h"
#include "diag_l2_iso14230.h"
#include "scantool.h"
#include "scantool_cli.h"

#define IS_KW1281(p) (((p)->diag_l2_kb1 == 0x01 && (p)->diag_l2_kb2 == 0x0a) ? 1 : 0)
#define IS_KWP2K(p) (((p)->diag_l2_kb2 == 0x0f) ? 1 : 0)


CVSID("$Id: scantool_vag.c,v 1.2 2009/06/25 02:13:18 fenugrec Exp $");

static int cmd_vag_help(int argc, char **argv);
static int cmd_vag_connect(int argc, char **argv);
static int cmd_vag_disconnect(int argc, char **argv);
static int cmd_vag_request(int argc, char **argv);
static int cmd_vag_reqdtc(int argc, char **argv);
static int cmd_vag_cleardtc(int argc, char **argv);
static int cmd_vag_groupread(int argc, char **argv);
static int cmd_vag_channelread(int argc, char **argv);
static int cmd_vag_sriread(int argc, char **argv);
static int cmd_vag_monitor(int argc, char **argv);
static int cmd_vag_stopmonitor(int argc, char **argv);
static int cmd_vag_showmonitor(int argc, char **argv);

const struct cmd_tbl_entry vag_cmd_table[] =
{
	{ "help", "help [command]", "Gives help for a command",
		cmd_vag_help, 0, NULL},
	{ "connect", "connect (<ECUId>)", "Connect to ECU", cmd_vag_connect, 0, NULL},
	{ "disconnect", "disconnect", "Disconnect from ECU", cmd_vag_disconnect,
		0, NULL},
	{ "request", "request <reqId> (<reqParm> <reqParm> ...)", "Send a single request to ECU", cmd_vag_request,
		0, NULL},
	{ "reqdtc", "reqdtc", "Request DTCs from ECU", cmd_vag_reqdtc,
		0, NULL},
	{ "cleardtc", "cleardtc", "Clear DTCs", cmd_vag_cleardtc,
		0, NULL},
	{ "groupread", "groupread (<grpId> <grpId> ...)", "Read sensor group data", cmd_vag_groupread,
		0, NULL},
	{ "channelread", "channelread (<chanId> <chanId> ...", "Read an adaptation channel", cmd_vag_channelread,
		0, NULL},
	{ "sriread", "sriread", "Read SRI channel values", cmd_vag_sriread,
		0, NULL},
	{ "monitor", "monitor <grpId> (<grpId> ...)", "Add sensor group to monitor watch list", cmd_vag_monitor,
		0, NULL},
	{ "stopmonitor", "stopmonitor <grpId> (<grpId> ...)", "Remove sensor group to monitor watch list", cmd_vag_stopmonitor,
		0, NULL},
	{ "showmonitor", "showmonitor", "Shows monitor watch list contents", cmd_vag_showmonitor,
		0, NULL},
	{ "up", "up", "Return to previous menu level",
		cmd_up, 0, NULL},
	{ "quit","quit", "Return to previous menu level",
		cmd_up, FLAG_HIDDEN, NULL},
	{ "exit", "exit", "Exit program",
		cmd_exit, 0, NULL},
	{ NULL, NULL, NULL, NULL, 0, NULL}
};


/*
 * Table of english descriptions of the VW ECU addresses
 */
struct vw_id_info
{
	const int id;
	const char *command;
} ;

const struct vw_id_info vw_ids[] =
{
	{DIAG_VAG_ECU_ENGINE, "Engine" },
	{DIAG_VAG_ECU_GEARBOX, "Gearbox" },
	{DIAG_VAG_ECU_ABS, "ABS" },
	{DIAG_VAG_ECU_AIRBAGS, "Airbag" },
	{DIAG_VAG_ECU_LOCKS, "Locking" },
	{0, NULL},
};

/*
 * KW1281 init
 */
int 
do_l2_kw1281_start(int destaddr)
{
	set_L1protocol = PROTOCOL_ISO9141;
	set_L2protocol = DIAG_L2_PROT_KW1281;
	set_speed = 9600;

	return do_l2_generic_start();

}

/*
 * (VAG) KWP20k init
 */
int
do_l2_kwp20k_start(int init_type)
{
	struct diag_l2_conn *d_conn;
	flag_type flags = 0;
	struct diag_msg	msg;
	int errVal=0, rv;
	uint8_t buff[64];

	set_speed = 9600;
 	flags = DIAG_L2_TYPE_PHYSCONN;
// 	flags = DIAG_L2_TYPE_FUNCADDR;
	flags |= (init_type & DIAG_L2_TYPE_INITMASK) ;

	d_conn = do_l2_common_start(DIAG_L1_ISO14230, DIAG_L2_PROT_ISO14230,
		flags, set_speed, set_destaddr, 0xf1);

	if (d_conn == NULL)
		return -1;

	/*
	 * diag_l2_p3max is used later for firing stay-alive timeouts, see diag_l2.c, and
	 * for monitoring in KWP2x. For KWP2x, the full diag_l2_p3max * 2/3
	 * (= 10/3 secs) appears too long between stay-alive pings, so
	 * increase the ping frequency:
	 */
        d_conn->diag_l2_p3max = ISO_14230_TIM_MAX_P3 / 5;

	/* Connected ! */
	global_l2_conn = d_conn;
	
	msg.len = 2;

 	if (diag_calloc(&msg.data, msg.len)) {
 	    fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KWPx\n", FL);
 	    return(DIAG_ERR_NOMEM);
 	}

	buff[0] = DIAG_KW2K_SI_STADS;
	buff[1] = 0x89;
	memcpy(msg.data, &buff[0], msg.len*sizeof(uint8_t));

//	diag_os_millisleep(25);
	/*
	 * Important to get a diag_l2_send call in here because the timestamps for calls to the
	 * timeout function (to send DIAG_KW2K_SI_TP messages to maintain the connection) are reset
	 * through diag_l2_send.  If don't do this then the timeout timer will wake up, see that
	 * dp->state == STATE_ESTABLISHED, compare timestamps find a completely old one and
	 * fire off an DIAG_KW2K_SI_TP message which will then disturb the still being initialised
	 * connection
	 */

	(void) diag_l2_send(global_l2_conn, &msg);
	if(errVal < 0) {
		fprintf(stderr, FLFMT "Failed to send request\n", FL);
		return CMD_FAILED;
	}
	free(msg.data);

	diag_os_millisleep(25);
	rv = diag_l2_recv(global_l2_conn, global_l2_conn->diag_l2_p3min, l2_iso14230_data_rcv, NULL);
// 	rv =diag_l2_recv(global_l2_conn, 1000, NULL, NULL);
	
	msg.len = 2;

 	if (diag_calloc(&msg.data, msg.len)) {
 	    fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KWPx\n", FL);
 	    return(DIAG_ERR_NOMEM);
 	}

	buff[0] = DIAG_KW2K_SI_REID;
// could try without 0x9B here: 2nd parameter seems to optional anyway
	buff[1] = 0x9B;
	memcpy(msg.data, &buff[0], msg.len*sizeof(uint8_t));

	diag_os_millisleep(25);
	(void) diag_l2_send(global_l2_conn, &msg);
	if(errVal < 0) {
		fprintf(stderr, FLFMT "Failed to send request\n", FL);
		return CMD_FAILED;
	}
	free(msg.data);

	diag_os_millisleep(25);
	rv = diag_l2_recv(global_l2_conn, global_l2_conn->diag_l2_p3min, l2_iso14230_data_rcv, NULL);
// 	rv =diag_l2_recv(global_l2_conn, 1000, NULL, NULL);
	
	return 0;
}


const struct protocol protocols_vag[] = {
   	{"KW1281", do_l2_kw1281_start, 0x01, PROTOCOL_ISO9141, DIAG_L2_TYPE_SLOWINIT},
	{"KWP2000", do_l2_kwp20k_start, DIAG_L2_TYPE_SLOWINIT, PROTOCOL_ISO14230, DIAG_L2_TYPE_SLOWINIT},
};

static int
cmd_vag_help(int argc, char **argv)
{
	return help_common(argc, argv, vag_cmd_table);
}

#ifdef WIN32
int
cmd_vag_connect(int argc,
char **argv)
#else
int
cmd_vag_connect(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int rv = 0, connected = 0;
	const struct protocol *p;

	if (global_state >= STATE_CONNECTED)
	{
		fprintf(stderr, FLFMT "Not needed, already connected to ECU %d\n", FL, set_destaddr);
		return CMD_OK;
	}

	if(argc == 2)
	  set_destaddr = atoi( argv[1]);
	else {
	  if(argc == 1)
	    set_destaddr = store_set_destaddr;
	  else {
		fprintf(stderr, FLFMT "Connect must be called with one optional argument <ECUId>\n", FL);
		return CMD_FAILED;
	  }
	}	    

	p=protocols_vag;
	if (set_L1protocol == 64) p++;

	if(set_L1protocol == 32 || set_L1protocol == 64) {
		fprintf(stderr,"Trying %s:\n", p->desc);
		rv = p->start(p->flags);
		if (rv == 0) {
			global_conmode = p->conmode;
			global_protocol = p->protoID;
			connected = 1;
			global_state = STATE_CONNECTED;
			printf("%s connected to ECU %d.\n", p->desc, set_destaddr);
		} else {
			fprintf(stderr, FLFMT "%s failed to connect to ECU %d\n", FL, p->desc, set_destaddr);
			return CMD_FAILED;
		}
	}
	
	for (p = protocols_vag; !connected && p < &protocols_vag[ARRAY_SIZE(protocols_vag)]; p++) {
		fprintf(stderr,"Trying %s:\n", p->desc);
		rv = p->start(p->flags);
		if (rv == 0) {
			global_conmode = p->conmode;
			global_protocol = p->protoID;
			connected = 1;
			global_state = STATE_CONNECTED;
			printf("%s connected to ECU %d.\n", p->desc, set_destaddr);
		} else {
			fprintf(stderr, FLFMT "%s failed to connect to ECU %d\n", FL, p->desc, set_destaddr);
			/* wait 3 secs before attempting next protocol */
 			diag_os_millisleep(ISO_14230_TIM_MAX_P3);
		}
	}

	if (rv<0)
	{
		printf("\nConnection to ECU failed %d\n", set_destaddr);
		printf("Please check :-\n");
		printf("	Adapter is connected to PC\n");
		printf("	Cable is connected to Vehicle\n");
		printf("	Vehicle is switched on\n");
		return CMD_FAILED;
	}
	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_disconnect(int argc,
char **argv)
#else
int
cmd_vag_disconnect(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	if (global_state < STATE_CONNECTED)
	{
		fprintf(stderr, FLFMT "Not connected to ECU\n", FL);
		return CMD_OK;
	}

	diag_l2_StopCommunications(global_l2_conn);
	diag_l2_close(global_l2_dl0d);

	global_l2_conn = NULL;
	global_state = STATE_IDLE;
	
	printf("\nDisconnected from ECU %d\n", set_destaddr);

	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_request(int argc,
char **argv)
#else
int
cmd_vag_request(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	struct diag_msg	msg;
	int errVal=0, i;
	uint8_t buff[257];

// if(0) {
 	if (global_state < STATE_CONNECTED) {
		fprintf(stderr, FLFMT "Not connected to ECU\n", FL);
		return CMD_OK;
	}

        if(argc < 2) {
		fprintf(stderr, FLFMT "This command must be called with a value\n", FL);
		return CMD_FAILED;
	}

	msg.len = (argc-1);

 	if (diag_calloc(&msg.data, msg.len)) {
 	    fprintf(stderr,
 		 	FLFMT "diag_calloc failed for KW1281\n", FL);
 	    return(DIAG_ERR_NOMEM);
 	}

	for(i=1; i<msg.len+1; i++) {
	  buff[i-1] = atoi(argv[i]);
// 	  printf("buff[%d]: <%x>\n", i-1, buff[i-1]);
	}
	memcpy(msg.data, &buff[0], msg.len*sizeof(uint8_t));

	(void) diag_l2_request(global_l2_conn, &msg, &errVal);
	if(errVal < 0) {
		fprintf(stderr, FLFMT "Failed to send request\n", FL);
		return CMD_FAILED;
	}
	free(msg.data);

	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_reqdtc(int argc,
char **argv)
#else
int
cmd_vag_reqdtc(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	char buff[4], *buff2[5];

	if(argc != 1) {
		fprintf(stderr, FLFMT "reqdtc does not support command line parameters\n", FL);
		return CMD_FAILED;
	}

	if(IS_KW1281(global_l2_conn)) {
	  snprintf(buff, 4, "%d", DIAG_VAG_CMD_DTC_RQST);
	  buff2[1] = buff;

	  return cmd_vag_request(2, buff2);
	}
	if(IS_KWP2K(global_l2_conn)) {
	  snprintf(buff, 4, "%d", DIAG_KW2K_SI_RDTCBS);
	  buff2[1] = malloc(4);
	  memcpy(buff2[1], &buff[0], 4);
	  snprintf(buff, 4, "%d", 2);
	  buff2[2] = malloc(4);
	  memcpy(buff2[2], &buff[0], 4);
	  snprintf(buff, 4, "%d", 255);
	  buff2[3] = malloc(4);
	  memcpy(buff2[3], &buff[0], 4);
	  snprintf(buff, 4, "%d", 255);
	  buff2[4] = malloc(4);
	  memcpy(buff2[4], &buff[0], 4);
	  
	  return cmd_vag_request(5, buff2);
	}
}

#ifdef WIN32
int
cmd_vag_cleardtc(int argc,
char **argv)
#else
int
cmd_vag_cleardtc(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	char buff[4], *buff2[2];

	if(argc != 1) {
		fprintf(stderr, FLFMT "cleardtc does not support command line parameters\n", FL);
		return CMD_FAILED;
	}

	snprintf(buff, 4, "%d", DIAG_VAG_CMD_DTC_CLEAR);
	buff2[1] = buff;

	return cmd_vag_request(2, buff2);
}

#ifdef WIN32
int
cmd_vag_groupread(int argc,
char **argv)
#else
int
cmd_vag_groupread(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int i, rv = 0, start=0, end=255;
	char buff[4], *buff2[3];

	if(IS_KW1281(global_l2_conn))
	  snprintf(buff, 4, "%d", DIAG_VAG_CMD_DATA_OTHER);
	if(IS_KWP2K(global_l2_conn)) {
	  start=1;
	  end=240;
	  snprintf(buff, 4, "%d", DIAG_KW2K_SI_RDDBLI);
	}
	buff2[1] = malloc(4);
	memcpy(buff2[1], &buff[0], 4);

	if(argc == 1) {
	  for(i=start; i<end; i++) {
	    snprintf(buff, 4, "%d", i);
	    buff2[2] = malloc(4);
	    memcpy(buff2[2], &buff[0], 4);

	    printf("Group read, group %d\n", i);
	    rv = cmd_vag_request(3, buff2);
	    if(rv != CMD_OK) {
		fprintf(stderr, FLFMT "Failed read group %d\n", FL, i);
		return CMD_FAILED;	      
	    }
	    diag_os_millisleep(25);
	  }
  	}
  	else {
        for(i=1; i<argc; i++) {
            snprintf(buff, 4, "%d", atoi(argv[i]));
            buff2[2] = malloc(4);
            memcpy(buff2[2], &buff[0], 4);

            printf("Group read, group %d\n", atoi(argv[i]));
            rv = cmd_vag_request(3, buff2);
            if(rv != CMD_OK) {
            fprintf(stderr, FLFMT "Failed read group %d\n", FL, atoi(argv[i]));
            return CMD_FAILED;	      
            }
        }
	}
	

	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_channelread(int argc,
char **argv)
#else
int
cmd_vag_channelread(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int i, rv = 0;
	char buff[4], *buff2[3];

	snprintf(buff, 4, "%d", DIAG_VAG_CMD_ADP_READ);
	buff2[1] = malloc(4);
	memcpy(buff2[1], &buff[0], 4);

	if(argc == 1) {
	  for(i=0; i<256; i++) {
	    snprintf(buff, 4, "%d", i);
	    buff2[2] = malloc(4);
	    memcpy(buff2[2], &buff[0], 4);

	    printf("Channel read, channel %d\n", i);
	    rv = cmd_vag_request(3, buff2);
	    if(rv != CMD_OK) {
		fprintf(stderr, FLFMT "Failed read channel %d\n", FL, i);
		return CMD_FAILED;	      
	    }
	  }
  	}
  	else {
	    i = atoi( argv[1]);
	    snprintf(buff, 4, "%d", i);
	    buff2[2] = malloc(4);
	    memcpy(buff2[2], &buff[0], 4);

	    printf("Channel read, channel %d\n", i);
	    rv = cmd_vag_request(3, buff2);
	    if(rv != CMD_OK) {
		fprintf(stderr, FLFMT "Failed read channel %d\n", FL, i);
		return CMD_FAILED;	      
	  }
	}

	
	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_sriread(int argc,
char **argv)
#else
int
cmd_vag_sriread(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int i, rv = 0;
	char buff[4], *buff2[3];
	int sri_channels[9] = {2, 40, 41, 42, 43, 44, 45, 47, 48};

	snprintf(buff, 4, "%d", DIAG_VAG_CMD_ADP_READ);
	buff2[1] = malloc(4);
	memcpy(buff2[1], &buff[0], 4);

	for(i=0; i<9; i++) {
	    snprintf(buff, 4, "%d", sri_channels[i]);
	    buff2[2] = malloc(4);
	    memcpy(buff2[2], &buff[0], 4);

	    printf("Channel read, channel %d\n", sri_channels[i]);
	    rv = cmd_vag_request(3, buff2);
	    if(rv != CMD_OK) {
		fprintf(stderr, FLFMT "Failed read channel %d\n", FL, sri_channels[i]);
		return CMD_FAILED;	      
	    }
	}

	
	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_monitor(int argc,
char **argv)
#else
int
cmd_vag_monitor(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int i, rv = 0;
	uint8_t group;
	char buff[4], *buff2[3];

	snprintf(buff, 4, "%d", DIAG_VAG_MONITOR);
	buff2[1] = malloc(4);
	memcpy(buff2[1], &buff[0], 4);

	if(argc < 2) {
	  fprintf(stderr, FLFMT "A group must be specified to add to monitor watch list\n", FL);
	  return CMD_FAILED;
	}
	
	for(i=1; i<argc; i++) {
	  group = atoi( argv[i]);
	  snprintf(buff, 4, "%d", group);
	  buff2[2] = malloc(4);
	  memcpy(buff2[2], &buff[0], 4);

	  printf("Monitoring group %d\n", group);
	  rv = cmd_vag_request(3, buff2);
	  if(rv != CMD_OK) {
	    fprintf(stderr, FLFMT "Failed to add group %d to monitor watch list\n", FL, group);
	    return CMD_FAILED;	      
	  }
	}
	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_stopmonitor(int argc,
char **argv)
#else
int
cmd_vag_stopmonitor(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	int i, rv = 0;
	uint8_t group;
	char buff[4], *buff2[3];

	snprintf(buff, 4, "%d", DIAG_VAG_STOP_MONITOR);
	buff2[1] = malloc(4);
	memcpy(buff2[1], &buff[0], 4);

	if(argc < 2) {
	  fprintf(stderr, FLFMT "A group must be specified to remove from monitor watch list\n", FL);
	  return CMD_FAILED;
	}
	
	for(i=1; i<argc; i++) {
	  group = atoi( argv[i]);
	  snprintf(buff, 4, "%d", group);
	  buff2[2] = malloc(4);
	  memcpy(buff2[2], &buff[0], 4);

	  printf("Stop monitoring group %d\n", group);
	  rv = cmd_vag_request(3, buff2);
	  if(rv != CMD_OK) {
	    fprintf(stderr, FLFMT "Failed to remove group %d from monitor watch list\n", FL, group);
	    return CMD_FAILED;	      
	  }
	}
	return CMD_OK;
}

#ifdef WIN32
int
cmd_vag_showmonitor(int argc,
char **argv)
#else
int
cmd_vag_showmonitor(int argc __attribute__((unused)),
char **argv __attribute__((unused)))
#endif
{
	char buff[4], *buff2[2];

	if(argc != 1) {
		fprintf(stderr, FLFMT "showmonitor does not support command line parameters\n", FL);
		return CMD_FAILED;
	}

	snprintf(buff, 4, "%d", DIAG_VAG_SHOW_MONITOR);
	buff2[1] = buff;

	return cmd_vag_request(2, buff2);

}

