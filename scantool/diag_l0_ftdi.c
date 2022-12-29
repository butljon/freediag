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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *************************************************************************
 *
 * diag_l0_ftdi.c
 *
 *     Cloned from diag_l0_vw. by Jonathan Butler
 *     jonathanbutler5@hotmail.com
 *
 *
 * Diag, Layer 0, interface for FT232 chipset compatible interfaces
 * such as Florian Schaeffer's USB KKL interface, see
 * http://www.blafusel.de/obd/obd2_usb-interf-b.html
 *
 * FT232 is a common chip for USB <-> RS232 (hardware) emulation.  My
 * laptop PC no longer had RS232 so I was forced to use an FT232 device
 * over the USB (2.0) port.
 *
 * The problem with FT232 chips is that the lowest supported transfer
 * rate is ~ 183 baud.  The chips appear to have a 3MHz internal clock
 * frequency and one sets the baud rate with a divisor.  The divisor is
 * only 14 bit, this giving the minimum baud rate (3000000 / 2^14), or
 * approximately 183.  This is much too fast for ISO9141, or e.g.
 * VAG KW1281, initialisation or for ISO14230, or e.g. VAG KWP2031,
 * slowinit.
 *
 * I found myself in the unfortunate situation of owning a laptop with no
 * RS232 port and a car (European VW Golf 3) with KW1281 ;-((
 *
 * As a work around, this module, diag_l0_ftdi.c calls a new function,
 * diag_tty_slow_write (see diag_tty.c for implementation) which uses
 * BREAK and then sleep ~ 200ms (actually 200ms - 5%) to transmit a 5
 * baud 0 bit and (non-BREAKs) sleep ~ 200ms to transmit a 5 baud 1 bit.
 *
 * This requires, however, an FT232 kernel driver that understands TIOCSBRK
 * and TIOCCBRK, which is not the case for the at-time-of-writing shipped with
 * standard kernels ftdi_sio.c module, see
 * http://lxr.free-electrons.com/source/drivers/usb/serial/ftdi_sio.c
 *
 * To build an ftdi kernel driver with ioctl that understands TIOCSBRK and
 * TIOCCBRK one can do the following.  i) Create a "pre-configured kernel
 * source tree" by doing: a) Find where the Linux sources for your
 * kernel are (usually under /usr/src/linux/).  b) Copy the complete kernel
 * source tree to a scratch location where you can build, e.g.
 * cp -r /usr/scr/linux ./kernel-build/  c) Locate the kernel .config file
 * that has been used to build your current, running kernel.  This should
 * be (linked to) /boot/config  d) Copy the kernel config file to where
 * you intend to build, e.g. cp /boot/config ./kernel-build/linux/.config
 * ii) Edit ./kernel-build/linux/drivers/usb/serial/ftdi_sio.c by adding
 * lines:
 *
 *     case TIOCSBRK:
 *          ftdi_break_ctl(tty , -1);
 *          return 0;
 *
 *     case TIOCCBRK:
 *          ftdi_break_ctl(tty , 0);
 *          return 0;
 *
 * to the "switch(cmd)" statement in the ftdi_sio.c::ftdi_ioctl function.
 * iii)  Compile the new ftdi_sio driver by calling
 * "make drivers/usb/serial/ftdi_sio.ko" in the kernel-build/linux/
 * directory.  iv) To install the new ftdi_sio.ko: a) Locate where
 * ftdi_sio.ko is currently located (usually in
 * /lib/modules/`uname -r`/kernel/drivers/usb/serial/ ).
 * b) Backup the current ftdi_sio.ko by copying it somewhere.  c) Install
 * the new version by copying, as root,
 * kernel-build/linux/drivers/usb/serial/ftdi_sio.ko to the directory
 * where the current ftdi_sio.ko resides.  Now when you plug an FT232
 * chipset OBDII dongle into your PC, the new ftdi_sio driver will be
 * loaded into the kernel and you are ready to do OBDII.
 *
 * The final thing to note of is that the FT232 module appears to
 * require the "old" diag_os_millisleep for some version <= 2.6 kernels.
 * To enable this, edit the #if statements surrounding diag_os_millisleep
 * in diag_os.c, change ms to 190 in diag_tty_slow_write in diag_tty.c 
 * and re-build freediag.  For 3.2 kernels, leave as is.
 *
 * That's it.  Have fun!
 *
 */


#ifdef WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <errno.h>
#include <stdlib.h>

#include "diag.h"
#include "diag_err.h"
#include "diag_tty.h"
#include "diag_l1.h"

CVSID("$Id: diag_l0_ftdi.c,v 1.0 2012/08/29 15:00:35 butljon Exp $");

/*
 * FT232 chipset compatible ISO-9141 'L and K' Line interface
 * under a POSIX like system connected to a serial port.
 */

struct diag_l0_ftdi_device
{
	int protocol;
	struct diag_serial_settings serial;
};

/* Global init flag */
static int diag_l0_ftdi_initdone;

extern const struct diag_l0 diag_l0_ftdi;

/*
 * Init must be callable even if no physical interface is
 * present, it's just here for the code here to initialise its
 * variables etc
 */
static int
diag_l0_ftdi_init(void)
{
	if (diag_l0_ftdi_initdone)
		return 0;
	diag_l0_ftdi_initdone = 1;

	/* Do required scheduling tweeks */
	diag_os_sched();

	return 0;
}

/*
 * Open the diagnostic device, returns a file descriptor
 * records original state of term interface so we can restore later
 */
static struct diag_l0_device *
diag_l0_ftdi_open(const char *subinterface, int iProtocol)
{
	int rv;
	struct diag_l0_device *dl0d;
	struct diag_l0_ftdi_device *dev;

	if (diag_l0_debug & DIAG_DEBUG_OPEN)
	{
		fprintf(stderr, FLFMT "open subinterface %s protocol %d\n",
			FL, subinterface, iProtocol);
	}

	diag_l0_ftdi_init();

	if ((rv=diag_calloc(&dev, 1)))
		return (struct diag_l0_device *)diag_pseterr(DIAG_ERR_NOMEM);

	dev->protocol = iProtocol;
	if ((rv=diag_tty_open(&dl0d, subinterface, &diag_l0_ftdi, (void *)dev)))
	{
		return (struct diag_l0_device *)diag_pseterr(rv);
	}

	/*
	 * We set RTS to low, and DTR high, because this allows some
	 * interfaces to work than need power from the DTR/RTS lines
	 */
	if (diag_tty_control(dl0d, 1, 0) < 0)
	{
		diag_tty_close(&dl0d);
		return (struct diag_l0_device *)diag_pseterr(DIAG_ERR_GENERAL);
	}

	(void)diag_tty_iflush(dl0d);	/* Flush unread input */

	return dl0d;
}

static int
diag_l0_ftdi_close(struct diag_l0_device **pdl0d)
{
	if (pdl0d && *pdl0d) {
		struct diag_l0_device *dl0d = *pdl0d;
		struct diag_l0_ftdi_device *dev =
			(struct diag_l0_ftdi_device *)diag_l0_dl0_handle(dl0d);

		if (diag_l0_debug & DIAG_DEBUG_CLOSE)
			fprintf(stderr, FLFMT "link %p closing\n",
				FL, dl0d);

		if (dev)
			free(dev);

		(void) diag_tty_close(pdl0d);
	}

	return 0;
}

/*
 * Fastinit:
 */
static int
diag_l0_ftdi_fastinit(struct diag_l0_device *dl0d)
{
	/* Send 25 ms break as initialisation pattern (TiniL) */
	diag_tty_break(dl0d, 25);

	return 0;
}

#if notdef
/* Do the 5 BAUD L line stuff while the K line is twiddling */
#define USDELAY 120
static void
diag_l0_ftdi_Lline(struct diag_l0_device *dl0d, struct diag_l0_ftdi_device *dev,
	uint8_t ecuaddr)
{
	/*
	 * The bus has been high for w0 ms already, now send the
	 * 8 bit ecuaddr at 5 baud LSB first
	 *
	 * NB, most OS delay implementations, other than for highest priority
	 * tasks on a real-time system, only promise to sleep "at least" what
	 * is requested, and only resume at a scheduling quantum, etc.
	 * Since baudrate must be 5baud +/ 5%, we use the -5% value
	 * and let the system extend as needed
	 */
	int i, rv;
	uint8_t cbuf;

	/* Initial state, DTR High, RTS low */

	/*
	 * Set DTR low during this, receive circuitry
	 * will see a break for that time, that we'll clear out later
	 */
	if (diag_tty_control(dl0d, 0, 1) < 0)
	{
		fprintf(stderr, FLFMT "open: Failed to set modem control lines\n",
			FL);
	}

	/* Set RTS low, for 200ms (Start bit) */
	if (diag_tty_control(dl0d, 0, 0) < 0)
	{
		fprintf(stderr, FLFMT "open: Failed to set modem control lines\n",
			FL);
		return;
	}
	diag_os_millisleep(190);		/* 200ms -5% */

	for (i=0; i<8; i++)
	{
		if (ecuaddr & (1<<i))
		{
			/* High */
			rv = diag_tty_control(dl0d, 0, 1);
		}
		else
		{
			/* Low */
			rv = diag_tty_control(dl0d, 0, 0);
		}
		if (rv < 0)
		{
			fprintf(stderr, FLFMT "open: Failed to set modem control lines\n",
				FL);
			return;
		}
		diag_os_millisleep(USDELAY);		    /* 200ms -5% */
	}
	/* And set high for the stop bit */
	if (diag_tty_control(dl0d, 0, 1) < 0)
	{
		fprintf(stderr, FLFMT "open: Failed to set modem control lines\n",
			FL);
		return;
	}
	diag_os_millisleep(USDELAY);		    /* 200ms -5% */

	/* Now put DTR/RTS back correctly so RX side is enabled */
	if (diag_tty_control(dl0d, 1, 0) < 0)
	{
		fprintf(stderr, FLFMT "open: Failed to set modem control lines\n",
			FL);
	}

	/* And clear out the break */
	(void) diag_tty_read(dl0d, &cbuf, 1, 20);

	return;
}
#endif

/*
 * Slowinit:
 *	We need to send a byte (the address) at 5 baud, then
 *	switch to the baud rate the ECU is using.  We try to determine the
 *      latter by how that 0x55 byte appeared when we sampled at 9600 baud.
 *
 * We can use the main chip to do this on the K line but on VAGtool interfaces
 * we also need to do this on the L line which is done by twiddling the RTS
 * line.
 */
static int
diag_l0_ftdi_slowinit(struct diag_l0_device *dl0d, struct diag_l1_initbus_args *in,
	struct diag_l0_ftdi_device *dev)
{
	char cbuf[MAXRBUF];
	int xferd, rv, tout;
	struct diag_serial_settings set;
	
	if (diag_l0_debug & DIAG_DEBUG_PROTO)
	{
		fprintf(stderr, FLFMT "slowinit link %p address 0x%x\n",
			FL, dl0d, in->addr);
	}

	
	/* Set to 7 O 1 */
	set.databits = diag_databits_7;
	set.stopbits = diag_stopbits_1;
	set.parflag = diag_par_o;
	/* and just to avoid divide by zero in diag_tty.c */
	set.speed = 9600;

	if(diag_tty_setup(dl0d, &set) < 0) {
		fprintf(stderr, 
			FLFMT "diag_tty_setup failed!\n", FL);
		return diag_iseterr(DIAG_ERR_GENERAL);	  
	}

	/* Wait W0 (2ms or longer) leaving the bus at logic 1 */
//	diag_os_millisleep(2);
	diag_os_millisleep(200);
//	diag_os_millisleep(100);

	/* Send the address as a single byte message */
  	diag_tty_slow_write(dl0d, &in->addr, 1);

#if notdef
	/* Do the L line stuff */
	diag_l0_ftdi_Lline(dl0d, in->addr);
#endif

	/*
	 * And read back the single byte echo, which shows TX completes
	 * - At 5 baud, it takes 2 seconds to send a byte ..
	 */
// 	diag_os_millisleep(200);
	
 	while ( (xferd = diag_tty_read(dl0d, &cbuf[0], 1, 2750)) <= 0)
	{
		if (xferd == DIAG_ERR_TIMEOUT)
		{
			if (diag_l0_debug & DIAG_DEBUG_PROTO)
				fprintf(stderr, FLFMT "slowinit link %p echo read timeout\n",
					FL, dl0d);
			return diag_iseterr(DIAG_ERR_TIMEOUT);
		}
		if (xferd == 0)
		{
			/* Error, EOF */
			fprintf(stderr, FLFMT "read returned EOF !!\n", FL);
			return diag_iseterr(DIAG_ERR_GENERAL);
		}
		if (errno != EINTR)
		{
			/* Error, EOF */
			perror("read");
			fprintf(stderr, FLFMT "read returned error %d !!\n", FL, errno);
			return diag_iseterr(DIAG_ERR_GENERAL);
		}
	}

	if(diag_tty_setup(dl0d, &dev->serial) < 0) {
		fprintf(stderr, 
			FLFMT "diag_tty_setup failed!\n", FL);
		return diag_iseterr(DIAG_ERR_GENERAL);	  
	}

 	if (dev->protocol == DIAG_L1_ISO9141)
 		tout = 750;		/* 2s is too long */
	else
		tout = 300;		/* 300ms according to ISO14230-2 */

	/*
	 * Don't be too fast, wait W1 (20 to 300ms) before proceeding
	 */
	diag_os_millisleep(100);

	
	/*
	 * Now use the 0x55 sync byte to work out what baud rate the ECU is using.
         * We are probing with 9600 baud, so everything is relative to that.
         * The 0x55 already appears to be transmitted 8N1 at the baud rate the ECU works with.
	 */
	rv = diag_tty_read(dl0d, cbuf, 1, tout);
//	rv = diag_tty_read(dl0d, cbuf, 1, 4000);
	if (rv < 0)
	{
		fprintf(stderr, FLFMT "slowinit link %p read timeout, rv: %d\n",
				FL, dl0d, rv);
		return diag_iseterr(rv);
// XXX
	  
	}
	else
	{
		if (diag_l0_debug & DIAG_DEBUG_PROTO)
			fprintf(stderr, FLFMT "slowinit link %p read 0x%x\n",
				FL, dl0d, cbuf[0]);

	}

        /*
         * Sampling at 9600 baud is good for detecting 9600 or 10400 baud (these
         * rates seeming common in KW1281 and KWP20xy VAG cars) because 9600 and
         * 10400 are coprime and there is thus no conflict of sampling at a point
         * where the line is transitioning high to low or vice versa.
         *
         * If ECU is using 9600 baud then we simply get 0x55 back.  Easy!  We will be
         * sampling at points (5+10k)/96 ms, k = 0,1,2,... relative to where the line initially
         * goes low to transmit the start bit.  Draw the diagram on a sheet of paper.
         * You will see that if ECU is sending its 0x55 8N1 at 10400 baud then, denoting the
         * start bit sent by ECU as the "zero'th" bit, our 5th and 6th samples, i.e. our
         * samples at (5+10k)/96 ms with k=5 and k=6, lie during transmition by ECU of its
         * 5th and 7th bits, respectively.  In other words, our 9600 baud sampling fails to
         * "see" the 6th bit of ECU's 0x55 8N1 at 10400 baud transmition.  We sample then
         * (start bit) 010101 (jump ECU's 6th bit) 101 (stop bit) 1, which is then, reversing
	 * the order of course and in 8N1, 10110101 or 0xb5.
	 *
	 * In practice, the problem with this is that our 5th and 6th samples lie just
	 * before (approx 4 micro secs) when ECU starts sending its 6th bit and just after
	 * (approx 4 micro secs) it stops, respectively.  In testing, presumably small 
	 * time lags of around 4 micro secs magnitude lead to obtaining 0x95 instead
	 * of 0xb5 even when ECU was using 10400 baud.  Although our 6th sample
	 * point is already 4 micro secs into ECU's 7th bit time frame, the line was still
	 * just lingering low from ECU sending its 6th (zero) bit.  Thus we sampled  
	 * (start bit) 010101 (sample 4 micro secs into ECU's 7th bit time frame, but due
	 * to small timming lag, line still low from ECU 6th bit transmition, so actually
	 * sample a zero) 001 (stop bit) 1, which is then 10010101 or 0x95.
	 *
	 * For good measure, although never experienced in testing, the other 2
	 * combinations are also included here.  The 4 combinations below are thus derived
	 * from the cases if we were to sample (start bit) 01010y01 (stop bit) 1, with
	 * y = 00, 01, 10, 11, which, reversing the order and remembering 8N1, give
	 * the 0x85, 0x95, 0xa5, 0xb5 in the switch statement below.
	 *
	 * To derive the hex numbers in the switch statement for 1200, 2400, 4800 baud
	 * take your diagram and just look which bits our 9600 baud sample points
	 * ( (5+10k)/96 ms, k = 0,1,2,... relative to where the line initially goes low
	 * to transmit the start bit) hit in ECU's transmittion pattern and play the
	 * same game as above: take note of the zero start bit, reverse the bit order
	 * and remember 8N1.  1200, 2400, 4800 baud are untested however as I have
	 * no access to ECUs using these rates.
	 *
	 * To my astonishment this all works - and *reliably* - with all ECUs in both
	 * my Golf III & IV models.  Have fun!
	 */

	/*
	 * If necessary, re-set speed, according to how 0x55 looks, prior to the
	 * (void)diag_tty_setup(dl0d, &dev->serial); call in diag_l0_ftdi_initbus.
	 */
	switch(cbuf[0] & 0xff) {
	  case 0x80:
		dev->serial.speed = 1200;
		break;
	  case 0x78:
		dev->serial.speed = 2400;
		break;
	  case 0x66:
		dev->serial.speed = 4800;
		break;
	  case 0x55:
		break;
	  case 0x85:
	  case 0x95:
	  case 0xa5:
	  case 0xb5:
		dev->serial.speed = 10400;
		break;
	  default:
		fprintf(stderr, FLFMT "ECU is using baud rate (under which 0x55 8N1 looks like %x when sampled 9600 baud) which is not equal to 1200, 2400, 4800, 9600 or 10400 and therefore not yet known to this module. Please add required code here!\n",
			FL, (cbuf[0] & 0xff));
		return diag_iseterr(-1);
	}

	printf("Using %d baud ", dev->serial.speed);

	return 0;
}

/*
 * Do wakeup on the bus
 */
static int
diag_l0_ftdi_initbus(struct diag_l0_device *dl0d, struct diag_l1_initbus_args *in)
{
        int rv = DIAG_ERR_INIT_NOTSUPP;

	struct diag_l0_ftdi_device *dev;

	dev = (struct diag_l0_ftdi_device *)diag_l0_dl0_handle(dl0d);

	if (diag_l0_debug & DIAG_DEBUG_IOCTL)
		fprintf(stderr, FLFMT "device link %p info %p initbus type %d\n",
			FL, dl0d, dev, in->type);

	if (!dev)
		return diag_iseterr(DIAG_ERR_GENERAL);

	
	(void)diag_tty_iflush(dl0d);	/* Flush unread input */
	/* Wait the idle time (Tidle > 300ms) */
	diag_os_millisleep(300);

	switch (in->type)
	{
	case DIAG_L1_INITBUS_FAST:
		rv = diag_l0_ftdi_fastinit(dl0d);
		break;

	case DIAG_L1_INITBUS_5BAUD:
		rv = diag_l0_ftdi_slowinit(dl0d, in, dev);
		break;

	default:
		rv = DIAG_ERR_INIT_NOTSUPP;
		break;
	}

	/*
	 * return the baud rate etc to what the user had set
	 * because the init routines will have messed them up
	 */
	if(diag_tty_setup(dl0d, &dev->serial) < 0) {
		fprintf(stderr, 
			FLFMT "diag_tty_setup failed!\n", FL);
		return diag_iseterr(DIAG_ERR_GENERAL);	  
	}

 	return rv;
}



/*
 * Send a load of data
 *
 * Returns 0 on success, -1 on failure
 */
#ifdef WIN32
static int
diag_l0_ftdi_send(struct diag_l0_device *dl0d,
const char *subinterface,
const void *data, size_t len)
#else
static int
diag_l0_ftdi_send(struct diag_l0_device *dl0d,
const char *subinterface __attribute__((unused)),
const void *data, size_t len)
#endif
{
	/*
	 * This will be called byte at a time unless P4 timing parameter is zero
	 * as the L1 code that called this will be adding the P4 gap between
	 * bytes
	 */
	ssize_t xferd;

	if (diag_l0_debug & DIAG_DEBUG_WRITE)
	{
		fprintf(stderr, FLFMT "device link %p send %ld bytes ",
			FL, dl0d, (long)len);
		if (diag_l0_debug & DIAG_DEBUG_DATA)
		{
			diag_data_dump(stderr, data, len);
		}
	}

	while ((size_t)(xferd = diag_tty_write(dl0d, data, len)) != len)
	{
		/* Partial write */
		if (xferd < 0)
		{
			/* error */
			if (errno != EINTR)
			{
				perror("write");
				fprintf(stderr, FLFMT "write returned error %d !!\n", FL, errno);
				return diag_iseterr(DIAG_ERR_GENERAL);
			}
			xferd = 0; /* Interrupted read, nothing transferred. */
		}
		/*
		 * Successfully wrote xferd bytes (or 0 && EINTR),
		 * so inc pointers and continue
		 */
		len -= xferd;
		data = (const void *)((const char *)data + xferd);
	}
	if ( (diag_l0_debug & (DIAG_DEBUG_WRITE|DIAG_DEBUG_DATA)) ==
			(DIAG_DEBUG_WRITE|DIAG_DEBUG_DATA) )
	{
		fprintf(stderr, "\n");
	}

	return 0;
}

/*
 * Get data (blocking), returns number of chars read, between 1 and len
 * If timeout is set to 0, this becomes non-blocking
 */
#ifdef WIN32
static int
diag_l0_ftdi_recv(struct diag_l0_device *dl0d,
const char *subinterface,
void *data, size_t len, int timeout)
#else
static int
diag_l0_ftdi_recv(struct diag_l0_device *dl0d,
const char *subinterface __attribute__((unused)),
void *data, size_t len, int timeout)
#endif
{
	int xferd;

	errno = EINTR;

	struct diag_l0_ftdi_device *dev;
	dev = (struct diag_l0_ftdi_device *)diag_l0_dl0_handle(dl0d);

	if (diag_l0_debug & DIAG_DEBUG_READ)
		fprintf(stderr,
			FLFMT "link %p recv upto %ld bytes timeout %d\n",
			FL, dl0d, (long)len, timeout);

	while ( (xferd = diag_tty_read(dl0d, data, len, timeout)) <= 0)
	{
	      /* No longer report TIMEOUT as an error.  ISO14230
	       * diag_l2_proto_14230_int_recv function for example appears to
	       * rely upon trying to read a whole buffer of data regardless
	       * of how much data is available, which of course results in
	       * a DIAG_ERR_TIMEOUT eventually.  I am not really sure about
	       * this implementation.  My instinct would be to controlled
	       * read the data you would expect to be getting and check that.
	       * After all, getting more or less data than you would expect
	       * or, for that matter, ending up with a DIAG_ERR_TIMEOUT
	       * for some other reason should probably be handled as an
	       * error situation.  Anyway, to minimise
	       * diag_l2_proto_14230_int_recv changes, we are going to now
	       * tolerate DIAG_ERR_TIMEOUT
	       */
		if (xferd == DIAG_ERR_TIMEOUT)
//   			return diag_iseterr(DIAG_ERR_TIMEOUT);
 			return DIAG_ERR_TIMEOUT;
		if (xferd == 0 && len != 0)
		{
			/* Error, EOF */
			fprintf(stderr, FLFMT "read returned EOF !!\n", FL);
			return diag_iseterr(DIAG_ERR_GENERAL);
		}
		if (errno != EINTR)
		{
			/* Error, EOF */
			fprintf(stderr, FLFMT "read returned error %d !!\n", FL, errno);
			return diag_iseterr(DIAG_ERR_GENERAL);
		}
	}

	return xferd;
}

/*
 * Set speed/parity etc
 */
static int
diag_l0_ftdi_setspeed(struct diag_l0_device *dl0d,
const struct diag_serial_settings *pset)
{
	struct diag_l0_ftdi_device *dev;
	dev = (struct diag_l0_ftdi_device *)diag_l0_dl0_handle(dl0d);

	dev->serial = *pset;

	return diag_tty_setup(dl0d, &dev->serial);
}

#ifdef WIN32
static int
diag_l0_ftdi_getflags(struct diag_l0_device *dl0d)
#else
static int
diag_l0_ftdi_getflags(struct diag_l0_device *dl0d __attribute__((unused)))
#endif
{
	/* All interface types here use same flags */
	return(
		DIAG_L1_SLOW | DIAG_L1_FAST | DIAG_L1_PREFFAST |
			DIAG_L1_HALFDUPLEX
		);
}

const struct diag_l0 diag_l0_ftdi = {
 	"FT232 Chipset USB Interface",
	"FTDI",
	DIAG_L1_ISO9141 | DIAG_L1_ISO14230 | DIAG_L1_RAW,
	diag_l0_ftdi_init,
	diag_l0_ftdi_open,
	diag_l0_ftdi_close,
	diag_l0_ftdi_initbus,
	diag_l0_ftdi_send,
	diag_l0_ftdi_recv,
	diag_l0_ftdi_setspeed,
	diag_l0_ftdi_getflags
};

#if defined(__cplusplus)
extern "C" {
#endif
extern int diag_l0_ftdi_add(void);
#if defined(__cplusplus)
}
#endif

int
diag_l0_ftdi_add(void) {
	return diag_l1_add_l0dev(&diag_l0_ftdi);
}
