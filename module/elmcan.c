/*
 * elmcan.c - ELM327 based CAN interface driver
 *            (tty line discipline)
 *
 * This file is derived from linux/drivers/net/can/slcan.c
 *
 * elmcan.c Author : Max Staudt <elmcan@enpas.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 */

#define pr_fmt(fmt) "[elmcan] " fmt


#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/workqueue.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>


MODULE_ALIAS_LDISC(N_ELMCAN);
MODULE_DESCRIPTION("ELM327 based CAN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Max Staudt <max-linux@enpas.org>");

/* Line discipline ID number */
#ifndef N_ELMCAN
#define N_ELMCAN 29
#endif

#define ELM327_CAN_CONFIG_SEND_SFF           0x8000
#define ELM327_CAN_CONFIG_VARIABLE_DLC       0x4000
#define ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF  0x2000
#define ELM327_CAN_CONFIG_BAUDRATE_MULT_8_7  0x1000

#define ELM327_MAGIC_CHAR 'y'
#define ELM327_MAGIC_STRING "y"
#define ELM327_READY_CHAR '>'


/* Bits in elm->cmds_todo */
enum ELM_TODO {
	ELM_TODO_CAN_DATA = 0,
	ELM_TODO_CANID_11BIT,
	ELM_TODO_CANID_29BIT_LOW,
	ELM_TODO_CANID_29BIT_HIGH,
	ELM_TODO_CAN_CONFIG,
	ELM_TODO_RESPONSES,
	ELM_TODO_SILENT_MONITOR,
	ELM_TODO_INIT
};


struct elmcan {
	/* This must be the first member when using alloc_candev() */
	struct can_priv can;

	/* TTY and netdev devices that we're bridging */
	struct tty_struct	*tty;
	struct net_device	*dev;

	char			ifname[IFNAMSIZ];

	/* Per-channel lock */
	spinlock_t		lock;

	/* Keep track of how many things are using this struct.
	 * Once it reaches 0, we are in the process of cleaning up,
	 * and new operations will be cancelled immediately.
	 * Use atomic_t rather than refcount_t because we deliberately
	 * decrement to 0, and refcount_dec() spills a WARN_ONCE in
	 * that case.
	 */
	atomic_t		refcount;

	/* TTY TX helpers */
	struct work_struct	tx_work;	/* Flushes TTY TX buffer   */
	unsigned char		txbuf[32];
	unsigned char		*txhead;	/* Pointer to next TX byte */
	int			txleft;		/* Bytes left to TX */

	/* TTY RX helpers */
	unsigned char rxbuf[256];
	int rxfill;

	/* State machine */
	enum {
		ELM_NOTINIT = 0,
		ELM_GETMAGICCHAR,
		ELM_GETPROMPT,
		ELM_RECEIVING,
	} state;

	int drop_next_line;

	/* The CAN frame and config the ELM327 is sending/using,
	 * or will send/use after finishing all cmds_todo */
	struct can_frame can_frame;
	unsigned short can_config;
	unsigned long can_bitrate;
	unsigned char can_bitrate_divisor;
	int silent_monitoring;

	/* Things we have yet to send */
	char **next_init_cmd;
	unsigned long cmds_todo;
};


/* A lock for all tty->disc_data handled by this ldisc.
 * This is to prevent a case where tty->disc_data is set to NULL,
 * yet someone is still trying to dereference it.
 * Without this, we cannot do a clean shutdown.
 */
static DEFINE_SPINLOCK(elmcan_discdata_lock);


static inline void elm327_hw_failure(struct elmcan *elm);



 /************************************************************************
  *		ELM327: Transmission				*
  *								*
  * (all functions assume elm->lock taken)			*
  ************************************************************************/

static void elm327_send(struct elmcan *elm, const void *buf, size_t len)
{
	int actual;

	memcpy(elm->txbuf, buf, len);

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);
	actual = elm->tty->ops->write(elm->tty, elm->txbuf, len);
	if (actual < 0) {
		pr_err("Failed to write to tty for %s.\n", elm->dev->name);
		elm327_hw_failure(elm);
	}

	elm->txleft = len - actual;
	elm->txhead = elm->txbuf + actual;
}


/*
 * Take the ELM327 out of almost any state and back into command mode
 *
 * Assumes elm->lock taken.
 */
static void elm327_kick_into_cmd_mode(struct elmcan *elm)
{
	if (elm->state != ELM_GETMAGICCHAR && elm->state != ELM_GETPROMPT) {
		elm327_send(elm, ELM327_MAGIC_STRING, 1);

		elm->state = ELM_GETMAGICCHAR;
		elm->rxfill = 0;
	}
}


/*
 * Schedule a CAN frame, and any necessary config changes,
 * to be sent down the TTY.
 *
 * Assumes elm->lock taken.
 */
static void elm327_send_frame(struct elmcan *elm, struct can_frame *frame)
{
	/* Schedule any necessary changes in ELM327's CAN configuration */
	if (elm->can_frame.can_id != frame->can_id) {
		/* Set the new CAN ID for transmission. */
		if ((frame->can_id & CAN_EFF_FLAG) ^ (elm->can_frame.can_id & CAN_EFF_FLAG)) {
			elm->can_config = (frame->can_id & CAN_EFF_FLAG ? 0 : ELM327_CAN_CONFIG_SEND_SFF)
					| ELM327_CAN_CONFIG_VARIABLE_DLC
					| ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF
					| elm->can_bitrate_divisor;

			set_bit(ELM_TODO_CAN_CONFIG, &elm->cmds_todo);
		}

		if (frame->can_id & CAN_EFF_FLAG) {
			clear_bit(ELM_TODO_CANID_11BIT, &elm->cmds_todo);
			set_bit(ELM_TODO_CANID_29BIT_LOW, &elm->cmds_todo);
			set_bit(ELM_TODO_CANID_29BIT_HIGH, &elm->cmds_todo);
		} else {
			set_bit(ELM_TODO_CANID_11BIT, &elm->cmds_todo);
			clear_bit(ELM_TODO_CANID_29BIT_LOW, &elm->cmds_todo);
			clear_bit(ELM_TODO_CANID_29BIT_HIGH, &elm->cmds_todo);
		}
	}

	/* Schedule the CAN frame itself. */
	elm->can_frame = *frame;
	set_bit(ELM_TODO_CAN_DATA, &elm->cmds_todo);

	elm327_kick_into_cmd_mode(elm);
}



 /************************************************************************
  *		ELM327: Initialization sequence			*
  *								*
  * (assumes elm->lock taken)					*
  ************************************************************************/

static char *elm327_init_script[] = {
	"AT WS\r",        /* v1.0: Warm Start */
	"AT PP FF OFF\r", /* v1.0: All Programmable Parameters Off */
	"AT M0\r",        /* v1.0: Memory Off */
	"AT AL\r",        /* v1.0: Allow Long messages */
	"AT BI\r",        /* v1.0: Bypass Initialization */
	"AT CAF0\r",      /* v1.0: CAN Auto Formatting Off */
	"AT CFC0\r",      /* v1.0: CAN Flow Control Off */
	"AT CF 000\r",    /* v1.0: Reset CAN ID Filter */
	"AT CM 000\r",    /* v1.0: Reset CAN ID Mask */
	"AT E1\r",        /* v1.0: Echo On */
	"AT H1\r",        /* v1.0: Headers On */
	"AT L0\r",        /* v1.0: Linefeeds Off */
	"AT SH 7DF\r",    /* v1.0: Set CAN sending ID to 0x7df */
	"AT ST FF\r",     /* v1.0: Set maximum Timeout for response after TX */
	"AT AT0\r",       /* v1.2: Adaptive Timing Off */
	"AT D1\r",        /* v1.3: Print DLC On */
	"AT S1\r",        /* v1.3: Spaces On */
	"AT TP B\r",      /* v1.0: Try Protocol B */
	NULL
};


static void elm327_init(struct elmcan *elm)
{
	elm->state = ELM_NOTINIT;
	elm->can_frame.can_id = 0x7df;
	elm->rxfill = 0;
	elm->drop_next_line = 0;

	/* We can only set the bitrate as a fraction of 500000.
	 * The bit timing constants in elmcan_bittiming_const will
	 * limit the user to the right values.
	 */
	elm->can_bitrate_divisor = 500000 / elm->can.bittiming.bitrate;
	elm->can_config = ELM327_CAN_CONFIG_SEND_SFF
			| ELM327_CAN_CONFIG_VARIABLE_DLC
			| ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF
			| elm->can_bitrate_divisor;

	/* Configure ELM327 and then start monitoring */
	elm->next_init_cmd = &elm327_init_script[0];
	set_bit(ELM_TODO_INIT, &elm->cmds_todo);
	set_bit(ELM_TODO_SILENT_MONITOR, &elm->cmds_todo);
	set_bit(ELM_TODO_RESPONSES, &elm->cmds_todo);
	set_bit(ELM_TODO_CAN_CONFIG, &elm->cmds_todo);

	elm327_kick_into_cmd_mode(elm);
}



 /************************************************************************
  *		ELM327: Reception -> netdev glue		*
  *								*
  * (assumes elm->lock taken)					*
  ************************************************************************/

static void elm327_feed_frame_to_netdev(struct elmcan *elm, const struct can_frame *frame)
{
	struct can_frame *cf;
	struct sk_buff *skb;

	if (!netif_running(elm->dev)) {
		return;
	}

	skb = alloc_can_skb(elm->dev, &cf);

	if (!skb)
		return;

	memcpy(cf, frame, sizeof(struct can_frame));

	elm->dev->stats.rx_packets++;
	elm->dev->stats.rx_bytes += frame->can_dlc;
	netif_rx_ni(skb);

	can_led_event(elm->dev, CAN_LED_EVENT_RX);
}



 /************************************************************************
  *		ELM327: "Panic" handler				*
  *								*
  * (assumes elm->lock taken)					*
  ************************************************************************/

/* Called when we're out of ideas and just want it all to end. */
static inline void elm327_hw_failure(struct elmcan *elm)
{
	struct can_frame frame = {0};

	frame.can_id = CAN_ERR_FLAG | CAN_ERR_RESTARTED;
	frame.can_dlc = CAN_ERR_DLC;
	elm327_feed_frame_to_netdev(elm, &frame);

	pr_err("ELM327 misbehaved. Re-initializing.\n");

	elm->can.can_stats.restarts++;
	elm327_init(elm);
}



 /************************************************************************
  *		ELM327: Reception parser			*
  *								*
  * (assumes elm->lock taken)					*
  ************************************************************************/

static void elm327_parse_error(struct elmcan *elm, int len)
{
	struct can_frame frame = {0};

	frame.can_id = CAN_ERR_FLAG;
	frame.can_dlc = CAN_ERR_DLC;

	switch(len) {
		case 17:
			if (!memcmp(elm->rxbuf, "UNABLE TO CONNECT", 17)) {
				pr_err("The ELM327 reported UNABLE TO CONNECT. Please check your setup.\n");
			}
			break;
		case 11:
			if (!memcmp(elm->rxbuf, "BUFFER FULL", 11)) {
				/* This case will only happen if the last data
				 * line was complete.
				 * Otherwise, elm327_parse_frame() will emit the
				 * error frame instead.
				 */
				frame.can_id |= CAN_ERR_CRTL;
				frame.data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
			}
			break;
		case 9:
			if (!memcmp(elm->rxbuf, "BUS ERROR", 9)) {
				frame.can_id |= CAN_ERR_BUSERROR;
			}
			if (!memcmp(elm->rxbuf, "CAN ERROR", 9)
				|| !memcmp(elm->rxbuf, "<RX ERROR", 9)) {
				frame.can_id |= CAN_ERR_PROT;
			}
			break;
		case 8:
			if (!memcmp(elm->rxbuf, "BUS BUSY", 8)) {
				frame.can_id |= CAN_ERR_PROT;
				frame.data[2] = CAN_ERR_PROT_OVERLOAD;
			}
			if (!memcmp(elm->rxbuf, "FB ERROR", 8)) {
				frame.can_id |= CAN_ERR_PROT;
				frame.data[2] = CAN_ERR_PROT_TX;
			}
			break;
		case 5:
			if (!memcmp(elm->rxbuf, "ERR", 3)) {
				pr_err("The ELM327 reported an ERR%c%c. Please power it off and on again.\n",
					elm->rxbuf[3], elm->rxbuf[4]);
				frame.can_id |= CAN_ERR_CRTL;
			}
			break;
		default:
			/* Don't emit an error frame if we're unsure */
			return;
	}

	elm327_feed_frame_to_netdev(elm, &frame);
}


static int elm327_parse_frame(struct elmcan *elm, int len)
{
	struct can_frame frame = {0};
	int hexlen;
	int datastart;
	int i;

	/* Find first non-hex and non-space character:
	 *  - In the simplest case, there is none.
	 *  - For RTR frames, 'R' is the first non-hex character.
	 *  - An error message may replace the end of the data line.
	 */
	for (hexlen = 0; hexlen <= len; hexlen++) {
		if (hex_to_bin(elm->rxbuf[hexlen]) < 0
		    && elm->rxbuf[hexlen] != ' ') {
			break;
		}
	}

	/* Use spaces in CAN ID to distinguish 29 or 11 bit address length.
	 * No out-of-bounds access:
	 * We use the fact that we can always read from elm->rxbuf.
	 */
	if (elm->rxbuf[2] == ' ' && elm->rxbuf[5] == ' '
		&& elm->rxbuf[8] == ' ' && elm->rxbuf[11] == ' '
		&& elm->rxbuf[13] == ' ') {
		frame.can_id = CAN_EFF_FLAG;
		datastart = 14;
	} else if (elm->rxbuf[3] == ' ' && elm->rxbuf[5] == ' ') {
		frame.can_id = 0;
		datastart = 6;
	} else {
		/* This is not a well-formatted data line.
		 * Assume it's an error message.
		 */
		return 1;
	}

	if (hexlen < datastart) {
		/* The line is too short to be a valid frame hex dump.
		 * Something interrupted the hex dump or it is invalid.
		 */
		return 1;
	}

	/* From here on all chars up to buf[hexlen] are hex or spaces,
	 * at well-defined offsets.
	 */

	/* Read CAN data length */
	frame.can_dlc = (hex_to_bin(elm->rxbuf[datastart - 2]) << 0);

	/* Read CAN ID */
	if (frame.can_id & CAN_EFF_FLAG) {
		frame.can_id |= (hex_to_bin(elm->rxbuf[0]) << 28)
			      | (hex_to_bin(elm->rxbuf[1]) << 24)
			      | (hex_to_bin(elm->rxbuf[3]) << 20)
			      | (hex_to_bin(elm->rxbuf[4]) << 16)
			      | (hex_to_bin(elm->rxbuf[6]) << 12)
			      | (hex_to_bin(elm->rxbuf[7]) << 8)
			      | (hex_to_bin(elm->rxbuf[9]) << 4)
			      | (hex_to_bin(elm->rxbuf[10]) << 0);
	} else {
		frame.can_id |= (hex_to_bin(elm->rxbuf[0]) << 8)
			      | (hex_to_bin(elm->rxbuf[1]) << 4)
			      | (hex_to_bin(elm->rxbuf[2]) << 0);
	}

	/* Check for RTR frame */
	if (elm->rxfill >= hexlen + 3
	    && elm->rxbuf[hexlen + 0] == 'R'
	    && elm->rxbuf[hexlen + 1] == 'T'
	    && elm->rxbuf[hexlen + 2] == 'R') {
		frame.can_id |= CAN_RTR_FLAG;
	}

	/* Is the line long enough to hold the advertised payload? */
	if (!(frame.can_id & CAN_RTR_FLAG) && (hexlen < frame.can_dlc * 3 + datastart)) {
		/* Incomplete frame. */

		/* Probably the ELM327's RS232 TX buffer was full.
		 * Emit an error frame and exit.
		 */
		frame.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
		frame.can_dlc = CAN_ERR_DLC;
		frame.data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		elm327_feed_frame_to_netdev(elm, &frame);

		/* Signal failure to parse.
		 * The line will be re-parsed as an error line, which will fail.
		 * However, this will correctly drop the state machine back into
		 * command mode.
		 */
		return 2;
	}

	/* Parse the data nibbles. */
	for (i = 0; i < frame.can_dlc; i++) {
		frame.data[i] = (hex_to_bin(elm->rxbuf[datastart+3*i]) << 4)
                                | (hex_to_bin(elm->rxbuf[datastart+3*i+1]) << 0);
	}

	/* Feed the frame to the network layer. */
	elm327_feed_frame_to_netdev(elm, &frame);

	return 0;
}


static void elm327_parse_line(struct elmcan *elm, int len)
{
	/* Skip empty lines */
	if (!len) {
		return;
	}

	/* Skip echo lines */
	if (elm->drop_next_line) {
		elm->drop_next_line = 0;
		return;
	} else if (elm->rxbuf[0] == 'A' && elm->rxbuf[1] == 'T') {
		return;
	}

	/* Regular parsing */
	switch(elm->state) {
		case ELM_RECEIVING:
			if (elm327_parse_frame(elm, len)) {
				/* Parse an error line. */
				elm327_parse_error(elm, len);

				/* After the error line, we expect a prompt. */
				elm->state = ELM_GETPROMPT;
			}
			break;
		default:
			break;
	}
}


static void elm327_handle_prompt(struct elmcan *elm)
{
	if (elm->cmds_todo) {
		struct can_frame *frame = &elm->can_frame;
		char txbuf[20];

		if (test_bit(ELM_TODO_INIT, &elm->cmds_todo)) {
			elm327_send(elm, *elm->next_init_cmd, strlen(*elm->next_init_cmd));
			elm->next_init_cmd++;
			if (!(*elm->next_init_cmd)) {
				clear_bit(ELM_TODO_INIT, &elm->cmds_todo);
				pr_info("%s: Initialization finished.\n", elm->dev->name);
			}

			/* Some chips are unreliable and need extra time after
			 * init commands, as seen with a clone.
			 * So let's do a dummy get-cmd-prompt dance.
			 */
			elm->state = ELM_NOTINIT;
			elm327_kick_into_cmd_mode(elm);
		} else if (test_and_clear_bit(ELM_TODO_SILENT_MONITOR, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATCSM%i\r", !(!(elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)));
		} else if (test_and_clear_bit(ELM_TODO_RESPONSES, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATR%i\r", !(elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY));
		} else if (test_and_clear_bit(ELM_TODO_CAN_CONFIG, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATPB%04X\r", elm->can_config);
		} else if (test_and_clear_bit(ELM_TODO_CANID_29BIT_HIGH, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATCP%02X\r", (frame->can_id & CAN_EFF_MASK) >> 24);
		} else if (test_and_clear_bit(ELM_TODO_CANID_29BIT_LOW, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATSH%06X\r", frame->can_id & CAN_EFF_MASK & ((1 << 24) - 1));
		} else if (test_and_clear_bit(ELM_TODO_CANID_11BIT, &elm->cmds_todo)) {
			snprintf(txbuf, sizeof(txbuf), "ATSH%03X\r", frame->can_id & CAN_SFF_MASK);
		} else if (test_and_clear_bit(ELM_TODO_CAN_DATA, &elm->cmds_todo)) {
			if (frame->can_id & CAN_RTR_FLAG) {
				snprintf(txbuf, sizeof(txbuf), "ATRTR\r");
			} else {
				int i;

				for (i = 0; i < frame->can_dlc; i++) {
					sprintf(&txbuf[2*i], "%02X", frame->data[i]);
				}

				sprintf(&txbuf[2*i], "\r");
			}

			elm->drop_next_line = 1;
			elm->state = ELM_RECEIVING;
		}

		elm327_send(elm, txbuf, strlen(txbuf));
	} else {
		/* Enter CAN monitor mode */
		elm327_send(elm, "ATMA\r", 5);
		elm->state = ELM_RECEIVING;
	}
}


static void elm327_drop_bytes(struct elmcan *elm, int i)
{
	memmove(&elm->rxbuf[0], &elm->rxbuf[i], sizeof(elm->rxbuf) - i);
	elm->rxfill -= i;
}


static void elm327_parse_rxbuf(struct elmcan *elm)
{
	int len;

	switch (elm->state) {
	case ELM_NOTINIT:
		elm->rxfill = 0;
		return;

	case ELM_GETMAGICCHAR:
	{
		/* Wait for 'y' or '>' */
		int i;

		for (i = 0; i < elm->rxfill; i++) {
			if (elm->rxbuf[i] == ELM327_MAGIC_CHAR) {
				elm327_send(elm, "\r", 1);
				elm->state = ELM_GETPROMPT;
				i++;
				break;
			} else if (elm->rxbuf[i] == ELM327_READY_CHAR) {
				elm327_send(elm, ELM327_MAGIC_STRING, 1);
				i++;
				break;
			}
		}

		elm327_drop_bytes(elm, i);

		return;
	}

	case ELM_GETPROMPT:
		/* Wait for '>' */
		if (elm->rxbuf[elm->rxfill - 1] == ELM327_READY_CHAR) {
			elm327_handle_prompt(elm);
		}

		elm->rxfill = 0;
		return;

	case ELM_RECEIVING:
		/* Find <CR> delimiting feedback lines. */
		for (len = 0;
		     (len < elm->rxfill) && (elm->rxbuf[len] != '\r');
		     len++) {
			/* empty loop */
		}

		if (len == sizeof(elm->rxbuf)) {
			/* Line exceeds buffer. It's probably all garbage.
			 * Did we even connect at the right baud rate?
			 */
			pr_err("RX buffer overflow. Faulty ELM327 connected?\n");
			elm327_hw_failure(elm);
		} else if (len == elm->rxfill) {
			if (elm->state == ELM_RECEIVING
				&& elm->rxbuf[elm->rxfill - 1] == ELM327_READY_CHAR) {
				/* The ELM327's AT ST response timeout ran out,
				 * so we got a prompt.
				 * Clear RX buffer and restart listening.
				 */
				elm->rxfill = 0;

				elm327_handle_prompt(elm);
				return;
			} else {
				/* We haven't received a full line yet.
				 * Wait for more data.
				 */
				return;
			}
		}

		/* We have a full line to parse. */
		elm327_parse_line(elm, len);

		/* Remove parsed data from RX buffer. */
		elm327_drop_bytes(elm, len+1);

		/* More data to parse? */
		if (elm->rxfill) {
			elm327_parse_rxbuf(elm);
		}
	}
}





 /************************************************************************
  *		netdev						*
  *								*
  * (takes elm->lock)						*
  ************************************************************************/

static int elmcan_netdev_init(struct net_device *dev)
{
	struct elmcan *elm = netdev_priv(dev);

	/* Copy the interface name here, so the SIOCGIFNAME case in
	 * elmcan_ldisc_ioctl() doesn't race against unregister_candev().
	 */
	memcpy(elm->ifname, dev->name, IFNAMSIZ);

	return 0;
}

/* Netdevice DOWN -> UP routine */
static int elmcan_netdev_open(struct net_device *dev)
{
	struct elmcan *elm = netdev_priv(dev);
	int err;

	spin_lock_bh(&elm->lock);
	if (elm->tty == NULL) {
		spin_unlock_bh(&elm->lock);
		return -ENODEV;
	}

	/* open_candev() checks for elm->can.bittiming.bitrate != 0 */
	err = open_candev(dev);
	if (err) {
		spin_unlock_bh(&elm->lock);
		return err;
	}

	/* Initialize the ELM327 */
	elm327_init(elm);
	spin_unlock_bh(&elm->lock);

	can_led_event(dev, CAN_LED_EVENT_OPEN);
	elm->can.state = CAN_STATE_ERROR_ACTIVE;
	netif_start_queue(dev);

	return 0;
}

/* Netdevice UP -> DOWN routine */
static int elmcan_netdev_close(struct net_device *dev)
{
	struct elmcan *elm = netdev_priv(dev);

	spin_lock_bh(&elm->lock);
	if (elm->tty) {
		/* TTY discipline is running. */

		/* Interrupt whatever we're doing right now */
		elm327_send(elm, ELM327_MAGIC_STRING, 1);

		/* Clear the wakeup bit, as the netdev will be down and thus
		 * the wakeup handler won't clear it
		 */
		clear_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);

		spin_unlock_bh(&elm->lock);

		flush_work(&elm->tx_work);
	} else {
		spin_unlock_bh(&elm->lock);
	}

	elm->can.state = CAN_STATE_STOPPED;
	netif_stop_queue(dev);
	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

/* Send a can_frame to a TTY queue. */
static netdev_tx_t elmcan_netdev_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct elmcan *elm = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *) skb->data;

	if (skb->len != sizeof(struct can_frame))
		goto out;

	if (!netif_running(dev))  {
		pr_warn("%s: xmit: iface is down\n", dev->name);
		goto out;
	}

	/* BHs are already disabled, so no spin_lock_bh().
	 * See Documentation/networking/netdevices.txt
	 */
	spin_lock(&elm->lock);
	if (elm->tty == NULL
		|| elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		spin_unlock(&elm->lock);
		goto out;
	}

	netif_stop_queue(dev);

	elm327_send_frame(elm, frame);
	spin_unlock(&elm->lock);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += frame->can_dlc;

	can_led_event(dev, CAN_LED_EVENT_TX);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int elmcan_netdev_change_mtu(struct net_device *dev, int new_mtu)
{
	return -EINVAL;
}

static const struct net_device_ops elmcan_netdev_ops = {
	.ndo_init       = elmcan_netdev_init,
	.ndo_open	= elmcan_netdev_open,
	.ndo_stop	= elmcan_netdev_close,
	.ndo_start_xmit	= elmcan_netdev_start_xmit,
	.ndo_change_mtu	= elmcan_netdev_change_mtu,
};





 /************************************************************************
  *		Line discipline					*
  *								*
  * (takes elm->lock)						*
  ************************************************************************/

/*
 * Get a reference to our struct, taking into account locks/refcounts.
 * This is to ensure ordering in case we are shutting down, and to ensure
 * there is a refcount at all (because tty->disc_data may be NULL).
 */
static struct elmcan* get_elm(struct tty_struct *tty)
{
	struct elmcan *elm;
	bool got_ref;

	/* Lock all elmcan TTYs, so tty->disc_data can't become NULL
	 * the moment before we increase the reference counter.
	 */
	spin_lock_bh(&elmcan_discdata_lock);
	elm = (struct elmcan *) tty->disc_data;

	if (!elm) {
		spin_unlock_bh(&elmcan_discdata_lock);
		return NULL;
	}

	got_ref = atomic_inc_not_zero(&elm->refcount);
	spin_unlock_bh(&elmcan_discdata_lock);

	if (!got_ref) {
		return NULL;
	}

	return elm;
}

static void put_elm(struct elmcan *elm)
{
	atomic_dec(&elm->refcount);
}



/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of ELM327 CAN data has been received, which can now be parsed
 * and sent on to some IP layer for further processing. This will not
 * be re-entered while running but other ldisc functions may be called
 * in parallel
 */
static void elmcan_ldisc_rx(struct tty_struct *tty,
			const unsigned char *cp, char *fp, int count)
{
	struct elmcan *elm = get_elm(tty);

	if (!elm)
		return;

	/* Read the characters out of the buffer */
	while (count-- && elm->rxfill < sizeof(elm->rxbuf)) {
		if (fp && *fp++) {
			pr_err("Error in received character stream. Check your wiring.");

			spin_lock_bh(&elm->lock);
			elm327_hw_failure(elm);
			spin_unlock_bh(&elm->lock);

			put_elm(elm);
			return;
		}
		if (*cp != 0) {
			elm->rxbuf[elm->rxfill++] = *cp;
		}
		cp++;
	}

	if (count >= 0) {
		pr_err("Receive buffer overflowed. Bad chip or wiring?");

		spin_lock_bh(&elm->lock);
		elm327_hw_failure(elm);
		spin_unlock_bh(&elm->lock);

		put_elm(elm);
		return;
	}

	spin_lock_bh(&elm->lock);
	elm327_parse_rxbuf(elm);
	spin_unlock_bh(&elm->lock);

	put_elm(elm);
}

/*
 * Write out remaining transmit buffer.
 * Scheduled when TTY is writable.
 */
static void elmcan_ldisc_tx_worker(struct work_struct *work)
{
	/* No need to use get_elm() here, as we'll always flush workers
	 * befory destroying the elmcan object.
	 */
	struct elmcan *elm = container_of(work, struct elmcan, tx_work);
	ssize_t actual;

	spin_lock_bh(&elm->lock);
	/* First make sure we're connected. */
	if (!elm->tty || !netif_running(elm->dev)) {
		spin_unlock_bh(&elm->lock);
		return;
	}

	if (elm->txleft <= 0)  {
		/* Our TTY write buffer is empty:
		 * We can start transmission of another packet
		 */
		clear_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);
		spin_unlock_bh(&elm->lock);
		netif_wake_queue(elm->dev);
		return;
	}

	actual = elm->tty->ops->write(elm->tty, elm->txhead, elm->txleft);
	if (actual < 0) {
		pr_err("Failed to write to tty for %s.\n", elm->dev->name);
		elm327_hw_failure(elm);
	}

	elm->txleft -= actual;
	elm->txhead += actual;
	spin_unlock_bh(&elm->lock);
}


/*
 * Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void elmcan_ldisc_tx_wakeup(struct tty_struct *tty)
{
	struct elmcan *elm = get_elm(tty);

	if (!elm)
		return;

	schedule_work(&elm->tx_work);

	put_elm(elm);
}



/* Some fake bit timings to allow bitrate setting */
static const struct can_bittiming_const elmcan_bittiming_const = {
	.name = "elmcan",
	.tseg1_min = 1,
	.tseg1_max = 1,
	.tseg2_min = 0,
	.tseg2_max = 0,
	.sjw_max = 1,
	.brp_min = 1,
	.brp_max = 500,
	.brp_inc = 1,
};

/*
 * Open the high-level part of the elmcan channel.
 * This function is called by the TTY module when the
 * elmcan line discipline is called for.
 *
 * Called in process context serialized from other ldisc calls.
 */
static int elmcan_ldisc_open(struct tty_struct *tty)
{
	struct net_device *dev;
	struct elmcan *elm;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!tty->ops->write)
		return -EOPNOTSUPP;


	/* OK.  Find a free elmcan channel to use. */
	dev = alloc_candev(sizeof(struct elmcan), 0);
	if (!dev)
		return -ENFILE;
	elm = netdev_priv(dev);

	/* Configure TTY interface */
	tty->receive_room = 65536; /* We don't flow control */
	elm->txleft = 0; /* Clear TTY TX buffer */
	spin_lock_init(&elm->lock);
	atomic_set(&elm->refcount, 1);
	INIT_WORK(&elm->tx_work, elmcan_ldisc_tx_worker);

	/* Configure CAN metadata */
	elm->can.state = CAN_STATE_STOPPED;
	elm->can.clock.freq = 1000000;
	elm->can.bittiming_const = &elmcan_bittiming_const;
	elm->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY;

	/* Configure netlink interface */
	elm->dev = dev;
	dev->netdev_ops = &elmcan_netdev_ops;

	/* Mark ldisc channel as alive */
	elm->tty = tty;
	tty->disc_data = elm;

	devm_can_led_init(elm->dev);

	/* Let 'er rip */
	err = register_candev(elm->dev);
	if (err) {
		free_candev(elm->dev);
		return err;
	}

	netdev_info(elm->dev, "elmcan on %s.\n", tty->name);

	return 0;
}

/*
 * Close down an elmcan channel.
 * This means flushing out any pending queues, and then returning.
 * This call is serialized against other ldisc functions:
 * Once this is called, no other ldisc function of ours is entered.
 *
 * We also use this function for a hangup event.
 */
static void elmcan_ldisc_close(struct tty_struct *tty)
{
	/* Use get_elm() to synchronize against other users */
	struct elmcan *elm = get_elm(tty);

	if (!elm)
		return;

	/* Tear down network side.
	 * unregister_netdev() calls .ndo_stop() so we don't have to.
	 */
	unregister_candev(elm->dev);

	/* Decrease the refcount twice, once for our own get_elm(),
	 * and once to remove the count of 1 that we set in _open().
	 * Once it reaches 0, we can safely destroy it.
	 */
	put_elm(elm);
	put_elm(elm);

	/* Spin until refcount reaches 0 */
	while(atomic_read(&elm->refcount) > 0)
		msleep(1);

	/* At this point, all ldisc calls to us will be no-ops.
	 * Since the refcount is 0, they are bailing immediately.
	 */

	/* Mark channel as dead */
	spin_lock_bh(&elm->lock);
	tty->disc_data = NULL;
	elm->tty = NULL;
	spin_unlock_bh(&elm->lock);

	/* Flush TTY side */
	flush_work(&elm->tx_work);

	netdev_info(elm->dev, "elmcan off %s.\n", tty->name);

	/* Free our memory */
	free_candev(elm->dev);
}

static int elmcan_ldisc_hangup(struct tty_struct *tty)
{
	elmcan_ldisc_close(tty);
	return 0;
}

/* Perform I/O control on an active elmcan channel. */
static int elmcan_ldisc_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct elmcan *elm = get_elm(tty);
	unsigned int tmp;

	/* First make sure we're connected. */
	if (!elm)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		tmp = strlen(elm->ifname) + 1;
		if (copy_to_user((void __user *)arg, elm->ifname, tmp)) {
			put_elm(elm);
			return -EFAULT;
		}

		put_elm(elm);
		return 0;

	case SIOCSIFHWADDR:
		put_elm(elm);
		return -EINVAL;

	default:
		put_elm(elm);
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static struct tty_ldisc_ops elmcan_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "elmcan",
	.receive_buf	= elmcan_ldisc_rx,
	.write_wakeup	= elmcan_ldisc_tx_wakeup,
	.open		= elmcan_ldisc_open,
	.close		= elmcan_ldisc_close,
	.hangup		= elmcan_ldisc_hangup,
	.ioctl		= elmcan_ldisc_ioctl,
};





 /************************************************************************
  *		Module init/exit				*
  ************************************************************************/

static int __init elmcan_init(void)
{
	int status;

	pr_info("ELM327 based best-effort CAN interface driver\n");
	pr_info("This device is severely limited as a CAN interface, see documentation.\n");

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_ELMCAN, &elmcan_ldisc);
	if (status) {
		pr_err("can't register line discipline\n");
	}
	return status;
}

static void __exit elmcan_exit(void)
{
	/* This will only be called when all channels have been closed by
	 * userspace - tty_ldisc.c takes care of the module's refcount.
	 */
	int status;

	status = tty_unregister_ldisc(N_ELMCAN);
	if (status) {
		pr_err("Can't unregister line discipline (error: %d)\n", status);
	}
}

module_init(elmcan_init);
module_exit(elmcan_exit);
