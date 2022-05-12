// SPDX-License-Identifier: GPL-2.0
/* ELM327 based CAN interface driver (tty line discipline)
 *
 * This driver started as a derivative of linux/drivers/net/can/slcan.c
 * and my thanks go to the original authors for their inspiration.
 *
 * can327.c Author : Max Staudt <max-linux@enpas.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 */

#define pr_fmt(fmt) "can327: " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <uapi/linux/tty.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/can/rx-offload.h>

/* Line discipline ID number.
 * Starting with Linux v5.18-rc1, N_DEVELOPMENT is defined as 29:
 * https://github.com/torvalds/linux/commit/c2faf737abfb10f88f2d2612d573e9edc3c42c37
 */
#ifndef N_DEVELOPMENT
#define N_DEVELOPMENT 29
#endif

/* Compatibility for Linux < 5.11 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
#define len can_dlc
#endif

#define ELM327_NAPI_WEIGHT 4

#define ELM327_SIZE_RXBUF 224
#define ELM327_SIZE_TXBUF 32

#define ELM327_CAN_CONFIG_SEND_SFF           0x8000
#define ELM327_CAN_CONFIG_VARIABLE_DLC       0x4000
#define ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF  0x2000
#define ELM327_CAN_CONFIG_BAUDRATE_MULT_8_7  0x1000

#define ELM327_DUMMY_CHAR 'y'
#define ELM327_DUMMY_STRING "y"
#define ELM327_READY_CHAR '>'

/* Bits in elm->cmds_todo */
enum elm327_to_to_do_bits {
	ELM327_TX_DO_CAN_DATA = 0,
	ELM327_TX_DO_CANID_11BIT,
	ELM327_TX_DO_CANID_29BIT_LOW,
	ELM327_TX_DO_CANID_29BIT_HIGH,
	ELM327_TX_DO_CAN_CONFIG_PART2,
	ELM327_TX_DO_CAN_CONFIG,
	ELM327_TX_DO_RESPONSES,
	ELM327_TX_DO_SILENT_MONITOR,
	ELM327_TX_DO_INIT
};

struct can327 {
	/* This must be the first member when using alloc_candev() */
	struct can_priv can;

	struct can_rx_offload offload;

	/* TTY buffers */
	u8 rxbuf[ELM327_SIZE_RXBUF];
	u8 txbuf[ELM327_SIZE_TXBUF] ____cacheline_aligned;

	/* Per-channel lock */
	spinlock_t lock;

	/* TTY and netdev devices that we're bridging */
	struct tty_struct *tty;
	struct net_device *dev;

	/* TTY buffer accounting */
	struct work_struct tx_work;	/* Flushes TTY TX buffer */
	u8 *txhead;			/* Next TX byte */
	size_t txleft;			/* Bytes left to TX */
	int rxfill;			/* Bytes already RX'd in buffer */

	/* State machine */
	enum {
		ELM327_STATE_NOTINIT = 0,
		ELM327_STATE_GETDUMMYCHAR,
		ELM327_STATE_GETPROMPT,
		ELM327_STATE_RECEIVING,
	} state;

	/* Things we have yet to send */
	char **next_init_cmd;
	unsigned long cmds_todo;

	/* The CAN frame and config the ELM327 is sending/using,
	 * or will send/use after finishing all cmds_todo
	 */
	struct can_frame can_frame_to_send;
	u16 can_config;
	u8 can_bitrate_divisor;

	/* Parser state */
	bool drop_next_line;

	/* Stop the channel on UART side hardware failure, e.g. stray
	 * characters or neverending lines. This may be caused by bad
	 * UART wiring, a bad ELM327, a bad UART bridge...
	 * Once this is true, nothing will be sent to the TTY.
	 */
	bool uart_side_failure;
};

static inline void elm327_uart_side_failure(struct can327 *elm);

static void elm327_send(struct can327 *elm, const void *buf, size_t len)
{
	int written;

	lockdep_assert_held(&elm->lock);

	if (elm->uart_side_failure)
		return;

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
	written = elm->tty->ops->write(elm->tty, elm->txbuf, len);
	if (written < 0) {
		netdev_err(elm->dev,
			   "Failed to write to tty %s.\n",
			   elm->tty->name);
		elm327_uart_side_failure(elm);
		return;
	}

	elm->txleft = len - written;
	elm->txhead = elm->txbuf + written;
}

/* Take the ELM327 out of almost any state and back into command mode.
 * We send ELM327_DUMMY_CHAR which will either abort any running
 * operation, or be echoed back to us in case we're already in command
 * mode.
 */
static void elm327_kick_into_cmd_mode(struct can327 *elm)
{
	lockdep_assert_held(&elm->lock);

	if (elm->state != ELM327_STATE_GETDUMMYCHAR &&
	    elm->state != ELM327_STATE_GETPROMPT) {
		elm327_send(elm, ELM327_DUMMY_STRING, 1);

		elm->state = ELM327_STATE_GETDUMMYCHAR;
	}
}

/* Schedule a CAN frame and necessary config changes to be sent to the TTY. */
static void elm327_send_frame(struct can327 *elm, struct can_frame *frame)
{
	lockdep_assert_held(&elm->lock);

	/* Schedule any necessary changes in ELM327's CAN configuration */
	if (elm->can_frame_to_send.can_id != frame->can_id) {
		/* Set the new CAN ID for transmission. */
		if ((frame->can_id & CAN_EFF_FLAG) ^
		    (elm->can_frame_to_send.can_id & CAN_EFF_FLAG)) {
			elm->can_config = (frame->can_id & CAN_EFF_FLAG
						? 0
						: ELM327_CAN_CONFIG_SEND_SFF)
					| ELM327_CAN_CONFIG_VARIABLE_DLC
					| ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF
					| elm->can_bitrate_divisor;

			set_bit(ELM327_TX_DO_CAN_CONFIG, &elm->cmds_todo);
		}

		if (frame->can_id & CAN_EFF_FLAG) {
			clear_bit(ELM327_TX_DO_CANID_11BIT, &elm->cmds_todo);
			set_bit(ELM327_TX_DO_CANID_29BIT_LOW, &elm->cmds_todo);
			set_bit(ELM327_TX_DO_CANID_29BIT_HIGH, &elm->cmds_todo);
		} else {
			set_bit(ELM327_TX_DO_CANID_11BIT, &elm->cmds_todo);
			clear_bit(ELM327_TX_DO_CANID_29BIT_LOW, &elm->cmds_todo);
			clear_bit(ELM327_TX_DO_CANID_29BIT_HIGH, &elm->cmds_todo);
		}
	}

	/* Schedule the CAN frame itself. */
	elm->can_frame_to_send = *frame;
	set_bit(ELM327_TX_DO_CAN_DATA, &elm->cmds_todo);

	elm327_kick_into_cmd_mode(elm);
}

/* ELM327 initialisation sequence.
 * The line length is limited by the buffer in elm327_handle_prompt().
 */
static char *elm327_init_script[] = {
	"AT WS\r",        /* v1.0: Warm Start */
	"AT PP FF OFF\r", /* v1.0: All Programmable Parameters Off */
	"AT M0\r",        /* v1.0: Memory Off */
	"AT AL\r",        /* v1.0: Allow Long messages */
	"AT BI\r",        /* v1.0: Bypass Initialisation */
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

static void elm327_init(struct can327 *elm)
{
	lockdep_assert_held(&elm->lock);

	elm->state = ELM327_STATE_NOTINIT;
	elm->can_frame_to_send.can_id = 0x7df; /* ELM327 HW default */
	elm->rxfill = 0;
	elm->drop_next_line = 0;

	/* We can only set the bitrate as a fraction of 500000.
	 * The bit timing constants in can327_bittiming_const will
	 * limit the user to the right values.
	 */
	elm->can_bitrate_divisor = 500000 / elm->can.bittiming.bitrate;
	elm->can_config = ELM327_CAN_CONFIG_SEND_SFF
			| ELM327_CAN_CONFIG_VARIABLE_DLC
			| ELM327_CAN_CONFIG_RECV_BOTH_SFF_EFF
			| elm->can_bitrate_divisor;

	/* Configure ELM327 and then start monitoring */
	elm->next_init_cmd = &elm327_init_script[0];
	set_bit(ELM327_TX_DO_INIT, &elm->cmds_todo);
	set_bit(ELM327_TX_DO_SILENT_MONITOR, &elm->cmds_todo);
	set_bit(ELM327_TX_DO_RESPONSES, &elm->cmds_todo);
	set_bit(ELM327_TX_DO_CAN_CONFIG, &elm->cmds_todo);

	elm327_kick_into_cmd_mode(elm);
}

static void elm327_feed_frame_to_netdev(struct can327 *elm,
					struct sk_buff *skb)
{
	lockdep_assert_held(&elm->lock);

	if (!netif_running(elm->dev))
		return;

	/* Queue for NAPI pickup.
	 * rx-offload will update stats and LEDs for us.
	 */
	if (can_rx_offload_queue_tail(&elm->offload, skb))
		elm->dev->stats.rx_fifo_errors++;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
	/* Wake NAPI */
	can_rx_offload_irq_finish(&elm->offload);
#endif
}

/* Called when we're out of ideas and just want it all to end. */
static inline void elm327_uart_side_failure(struct can327 *elm)
{
	struct can_frame *frame;
	struct sk_buff *skb;

	lockdep_assert_held(&elm->lock);

	elm->uart_side_failure = true;

	clear_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);

	elm->can.can_stats.bus_off++;
	netif_stop_queue(elm->dev);
	elm->can.state = CAN_STATE_BUS_OFF;
	can_bus_off(elm->dev);

	netdev_err(elm->dev, "ELM327 misbehaved. Blocking further communication.\n");

	skb = alloc_can_err_skb(elm->dev, &frame);
	if (!skb)
		return;

	frame->can_id |= CAN_ERR_BUSOFF;
	elm327_feed_frame_to_netdev(elm, skb);
}

/* Compare buffer to string length, then compare buffer to fixed string.
 * This ensures two things:
 *  - It flags cases where the fixed string is only the start of the
 *    buffer, rather than exactly all of it.
 *  - It avoids byte comparisons in case the length doesn't match.
 *
 * strncmp() cannot be used here because it accepts the following wrong case:
 *   strncmp("CAN ER", "CAN ERROR", 6);
 * This must fail, hence this helper function.
 */
static inline int check_len_then_cmp(const u8 *mem, size_t mem_len, const char *str)
{
	size_t str_len = strlen(str);

	return (mem_len == str_len) && !memcmp(mem, str, str_len);
}

static void elm327_parse_error(struct can327 *elm, size_t len)
{
	struct can_frame *frame;
	struct sk_buff *skb;

	lockdep_assert_held(&elm->lock);

	skb = alloc_can_err_skb(elm->dev, &frame);
	if (!skb)
		/* It's okay to return here:
		 * The outer parsing loop will drop this UART buffer.
		 */
		return;

	/* Filter possible error messages based on length of RX'd line */
	if (check_len_then_cmp(elm->rxbuf, len, "UNABLE TO CONNECT")) {
		netdev_err(elm->dev,
			   "ELM327 reported UNABLE TO CONNECT. Please check your setup.\n");
	} else if (check_len_then_cmp(elm->rxbuf, len, "BUFFER FULL")) {
		/* This will only happen if the last data line was complete.
		 * Otherwise, elm327_parse_frame() will heuristically
		 * emit this kind of error frame instead.
		 */
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	} else if (check_len_then_cmp(elm->rxbuf, len, "BUS ERROR")) {
		frame->can_id |= CAN_ERR_BUSERROR;
	} else if (check_len_then_cmp(elm->rxbuf, len, "CAN ERROR")) {
		frame->can_id |= CAN_ERR_PROT;
	} else if (check_len_then_cmp(elm->rxbuf, len, "<RX ERROR")) {
		frame->can_id |= CAN_ERR_PROT;
	} else if (check_len_then_cmp(elm->rxbuf, len, "BUS BUSY")) {
		frame->can_id |= CAN_ERR_PROT;
		frame->data[2] = CAN_ERR_PROT_OVERLOAD;
	} else if (check_len_then_cmp(elm->rxbuf, len, "FB ERROR")) {
		frame->can_id |= CAN_ERR_PROT;
		frame->data[2] = CAN_ERR_PROT_TX;
	} else if (len == 5 && !memcmp(elm->rxbuf, "ERR", 3)) {
		/* ERR is followed by two digits, hence line length 5 */
		netdev_err(elm->dev, "ELM327 reported an ERR%c%c. Please power it off and on again.\n",
			   elm->rxbuf[3], elm->rxbuf[4]);
		frame->can_id |= CAN_ERR_CRTL;
	} else {
		/* Something else has happened.
		 * Maybe garbage on the UART line.
		 * Emit a generic error frame.
		 */
	}

	elm327_feed_frame_to_netdev(elm, skb);
}

/* Parse CAN frames coming as ASCII from ELM327.
 * They can be of various formats:
 *
 * 29-bit ID (EFF):  12 34 56 78 D PL PL PL PL PL PL PL PL
 * 11-bit ID (!EFF): 123 D PL PL PL PL PL PL PL PL
 *
 * where D = DLC, PL = payload byte
 *
 * Instead of a payload, RTR indicates a remote request.
 *
 * We will use the spaces and line length to guess the format.
 */
static int elm327_parse_frame(struct can327 *elm, size_t len)
{
	struct can_frame *frame;
	struct sk_buff *skb;
	int hexlen;
	int datastart;
	int i;

	lockdep_assert_held(&elm->lock);

	skb = alloc_can_skb(elm->dev, &frame);
	if (!skb)
		return -ENOMEM;

	/* Find first non-hex and non-space character:
	 *  - In the simplest case, there is none.
	 *  - For RTR frames, 'R' is the first non-hex character.
	 *  - An error message may replace the end of the data line.
	 */
	for (hexlen = 0; hexlen <= len; hexlen++) {
		if (hex_to_bin(elm->rxbuf[hexlen]) < 0 &&
		    elm->rxbuf[hexlen] != ' ') {
			break;
		}
	}

	/* Sanity check whether the line is really a clean hexdump,
	 * or terminated by an error message, or contains garbage.
	 */
	if (hexlen < len &&
	    !isdigit(elm->rxbuf[hexlen]) &&
	    !isupper(elm->rxbuf[hexlen]) &&
	    '<' != elm->rxbuf[hexlen] &&
	    ' ' != elm->rxbuf[hexlen]) {
		/* The line is likely garbled anyway, so bail.
		 * The main code will restart listening.
		 */
		return -ENODATA;
	}

	/* Use spaces in CAN ID to distinguish 29 or 11 bit address length.
	 * No out-of-bounds access:
	 * We use the fact that we can always read from elm->rxbuf.
	 */
	if (elm->rxbuf[2] == ' ' && elm->rxbuf[5] == ' ' &&
	    elm->rxbuf[8] == ' ' && elm->rxbuf[11] == ' ' &&
	    elm->rxbuf[13] == ' ') {
		frame->can_id = CAN_EFF_FLAG;
		datastart = 14;
	} else if (elm->rxbuf[3] == ' ' && elm->rxbuf[5] == ' ') {
		datastart = 6;
	} else {
		/* This is not a well-formatted data line.
		 * Assume it's an error message.
		 */
		return -ENODATA;
	}

	if (hexlen < datastart) {
		/* The line is too short to be a valid frame hex dump.
		 * Something interrupted the hex dump or it is invalid.
		 */
		return -ENODATA;
	}

	/* From here on all chars up to buf[hexlen] are hex or spaces,
	 * at well-defined offsets.
	 */

	/* Read CAN data length */
	frame->len = (hex_to_bin(elm->rxbuf[datastart - 2]) << 0);

	/* Read CAN ID */
	if (frame->can_id & CAN_EFF_FLAG) {
		frame->can_id |= (hex_to_bin(elm->rxbuf[0]) << 28)
			       | (hex_to_bin(elm->rxbuf[1]) << 24)
			       | (hex_to_bin(elm->rxbuf[3]) << 20)
			       | (hex_to_bin(elm->rxbuf[4]) << 16)
			       | (hex_to_bin(elm->rxbuf[6]) << 12)
			       | (hex_to_bin(elm->rxbuf[7]) << 8)
			       | (hex_to_bin(elm->rxbuf[9]) << 4)
			       | (hex_to_bin(elm->rxbuf[10]) << 0);
	} else {
		frame->can_id |= (hex_to_bin(elm->rxbuf[0]) << 8)
			       | (hex_to_bin(elm->rxbuf[1]) << 4)
			       | (hex_to_bin(elm->rxbuf[2]) << 0);
	}

	/* Check for RTR frame */
	if (elm->rxfill >= hexlen + 3 &&
	    !memcmp(&elm->rxbuf[hexlen], "RTR", 3)) {
		frame->can_id |= CAN_RTR_FLAG;
	}

	/* Is the line long enough to hold the advertised payload?
	 * Note: RTR frames have a DLC, but no actual payload.
	 */
	if (!(frame->can_id & CAN_RTR_FLAG) &&
	    (hexlen < frame->len * 3 + datastart)) {
		/* Incomplete frame.
		 * Probably the ELM327's RS232 TX buffer was full.
		 * Emit an error frame and exit.
		 */
		frame->can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
		frame->len = CAN_ERR_DLC;
		frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		elm327_feed_frame_to_netdev(elm, skb);

		/* Signal failure to parse.
		 * The line will be re-parsed as an error line, which will fail.
		 * However, this will correctly drop the state machine back into
		 * command mode.
		 */
		return -ENODATA;
	}

	/* Parse the data nibbles. */
	for (i = 0; i < frame->len; i++) {
		frame->data[i] = (hex_to_bin(elm->rxbuf[datastart + 3*i]) << 4)
			       | (hex_to_bin(elm->rxbuf[datastart + 3*i + 1]));
	}

	/* Feed the frame to the network layer. */
	elm327_feed_frame_to_netdev(elm, skb);

	return 0;
}

static void elm327_parse_line(struct can327 *elm, size_t len)
{
	lockdep_assert_held(&elm->lock);

	/* Skip empty lines */
	if (!len)
		return;

	/* Skip echo lines */
	if (elm->drop_next_line) {
		elm->drop_next_line = 0;
		return;
	} else if (!memcmp(elm->rxbuf, "AT", 2)) {
		return;
	}

	/* Regular parsing */
	if (elm->state == ELM327_STATE_RECEIVING &&
	    elm327_parse_frame(elm, len)) {
		/* Parse an error line. */
		elm327_parse_error(elm, len);

		/* Start afresh. */
		elm327_kick_into_cmd_mode(elm);
	}
}

static void elm327_handle_prompt(struct can327 *elm)
{
	struct can_frame *frame = &elm->can_frame_to_send;
	/* Size this buffer for the largest ELM327 line we may generate,
	 * which is currently an 8 byte CAN frame's payload hexdump.
	 * Items in elm327_init_script must fit here, too!
	 */
	char local_txbuf[sizeof("0102030405060708\r")];

	lockdep_assert_held(&elm->lock);

	if (!elm->cmds_todo) {
		/* Enter CAN monitor mode */
		elm327_send(elm, "ATMA\r", 5);
		elm->state = ELM327_STATE_RECEIVING;

		/* We will be in the default state once this command is
		 * sent, so enable the TX packet queue.
		 */
		netif_wake_queue(elm->dev);

		return;
	}

	/* Reconfigure ELM327 step by step as indicated by elm->cmds_todo */
	if (test_bit(ELM327_TX_DO_INIT, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "%s",
			 *elm->next_init_cmd);

		elm->next_init_cmd++;
		if (!(*elm->next_init_cmd)) {
			clear_bit(ELM327_TX_DO_INIT, &elm->cmds_todo);
			/* Init finished. */
		}

	} else if (test_and_clear_bit(ELM327_TX_DO_SILENT_MONITOR, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATCSM%i\r",
			 !(!(elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)));

	} else if (test_and_clear_bit(ELM327_TX_DO_RESPONSES, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATR%i\r",
			 !(elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY));

	} else if (test_and_clear_bit(ELM327_TX_DO_CAN_CONFIG, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATPC\r");
		set_bit(ELM327_TX_DO_CAN_CONFIG_PART2, &elm->cmds_todo);

	} else if (test_and_clear_bit(ELM327_TX_DO_CAN_CONFIG_PART2, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATPB%04X\r",
			 elm->can_config);

	} else if (test_and_clear_bit(ELM327_TX_DO_CANID_29BIT_HIGH, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATCP%02X\r",
			 (frame->can_id & CAN_EFF_MASK) >> 24);

	} else if (test_and_clear_bit(ELM327_TX_DO_CANID_29BIT_LOW, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATSH%06X\r",
			 frame->can_id & CAN_EFF_MASK & ((1 << 24) - 1));

	} else if (test_and_clear_bit(ELM327_TX_DO_CANID_11BIT, &elm->cmds_todo)) {
		snprintf(local_txbuf, sizeof(local_txbuf),
			 "ATSH%03X\r",
			 frame->can_id & CAN_SFF_MASK);

	} else if (test_and_clear_bit(ELM327_TX_DO_CAN_DATA, &elm->cmds_todo)) {
		if (frame->can_id & CAN_RTR_FLAG) {
			/* Send an RTR frame. Their DLC is fixed.
			 * Some chips don't send them at all.
			 */
			snprintf(local_txbuf, sizeof(local_txbuf),
				 "ATRTR\r");
		} else {
			/* Send a regular CAN data frame */
			int i;

			for (i = 0; i < frame->len; i++) {
				snprintf(&local_txbuf[2 * i], sizeof(local_txbuf),
					 "%02X",
					 frame->data[i]);
			}

			snprintf(&local_txbuf[2 * i], sizeof(local_txbuf),
				 "\r");
		}

		elm->drop_next_line = 1;
		elm->state = ELM327_STATE_RECEIVING;

		/* We will be in the default state once this command is
		 * sent, so enable the TX packet queue.
		 */
		netif_wake_queue(elm->dev);
	}

	elm327_send(elm, local_txbuf, strlen(local_txbuf));
}

static bool elm327_is_ready_char(char c)
{
	/* Bits 0xc0 are sometimes set (randomly), hence the mask.
	 * Probably bad hardware.
	 */
	return (c & 0x3f) == ELM327_READY_CHAR;
}

static void elm327_drop_bytes(struct can327 *elm, size_t i)
{
	lockdep_assert_held(&elm->lock);

	memmove(&elm->rxbuf[0], &elm->rxbuf[i], ELM327_SIZE_RXBUF - i);
	elm->rxfill -= i;
}

static void elm327_parse_rxbuf(struct can327 *elm)
{
	size_t len;
	int i;

	lockdep_assert_held(&elm->lock);

	switch (elm->state) {
	case ELM327_STATE_NOTINIT:
		elm->rxfill = 0;
		break;

	case ELM327_STATE_GETDUMMYCHAR:
	{
		/* Wait for 'y' or '>' */
		for (i = 0; i < elm->rxfill; i++) {
			if (elm->rxbuf[i] == ELM327_DUMMY_CHAR) {
				elm327_send(elm, "\r", 1);
				elm->state = ELM327_STATE_GETPROMPT;
				i++;
				break;
			} else if (elm327_is_ready_char(elm->rxbuf[i])) {
				elm327_send(elm, ELM327_DUMMY_STRING, 1);
				i++;
				break;
			}
		}

		elm327_drop_bytes(elm, i);

		break;
	}

	case ELM327_STATE_GETPROMPT:
		/* Wait for '>' */
		if (elm327_is_ready_char(elm->rxbuf[elm->rxfill - 1]))
			elm327_handle_prompt(elm);

		elm->rxfill = 0;
		break;

	case ELM327_STATE_RECEIVING:
		/* Find <CR> delimiting feedback lines. */
		for (len = 0;
		     (len < elm->rxfill) && (elm->rxbuf[len] != '\r');
		     len++) {
			/* empty loop */
		}

		if (len == ELM327_SIZE_RXBUF) {
			/* Line exceeds buffer. It's probably all garbage.
			 * Did we even connect at the right baud rate?
			 */
			netdev_err(elm->dev,
				   "RX buffer overflow. Faulty ELM327 or UART?\n");
			elm327_uart_side_failure(elm);
			break;
		} else if (len == elm->rxfill) {
			if (elm327_is_ready_char(elm->rxbuf[elm->rxfill - 1])) {
				/* The ELM327's AT ST response timeout ran out,
				 * so we got a prompt.
				 * Clear RX buffer and restart listening.
				 */
				elm->rxfill = 0;

				elm327_handle_prompt(elm);
				break;
			}

			/* No <CR> found - we haven't received a full line yet.
			 * Wait for more data.
			 */
			break;
		}

		/* We have a full line to parse. */
		elm327_parse_line(elm, len);

		/* Remove parsed data from RX buffer. */
		elm327_drop_bytes(elm, len + 1);

		/* More data to parse? */
		if (elm->rxfill)
			elm327_parse_rxbuf(elm);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,10,0)
/* Dummy needed to use can_rx_offload */
static struct sk_buff *can327_mailbox_read(struct can_rx_offload *offload,
					   unsigned int n, u32 *timestamp,
					   bool drop)
{
	WARN_ON_ONCE(1); /* This function is a dummy, so don't call it! */

	return ERR_PTR(-ENOBUFS);
}
#endif

static int can327_netdev_open(struct net_device *dev)
{
	struct can327 *elm = netdev_priv(dev);
	int err;

	spin_lock_bh(&elm->lock);

	if (!elm->tty) {
		spin_unlock_bh(&elm->lock);
		return -ENODEV;
	}

	if (elm->uart_side_failure)
		netdev_warn(elm->dev, "Reopening netdev after a UART side fault has been detected.\n");

	/* Clear TTY buffers */
	elm->rxfill = 0;
	elm->txleft = 0;

	/* open_candev() checks for elm->can.bittiming.bitrate != 0 */
	err = open_candev(dev);
	if (err) {
		spin_unlock_bh(&elm->lock);
		return err;
	}

	elm327_init(elm);
	spin_unlock_bh(&elm->lock);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,10,0)
	elm->offload.mailbox_read = can327_mailbox_read;
	err = can_rx_offload_add_fifo(dev, &elm->offload, ELM327_NAPI_WEIGHT);
#else
	err = can_rx_offload_add_manual(dev, &elm->offload, ELM327_NAPI_WEIGHT);
#endif
	if (err) {
		close_candev(dev);
		return err;
	}

	can_rx_offload_enable(&elm->offload);

	can_led_event(dev, CAN_LED_EVENT_OPEN);
	elm->can.state = CAN_STATE_ERROR_ACTIVE;
	netif_start_queue(dev);

	return 0;
}

static int can327_netdev_close(struct net_device *dev)
{
	struct can327 *elm = netdev_priv(dev);

	/* Interrupt whatever the ELM327 is doing right now */
	spin_lock_bh(&elm->lock);
	elm327_send(elm, ELM327_DUMMY_STRING, 1);
	spin_unlock_bh(&elm->lock);

	netif_stop_queue(dev);

	/* Give UART one final chance to flush. */
	clear_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);
	flush_work(&elm->tx_work);

	can_rx_offload_disable(&elm->offload);
	elm->can.state = CAN_STATE_STOPPED;
	can_rx_offload_del(&elm->offload);
	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

/* Send a can_frame to a TTY. */
static netdev_tx_t can327_netdev_start_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	struct can327 *elm = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	/* BHs are already disabled, so no spin_lock_bh().
	 * See Documentation/networking/netdevices.txt
	 */
	spin_lock(&elm->lock);

	/* We shouldn't get here after a hardware fault:
	 * can_bus_off() calls netif_carrier_off()
	 */
	WARN_ON_ONCE(elm->uart_side_failure);

	if (!elm->tty ||
	    elm->uart_side_failure ||
	    elm->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		spin_unlock(&elm->lock);
		goto out;
	}

	netif_stop_queue(dev);

	elm327_send_frame(elm, frame);
	spin_unlock(&elm->lock);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += frame->len;

	can_led_event(dev, CAN_LED_EVENT_TX);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops can327_netdev_ops = {
	.ndo_open	= can327_netdev_open,
	.ndo_stop	= can327_netdev_close,
	.ndo_start_xmit	= can327_netdev_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

static bool can327_is_valid_rx_char(char c)
{
	return (isdigit(c) ||
		isupper(c) ||
		c == ELM327_DUMMY_CHAR ||
		c == ELM327_READY_CHAR ||
		c == '<' ||
		c == 'a' ||
		c == 'b' ||
		c == 'v' ||
		c == '.' ||
		c == '?' ||
		c == '\r' ||
		c == ' ');
}

/* Handle incoming ELM327 ASCII data.
 * This will not be re-entered while running, but other ldisc
 * functions may be called in parallel.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static void can327_ldisc_rx(struct tty_struct *tty,
			    const unsigned char *cp, char *fp, int count)
#else
static void can327_ldisc_rx(struct tty_struct *tty,
			    const unsigned char *cp, const char *fp, int count)
#endif
{
	struct can327 *elm = (struct can327 *)tty->disc_data;

	spin_lock_bh(&elm->lock);

	if (elm->uart_side_failure)
		goto out;

	while (count-- && elm->rxfill < ELM327_SIZE_RXBUF) {
		if (fp && *fp++) {
			netdev_err(elm->dev, "Error in received character stream. Check your wiring.");

			elm327_uart_side_failure(elm);

			goto out;
		}

		/* Ignore NUL characters, which the PIC microcontroller may
		 * inadvertently insert due to a known hardware bug.
		 * See ELM327 documentation, which refers to a Microchip PIC
		 * bug description.
		 */
		if (*cp != 0) {
			/* Check for stray characters on the UART line.
			 * Likely caused by bad hardware.
			 */
			if (!can327_is_valid_rx_char(*cp)) {
				netdev_err(elm->dev,
					   "Received illegal character %02x.\n",
					   *cp);
				elm327_uart_side_failure(elm);

				goto out;
			}

			elm->rxbuf[elm->rxfill++] = *cp;
		}

		cp++;
	}

	if (count >= 0) {
		netdev_err(elm->dev, "Receive buffer overflowed. Bad chip or wiring?");

		elm327_uart_side_failure(elm);

		goto out;
	}

	elm327_parse_rxbuf(elm);

out:
	spin_unlock_bh(&elm->lock);
}

/* Write out remaining transmit buffer.
 * Scheduled when TTY is writable.
 */
static void can327_ldisc_tx_worker(struct work_struct *work)
{
	struct can327 *elm = container_of(work, struct can327, tx_work);
	ssize_t written;

	if (elm->uart_side_failure)
		return;

	spin_lock_bh(&elm->lock);

	if (elm->txleft) {
		written = elm->tty->ops->write(elm->tty, elm->txhead, elm->txleft);
		if (written < 0) {
			netdev_err(elm->dev,
				   "Failed to write to tty %s.\n",
				   elm->tty->name);
			elm327_uart_side_failure(elm);
			spin_unlock_bh(&elm->lock);
			return;
		}

		elm->txleft -= written;
		elm->txhead += written;
	}

	if (!elm->txleft)  {
		clear_bit(TTY_DO_WRITE_WAKEUP, &elm->tty->flags);
		spin_unlock_bh(&elm->lock);
	} else {
		spin_unlock_bh(&elm->lock);
	}
}

/* Called by the driver when there's room for more data. */
static void can327_ldisc_tx_wakeup(struct tty_struct *tty)
{
	struct can327 *elm = (struct can327 *)tty->disc_data;

	schedule_work(&elm->tx_work);
}

/* ELM327 can only handle bitrates that are integer divisors of 500 kHz,
 * or 7/8 of that. Divisors are 1 to 64.
 * Currently we don't implement support for 7/8 rates.
 */
static const u32 can327_bitrate_const[64] = {
	 7812,  7936,  8064,  8196,  8333,  8474,  8620,  8771,
	 8928,  9090,  9259,  9433,  9615,  9803, 10000, 10204,
	10416, 10638, 10869, 11111, 11363, 11627, 11904, 12195,
	12500, 12820, 13157, 13513, 13888, 14285, 14705, 15151,
	15625, 16129, 16666, 17241, 17857, 18518, 19230, 20000,
	20833, 21739, 22727, 23809, 25000, 26315, 27777, 29411,
	31250, 33333, 35714, 38461, 41666, 45454, 50000, 55555,
	62500, 71428, 83333, 100000, 125000, 166666, 250000, 500000
};

/* Dummy needed to use bitrate_const */
static int can327_do_set_bittiming(struct net_device *netdev)
{
	return 0;
}

static int can327_ldisc_open(struct tty_struct *tty)
{
	struct net_device *dev;
	struct can327 *elm;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!tty->ops->write)
		return -EOPNOTSUPP;

	dev = alloc_candev(sizeof(struct can327), 0);
	if (!dev)
		return -ENFILE;
	elm = netdev_priv(dev);

	/* Configure TTY interface */
	tty->receive_room = 65536; /* We don't flow control */
	spin_lock_init(&elm->lock);
	INIT_WORK(&elm->tx_work, can327_ldisc_tx_worker);

	/* Configure CAN metadata */
	elm->can.bitrate_const = can327_bitrate_const;
	elm->can.bitrate_const_cnt = ARRAY_SIZE(can327_bitrate_const);
	elm->can.do_set_bittiming = can327_do_set_bittiming;
	elm->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY;

	/* Configure netdev interface */
	elm->dev = dev;
	dev->netdev_ops = &can327_netdev_ops;

	/* Mark ldisc channel as alive */
	elm->tty = tty;
	tty->disc_data = elm;

	devm_can_led_init(elm->dev);

	/* Let 'er rip */
	err = register_candev(elm->dev);
	if (err)
		goto out_err;

	netdev_info(elm->dev, "can327 on %s.\n", tty->name);

	return 0;

out_err:
	free_candev(elm->dev);
	return err;
}

/* Close down a can327 channel.
 * This means flushing out any pending queues, and then returning.
 * This call is serialized against other ldisc functions:
 * Once this is called, no other ldisc function of ours is entered.
 *
 * We also use this function for a hangup event.
 */
static void can327_ldisc_close(struct tty_struct *tty)
{
	struct can327 *elm = (struct can327 *)tty->disc_data;

	/* unregister_netdev() calls .ndo_stop() so we don't have to.
	 * Our .ndo_stop() also flushes the TTY write wakeup handler,
	 * so we can safely set elm->tty = NULL after this.
	 */
	unregister_candev(elm->dev);

	/* Mark channel as dead */
	spin_lock_bh(&elm->lock);
	tty->disc_data = NULL;
	elm->tty = NULL;
	spin_unlock_bh(&elm->lock);

	netdev_info(elm->dev, "can327 off %s.\n", tty->name);

	free_candev(elm->dev);
}

static int can327_ldisc_ioctl(struct tty_struct *tty,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
			      struct file *file,
#endif
			      unsigned int cmd, unsigned long arg)
{
	struct can327 *elm = (struct can327 *)tty->disc_data;
	unsigned int tmp;

	switch (cmd) {
	case SIOCGIFNAME:
		tmp = strnlen(elm->dev->name, IFNAMSIZ - 1) + 1;
		if (copy_to_user((void __user *)arg, elm->dev->name, tmp))
			return -EFAULT;
		return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,16,0)
		return tty_mode_ioctl(tty, file, cmd, arg);
#else
		return tty_mode_ioctl(tty, cmd, arg);
#endif
	}
}

static struct tty_ldisc_ops can327_ldisc = {
	.owner		= THIS_MODULE,
	.name		= "can327",
	.num		= N_DEVELOPMENT,
	.receive_buf	= can327_ldisc_rx,
	.write_wakeup	= can327_ldisc_tx_wakeup,
	.open		= can327_ldisc_open,
	.close		= can327_ldisc_close,
	.ioctl		= can327_ldisc_ioctl,
};

static int __init can327_init(void)
{
	int status;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
	status = tty_register_ldisc(N_DEVELOPMENT, &can327_ldisc);
#else
	status = tty_register_ldisc(&can327_ldisc);
#endif
	if (status)
		pr_err("Can't register line discipline\n");

	return status;
}

static void __exit can327_exit(void)
{
	/* This will only be called when all channels have been closed by
	 * userspace - tty_ldisc.c takes care of the module's refcount.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
	int status;

	status = tty_unregister_ldisc(N_DEVELOPMENT);
	if (status)
		pr_err("Can't unregister line discipline (error: %d)\n",
		       status);
#else
	tty_unregister_ldisc(&can327_ldisc);
#endif
}

module_init(can327_init);
module_exit(can327_exit);

MODULE_ALIAS_LDISC(N_DEVELOPMENT);
MODULE_DESCRIPTION("ELM327 based CAN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Max Staudt <max-linux@enpas.org>");