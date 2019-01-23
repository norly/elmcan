Linux SocketCAN driver for ELM327
==================================

Authors
--------

Max Staudt <elmcan@enpas.org>



Motivation
-----------

CAN adapters are expensive, few, and far between.
ELM327 interfaces are cheap and plentiful.

This driver aims to lower the initial cost for hackers interested in
working with CAN buses.



Introduction
-------------

This driver is an effort to turn abundant ELM327 based OBD interfaces
into full-fledged (as far as possible) CAN interfaces.

Since the ELM327 was never meant to be a stand-alone CAN controller,
the driver has to switch between its modes asa quickly as possible in
order to approximate full-duplex operation.

As such, elmcan is a best-effort driver. However, this is more than
enough to implement simple request-response protocols (such as OBD II),
and to monitor broadcast messages on a bus (such as in a vehicle).

Most ELM327s come as nondescript serial devices, attached via USB or
Bluetooth. The driver cannot recognize them by itself, and as such it
is up to the user to attach it in form of a TTY line discipline
(similar to PPP, SLIP, slcan, ...).

This driver is meant for ELM327 versions 1.4b and up, see below for
known limitations in older controllers and clones.



Data sheet
-----------

The official data sheets can be found at ELM electronics' home page:

  https://www.elmelectronics.com/



How to check the controller version
------------------------------------

Use a terminal program to attach to the controller.

After issuing the "``AT WS``" command, the controller will respond with
its version::

    >AT WS


    ELM327 v1.4b

    >



How to attach the line discipline
----------------------------------

Every ELM327 chip is factory programmed to operate at a serial setting
of 38400 baud/s, 8 data bits, no parity, 1 stopbit.

The line discipline can be attached on a command prompt as follows::

    sudo ldattach \
           --debug \
           --speed 38400 \
           --eightbits \
           --noparity \
           --onestopbit \
           --iflag -ICRNL,INLCR,-IXOFF \
           26 \
           /dev/ttyUSB0

To change the ELM327's serial settings, please refer to its data
sheet. This needs to be done before attaching the line discipline.



Known limitations of the controller
------------------------------------

- All versions

  No automatic full duplex operation is supported. The driver will
  switch between input/output mode as quickly as possible.

  The length of outgoing RTR frames cannot be set. In fact, some
  clones (tested with one identifying as "``v1.5``") are unable to
  send RTR frames at all.

  We don't have a way to get real-time notifications on CAN errors.
  While there is a command (``AT CS``) to retrieve some basic stats,
  we don't poll it as it would force us to interrupt reception mode.


- Versions prior to 1.4b

  These versions do not send CAN ACKs when in monitoring mode (AT MA).
  However, they do send ACKs while waiting for a reply immediately
  after sending a frame. The driver maximizes this time to make the
  controller as useful as possible.

  Starting with version 1.4b, the ELM327 supports the "``AT CSM``"
  command, and the "listen-only" CAN option will take effect.


- Versions prior to 1.4

  These chips do not support the "``AT PB``" command, and thus cannot
  change bitrate or SFF/EFF mode on-the-fly. This will have to be
  programmed by the user before attaching the line discipline. See the
  data sheet for details.


- Versions prior to 1.3

  These chips cannot be used at all with elmcan. They do not support
  the "``AT D1``", which is necessary to avoid parsing conflicts on
  incoming data, as well as distinction of RTR frame lengths.

  Specifically, this allows for easy distinction of SFF and EFF
  frames, and to check whether frames are complete. While it is possible
  to deduce the type and length from the length of the line the ELM327
  sends us, this method fails when the ELM327's UART output buffer
  overruns. It may abort sending in the middle of the line, which will
  then be mistaken for something else.



Known limitations of the driver
--------------------------------

- No 8/7 timing.

  ELM327 can only set CAN bitrates that are of the form 500000/n, where
  n is an integer divisor.
  However there is an exception: With a separate flag, it may set the
  speed to be 8/7 of the speed indicated by the divisor.
  This mode is not currently implemented.

- No evaluation of command responses.

  The ELM327 will reply with OK when a command is understood, and with ?
  when it is not. The driver does not currently check this, and simply
  assumes that the chip understands every command.
  The driver is built such that functionality degrades gracefully
  nevertheless. See the section on known limitations of the controller.

- No use of hardware CAN ID filtering

  An ELM327's UART sending buffer will easily overflow on heavy CAN bus
  load, resulting in the "``BUFFER FULL``" message. Using the hardware
  filters available through "``AT CF xxx``" and "``AT CM xxx``" would be
  helpful here, however SocketCAN does not currently provide a facility
  to make use of such hardware features.

- No BUS-OFF state and automatic restart

  We currently reset the ELM327 and generate error frames manually.
  In the future, we may be able to use ``can_bus_off()`` and its siblings.



Communication example
----------------------

This is a short and incomplete introduction on how to talk to an ELM327.


The ELM327 has two modes:

- Command mode
- Reception mode

In command mode, it expects one command per line, terminated by CR.
By default, the prompt is a "``>``", after which a command can be
entered::

    >ATE1
    OK
    >

The init script in the driver switches off several configuration options
that are only meaningful in the original OBD scenario the chip is meant
for, and are actually a hindrance for elmcan.


When a command is not recognized, such as by an older version of the
ELM327, a question mark is printed as a response instead of OK::

    >ATUNKNOWN
    ?
    >

At present, elmcan does not evaluate this response and silently assumes
that all commands are recognized. It is structured such that it will
degrade gracefully when a command is unknown. See the sections above on
known limitations for details.


When a CAN frame is to be sent, the target address is configured, after
which the frame is sent as a command that consists of the data's hex
dump::

    >ATSH123
    OK
    >DEADBEEF12345678
    OK
    >

The above interaction sends the frame "``DE AD BE EF 12 34 56 78``" with
the 11 bit CAN ID ``0x123``.
For this to function, the controller must be configured for 11 bit CAN
ID sending mode (using "``AT PB``", see code or datasheet).


Once a frame has been sent and wait-for-reply mode is on (ATR1,
configured on listen-only=off), or when the reply timeout expires and
the driver sets the controller into monitoring mode (``ATMA``), the ELM327
will send one line for each received CAN frame, consisting of CAN ID,
DLC, and data::

    123 8 DEADBEEF12345678

For 29 bit CAN frames, the address format is slightly different, which
elmcan uses to tell the two apart::

    12 34 56 78 8 DEADBEEF12345678

The ELM327 will receive both 11 and 29 bit frames - the current CAN
config (``ATPB``) does not matter.


If the ELM327's internal UART sending buffer runs full, it will abort
the monitoring mode, print "BUFFER FULL" and drop back into command
mode. Note that in this case, unlike with other error messages, the
error message may appear on the same line as the last (usually
incomplete) data frame::

    12 34 56 78 8 DEADBEEF123 BUFFER FULL



To Do list for future development
----------------------------------

- Handle ``write()`` error

- Rename ``elm327_panic()``

- No auto-restart in ``elm327_panic()``?

- Stop current function when in ``elm327_panic()``

- DMA capable rx/tx buffers

- fixup constants, constant for '``>``'

- flushing of ``tx_work`` is too late in ``ldisc_close()``
