Using ELM327 driver out-of-tree
================================

Requirements
-------------

This requires Linux 4.11 (for 431af779256c) at the very least.

Also, can327 depends on ``can-dev``:

    sudo modprobe can-dev



Install
-------
    cd module/

    sudo dkms install .




Porting and alternative licences
=================================

This driver started as a derivative of linux/drivers/net/can/slcan.c
and my thanks go to the original authors for their inspiration, even
after almost none of their code is left.

This code barely bears any resemblance to slcan anymore, and whatever
may be left is Linux specific boilerplate anyway, however I am leaving
the GPL-2.0 identifier at the top just to be sure.

Please feel free to use my own code, especially the ELM327 communication
logic, in accordance with SPDX-License-Identifier BSD-3-Clause to port
this driver to other systems.

If in doubt, feel free to ask me to clarify which code is mine.




Thanks
=======

Several people have encouraged me to finish this - thank you all.

Thanks to Oliver Neukum for his early (pre-v1) reviews and suggestions.

Thanks to Greg Kroah-Hartman, Marc Kleine-Budde for their upstream reviews.

Thanks to Vincent Mailhol for the extra upstream review help and push,
unexpected and very appreciated.

Thanks to tomaszduda23 for the dkms.conf file.
