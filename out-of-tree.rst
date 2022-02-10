Using ELM327 driver out-of-tree
================================

Requirements
-------------

This requires Linux 4.11 (for 431af779256c), and has been tested on 4.19.

Also, elmcan depends on ``can-dev``:

    sudo modprobe can-dev



Install
-------
    cd module/

    sudo dkms install .




Thanks
=======

Thanks go out to Oliver Neukum for his early reviews and suggestions.

Several more people have encouraged me to finish this - thank you all.
