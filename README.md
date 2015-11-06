# bsc-slave

Here you can find the source code of the BSC-SLAVE kernel module for 
the BCM2835 ARM soc used for the raspberry pi. I have tested it for 
the rasperry pi 2 but it should work for the model B+ too.

There is a precompiled version of the module available (bsc-slave.ko)
wich you can simply insert with

#> insmod bsc-slave.ko

It is compiled for the kernel version 3.18.12-v7+

If you d'ont have this kernel version (verify with: uname -r)you need to 
compile the module yourself. This is what the Makefile is for. 

There are many possibilities. I chose installing rpi-update. You can find 
instructions on: https://github.com/Hexxeh/rpi-update

Than you need to install the kernel sources. Instructions you can find on:
https://github.com/notro/rpi-source/wiki
 
The file: i2ccat.c is just an exemple of how to talk to the i2c controller 
of the pi. You give yourself a slave address and than read the FIFOs by means
of the read and write system call.

This is my first kernel module and I am not a professional programmer so
I would be glad if someone more experienced makes this module better. At 
the moment only one process can access the bus. 

The controller also supports SPI Slave mode. I had no chance to make that 
running. If you do I would be very happy if you could complete the module. 

SDA is on GPIO18 (PIN HEADER 12) while SCL is on GPIO19 (PIN HEADER 35)
