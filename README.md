# nRF24L01+ library for Atmel XMega

Library for the Nordic nRF24L01+ transceiver to be used on Atmel XMega
chips.  Includes sample sender and receiver code designed to
demonstrate over-the-air programming.  All code tested on a pair of
ATXmega32a4u chips.

The sender code accepts commands over a serial connection to send and
receive data.  Also includes a simple python script
(`sender/upload_ota.py`) to upload hex files to the receiver.  To use,
pass in the type of chip and the hex file to upload:

    ./upload_ota.py -p atxmega32a4u ../receiver/receiver.hex 

The receiver code uses the
[XBoot](https://github.com/alexforencich/xboot) API to update the
firmware on the receiver based on data from the sender.

### License

Unless otherwise noted, all source is released under the MIT license.
Included code from the Atmel Software Framework is licensed under a
BSD 3-clause license as spelled out in those files.
