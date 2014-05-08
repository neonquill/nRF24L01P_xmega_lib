#!/usr/bin/env python3

import serial
import struct

ser = serial.Serial('/dev/tty.usbserial-A40188LY', 115200, timeout = 1)

def send_packet(pkt):
    cmd = b'W' + struct.pack('B', len(pkt)) + pkt
    ser.write(cmd)
    print("#XXX Sent cmd {}".format(cmd))

pkt = struct.pack('bbb', 24, 36, 43)
send_packet(pkt)
