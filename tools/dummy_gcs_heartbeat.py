#!/usr/bin/env python3
import socket
import struct
import time

def x25_crc(data: bytes) -> int:
    crc = 0xffff
    for b in data:
        tmp = b ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc

def mavlink_v1_heartbeat(seq: int) -> bytes:
    # MAV_TYPE_GCS=6, MAV_AUTOPILOT_INVALID=8, MAV_STATE_ACTIVE=4
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)
    header = struct.pack("<BBBBB", len(payload), seq & 0xff, 255, 190, 0)
    crc = x25_crc(header + payload + bytes([50]))
    return bytes([0xFE]) + header + payload + struct.pack("<H", crc)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 14550))
sock.setblocking(False)

peers = set()
seq = 0
print("dummy GCS heartbeat listening on UDP 14550")

while True:
    try:
        while True:
            data, addr = sock.recvfrom(4096)
            if addr not in peers:
                peers.add(addr)
                print("PX4 peer:", addr)
    except BlockingIOError:
        pass

    pkt = mavlink_v1_heartbeat(seq)
    seq += 1
    for peer in list(peers):
        sock.sendto(pkt, peer)

    time.sleep(1.0)
