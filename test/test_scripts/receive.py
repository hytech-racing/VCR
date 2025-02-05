import socket
import struct

MCAST_GRP = '239.1.1.1'  # Multicast IP matching the Teensy's group
MCAST_PORT = 5000        # Port matching the Teensy's receiver

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

# Send a float value
import struct
data = struct.pack('f', 42.42)  # Convert float to bytes
sock.sendto(data, (MCAST_GRP, MCAST_PORT))
