# from pythonosc import udp_client

# client = udp_client.SimpleUDPClient("127.0.0.1", 1001) # 169.254.60.1
# client.send_message("/striker", ['s', 10, 1, 50, 90000])

import socket

msg = list("/striker")
msg.extend(['s', 8, 1, 35, 67620])
bin_msg = []
for m in msg:
    if (type(m) is str):
        bin_msg.append(ord(m))
    elif m > 255:
        bin_msg.extend(m.to_bytes(4, 'little'))
    else:
        bin_msg.append(m)

print(bin_msg)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytes(bin_msg), ("169.254.60.1", 1001))