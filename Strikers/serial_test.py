import serial
import time

striker_controller = serial.Serial('/dev/tty.usbmodemFFFFFFFEFFFF1', baudrate=115200, timeout=0.1)
time.sleep(1)
print(striker_controller.read_all().decode())

# msg = f"r{0}{0}{0}{0}"
# striker_controller.write(bytes(msg, 'utf-8'))
# time.sleep(5)

# data = [('s', 0b00000001, 80), ('s', 0b00000100, 88), ('s', 0b00001000, 96), ('s', 0b00000001, 48), ('s', 0b00000001, 64), ('s', 0b00000001, 96)]
data = [('s', 1, 80, 0), ('s', 2, 80, 0), ('s', 3, 80, 0), ('s', 4, 80, 0), ('s', 5, 80, 0), ('s', 6, 80, 0), ('s', 7, 80, 0)]
# data = [('s', 6, 32, 0), ('s', 6, 64, 0), ('s', 6, 96, 0), ('s', 6, 127, 0)]

# data = [('c', 1, 3, 30), ('c', 1, 3, 0), ('c', 1, 3, 60), ('c', 1, 3, 0), ('c', 1, 3, 90), ('c', 1, 3, 0), ('c', 1, 3, 10)]
for d in data:
    for i in range(4):
        id = 1 << (d[1] - 1)
        msg = f"{d[0]}{chr(id)}{chr((d[2] >> 8) & 0xFF)}{chr(d[2] & 0xFF)}{chr((d[3] >> 8) & 0xFF)}{chr(d[3] & 0xFF)}\n"
        # print(d[0], id, d[2], d[3], msg, end="")
        striker_controller.write(bytes(msg, 'utf-8'))
        time.sleep(0.5)
        while (striker_controller.in_waiting > 0):
            print(striker_controller.readline().decode().strip('\n\r'))

time.sleep(0.1)

while (striker_controller.in_waiting > 0):
    print(striker_controller.readline().decode().strip('\n\r'))
    time.sleep(0.0011)

striker_controller.close()