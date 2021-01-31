import serial
import time
from config import UBIT_COM_PORT

print("Gently rotate it around in all directions")

ser = serial.Serial(UBIT_COM_PORT, 115200)
# clean up input buffer
timeout = time.time()+3
while time.time() < timeout:
    ser.readline()

# clear calibration file
with open('calibration.txt', 'w') as f:
    pass

while True:
    data = ser.readline().decode('utf-8').split(',')
    if len(data) == 3:
        print(data)
        # with open('calibration.txt', 'a') as f:
        #     mag = list(map(lambda x: int(x), data[7:10]))
        #     print(mag)
        #     f.write("{}    {}    {}\n".format(mag[0], mag[1], mag[2]))