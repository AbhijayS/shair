import serial
import time

print("Gently rotate it around in all directions")

ser = serial.Serial("COM10", 115200)
# clean up input buffer
timeout = time.time()+3
while time.time() < timeout:
    ser.readline()

# clear calibration file
with open('calibration.txt', 'w') as f:
    pass

ts = 0
while True:
    data = ser.readline().decode('utf-8').split(',')
    t = int(data[0])

    # sample every 50ms
    if t-ts >= 50:
        with open('calibration.txt', 'a') as f:
            mag = list(map(lambda x: int(x), data[7:10]))
            print(mag)
            f.write("{}    {}    {}\n".format(mag[0], mag[1], mag[2]))
        ts = t