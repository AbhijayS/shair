import serial
import time

s = serial.Serial("COM12", 115200, timeout=0.01)
timeout = time.time()+1
while time.time() < timeout:
    s.readline()

ts = 0
b = ''
while True:
    if s.in_waiting:
        raw = s.read_all().decode('utf-8')
        if '\n' in raw:
            raw = raw.split('\n')
            if len(raw) > 2:
                # ...stuff\nstuff\nstuff
                #           ^^^^^  ^^^^^
                #           data   buff
                data = raw[-2]
                b = raw[-1]
            else:
                # stuff\nstuff
                # ^^^^^  ^^^^^
                # data   buff
                data = b + raw[0]
                b = raw[1]
            print(data)
        else:
            b = b + raw

# ts = 0
# while True:
#     t = int(s.readline().decode('utf-8').split(',')[0])
#     if t < ts:
#         ts = ts-1000
#     print(t-ts)
#     ts = t