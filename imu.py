import serial
import time
from ahrs.filters import Madgwick
from ahrs import Quaternion
import numpy as np
import math
import operator
import config

class IMU:
    # constants
    FREQUENCY = 100
    GAIN = 0.2
    GRAVITY = 9.80665
    SENSOR_SIGN = [1,1,1,-1,-1,-1,1,1,1] # order: gyrxyz, acclxyz, magxyz
    MAG_HARD_IRON = config.MAG_HARD_IRON
    MAG_SOFT_IRON = config.MAG_SOFT_IRON

    def __init__(self):
        ser = serial.Serial(config.ARDUINO_COM_PORT, config.ARDUINO_BAUD_RATE)
        self.ser_buff = ''

        # clean up input buffer
        timeout = time.time()+1
        while time.time() < timeout:
            ser.read_all()

        self.Q = [1.0, 0.0, 0.0, 0.0]
        self.madwick = Madgwick()
        # self.madwick.frequency = self.FREQUENCY
        self.madwick.beta = self.GAIN
        self.timestamp = 0

        # calculate offsets
        ACC_OFFSETS = [0]*3 #xyz
        GYR_OFFSETS = [0]*3 #xyz
        for i in range(32):
            data = ser.readline().decode('utf-8').split(',')
            ax,ay,az = list(map(lambda x: self.accel_meter_per_sec(int(x)), data[1:4]))
            gx,gy,gz = list(map(lambda x: self.gyro_rad_per_sec(int(x)), data[4:7]))
            GYR_OFFSETS[0] = GYR_OFFSETS[0] + gx
            GYR_OFFSETS[1] = GYR_OFFSETS[1] + gy
            GYR_OFFSETS[2] = GYR_OFFSETS[2] + gz
            ACC_OFFSETS[0] = ACC_OFFSETS[0] + ax
            ACC_OFFSETS[1] = ACC_OFFSETS[1] + ay
            ACC_OFFSETS[2] = ACC_OFFSETS[2] + az
        GYR_OFFSETS = list(map(lambda x: x/32, GYR_OFFSETS))
        ACC_OFFSETS = list(map(lambda x: x/32, ACC_OFFSETS))

        self.ACC_OFFSETS = ACC_OFFSETS
        self.GYR_OFFSETS = GYR_OFFSETS
        self.ser = ser

        # initalize imu
        timeout = time.time()+10
        while time.time() < timeout:
            self.update()
        
        print("IMU ready to go!")

    # https://www.pololu.com/file/0J1087/LSM6DS33.pdf
    def accel_meter_per_sec(self, a_raw):
        return a_raw * 0.061 * self.GRAVITY / 1000

    # https://www.pololu.com/file/0J1087/LSM6DS33.pdf
    def gyro_rad_per_sec(self, g_raw):
        return math.radians(g_raw * 8.75 / 1000)

    # https://www.pololu.com/file/0J1089/LIS3MDL.pdf
    def mag_milli_tesla(self, m_raw):
        return m_raw / 68420.0
    
    # call this update loop at least self.FREQUENCY or better
    def update(self):
        if self.ser.in_waiting:
            raw = self.ser.read_all().decode('utf-8')

            if '\n' in raw:
                raw = raw.split('\n')
                if len(raw) > 2:
                    data = (raw[-2]).split(',')
                else:
                    data = (self.ser_buff + raw[-2]).split(',')
                self.ser_buff = raw[-1]

                if len(data) != 10:
                    print(data)
                    raise Exception("Serial data ill-formatted")
                    
                t = int(data[0])

                self.madwick.Dt = (t-self.timestamp)/1000

                # TODO: use numpy here; make it faster
                # a = accel_meter_per_sec(sign*(a_raw-offset))
                ax,ay,az = list(map(lambda x : self.accel_meter_per_sec(x), map(operator.mul, self.SENSOR_SIGN[3:6], map(operator.sub, map(lambda x: int(x), data[1:4]), self.ACC_OFFSETS))))
                # g = gyro_rad_per_sec(sign*(a_raw-offset))
                gx,gy,gz = list(map(lambda x : self.gyro_rad_per_sec(x), map(operator.mul, self.SENSOR_SIGN[0:3], map(operator.sub, map(lambda x: int(x), data[4:7]), self.GYR_OFFSETS))))
                # ax,ay,az = list(map(lambda x: accel_meter_per_sec(int(x)), data[1:4]))
                # gx,gy,gz = list(map(lambda x: gyro_rad_per_sec(int(x)), data[4:7]))

                # m = matrix*(m-bias)
                mx,my,mz = list(map(operator.sub, map(lambda x: int(x), data[7:10]), self.MAG_HARD_IRON))
                mx = mx*self.MAG_SOFT_IRON[0] + my*self.MAG_SOFT_IRON[1] + mz*self.MAG_SOFT_IRON[2]
                my = mx*self.MAG_SOFT_IRON[3] + my*self.MAG_SOFT_IRON[4] + mz*self.MAG_SOFT_IRON[5]
                mz = mx*self.MAG_SOFT_IRON[6] + my*self.MAG_SOFT_IRON[7] + mz*self.MAG_SOFT_IRON[8]
                mx,my,mz = list(map(lambda x: self.mag_milli_tesla(int(x)), (mx,my,mz)))

                self.timestamp = t
                self.Q = self.madwick.updateMARG(self.Q, gyr=np.array([gx,gy,gz]), acc=np.array([ax,ay,az]), mag=np.array([mx,my,mz]))
            else:
                self.ser_buff = self.ser_buff + raw

    def get_euler_angles(self):
        return list(map(lambda x : math.degrees(x), Quaternion(self.Q).to_angles()))

def main():
    myIMU = IMU()

    # clear log file
    with open('imu.data', 'w') as f:
        pass

    while True:
        myIMU.update()
        with open('imu.data', 'a') as f:
            euler = myIMU.get_euler_angles()
            f.write(str(euler[0]) + "," + str(euler[1]) + "," + str(euler[2]) + "\n")
            # print(euler)
            time.sleep(.1)
        
if __name__ == "__main__":
    main()