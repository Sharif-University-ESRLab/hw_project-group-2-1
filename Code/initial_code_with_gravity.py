import time
from math import radians

import numpy as np
import quaternion
from ahrs.filters import Madgwick
from magno_gy import gy801


sensors = gy801()
gyro = sensors.gyro
accel = sensors.accel
compass = sensors.compass
barometer = sensors.baro

def read_accel():
    return accel.getX(), accel.getY(), accel.getZ()
    
def read_gyro():
    return radians(gyro.getX()), radians(gyro.getY()), radians(gyro.getZ())

def read_magnet():
    return compass.getX(), compass.getY(), compass.getZ()

madgwick_wo_magnet = Madgwick()
madgwick_w_magnet = Madgwick()
Q_w_magnet = [1., 0., 0., 0.]

t = 0
v_wo = np.array([0., 0., 0.])
v_w = np.array([0., 0., 0.])

x_wo = np.array([0., 0., 0.])
x_w = np.array([0., 0., 0.])


def calcualte_rotation_matrix(Q):
    np_quaternion = np.quaternion(Q[0], Q[1], Q[2], Q[3])
    return quaternion.as_rotation_matrix(np_quaternion)    

class IMUData:
    def __init__(self, gyr, acc, mag) -> None:
        self.gyr = gyr
        self.acc = acc
        self.mag = mag

def get_imu_data() -> IMUData:
    acc = read_accel()
    gyr = read_gyro()
    mag = read_magnet()    

    return IMUData(gyr, acc, mag)

class AccelartionCalculator:
    def __init__(self):
        self.q_prev = [1., 0., 0., 0.]

    def get_q(self, imu_data: IMUData, dt):
        pass

    def get_acceleration(self, imu_data: IMUData, dt, a0=np.array([0, 0, 0])):
        q = self.get_q(imu_data, dt)
        rotation_matrix = calcualte_rotation_matrix(q)
        a = np.dot(rotation_matrix, imu_data.acc) - a0

        self.q_prev = q
        return a

class AccelartionCalculatorWithOutMagnet(AccelartionCalculator):
    def get_q(self, imu_data: IMUData, dt):
        q = madgwick_w_magnet.updateIMU(self.q_prev, gyr=imu_data.gyr, acc=imu_data.acc, dt=dt)
        return q

class AccelartionCalculatorWithMagnet(AccelartionCalculator):
    def get_q(self, imu_data: IMUData, dt):
        q = madgwick_w_magnet.updateMARG(self.q_prev, gyr=imu_data.gyr, acc=imu_data.acc, mag=imu_data.mag, dt=dt)
        return q
    

a_calculator_w = AccelartionCalculatorWithMagnet()
a_calculator_wo = AccelartionCalculatorWithOutMagnet()

prev_time = time.time()
imu_data = get_imu_data()
dt = time.time() - prev_time
prev_time = time.time()
a0_w = a_calculator_w.get_acceleration(imu_data, dt)
a0_wo = a_calculator_wo.get_acceleration(imu_data, dt)


prev_time = time.time()
while True:
    t += 1
    curr_time = time.time()
    dt = curr_time - prev_time
    prev_time = curr_time
    imu_data = get_imu_data()

    a_wo = a_calculator_wo.get_acceleration(imu_data, dt, a0=a0_wo)
    a_w = a_calculator_w.get_acceleration(imu_data, dt, a0=a0_w)

    v_wo += a_wo * dt
    v_w += a_w * dt

    x_wo += v_wo * dt
    x_w += v_w * dt

    if t  % 100 == 0:
        print(f'acceleration without magnometer:\t\t{a_wo}')
        print(f'acceleration with magnometer:\t\t{a_w}')
        print('*'*30)
        print(f'velocity without magnometer:\t\t{v_wo}')
        print(f'velocity with magnometer:\t\t{v_w}')
        print('*'*30)
        print(f'location without magnometer:\t\t{x_wo}')
        print(f'location with magnometer:\t\t{x_w}')

        print('\n'*4)


