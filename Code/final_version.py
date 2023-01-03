import time
from math import radians

import numpy as np
import quaternion
from ahrs.filters import Madgwick, Mahony, EKF
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


ALGO_NAME = 'Madgwick'
algo_wo_magnet = Madgwick()
algo_w_magnet = Madgwick()
# algo_wo_magnet = Mahony()
# algo_w_magnet = Mahony()
# algo_wo_magnet = EKF(frame='NED')


t = 0
v_wo = np.array([0., 0., 0.])
v_w = np.array([0., 0., 0.])

x_wo = np.array([0., 0., 0.])
x_w = np.array([0., 0., 0.])


class Config:
    def __init__(self) -> None:
        self.sliding_window = True
        self.moving_average = False

    def get_name(self):
        if self.sliding_window:
            return 'SW'
        elif self.moving_average:
            return 'MA'
        else:
            return 'SIMPLE'

class Logger:
    def __init__(self, algo_name: str, config_name: str, adjustment_a0: bool) -> None:
        self.a_list = []
        self.v_list = []
        self.x_list = []
        self.algo_name = algo_name
        self.config_name = config_name
        self.adjustment_a0 = 'adjust' if adjustment_a0 else 'NOadjust'

    def update(self, a, v, x):
        self.a_list.append(a.tolist())
        self.v_list.append(v.tolist())
        self.x_list.append(x.tolist())

    def print(self):
        with open(f'out/output_a_{self.adjustment_a0}_{self.algo_name}_{self.config_name}.txt', 'w') as f:
            for a in self.a_list:
                print(a, file=f)
        with open(f'out/output_v_{self.adjustment_a0}_{self.algo_name}_{self.config_name}.txt', 'w') as f:
            for v in self.v_list:
                print(v, file=f)
        with open(f'out/output_x_{self.adjustment_a0}_{self.algo_name}_{self.config_name}.txt', 'w') as f:
            for x in self.x_list:
                print(x, file=f)


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
        self.q_prev = np.array([1., 0., 0., 0.])

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
        q = algo_wo_magnet.updateIMU(self.q_prev, gyr=imu_data.gyr, acc=imu_data.acc, dt=dt)
        # q = algo_wo_magnet.update(self.q_prev, gyr=imu_data.gyr, acc=imu_data.acc, dt=dt)
        # uncomment above for ekf
        return q

class AccelartionCalculatorWithMagnet(AccelartionCalculator):
    def get_q(self, imu_data: IMUData, dt):
        q = algo_w_magnet.updateMARG(self.q_prev, gyr=imu_data.gyr, acc=imu_data.acc, mag=imu_data.mag, dt=dt)
        return q

# a_calculator_w = AccelartionCalculatorWithMagnet()
a_calculator_w = AccelartionCalculatorWithOutMagnet()
a_calculator_wo = AccelartionCalculatorWithOutMagnet()

prev_time = time.time()
imu_data = get_imu_data()
dt = time.time() - prev_time
prev_time = time.time()
a0_w = a_calculator_w.get_acceleration(imu_data, dt)
a0_wo = a_calculator_wo.get_acceleration(imu_data, dt)


ADJUSTMENT_A0 = True
if ADJUSTMENT_A0:
    # adjust a0
    for i in range(1000):
        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time
        imu_data = get_imu_data()

        a_wo = a_calculator_wo.get_acceleration(imu_data, dt, a0_wo)
        a_w = a_calculator_w.get_acceleration(imu_data, dt, a0_w)

        a0_wo = a0_wo + 0.1 * a_wo
        a0_w = a0_w + 0.1 * a_w

print('INITIAL ACCELERATION ESTIMATED')
print(f'A0_wo: {a0_wo}')
print(f'A0_w: {a0_w}')
prev_time = time.time()
SLIDING_SIZE = 5
MA_RATE = 0.8
sliding_window_a_wo = np.array([a0_wo] * SLIDING_SIZE)
sliding_window_a_w = np.array([a0_w] * SLIDING_SIZE)
a_wo_prev = a0_wo
a_w_prev = a0_w
config = Config()
logger = Logger(algo_name=ALGO_NAME, config_name=config.get_name(), adjustment_a0=ADJUSTMENT_A0)
NUM_ITERATION = 40
while True:
    t += 1
    curr_time = time.time()
    dt = curr_time - prev_time
    prev_time = curr_time
    imu_data = get_imu_data()

    a_wo = a_calculator_wo.get_acceleration(imu_data, dt, a0_wo)
    a_w = a_calculator_w.get_acceleration(imu_data, dt, a0_w)
    if config.sliding_window:
        sliding_window_a_wo = np.delete(sliding_window_a_wo, 0, axis=0)
        sliding_window_a_w = np.delete(sliding_window_a_w, 0, axis=0)
        sliding_window_a_wo = np.vstack((sliding_window_a_wo, a_wo))
        sliding_window_a_w = np.vstack((sliding_window_a_w, a_w))
        a_wo = sliding_window_a_wo.mean(axis=0)
        a_w = sliding_window_a_w.mean(axis=0)

    elif config.moving_average:
        a_wo = a_wo * MA_RATE + a_wo_prev * (1 - MA_RATE)
        a_w = a_w * MA_RATE + a_w_prev * (1 - MA_RATE)
        a_wo_prev = a_wo
        a_w_prev = a_w

    v_wo += a_wo * dt
    v_w += a_w * dt

    x_wo += v_wo * dt
    x_w += v_w * dt
    if t % 100 == 0:
        # logger.update(a_wo, v_wo, x_wo)
        print('acceleration')
        print(a_wo)
        # print(a_w)
        print('*'*30)
        print('velocity')
        print(v_wo)
        # print(v_w)
        print('*'*30)
        print('location')
        print(x_wo)
        # print(x_w)
        print('\n'*4)

    if t % (100 * NUM_ITERATION) == 0:
        # logger.print()
        exit(0)
