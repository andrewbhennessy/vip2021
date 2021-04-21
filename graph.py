import serial
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from numba import jit
import time
import matplotlib.pyplot as plt

#@jit(parallel=True)
def engage_radar():
    my_filter = KalmanFilter(dim_x=2, dim_z=1)

    my_filter.x = np.array([[2.],
                            [0.]])  # initial state (location and velocity)

    my_filter.F = np.array([[1., 1.],
                            [0., 1.]])  # state transition matrix

    my_filter.H = np.array([[1., 0.]])  # Measurement function
    my_filter.P *= 1000.  # covariance matrix
    my_filter.R = 5  # state uncertainty



    ser = serial.Serial(
        port='COM3',
        baudrate=115200,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE
    )

    ser.isOpen()

    times = []
    raw = []
    filtered = []
    currentTime = int(np.round(time.time() % 60))
    while (int(np.round(time.time()%60,0)) - currentTime) < 10:
        while ser.inWaiting() >= 14:
            f = str(ser.read(14).hex()).strip("f")[6:].lstrip("0")
            if f == "00" or f == "":
                pass
            else:
                my_filter.predict()
                my_filter.update(int(f.upper(), 16))
                times.append(np.round(time.time()%60,2)-currentTime)
                if len(times) < 2:
                    dt = 0.1
                else:
                    dt = times[-1] - times[-2]
                my_filter.Q = Q_discrete_white_noise(2, dt, .1)
                raw.append(int(f.upper(),16))
                # do something with the output
                x = my_filter.x
                print(x[0][0])
                filtered.append(x[0][0])

    for i in range(len(times)):
        print("{},{},{}".format(times[i], raw[i], filtered[i]))


    plt.scatter(times, raw, s=5, c='red')
    plt.plot(times, filtered)
    plt.title("Kalman Filtered Discrete Radar Measurements vs time")
    plt.xlabel("Time(s)")
    plt.ylabel("Distance(cm)")
    plt.show()



if __name__ == '__main__':
    engage_radar()
