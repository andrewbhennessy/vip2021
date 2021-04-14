import serial
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time

def engage_radar():
    my_filter = KalmanFilter(dim_x=2, dim_z=1)

    my_filter.x = np.array([[2.],
                            [0.]])  # initial state (location and velocity)

    my_filter.F = np.array([[1., 1.],
                            [0., 1.]])  # state transition matrix

    my_filter.H = np.array([[1., 0.]])  # Measurement function
    my_filter.P *= 1000.  # covariance matrix
    my_filter.R = 5  # state uncertainty

      # process uncertainty
    #TODO:Figure out which serial port py uses
    ser = serial.Serial(
        port='COM3',
        baudrate=115200,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE
    )

    ser.isOpen()

    times = []
    currentTime = int(np.round(time.time() % 60))
    while 1:
        while ser.inWaiting() >= 14:
            for elem in range(13):
                #try:
                f = str(ser.read(14).hex()).strip("f")[6:].lstrip("0")
                if f == "00" or f == "":
                    pass
                else:
                    my_filter.predict()
                    my_filter.update(int(f.upper(), 16))
                    times.append(np.round(time.time()%60,2)-currentTime)
                    if len(times) == 0 or len(times) == 1:
                        dt = 0.1
                    else:
                        times = times[-2:len(times)]
                        dt = times[-1] - times[-2]
                    my_filter.Q = Q_discrete_white_noise(2, dt, .1)
                    # do something with the output
                    x = my_filter.x
                    print(x[0][0])



    for i in range(len(times)):
        print("{},{},{}".format(times[i], raw[i], filtered[i]))
    return

if __name__ == '__main__':
    engage_radar()
