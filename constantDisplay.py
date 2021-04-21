import serial
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time
from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

def engage_radar():
    i2c = busio.I2C(SCL, SDA)

    disp = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

    disp.fill(0)
    disp.show()

    width = disp.width
    height = disp.height
    image = Image.new("1", (width, height))

    draw = ImageDraw.Draw(image)

    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Draw some shapes.
    # First define some constants to allow easy resizing of shapes.
    padding = -2
    top = padding
    bottom = height - padding
    # Move left to right keeping track of the current x position for drawing shapes.
    dispCursor = 0

    font = ImageFont.load_default()

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
        port='/dev/ttyUSB0',
        baudrate=115200,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE
    )
    ser.isOpen()
    draw.text((dispCursor, top + 0), "ENGAGING RADAR STANDBY...", font=font, fill=255)
    disp.image(image)
    disp.show()
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
                    #print(x[0][0])
                    text = "Distance(cm): {}".format(str(np.round(x[0][0],2)))
                    draw.rectangle((0,0,width,height),outline=0,fill=0)
                    draw.text((dispCursor, top + 0), text, font=font, fill=255)
                    disp.image(image)
                    disp.show()



    for i in range(len(times)):
        print("{},{},{}".format(times[i], raw[i], filtered[i]))
    return

if __name__ == '__main__':
    engage_radar()
