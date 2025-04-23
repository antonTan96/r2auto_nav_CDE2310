import time
import busio
import board
import adafruit_amg88xx
import numpy as np
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c,0x68)
img = np.zeros((8,8))
cols = np.zeros(8)


while True:
    for i in range(8):
        for j in range(8):
            if amg.pixels[i][j] > cols[j]:
                cols[j] = amg.pixels[i][j]
            #img[i][j] = amg.pixels[i][j]
        
    #print(img)
    print(cols)
    cols = np.zeros(8)


