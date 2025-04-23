Can use the attached scripts to test individual components:

# PCA9685 Servo Motor Driver:
- reload.py
- firing.py

# AMG8833 IR Camera:
- ir.py

# I2C address
To confirm I2C addresses on the bus, run 
```
i2cdetect -y 1
```
AMG8833: 0x69 or 0x68
PCA9685: 0x40
Update addresses accordingly in the respective script if there are any variations
