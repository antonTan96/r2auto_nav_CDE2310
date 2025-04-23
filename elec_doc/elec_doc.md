# For electrical documentation

###Components used
- TurtleBot components
- Adafruit PCA9685 16-Channel Servo Driver
- SG90 Servo x9
- LM2596 DC-DC converter
- AMG8833 IR Sensor

###Connections
1. Solder female jumper wires to all 4 pins (IN+, IN-, OUT+, OUT-) of the LM2596 DC-DC converter. Connect IN+ and IN- to the 12V and GND on the OpenCR board. Connect OUT+ and OUT- to V+ and GND on the PCA9685 Servo Driver.
2. Solder male jumper wires to VIN, GND, SCL and SDA on the AMG8833 IR Sensor.
3. Solder male jumper wires to VCC, GND, SCL and SDA on the PCA9685 Servo Driver.
4. Splice female jumper wires to form a "Y" shape. Make 4 of these. 
- Use one of the "Y" shaped wires to connect GND on the RaspberryPi, GND on the AMG8833, and GND on the PCA9685.
- Take another one and connect 3V3 power on the RaspberryPi, VIN on the AMG8833, and VCC on the PCA9685.
- The third one should connect GPIO Pin 2 (SDA) on the RaspberryPI, SDA on the AMG8833, and SDA on the PCA9685.
- The fourth on should connect GPIO Pin 3 (SCL) on the RaspberryPi, SCL on the AMG8833, and SCL on the PCA9685.
Note: Ideally, the length of these wires should be within 10cm to ensure the best I2C communication between the RaspberryPi and the IR sensor. 
