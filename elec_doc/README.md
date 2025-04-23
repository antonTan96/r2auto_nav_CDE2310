# Electrical documentation

### Components used
- TurtleBot components
- Adafruit PCA9685 16-Channel Servo Driver
- SG90 Servo x9
- LM2596 DC-DC converter
- AMG8833 IR Sensor

### Connections
1. Solder female jumper wires to IN+ and IN-, and male jumper wires to OUT+ and OUT- of the LM2596 DC-DC converter. Connect IN+ and IN- to the 12V and GND on the OpenCR board, respectively. Connect OUT+ and OUT- to V+ and GND of the power terminal on the PCA9685 Servo Driver, respectively.
2. Solder male jumper wires to VIN, GND, SCL and SDA on the AMG8833 IR Sensor.
3. Splice female jumper wires to form a "Y" shape. Make 4 of these.
    - Use one of the "Y" shaped wires to connect GND on the RaspberryPi, GND on the AMG8833, and GND on the PCA9685.
    - Take another one and connect 3V3 power on the RaspberryPi, VIN on the AMG8833, and VCC on the PCA9685.
    - The third one should connect GPIO Pin 2 (SDA) on the RaspberryPi, SDA on the AMG8833, and SDA on the PCA9685.
    - The fourth one should connect GPIO Pin 3 (SCL) on the RaspberryPi, SCL on the AMG8833, and SCL on the PCA9685.
    - Note: Keep the length of the wires to a minimum to ensure the best I2C communication between the RaspberryPi and the AMG8833.
4.  Connect the nine servos to the output ports 0, 1, 2, 4, 5, 6, 8, 9, 10 of the PCA9685. Ensure that the ground, power, and signal wires are connected appropriately with their respective pins.

### Wiring Diagram and System Architecture
- [Wiring Diagram and System Architecture](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/elec_doc/Electrical%20diagram%20and%20Electronics%20system%20architecture.pdf)
### Power Calculation
- [Power Calculation](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/elec_doc/Power%20calculations.pdf)
### Component Testing Scripts
- [Component Testing](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/elec_doc/testing_code)
