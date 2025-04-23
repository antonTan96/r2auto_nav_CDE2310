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
### Selection of Sensors and Actuators
**Selection of IR Sensors**
| **Sensor**       | **Image Output** | **FOV**  | **Temp Accuracy (°C)** | **Temp Range (°C)** | **Comms Protocol** | **Price (SGD)** |
|------------------|------------------|----------|-------------------------|---------------------|--------------------|-----------------|
| MLX90614xAA      | 1 x 1            | NIL      | ± 0.5                   | -70 – 380           | I2C                | ~20             |
| AMG8833          | 8 x 8            | 80°      | ± 2.5                   | 0 – 80              | I2C                | ~30             |
| MLX90640BAA      | 32 x 24          | 110°     | ± 2                     | -40 – 300           | I2C                | ~50             |
| MLX90640BAB      | 32 x 24          | 55°      | ± 2                     | -40 – 300           | I2C                | ~50             |
Initial selection was of MLX90640BAA based on the following rationale:
![image](https://github.com/user-attachments/assets/514c6c86-3d02-4ab1-abd0-d084bef3e5c8)
**Selection of SG90 Servos**
![image](https://github.com/user-attachments/assets/96a5a5db-11c6-4998-a126-2da009de524d)
**Selection of PCA9685**
![image](https://github.com/user-attachments/assets/00264b60-6474-45e2-b155-642774d7f6c5)
**Selection of LM2596**
![image](https://github.com/user-attachments/assets/b3eeb559-1755-4259-bf56-b0483e11d7d1)


Due to inability to source MLX90640BAA in time and concerns over computational power, we decided to proceed with the AMG8833
### Wiring Diagram and System Architecture
- [Wiring Diagram and System Architecture](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/elec_doc/Electrical%20diagram%20and%20Electronics%20system%20architecture.pdf)
### Power Calculation
- [Power Calculation](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/elec_doc/Power%20calculations.pdf)
### Component Testing Scripts
- [Component Testing](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/elec_doc/testing_code)
