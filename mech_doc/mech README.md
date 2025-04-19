# TurtleBot ICBM mechanical documentation

This document provides detailed instructions for printing, assembling, and integrating the ICBM launch piece onto your TurtleBot.

---

## Components

### TurtleBot3 Subcomponents
- LiDAR
- RaspberryPi
- OpenCR 1.0
- 2x Dynamixel motors
- LiPo Battery
- USB2LDS

### Bill of Materials
- Adafruit PCA9685 16-Channel Servo Driver
- LM2596 DC-DC converter
- AMG8833 IR Sensor
- SG90 Servos x9
- Springs x9
- Alt Tab PLA Filament 1kg

### Printed Parts
- **Main Launch Piece (ICBM)**: Printed as one complete piece.
- **Lids (9 pieces)**: Individually printed lids to secure springs.

### Hardware
- **Springs (9 pieces)**: Fit into grooves on the main launch piece and lids.
- **Locking Pins (6 pieces)**: Used to secure the main launch piece to the waffle plate.
- **DC-DC Converter and Wiring**
- **Servo Motors**
- **Servo Controller Board**

---

## Assembly Instructions

### Step 1: Preparation
1. **Remove TurtleBot Layers**: Remove the top two layers of your TurtleBot to expose the OpenCR board layer.
2. **Remove OpenCR Board**: Carefully remove the OpenCR board to access the waffle plate.

### Step 2: Installing the ICBM
1. **Positioning the Launch Piece**:
   - Place the main launch piece on the waffle plate.
   - Identify two extrusion plates at the center; the thinner extrusion fits between the two plate support pieces on the board, while the thicker one faces the opposite side.

2. **Securing the Launch Piece**:
   - Align the holes on both extrusion plates with the corresponding holes on the waffle plate.
   - Use the locking pins (total of 6 pins, 3 for each extrusion) to firmly secure the launch piece onto the waffle plate.

### Step 3: Reinstalling Components
1. **Replace OpenCR Board**:
   - Carefully reposition and lock down the OpenCR board on the PCB, further securing the extrusion plates.

2. **Reassemble TurtleBot Layers**:
   - Reattach the previously removed top layers onto the TurtleBot.

### Step 4: Lid and Spring Assembly
1. **Spring Installation**:
   - Glue each spring into the groove of its respective lid using **superglue** (strongest recommended)

2. **Lid Installation**:
   - Once the glue sets, glue the lid-and-spring assembly into the corresponding groove on the main launch piece. This prevents the lids and springs from dislodging during launches.

---

## Important Installation Notes

### Wiring and Electronics
- **DC-DC Converter and Wiring**: Ensure these components are installed before attaching the ICBM to simplify wiring.
- **Servo Motors and Controller Board**: Install these components after the ICBM is fully secured, to avoid wiring complications.

### Servo Installation
- Servos should be installed into the servo housing on the ICBM in an anti-clockwise direction. This helps prevent potential jamming and lockups during the ping-pong ball firing sequence.

---

## Spring Specifications
- **Maximum Compression**: 3.75 cm
- **Spring Constant (k)**: ~18.77 N/m

---

## Launch Calculations
- **Mass of Ping Pong Ball**: 2.7 g
- **Required Launch Energy**: ~0.0332 J
- **Required Launch Height**: 50-150 cm
- **Calculated Launch Height**: 152 cm

---

## Mechanical and Assembly Recommendations
- Ensure all locking pins and glued components have fully set before testing the launch mechanism.
- Double-check servo alignments and wiring connections to ensure smooth operation.
- Conduct initial launch tests at low servo power settings to verify mechanical stability and integrity.

---

## Design Reasoning
- Use of spring loaded launcher makes it easy to design and iterate
- Using servos to hold down the launch pins is relatively simple to implement compared to other launch mechanisms such as flywheels
- Reload mechanism was considered but it required too much power and complexity to reset the spring and servo
- Using single use launch mechanism allowed us to design a concentric launch platform which has the added benefit of not affecting CG as much
- Design is easy to replicate and print as well due to its simplicity

---

## Additional Details
- Common issue face is the DC-DC converter wire disconnecting from the OpenCR board which required the disassembly of the entire TurtleBot
- Ensure all wires are secured down

---

For further support or inquiries, please contact CDE2310 Group 3 AY24/25 or refer to the provided project documentation.
```

