import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
active = [0,1,2,4,5,6,8,9,10]

lock = 0
fire = 180

count = 0
tube_no = 0

while count < 3:
    input("Enter to fire when ready")
    kit.servo[active[tube_no]].angle = fire
    print("Tube " + str(tube_no) + " (Servo Channel " + str(active[tube_no]) + ") Firing" )
    time.sleep(2)
    tube_no += 1
    kit.servo[active[tube_no]].angle = fire
    print("Tube " + str(tube_no) + " (Servo Channel " + str(active[tube_no]) + ") Firing" )
    time.sleep(4)
    tube_no += 1
    kit.servo[active[tube_no]].angle = fire
    print("Tube " + str(tube_no) + " (Servo Channel " + str(active[tube_no]) + ") Firing" )
    tube_no += 1
    count += 1
for pin in active:
    kit.servo[pin].angle = lock
print("All Tubes fired")
