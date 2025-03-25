import time
from adafruit_servokit import ServoKit

idle = 0
fire = 180

kit = ServoKit(channels=16)
active = [0,1,2,4,5,6,8,9,10]
try:
    to_reload = input("Enter tube to reload (-1 for all)")
    tube_no = int(to_reload)
    # for all tube reloads
    if tube_no < 0:
        for i in range(9): 
            kit.servo[active[i]].angle = fire
        for i in range(9):
            next = input("Press Enter When Tube " + str(i) + " (Servo Channel " + str(active[i]) + ") Ready")
            kit.servo[active[i]].angle = idle
            print("Tube " + str(i) + " (Servo Channel " + str(active[i]) + ") Ready" )
        print("All Servo Ready")
    else: 
        kit.servo[active[tube_no]].angle = fire
        next = input("Press Enter When Tube " + str(tube_no) + " (Servo Channel " + str(active[tube_no]) + ") Ready")
        kit.servo[active[tube_no]].angle = idle
        print("Tube " + str(tube_no) + " (Servo Channel " + str(active[tube_no]) + ") Ready" )
except Exception as e:
    while(True):
        print(1)




