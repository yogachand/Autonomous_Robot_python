#!/usr/bin/env python3

from roboclaw_3 import Roboclaw
from time import sleep 
try:
    print("Initializing RoboClaw...")
    roboclaw = Roboclaw("/dev/ttyACM0", 38400)
    
    print("Opening connection...")
    roboclaw.Open()
    print("Sending command to motor...")
    roboclaw.ForwardM1(0x80, 20)
    roboclaw.ForwardM2(0x80, 20)
    sleep(1)
    motor_1_count = roboclaw.ReadEncM1(0x80)
    motor_2_count = roboclaw.ReadEncM2(0x80)
    print (motor_1_count,motor_2_count)
    roboclaw.BackwardM1(0x80, 0)
    roboclaw.BackwardM2(0x80, 0)
 


    # sleep(1)

except FileNotFoundError:
    print("❌ Serial port not found (expected - hardware not connected)")