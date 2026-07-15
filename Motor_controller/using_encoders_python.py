#!/usr/bin/env python3

from roboclaw_3 import Roboclaw
from time import sleep

roboclaw = Roboclaw("/dev/ttyS0", 38400)
roboclaw.Open()


# Read encoder
motor_1_count = roboclaw.ReadEncM1(0x80)
print ("Original:")
print (motor_1_count)
motor_2_count = roboclaw.ReadEncM2(0X80)
print ("Original:")
print (motor_2_count)

# sleep(2)

# # Set encoder and then read and print to test operation
# roboclaw.SetEncM1(0x80, 10000)
# motor_1_count = roboclaw.ReadEncM1(0x80)
# print ("After setting count:")
# print (motor_1_count)

# sleep(2)

# # Reset encoders and read and print value to test operation
# roboclaw.ResetEncoders(0x80)
# motor_1_count = roboclaw.ReadEncM1(0x80)
# print ("After resetting:")
# print (motor_1_count)

# sleep(2)

# # Position motor, these values may have to be changed to suit the motor/encoder combination in use

# roboclaw.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
