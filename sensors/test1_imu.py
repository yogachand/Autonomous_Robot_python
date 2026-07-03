#!/usr/bin/env python3

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.imu import MsgImuRaw
from time import time
import math
import json
# import csv

HOST = '195.37.48.233'
PORT = 55555
raw_count = 4096
GYRO_SENSITIVITY = 131.2
g = 9.8065

start_time = time()
previous_time = None

velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

distance_x = 0.0
distance_y = 0.0
distance_z = 0.0

x = y = z = 0.0
speed = 0.0
distance = 0.0
count = 0

try:
    with TCPDriver(HOST, PORT) as driver:
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("initate connection")

            with open("imu_calibration.json", "r") as f:
                calib = json.load(f)

            # with open("imu_log.csv", "w", newline="") as csvfile:
            #     writer = csv.writer(csvfile)
            #     writer.writerow([
            #         # "timestamp", "dt",
            #         "acc_x_raw", "acc_y_raw", "acc_z_raw",
            #         "acc_x", "acc_y", "acc_z",
            #         "vel_x", "vel_y", "vel_z",
            #         "dist_x", "dist_y", "dist_z",
            #         "speed", "distance"
            #     ])

            for msg, metadata in handler:
                if not isinstance(msg, MsgImuRaw):
                    continue

                # count += 1
                # if count >= 100:
                #     break

                current_time = time()

                if previous_time is None:
                    previous_time = current_time
                    continue

                dt = current_time - previous_time
                previous_time = current_time

                x_raw = msg.acc_x
                y_raw = msg.acc_y
                z_raw = msg.acc_z
                print(msg.acc_x)
                x = ((x_raw / raw_count)*g - calib["x_bias"]) 
                y = ((y_raw / raw_count)*g- calib["y_bias"]) 
                z = ((z_raw / raw_count)*g - calib["z_bias"])
                if abs(x)< calib["deadzone_x_a"]:
                    x=0
                else:
                    if x>0:
                        x-=calib["deadzone_x_a"]
                    else:
                        x+=calib["deadzone_x_a"]
                if abs(y)< calib["deadzone_y_a"]:
                    y=0
                else:
                    if y>0:
                        y-=calib["deadzone_y_a"]
                    else:
                        y+=calib["deadzone_y_a"]
                if abs(z)< calib["deadzone_z_a"]:
                    z=0
                else:
                    if z>0:
                        z-=calib["deadzone_z_a"]
                    else:
                        z+=calib["deadzone_z_a"]
                
                velocity_x += x * dt
                velocity_y += y * dt
                velocity_z += z * dt

                distance_x += velocity_x * dt
                distance_y += velocity_y * dt
                distance_z += velocity_z * dt

                print(dt)

                speed = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
                distance = math.sqrt(distance_x**2 + distance_y**2 + distance_z**2)

                # writer.writerow([
                #     x_raw,    y_raw,    z_raw,
                #     x,        y,        z,
                #     velocity_x,    velocity_y,   velocity_z,
                #     distance_x,    distance_y,    distance_z,
                #     speed,     distance
                # ])
                # csvfile.flush()

                print("acc:", x, y, z)
                print("vel:", velocity_x, velocity_y, velocity_z)
                print("dist:", distance_x, distance_y, distance_z)
                print("speed:", speed, "distance:", distance)

except KeyboardInterrupt:
    print("Stopped by user.")