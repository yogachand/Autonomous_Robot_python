#!/usr/bin/env python3

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.imu import MsgImuRaw
from time import time
from collections import deque
import math
import csv                     
import numpy as np 
import json

HOST = '195.37.48.233'
PORT = 55555
raw_count = 4096
g = 9.8065

previous_time = None

velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0
distance_x, distance_y, distance_z = 0.0, 0.0, 0.0

speed = 0.0
distance = 0.0

# Slightly higher alpha means faster tracking response for quick short movements
alpha_ = 0.40

filter_acc_x, filter_acc_y, filter_acc_z = None, None, None

# A shorter window (8-10 samples) reacts faster to sudden stops
window_len = 2
window_x_acc = deque(maxlen=window_len)
window_y_acc = deque(maxlen=window_len)
window_z_acc = deque(maxlen=window_len)

try:
    with TCPDriver(HOST, PORT) as driver:
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("Initiating connection...")

            with open("imu_calibration.json", "r") as f:
                calib = json.load(f)

            with open("imu_bias_profile.csv", "r") as f:
                reader = csv.reader(f)
                next(reader) 
                for row in reader:
                    if not row or row[0] == "": 
                        break
                    calib[row[0]] = float(row[1])
                    calib[row[2]] = float(row[3])
            
            # Using a tight multiplier for stopping detection
            THRESHOLD_MULTIPLIER = 2.0
            var_thr_acc_x = calib["x_accl_variance"] * THRESHOLD_MULTIPLIER
            var_thr_acc_y = calib["y_accl_variance"] * THRESHOLD_MULTIPLIER
            var_thr_acc_z = calib["z_accl_variance"] * THRESHOLD_MULTIPLIER

            filename = "acceleration_without_gyroscope"
            with open(filename, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                # Header row
                writer.writerow([
                    "time", "acc_x", "acc_y", "acc_z", 
                    "vel_x", "vel_y", "vel_z", "speed", "distance_cm"
                ])
            
                print("Ready! Move the IMU linearly now...")
            
                for msg, metadata in handler:
                    if isinstance(msg, MsgImuRaw):  

                        # Apply calibration bias offsets
                        raw_x_acc = ((msg.acc_x / raw_count) * g) - calib["x_accl_bias"] 
                        raw_y_acc = ((msg.acc_y / raw_count) * g) - calib["y_accl_bias"] 
                        raw_z_acc = ((msg.acc_z / raw_count) * g) - calib["z_accl_bias"] 

                        current_time = time()
                        if previous_time is None:
                            previous_time = current_time
                            continue

                        dt = current_time - previous_time
                        previous_time = current_time
                        
                        # Low pass filter 
                        if filter_acc_x is None:
                            filter_acc_x, filter_acc_y, filter_acc_z = raw_x_acc, raw_y_acc, raw_z_acc
                        else:
                            filter_acc_x = alpha_ * raw_x_acc + (1 - alpha_) * filter_acc_x
                            filter_acc_y = alpha_ * raw_y_acc + (1 - alpha_) * filter_acc_y
                            filter_acc_z = alpha_ * raw_z_acc + (1 - alpha_) * filter_acc_z
                        
                        window_x_acc.append(filter_acc_x)
                        window_y_acc.append(filter_acc_y)
                        window_z_acc.append(filter_acc_z)

                        if len(window_x_acc) == window_len:
                            # --- FIX 1: Removed np.abs(). We need the true directional mean. ---
                            mean_x = np.mean(window_x_acc)
                            var_x_acc = np.var(window_x_acc, ddof=1)
                            mean_y = np.mean(window_y_acc)
                            var_y_acc = np.var(window_y_acc, ddof=1)
                            mean_z = np.mean(window_z_acc)
                            var_z_acc = np.var(window_z_acc, ddof=1)

                            # --- FIX 2: Check absolute thresholds on the *mean*, not during integration ---
                            x_stationary = (var_x_acc < var_thr_acc_x) and ((mean_x) < 2 * calib["deadzone_x_a"])
                            y_stationary = (var_y_acc < var_thr_acc_y) and ((mean_y) < 2 * calib["deadzone_y_a"])
                            z_stationary = (var_z_acc < var_thr_acc_z) and ((mean_z) < 2 * calib["deadzone_z_a"])

                            # # --- FIX 3: Do not hard-wipe velocity instantly if variance drops ---
                            # # Let the real acceleration handle the math. Only force 0 if truly stationary.
                            # x = 0.0 if x_stationary else filter_acc_x
                            # y = 0.0 if y_stationary else filter_acc_y
                            # z = 0.0 if z_stationary else filter_acc_z

                            # if x_stationary: velocity_x = 0.0
                            # if y_stationary: velocity_y = 0.0
                            # if z_stationary: velocity_z = 0.0


                            if x_stationary and y_stationary and z_stationary:
                                x,y,z = 0, 0, 0
                                velocity_x,velocity_y,velocity_z = 0, 0, 0
                            else:
                                x,y,z=filter_acc_x,filter_acc_y,filter_acc_z
                        else:
                            x, y, z = 0.0, 0.0, 0.0
                            velocity_x,velocity_y,velocity_z=0,0,0

                        # --- FIX 4: Removed `if abs(x) > 0.01` restriction ---
                        # Allowing true positive and negative integration down to 0.0

                        # Numerical integration step
                        if abs(x) > 0.02:  # Sightly higher threshold for active integration
                            velocity_x += x * dt
                        else:
                            velocity_x *= 0.85  # Digital friction: quickly drain lingering velocity to 0

                        if abs(y) > 0.02:
                            velocity_y += y * dt
                        else:
                            velocity_y *= 0.85

                        if abs(z) > 0.02:
                            velocity_z += z * dt
                        else:
                            velocity_z *= 0.85

                        # Software Deadband applied safely to clean up lingering micro-drift
                        if abs(velocity_x) < 0.003: velocity_x = 0.0
                        if abs(velocity_y) < 0.003: velocity_y = 0.0
                        if abs(velocity_z) < 0.003: velocity_z = 0.0

                        # Calculate instantaneous speed
                        speed = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
                        
                        # Accumulate real total distance traveled along the path over time
                        distance += speed * dt
                        distance_cm = distance * 100
                        
                        status = "STATIONARY" if (velocity_x == 0 and velocity_y == 0 and velocity_z == 0) else "MOVING"
                        print("acceleration", x, y, z)
                        
                        print("velocity",velocity_x,velocity_y,velocity_z)
                        
                        print("speed", speed)
                        
                        # print("variance", var_thr_acc_x, var_thr_acc_y, var_thr_acc_z)
                        print("distance", distance_cm)
                        print("------------------------------------------------")
                        writer.writerow([
                                current_time, x, y, z,
                                velocity_x, velocity_y, velocity_z, speed, distance_cm
                            ])

except KeyboardInterrupt:                   
    print("\nStopped by user.")