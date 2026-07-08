#!/usr/bin/env python3

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.imu import MsgImuRaw
from time import time
from collections import deque
import math
import json
import numpy as np 


HOST = '195.37.48.233'
PORT = 55555
raw_count = 4096
GYRO_SENSITIVITY = 131.2
g = 9.8065
count = 0

previous_time = None

velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

distance_x = 0.0
distance_y = 0.0
distance_z = 0.0

speed = 0.0
distance = 0.0
alpha_ = 0.386

filter_acc_x = None
filter_acc_y = None
filter_acc_z = None
filter_gyr_x = None
filter_gyr_y = None
filter_gyr_z = None

window_len=10
window_x_acc=deque(maxlen=window_len)
window_y_acc=deque(maxlen=window_len)
window_z_acc=deque(maxlen=window_len)

var_thr_acc_x=0.01
var_thr_acc_y=0.01
var_thr_acc_z=0.01

try:
    with TCPDriver(HOST, PORT) as driver:
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("initate connection")

            with open("imu_calibration.json", "r") as f:
                calib = json.load(f)
        
            for msg, metadata in handler:
                if isinstance(msg, MsgImuRaw):  
                    # if count >= 10:
                    #     break

                    raw_x_acc = ((msg.acc_x / raw_count)*g - calib["x_bias"]) 
                    raw_y_acc = ((msg.acc_y / raw_count)*g- calib["y_bias"]) 
                    raw_z_acc = ((msg.acc_z / raw_count)*g - calib["z_bias"]) 


                    # raw_x_gyr = msg.gyr_x - calib["x_g_bias"]
                    # raw_y_gyr = msg.gyr_y - calib["y_g_bias"]
                    # raw_z_gyr = msg.gyr_z - calib["z_g_bias"]

                    current_time = time()

                    if previous_time is None:
                        previous_time = current_time
                        continue

                    dt = current_time - previous_time
                    previous_time = current_time
                    # low pass filter for linear acceleartion 
                    if filter_acc_x is None:
                        filter_acc_x = raw_x_acc
                        filter_acc_y = raw_y_acc
                        filter_acc_z = raw_z_acc
                    else:
                        filter_acc_x = alpha_ * raw_x_acc + (1 - alpha_) * filter_acc_x
                        filter_acc_y = alpha_ * raw_y_acc + (1 - alpha_) * filter_acc_y
                        filter_acc_z = alpha_ * raw_z_acc + (1 - alpha_) * filter_acc_z
                    #low pass filter for the gyroscope 
                    # if filter_gyr_x is None:
                    #     filter_gyr_x = raw_x_gyr
                    #     filter_gyr_y = raw_y_gyr
                    #     filter_gyr_z = raw_z_gyr
                    # else:
                    #     filter_gyr_x = alpha_ * raw_x_gyr + (1 - alpha_) * filter_gyr_x
                    #     filter_gyr_y = alpha_ * raw_y_gyr + (1 - alpha_) * filter_gyr_y
                    #     filter_gyr_z = alpha_ * raw_z_gyr + (1 - alpha_) * filter_gyr_z
                    x = filter_acc_x
                    y = filter_acc_y
                    z = filter_acc_z
                    
                    window_x_acc.append(filter_acc_x)
                    window_y_acc.append(filter_acc_y)
                    window_z_acc.append(filter_acc_z)

                    if len(window_x_acc) == window_len:
                        mean_abs_x=np.mean(np.abs(window_x_acc))
                        var_x_acc = np.var(window_x_acc)
                        mean_abs_y=np.mean(np.abs(window_y_acc))
                        var_y_acc = np.var(window_y_acc)
                        mean_abs_z=np.mean(np.abs(window_z_acc))
                        var_z_acc = np.var(window_z_acc)

                        x_stationay= (var_x_acc < var_thr_acc_x) and (mean_abs_x < 1.1 * calib["deadzone_x_a"])
                        y_stationay= (var_y_acc < var_thr_acc_y) and (mean_abs_y < 1.1 * calib["deadzone_y_a"])
                        z_stationay= (var_z_acc < var_thr_acc_z) and (mean_abs_z < 1.1 * calib["deadzone_z_a"])

                        if x_stationay and y_stationay and z_stationay:
                            x, y, z = 0, 0, 0
                            velocity_x,velocity_y,velocity_z =0, 0, 0 
                        else:
                            x,y,z=filter_acc_x,filter_acc_y,filter_acc_z

                    else:
                        x, y, z = 0, 0, 0
                        velocity_x,velocity_y,velocity_z =0, 0, 0 
                            # xgyr = filter_gyr_x / GYRO_SENSITIVITY
                            # ygyr = filter_gyr_y / GYRO_SENSITIVITY
                            # zgyr = filter_gyr_z / GYRO_SENSITIVITY

                    velocity_x += x * dt
                    velocity_y += y * dt
                    velocity_z += z * dt

                    distance_x += velocity_x * dt
                    distance_y += velocity_y * dt
                    distance_z += velocity_z * dt

                    speed = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
                    distance = math.sqrt(distance_x**2 + distance_y**2 + distance_z**2)
                    
                    print("calibrated_acceleration",raw_x_acc,raw_y_acc,raw_z_acc)
                    print("low pass filter value:", filter_acc_x, filter_acc_y, filter_acc_z)
                    print("acc:", x, y, z)
                    # print("gyr", xgyr, ygyr, zgyr)
                    print("vel:", velocity_x, velocity_y, velocity_z)
                    print("dist:", distance_x, distance_y, distance_z)
                    print("speed:", speed, "distance:", distance)

except KeyboardInterrupt:
    print("Stopped by user.")