#!/usr/bin/env python3

from sbp.client.drivers.network_drivers import TCPDriver  # ethernet connection tool
from sbp.client import Handler, Framer                    # message reader tools
from sbp.imu import MsgImuRaw                             # IMU message blueprints
import time       
import math         
import json
import numpy as np                    

# ── STEP 2: define where sensor is ────────────────────
HOST = '195.37.48.233'
PORT = 55555
RAW_COUNT = 4096            # Accelerometer sensitivity factor
GYRO_SENSITIVITY = 131.2    # LSB/°/s — matches datasheet language
G_METERS_PER_SEC2 = 9.8065  # Standard gravity

# ── STEP 3: connect safely ────────────────────────────
try:                                      
    with TCPDriver(HOST, PORT) as driver: 
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("Initiating connection and collecting calibration samples...")

            # Correctly initialize separate empty lists
            x_acc, y_acc, z_acc = [], [], []
            x_gyr, y_gyr, z_gyr = [], [], []
            samples_needed = 1000
            count = 0

            # Stream incoming data until we have enough samples
            for msg, metadata in handler:
                if isinstance(msg, MsgImuRaw):
                    x_acc.append(msg.acc_x)
                    y_acc.append(msg.acc_y)
                    z_acc.append(msg.acc_z)
                    x_gyr.append(msg.gyr_x)
                    y_gyr.append(msg.gyr_y)
                    z_gyr.append(msg.gyr_z)
                    
                    count += 1
                    if count >= samples_needed:
                        break

            # Convert to numpy arrays for calculation)
            x_a, y_a, z_a = np.array(x_acc), np.array(y_acc), np.array(z_acc)
            x_g, y_g, z_g = np.array(x_gyr), np.array(y_gyr), np.array(z_gyr)

            # Calculate means (biases in raw counts)
            bias_x_a, bias_y_a, bias_z_a = np.mean(x_a), np.mean(y_a), np.mean(z_a)
            bias_x_g, bias_y_g, bias_z_g = np.mean(x_g), np.mean(y_g), np.mean(z_g)

            # Calculate deadzones (max peak-to-peak noise variation * 1.1)
            deadzone_x_a = np.max(np.abs(x_a - bias_x_a)) 
            deadzone_y_a = np.max(np.abs(y_a - bias_y_a)) 
            deadzone_z_a = np.max(np.abs(z_a - bias_z_a)) 
            
            deadzone_x_g = np.max(np.abs(x_g - bias_x_g)) 
            deadzone_y_g = np.max(np.abs(y_g - bias_y_g)) 
            deadzone_z_g = np.max(np.abs(z_g - bias_z_g)) 

            # Convert raw biases into physical units (m/s² and degrees/s)
            x_bias = (bias_x_a / RAW_COUNT) * G_METERS_PER_SEC2
            y_bias = (bias_y_a / RAW_COUNT) * G_METERS_PER_SEC2
            z_bias = (bias_z_a / RAW_COUNT) * G_METERS_PER_SEC2
            
            x_g_bias = bias_x_g / GYRO_SENSITIVITY
            y_g_bias = bias_y_g / GYRO_SENSITIVITY
            z_g_bias = bias_z_g / GYRO_SENSITIVITY

            # Save the calibration profiles to JSON
            calibration_data = {
                "x_bias": x_bias, "y_bias": y_bias, "z_bias": z_bias,
                "x_g_bias": x_g_bias, "y_g_bias": y_g_bias, "z_g_bias": z_g_bias,
                "deadzone_x_a": deadzone_x_a / RAW_COUNT * G_METERS_PER_SEC2,
                "deadzone_y_a": deadzone_y_a / RAW_COUNT * G_METERS_PER_SEC2,
                "deadzone_z_a": deadzone_z_a / RAW_COUNT * G_METERS_PER_SEC2,
                "deadzone_x_g": deadzone_x_g / GYRO_SENSITIVITY,
                "deadzone_y_g": deadzone_y_g / GYRO_SENSITIVITY,
                "deadzone_z_g": deadzone_z_g / GYRO_SENSITIVITY
            }

            with open("imu_calibration.json", "w") as f:
                json.dump(calibration_data, f, indent=4)
            
            print("Calibration complete. Values saved to imu_calibration.json")

except KeyboardInterrupt:
    print("\nStopped by user.")