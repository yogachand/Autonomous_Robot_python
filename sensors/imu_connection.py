#!/usr/bin/env python3
from sbp.client.drivers.network_drivers import TCPDriver  # ethernet connection tool
from sbp.client import Handler, Framer                    # message reader tools
from sbp.imu import MsgImuRaw                             # IMU message blueprints
import math
import time
import json
import csv
import numpy as np 

HOST = '195.37.48.233'
PORT = 55555
RAW_COUNT = 4096            # Accelerometer sensitivity factor
GYRO_SENSITIVITY = 131.2    # LSB/°/s — matches datasheet language
G_METERS_PER_SEC2 = 9.8065

#connecting the IMU sensor

def connect_pikisi(HOST,PORT):
    driver = TCPDriver(HOST,PORT)
    handler= Handler(Framer(driver.read,driver.write))
    return driver,handler

# collecting the IMU data 

def collect_imu_data(handler):
    x_acc, y_acc, z_acc = [], [], []
    x_gyr, y_gyr, z_gyr = [], [], []
    samples_needed = 500
    count = 0 
    for msg, metadata in handler:
        if isinstance(msg, MsgImuRaw):
            x_acc.append((msg.acc_x / RAW_COUNT) * G_METERS_PER_SEC2)
            y_acc.append((msg.acc_y / RAW_COUNT) * G_METERS_PER_SEC2)
            z_acc.append((msg.acc_z / RAW_COUNT) * G_METERS_PER_SEC2)
            
            x_gyr.append(msg.gyr_x / GYRO_SENSITIVITY)
            y_gyr.append(msg.gyr_y / GYRO_SENSITIVITY)
            z_gyr.append(msg.gyr_z / GYRO_SENSITIVITY)

            count += 1 
            if count >= samples_needed:
                break
    return x_acc,y_acc,z_acc,x_gyr,y_gyr,z_gyr

def converting_data_deadzones(samples):
    x_a,y_a,z_a = np.array(x_acc), np.array(y_acc), np.array(z_acc)
    x_g, y_g, z_g = np.array(x_gyr), np.array(y_gyr), np.array(z_gyr)

    bias_x_a, bias_y_a, bias_z_a = np.mean(x_a), np.mean(y_a), np.mean(z_a)
    bias_x_g, bias_y_g, bias_z_g = np.mean(x_g), np.mean(y_g), np.mean(z_g)
    x_var_acc,y_var_acc,z_var_acc = np.var(x_a), np.var(y_a), np.var(z_a)
    x_var_gyo,y_var_gyo,z_var_gyo = np.var(x_g), np.var(y_g), np.var(z_g)

    deadzone_x_a = np.max(np.abs(x_a - bias_x_a)) 
    deadzone_y_a = np.max(np.abs(y_a - bias_y_a)) 
    deadzone_z_a = np.max(np.abs(z_a - bias_z_a)) 
    
    deadzone_x_g = np.max(np.abs(x_g - bias_x_g)) 
    deadzone_y_g = np.max(np.abs(y_g - bias_y_g)) 
    deadzone_z_g = np.max(np.abs(z_g - bias_z_g)) 


def save_data():





def run_calibration(HOST,PORT):
    driver,handler =connect_pikisi(HOST,PORT)
    try:
        with driver,handler:
            samples=collect_imu_data(handler)

            converting_data_deadzones(samples)
            save_data()
    except KeyboardInterrupt:
        print("stopped by user")

    

if __name__ == "__main__":
    run_calibration(HOST, PORT)








