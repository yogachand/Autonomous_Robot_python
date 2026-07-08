#!/usr/bin/env python3

from sbp.client.drivers.network_drivers import TCPDriver  
from sbp.client import Handler, Framer                    
from sbp.imu import MsgImuRaw                             
import csv                                                
import numpy as np  # Used to calculate the average values                  

HOST = '195.37.48.233'
PORT = 55555
RAW_COUNT = 4096            # Accelerometer sensitivity factor
GYRO_SENSITIVITY = 131.2    # LSB/°/s — matches datasheet language
G_METERS_PER_SEC2 = 9.8065  # Standard gravity

try:                                      
    with TCPDriver(HOST, PORT) as driver: 
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("Keep sensor perfectly still. Collecting calibration data...")

            x_acc, y_acc, z_acc = [], [], []
            x_gyr, y_gyr, z_gyr = [], [], []

            samples_needed = 500
            count = 0

            for msg, metadata in handler:
                if isinstance(msg, MsgImuRaw):
                    # FIX: Correctly mapping acc_x, acc_y, and acc_z
                    x_acc.append((msg.acc_x / RAW_COUNT) * G_METERS_PER_SEC2)
                    y_acc.append((msg.acc_y / RAW_COUNT) * G_METERS_PER_SEC2)
                    z_acc.append((msg.acc_z / RAW_COUNT) * G_METERS_PER_SEC2)
                    
                    x_gyr.append(msg.gyr_x / GYRO_SENSITIVITY)
                    y_gyr.append(msg.gyr_y / GYRO_SENSITIVITY)
                    z_gyr.append(msg.gyr_z / GYRO_SENSITIVITY)
                    
                    count += 1
                    if count >= samples_needed:
                        break

            # ── STEP 4: Calculate the average Bias Offset and Variances ──
            analysis_data = [
                ["x_accl_bias", np.mean(x_acc), "x_accl_variance", np.var(x_acc, ddof=1)],
                ["y_accl_bias", np.mean(y_acc), "y_accl_variance", np.var(y_acc, ddof=1)],
                ["z_accl_bias", np.mean(z_acc), "z_accl_variance", np.var(z_acc, ddof=1)],
                ["x_gyro_bias", np.mean(x_gyr), "x_gyro_variance", np.var(x_gyr, ddof=1)],
                ["y_gyro_bias", np.mean(y_gyr), "y_gyro_variance", np.var(y_gyr, ddof=1)],
                ["z_gyro_bias", np.mean(z_gyr), "z_gyro_variance", np.var(z_gyr, ddof=1)],
            ]
            
            # Calculate the global averages your AWGF-ZVD algorithm requests
            global_sigma_a_sq = np.mean([analysis_data[0][3], analysis_data[1][3], analysis_data[2][3]])
            global_sigma_w_sq = np.mean([analysis_data[3][3], analysis_data[4][3], analysis_data[5][3]])
            
            # ── STEP 5: Save to CSV ────────────────────────
            filename = "imu_bias_profile.csv"
            
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                
                # Write the individual axis data
                writer.writerow(["Parameter_Bias", "Bias_Value", "Parameter_Variance", "Variance_Value"])
                for row in analysis_data:
                    writer.writerow(row)
                
                # Write the global algorithm baselines at the bottom
                writer.writerow([])  # Blank spacer line
                writer.writerow(["Algorithm Global Parameter", "Value"])
                writer.writerow(["Acceleration Noise Variance (σa²)", global_sigma_a_sq])
                writer.writerow(["Angular Rate Noise Variance (σω²)", global_sigma_w_sq])
            
            print(f"Calibration complete! Calculated offsets and noise baselines saved to {filename}")

except KeyboardInterrupt:
    print("\nStopped by user.")