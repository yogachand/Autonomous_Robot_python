#!/usr/bin/env python3
import numpy as np
dt = 0.01  # 100 Hz sampling rate (typical for BMI160)
total_time = 10.0  # 10 seconds of motion
time_steps = np.arange(0, total_time, dt)
N = len(time_steps)

# BMI160 Typicals: Noise Density & Accel/Gyro Offsets
accel_noise_density = 180e-6 * 9.81  # 180 ug/sqrt(Hz) -> m/s^2/sqrt(Hz)
gyro_noise_density = np.radians(0.008)  # 0.008 deg/s/sqrt(Hz) -> rad/s/sqrt(Hz)

# # Real physical bias present in the hardware
# true_accel_bias = np.array([0.05, -0.03, 0.02])  # m/s^2
# true_gyro_bias = np.radians(np.array([0.5, -0.3, 0.1]))  # rad/s

# Real physical bias present in the hardware
# ---------------------------------------------------------
# CONVERT YOUR REAL BMI160 BIAS DATA CORRECTLY
# ---------------------------------------------------------
# 1. Accelerometer: Convert Gs to m/s^2 (Multiply by 9.80665)
raw_x_accel_g = -0.029118408203125
raw_y_accel_g = -0.06372705078125
raw_z_accel_g = -0.97757275390625  # Contains Earth gravity!

# Strip out 1.0 G of gravity from the Z axis to isolate the actual sensor error
z_accel_error_g = raw_z_accel_g - (-1.0) 

# Final converted acceleration biases in m/s^2
true_accel_bias = np.array([
    raw_x_accel_g * 9.80665,
    raw_y_accel_g * 9.80665,
    z_accel_error_g * 9.80665
])

# 2. Gyroscope: Convert Degrees/sec to Radians/sec
raw_x_gyro_dps = -0.2657545731707317
raw_y_gyro_dps = -1.5317378048780488
raw_z_gyro_dps = 0.14501524390243903

true_gyro_bias = np.radians(np.array([
    raw_x_gyro_dps,
    raw_y_gyro_dps,
    raw_z_gyro_dps
]))

true_accel = np.zeros((N, 3))
true_gyro = np.zeros((N, 3))

print("Corrected Accel Biases (m/s^2):", true_accel_bias)
print("Corrected Gyro Biases (rad/s): ", true_gyro_bias)