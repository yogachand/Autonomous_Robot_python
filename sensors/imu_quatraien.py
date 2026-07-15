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

from ahrs.filters import Madgwick
from ahrs import Quaternion

HOST = '195.37.48.233'
PORT = 55555
raw_count = 4096
g = 9.8065
GYRO_SENSITIVITY = 131.2

previous_time = None

velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0
distance_x, distance_y, distance_z = 0.0, 0.0, 0.0

speed = 0.0
distance = 0.0

# --- Quaternion orientation state (replaces roll/pitch Euler state) ---
# Identity quaternion: (w, x, y, z) = (1, 0, 0, 0) -> no rotation yet
q = np.array([1.0, 0.0, 0.0, 0.0])
madgwick = Madgwick()   # you can tune madgwick.gain if convergence feels too slow/fast

filter_acc_x, filter_acc_y, filter_acc_z = None, None, None
filter_gyr_x, filter_gyr_y, filter_gyr_z = None, None, None

# Slightly higher alpha means faster tracking response for quick short movements
alpha_ = 0.40

# A shorter window (8-10 samples) reacts faster to sudden stops
window_len = 10
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

                        raw_x_gyr = math.radians((msg.gyr_x / GYRO_SENSITIVITY) - calib["x_g_bias"])
                        raw_y_gyr = math.radians((msg.gyr_y / GYRO_SENSITIVITY) - calib["y_g_bias"])
                        raw_z_gyr = math.radians((msg.gyr_z / GYRO_SENSITIVITY) - calib["z_g_bias"])

                        current_time = time()
                        imu_time = (msg.tow + msg.tow_f / 256.0) / 1000.0  # -> seconds
                        if previous_time is None:
                            previous_time = imu_time
                            continue

                        dt = imu_time - previous_time
                        previous_time = imu_time
                        if dt <= 0 or dt > 1.0:
                            continue

                        # Low pass filter (unchanged from your original - EMA weights sum to 1.0)
                        if filter_acc_x is None:
                            filter_acc_x, filter_acc_y, filter_acc_z = raw_x_acc, raw_y_acc, raw_z_acc
                        else:
                            filter_acc_x = alpha_ * raw_x_acc + (1 - alpha_) * filter_acc_x
                            filter_acc_y = alpha_ * raw_y_acc + (1 - alpha_) * filter_acc_y
                            filter_acc_z = alpha_ * raw_z_acc + (1 - alpha_) * filter_acc_z

                        if filter_gyr_x is None:
                            filter_gyr_x, filter_gyr_y, filter_gyr_z = raw_x_gyr, raw_y_gyr, raw_z_gyr
                        else:
                            filter_gyr_x = alpha_ * raw_x_gyr + (1 - alpha_) * filter_gyr_x
                            filter_gyr_y = alpha_ * raw_y_gyr + (1 - alpha_) * filter_gyr_y
                            filter_gyr_z = alpha_ * raw_z_gyr + (1 - alpha_) * filter_gyr_z

                        # --- Quaternion orientation update (replaces the Euler complementary filter) ---
                        # Madgwick fuses gyro (short-term) + accel (long-term gravity reference) into
                        # one quaternion update, same role as your old roll/pitch complementary filter,
                        # but also tracks yaw and avoids gimbal lock.
                        gyr_vec = np.array([raw_x_gyr, raw_y_gyr, raw_z_gyr])
                        acc_vec = np.array([raw_x_acc, raw_y_acc, raw_z_acc])
                        # Madgwick's accel correction assumes acc_vec is dominated by gravity (~1g).
                        # It still works reasonably during mild motion, same assumption your old
                        # accel_roll/accel_pitch calculation was already making.
                        q = madgwick.updateIMU(q, gyr=gyr_vec, acc=acc_vec, dt=dt)

                        # --- Rotate the filtered acceleration vector into the navigation (world) frame ---
                        # This is the q * A * q_conjugate operation, done via the equivalent rotation matrix.
                        quat = Quaternion(q)
                        R = quat.to_DCM()  # 3x3 rotation matrix, body frame -> navigation frame
                        acc_body = np.array([filter_acc_x, filter_acc_y, filter_acc_z])
                        acc_nav = R @ acc_body   # rotated acceleration, still includes gravity

                        # Gravity in the navigation frame is a fixed known vector: (0, 0, -g) or (0,0,g)
                        # depending on your convention. Since your original code treated "g" as
                        # acting along the sensor's own up-axis at rest, we subtract it the same
                        # way here, just now the subtraction happens after proper full 3D rotation
                        # instead of only accounting for roll/pitch.
                        gravity_nav = np.array([0.0, 0.0, g])

                        acc_linear = acc_nav - gravity_nav

                        x, y, z = acc_linear[0], acc_linear[1], acc_linear[2]

                        window_x_acc.append(x)
                        window_y_acc.append(y)
                        window_z_acc.append(z)

                        if len(window_x_acc) == window_len:
                            mean_x = np.mean(window_x_acc)
                            var_x_acc = np.var(window_x_acc, ddof=1)
                            mean_y = np.mean(window_y_acc)
                            var_y_acc = np.var(window_y_acc, ddof=1)
                            mean_z = np.mean(window_z_acc)
                            var_z_acc = np.var(window_z_acc, ddof=1)

                            x_stationary = (var_x_acc < var_thr_acc_x) and (abs(mean_x) < 1.1 * calib["deadzone_x_a"])
                            y_stationary = (var_y_acc < var_thr_acc_y) and (abs(mean_y) < 1.1 * calib["deadzone_y_a"])
                            z_stationary = (var_z_acc < var_thr_acc_z) and (abs(mean_z) < 1.1 * calib["deadzone_z_a"])

                            if x_stationary and y_stationary and z_stationary:
                                x, y, z = 0.0, 0.0, 0.0
                                velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0
                        else:
                            x, y, z = 0.0, 0.0, 0.0
                            velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0

                        velocity_x += x * dt
                        velocity_y += y * dt
                        velocity_z += z * dt

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

                        roll, pitch, yaw = quat.to_angles()  # radians, for comparison/debug only

                        print("quaternion", q)
                        print("roll_pitch_yaw", roll, pitch, yaw)
                        print("filter_acceleration", filter_acc_x, filter_acc_y, filter_acc_z)
                        print("acceleration_nav_frame", x, y, z)
                        print("velocity", velocity_x, velocity_y, velocity_z)
                        print("speed", speed)
                        print("distance", distance_cm)
                        print("------------------------------------------------")
                        writer.writerow([
                                current_time, x, y, z,
                                velocity_x, velocity_y, velocity_z, speed, distance_cm
                            ])

except KeyboardInterrupt:
    print("\nStopped by user.")