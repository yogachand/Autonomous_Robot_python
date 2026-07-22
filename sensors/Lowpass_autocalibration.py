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
g = 9.8
GYRO_SENSITIVITY = 131.2

previous_time = None

velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0
distance_x, distance_y, distance_z = 0.0, 0.0, 0.0

speed = 0.0
distance = 0.0

# roll/pitch must persist across loop iterations, not reset each cycle
roll, pitch = 0.0, 0.0

# alpha_ is for raw signal smoothing (low-pass on noisy accel/gyro samples).
# Higher alpha_ = more of each new noisy sample passes through = less smoothing.
alpha_ = 0.40

# alpha_cf is for the gyro/accel complementary filter fusing orientation.
# This is a DIFFERENT equation than alpha_ above: alpha_cf weights the
# gyro-integrated (smooth, drift-prone) angle, and (1 - alpha_cf) weights
# the raw accelerometer-derived angle (noisy, and corrupted by any real
# linear acceleration, not just gravity). So HIGHER alpha_cf means LESS of
# the noisy/corrupted accel angle leaks in per sample - opposite direction
# from alpha_. Standard complementary filters use ~0.95-0.99 here.
# (Was 0.40, which let real linear-acceleration events masquerade as tilt
# and made roll/pitch climb continuously during motion instead of staying
# near zero. Confirmed fixed by raising to 0.98 - roll/pitch now oscillate
# in a tight, bounded band during motion instead of drifting.)
alpha_cf = 0.98

accel_roll = 0
accel_pitch = 0
filter_acc_x, filter_acc_y, filter_acc_z = None, None, None
filter_gyr_x, filter_gyr_y, filter_gyr_z = None, None, None

# Widened from 10 -> 20: a variance ESTIMATE from only 10 samples is itself
# statistically noisy - even genuinely stationary Gaussian noise will randomly
# overshoot a tight threshold a meaningful fraction of the time just by
# sampling chance, not real motion. A bigger window makes the variance
# estimate much more stable at the cost of slightly slower reaction time.
window_len = 10
window_x_acc = deque(maxlen=window_len)
window_y_acc = deque(maxlen=window_len)
window_z_acc = deque(maxlen=window_len)

# --- DEBOUNCE STATE ---
# Requires several consecutive windows to agree before flipping stationary<->moving.
# Without this, a single noisy window can falsely flip to MOVING even at true
# rest, and integration of that window's tiny nonzero (x,y,z) into velocity is
# enough to make `distance` creep upward indefinitely even though nothing moved.
DEBOUNCE_COUNT = 3
moving_streak = 0
stationary_streak = 0
is_stationary_state = True  # start assuming stationary, since calibration required stillness

# --- AUTO-CALIBRATION SETTINGS ---
# Recalibrates bias + noise variance from the CURRENT session at startup,
# instead of trusting a bias file saved from a previous session. This fixes
# the bug where a stale x_accl_bias (from an earlier, colder, or slightly
# differently-mounted session) left a ~0.10-0.14 m/s^2 residual on the
# x-axis that the ZUPT deadzone check could never pass, so velocity/distance
# kept accumulating even at genuine rest.
CALIB_WARMUP_N = 200     # ~samples to average over; keep IMU still during this
# Widened from 2.0 -> 3.5: with the larger window above the variance estimate
# is more stable, but the multiplier is widened too so genuine sensor noise
# (which has some natural sample-to-sample spread) doesn't spuriously trip
# the "moving" state on its own.
THRESHOLD_MULTIPLIER = 5
DEADZONE_SIGMA = 5.0          # deadzone = this many std devs of the warm-up noise

calib_x_acc, calib_y_acc, calib_z_acc = [], [], []
calib_x_gyr, calib_y_gyr, calib_z_gyr = [], [], []
calib = {}
runtime_calibrated = False

try:
    with TCPDriver(HOST, PORT) as driver:
        with Handler(Framer(driver.read, driver.write)) as handler:
            print("Initiating connection...")
            print(f"Calibrating from live data - keep the IMU perfectly still "
                  f"for the next ~{CALIB_WARMUP_N} samples...")

            filename = "acceleration_without_gyroscope"
            with open(filename, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    "time", "acc_x", "acc_y", "acc_z",
                    "vel_x", "vel_y", "vel_z", "speed", "distance_cm"
                ])

                for msg, metadata in handler:
                    if isinstance(msg, MsgImuRaw):

                        # --- AUTO-CALIBRATION PHASE ---
                        # Collect raw (pre-bias) samples first; compute bias as their
                        # mean directly (not mean - g) because on the axis aligned with
                        # gravity the raw reading itself already sits near -g at rest,
                        # so the bias needs to absorb that full value to zero it out -
                        # matching how your existing calibration files were structured
                        # (e.g. z bias ~ -9.65, not ~0.15).
                        if not runtime_calibrated:
                            rx = (msg.acc_x / raw_count) * g
                            ry = (msg.acc_y / raw_count) * g
                            rz = (msg.acc_z / raw_count) * g
                            rgx = msg.gyr_x / GYRO_SENSITIVITY
                            rgy = msg.gyr_y / GYRO_SENSITIVITY
                            rgz = msg.gyr_z / GYRO_SENSITIVITY

                            calib_x_acc.append(rx)
                            calib_y_acc.append(ry)
                            calib_z_acc.append(rz)
                            calib_x_gyr.append(rgx)
                            calib_y_gyr.append(rgy)
                            calib_z_gyr.append(rgz)

                            if len(calib_x_acc) >= CALIB_WARMUP_N:
                                calib["x_accl_bias"] = float(np.mean(calib_x_acc))
                                calib["y_accl_bias"] = float(np.mean(calib_y_acc))
                                calib["z_accl_bias"] = float(np.mean(calib_z_acc))
                                calib["x_g_bias"] = float(np.mean(calib_x_gyr))
                                calib["y_g_bias"] = float(np.mean(calib_y_gyr))
                                calib["z_g_bias"] = float(np.mean(calib_z_gyr))

                                std_x = float(np.std(calib_x_acc, ddof=1))
                                std_y = float(np.std(calib_y_acc, ddof=1))
                                std_z = float(np.std(calib_z_acc, ddof=1))

                                calib["x_accl_variance"] = std_x ** 2
                                calib["y_accl_variance"] = std_y ** 2
                                calib["z_accl_variance"] = std_z ** 2

                                # Deadzone derived from THIS session's own noise floor,
                                # instead of a fixed number from a possibly stale file -
                                # so it always matches current temperature/mounting state.
                                calib["deadzone_x_a"] = DEADZONE_SIGMA * std_x
                                calib["deadzone_y_a"] = DEADZONE_SIGMA * std_y
                                calib["deadzone_z_a"] = DEADZONE_SIGMA * std_z

                                print("\n--- Runtime calibration complete ---")
                                print("bias  (x,y,z):", calib["x_accl_bias"],
                                      calib["y_accl_bias"], calib["z_accl_bias"])
                                print("gyro bias (x,y,z):", calib["x_g_bias"],
                                      calib["y_g_bias"], calib["z_g_bias"])
                                print("deadzone (x,y,z):", calib["deadzone_x_a"],
                                      calib["deadzone_y_a"], calib["deadzone_z_a"])
                                print("-------------------------------------\n")

                                # Save this session's calibration to its OWN file (not
                                # hardcoded/kept only in-memory in the script), so you can
                                # inspect it afterward or compare across runs/sessions.
                                # Timestamped so repeated runs don't overwrite each other.
                                calib_out_path = f"imu_calibration_runtime_{int(time())}.json"
                                with open(calib_out_path, "w") as calib_out_file:
                                    json.dump(calib, calib_out_file, indent=4)
                                print(f"Saved this session's calibration to {calib_out_path}\n")

                                var_thr_acc_x = calib["x_accl_variance"] * THRESHOLD_MULTIPLIER
                                var_thr_acc_y = calib["y_accl_variance"] * THRESHOLD_MULTIPLIER
                                var_thr_acc_z = calib["z_accl_variance"] * THRESHOLD_MULTIPLIER

                                runtime_calibrated = True
                                print("Ready! Move the IMU linearly now...")
                            continue  # don't run the main pipeline until calibration is done

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
                        print("raw_acceleration_after_biasing", raw_x_acc, raw_y_acc, raw_z_acc)
                        if dt <= 0 or dt > 1.0:
                            continue

                        # Low pass filter
                        if filter_acc_x is None:
                            filter_acc_x, filter_acc_y, filter_acc_z = raw_x_acc, raw_y_acc, raw_z_acc
                        else:
                            filter_acc_x = alpha_ * raw_x_acc + (1 - alpha_) * filter_acc_x
                            filter_acc_y = alpha_ * raw_y_acc + (1 - alpha_) * filter_acc_y
                            filter_acc_z = alpha_ * raw_z_acc + (1 - alpha_) * filter_acc_z

                        if filter_gyr_x is None:
                            filter_gyr_x, filter_gyr_y, filter_gyr_z = raw_x_gyr, raw_y_gyr, raw_z_gyr
                            roll = accel_roll
                            pitch = accel_pitch
                        else:
                            filter_gyr_x = alpha_ * raw_x_gyr + (1 - alpha_) * filter_gyr_x
                            filter_gyr_y = alpha_ * raw_y_gyr + (1 - alpha_) * filter_gyr_y
                            filter_gyr_z = alpha_ * raw_z_gyr + (1 - alpha_) * filter_gyr_z

                            accel_roll = math.atan2(-filter_acc_x, g)
                            accel_pitch = math.atan2(filter_acc_y, g)
                            roll = alpha_cf * (roll + raw_x_gyr * dt) + (1 - alpha_cf) * accel_roll
                            pitch = alpha_cf * (pitch + raw_y_gyr * dt) + (1 - alpha_cf) * accel_pitch

                            print("acc_roll and pitch", accel_roll, accel_pitch)
                            print("roll and pitch", roll, pitch)

                        gx = -g * math.sin(pitch)
                        gy = g * math.cos(pitch) * math.sin(roll)
                        gz = g * math.cos(pitch) * math.cos(roll)

                        x = filter_acc_x - gx
                        y = filter_acc_y - gy
                        z = filter_acc_z - (gz - g)

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

                            x_stationary = (var_x_acc < var_thr_acc_x) and (abs(mean_x) < calib["deadzone_x_a"])
                            y_stationary = (var_y_acc < var_thr_acc_y) and (abs(mean_y) < calib["deadzone_y_a"])
                            z_stationary = (var_z_acc < var_thr_acc_z) and (abs(mean_z) < calib["deadzone_z_a"])

                            window_says_stationary = x_stationary and y_stationary and z_stationary

                            # DEBOUNCE: only flip the committed state after DEBOUNCE_COUNT
                            # consecutive windows agree - one noisy/lucky window can no
                            # longer flip stationary->moving (or moving->stationary) alone.
                            if window_says_stationary:
                                stationary_streak += 1
                                moving_streak = 0
                            else:
                                moving_streak += 1
                                stationary_streak = 0

                            if not is_stationary_state and stationary_streak >= DEBOUNCE_COUNT:
                                is_stationary_state = True
                            elif is_stationary_state and moving_streak >= DEBOUNCE_COUNT:
                                is_stationary_state = False

                            if is_stationary_state:
                                x, y, z = 0.0, 0.0, 0.0
                                velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0
                        else:
                            x, y, z = 0.0, 0.0, 0.0
                            velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0

                        velocity_x += x * dt
                        velocity_y += y * dt
                        velocity_z += z * dt

                        if abs(velocity_x) < 0.003: velocity_x = 0.0
                        if abs(velocity_y) < 0.003: velocity_y = 0.0
                        if abs(velocity_z) < 0.003: velocity_z = 0.0

                        speed = math.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)

                        # distance is cumulative PATH LENGTH (like an odometer) - it should
                        # not, and will not, reset to 0 when ZUPT zeros velocity at a stop.
                        # It should simply stop increasing once speed correctly hits 0.
                        distance += speed * dt
                        distance_cm = distance * 100

                        status = "STATIONARY" if is_stationary_state else "MOVING"

                        print("filter_acceleration", filter_acc_x, filter_acc_y, filter_acc_z)
                        print("filtered_gyro", filter_gyr_x, filter_gyr_y, filter_gyr_z)
                        print("acceleration", x, y, z)
                        print("velocity", velocity_x, velocity_y, velocity_z)
                        print("speed", speed)
                        print("distance", distance_cm)
                        print("status", status)
                        print("------------------------------------------------")
                        writer.writerow([
                            current_time, x, y, z,
                            velocity_x, velocity_y, velocity_z, speed, distance_cm
                        ])

except KeyboardInterrupt:
    print("\nStopped by user.")