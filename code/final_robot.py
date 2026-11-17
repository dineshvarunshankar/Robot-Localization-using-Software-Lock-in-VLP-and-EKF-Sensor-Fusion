import argparse
import sys
import time
import math
import threading
import collections
import serial
import numpy as np
import cv2
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description="VLP + EKF robot localization")
parser.add_argument("--port", default="/dev/cu.usbmodem1101", help="Pico serial port")
parser.add_argument("--calib", nargs=2, type=int, default=[252, 370], metavar=("LEFT_X", "RIGHT_X"),
                    help="Beacon pixel x-coordinates from beacons_pixel_extractor.py")
args = parser.parse_args()

# Configuration
USE_VLP = True

# Calibration
TICKS_PER_METER = 1148.0
CALIB_PIXEL_L, CALIB_PIXEL_R = args.calib
CALIB_DIST      = 1.25
BEACON_WIDTH    = 0.4    # Beacon spacing (m)

# Vision parameters
TARGET_FREQ    = 14.1
WIN_SIZE       = 32
SNR_THRESH     = 5.0
LOCK_COUNT     = 30
REFLECTION_TOL = 30
PROC_W         = 160
PROC_H         = 120
FLIP_CODE      = -1     # Camera mounting rotation
FRAME_W        = 640
FRAME_H        = 480
CAM_FPS        = 60

if not 0 <= CALIB_PIXEL_L < CALIB_PIXEL_R < FRAME_W:
    parser.error(f"--calib needs 0 <= LEFT_X < RIGHT_X < {FRAME_W}")

# Hardware
SERIAL_PORT = args.port
BAUD_RATE   = 115200
CAM_INDEX   = 0

# Robot frame: positive Y points left.
BEACON_LEFT  = np.array([CALIB_DIST,  BEACON_WIDTH / 2.0])
BEACON_RIGHT = np.array([CALIB_DIST, -BEACON_WIDTH / 2.0])

px_width = abs(CALIB_PIXEL_R - CALIB_PIXEL_L)
FOCAL_LENGTH = (px_width * CALIB_DIST) / BEACON_WIDTH
print(f"Computed Focal Length: {FOCAL_LENGTH:.2f}")

RUNNING = True
data_lock = threading.Lock()

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except serial.SerialException:
    print(f"Could not open {SERIAL_PORT}. Check USB connection.")
    sys.exit()

current_key = None
last_key_time = 0.0

def heartbeat_loop():
    """Sends control commands to the robot periodically."""
    global current_key
    while RUNNING:
        if time.time() - last_key_time > 0.1:
            current_key = None

        cmd = current_key.encode() if current_key else b' '
        ser.write(cmd)
        time.sleep(0.05)

threading.Thread(target=heartbeat_loop, daemon=True).start()

class ExtendedKalmanFilter:
    def __init__(self):
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1

        # Process covariance
        self.Q = np.diag([0.000008, 0.000008, 0.0002])

        # Bearing variance (rad²)
        self.R_bearing = (math.radians(140.0))**2

    def normalize_angle(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def predict(self, dist, d_theta):
        th = self.x[2, 0]
        self.x[0, 0] += dist * math.cos(th)
        self.x[1, 0] += dist * math.sin(th)
        self.x[2, 0] = self.normalize_angle(self.x[2, 0] + d_theta)

        F = np.eye(3)
        F[0, 2] = -dist * math.sin(th)
        F[1, 2] =  dist * math.cos(th)

        self.P = F @ self.P @ F.T + self.Q

    def correct_bearing(self, beacon_xy, z_bearing):
        dx = beacon_xy[0] - self.x[0, 0]
        dy = beacon_xy[1] - self.x[1, 0]
        q = dx**2 + dy**2
        if q < 1e-4: return

        pred = math.atan2(dy, dx) - self.x[2, 0]
        pred = self.normalize_angle(pred)
        y = self.normalize_angle(z_bearing - pred)

        H = np.zeros((1, 3))
        H[0, 0] =  dy / q
        H[0, 1] = -dx / q
        H[0, 2] = -1.0

        S = H @ self.P @ H.T + self.R_bearing
        S_val = float(S[0, 0])

        # Normalized innovation squared gate
        if S_val > 0 and (y**2) / S_val <= 6.0:
            K = (self.P @ H.T) / S_val
            self.x = self.x + K * y
            self.x[2, 0] = self.normalize_angle(self.x[2, 0])
            self.P = (np.eye(3) - K @ H) @ self.P

class BeaconTracker:
    def __init__(self, color):
        self.color = color
        self.history = collections.deque(maxlen=LOCK_COUNT)
        self.locked = False
        self.current_pos = None

    def update(self, pos):
        if pos is not None:
            self.history.append(pos)
            self.locked = (len(self.history) == LOCK_COUNT)
            self.current_pos = pos
        else:
            # Invalidate observations when detection is lost.
            self.locked = False
            self.history.clear()
            self.current_pos = None

    def get_smoothed_pos(self):
        if not self.locked or len(self.history) < 5: return None
        recent = list(self.history)[-10:]
        avg_x = int(sum(p[0] for p in recent) / len(recent))
        avg_y = int(sum(p[1] for p in recent) / len(recent))
        return (avg_x, avg_y)

ekf_odom  = ExtendedKalmanFilter()
ekf_fused = ExtendedKalmanFilter()

tracker_L = BeaconTracker((0, 255, 0))
tracker_R = BeaconTracker((0, 0, 255))

path_odom = {'x': [], 'y': []}

def sensor_loop():
    last_ticks = 0.0
    gyro_bias = 0.0
    calibrated = False
    samples = []
    start_t = time.time()
    last_t = time.time()
    path_decimate_counter = 0

    print("Calibrating Gyro....")

    while RUNNING:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line: continue

            ticks, gz = map(float, line.split(','))

            if not calibrated:
                samples.append(gz)
                if time.time() - start_t > 10.0:
                    gyro_bias = sum(samples) / len(samples)
                    last_ticks = ticks
                    last_t = time.time()
                    calibrated = True
                    print(f"READY! VLP Mode: {USE_VLP}")
            else:
                now = time.time()
                dt = now - last_t
                last_t = now

                delta = ticks - last_ticks
                last_ticks = ticks
                dist = delta / TICKS_PER_METER

                is_moving = abs(delta) > 0

                # Suppress gyro drift when the encoder reports no motion.
                if not is_moving:
                    d_theta = 0.0
                elif abs(gz - gyro_bias) < 1.0:
                    d_theta = 0.0
                else:
                    d_theta = math.radians(gz - gyro_bias) * dt

                with data_lock:
                    ekf_odom.predict(dist, d_theta)
                    ekf_fused.predict(dist, d_theta)

                    path_decimate_counter += 1
                    if path_decimate_counter % 10 == 0:
                        path_odom['x'].append(ekf_odom.x[0, 0])
                        path_odom['y'].append(ekf_odom.x[1, 0])

        except ValueError:
            continue  # Malformed serial line

def run_sensor_loop():
    global RUNNING
    try:
        sensor_loop()
    finally:
        RUNNING = False

threading.Thread(target=run_sensor_loop, daemon=True).start()

for k in ('keymap.save', 'keymap.quit'):
    plt.rcParams[k] = []  # Free 's' and 'q' for robot controls
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))

line_odom, = ax.plot([], [], color='blue', linewidth=2.5, alpha=0.7, label="Encoder Path")
dot_odom,  = ax.plot([], [], 'ro', markersize=9, label="Encoder Pos")
dot_ekf,   = ax.plot([], [], 'go', markersize=9, label="VLP Pos")
ax.plot([BEACON_LEFT[0], BEACON_RIGHT[0]],
        [BEACON_LEFT[1], BEACON_RIGHT[1]],
        'k^', markersize=12, label="Beacons")

ax.legend(loc='upper left')
ax.grid(True)
ax.axis('equal')
ax.set_title("Blue=Path | Red=Encoder Pos | Green=VLP Pos")

def on_press(event):
    global current_key, last_key_time, RUNNING
    if event.key in ['w', 'a', 's', 'd', 'x']:
        current_key = event.key
        last_key_time = time.time()
    elif event.key == 'q':
        RUNNING = False

fig.canvas.mpl_connect('key_press_event', on_press)

def detect_beacons(frame_buffer, f_idx):
    aligned = np.roll(frame_buffer, -f_idx, axis=0)
    ac = aligned - np.mean(aligned, axis=0)
    fft = np.fft.rfft(ac, axis=0)
    mags = np.abs(fft)
    snr_map = mags[target_bin] / (np.mean(mags) + 0.001)

    mask = np.uint8(snr_map > SNR_THRESH) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for c in contours:
        if cv2.contourArea(c) < 2.0: continue

        M = cv2.moments(c)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            rx = int(cx * (FRAME_W / PROC_W))
            ry = int(cy * (FRAME_H / PROC_H))
            blobs.append((rx, ry))

    # Prefer upper blobs over floor reflections.
    blobs.sort(key=lambda p: p[1])
    clean = []

    while blobs:
        best = blobs.pop(0)
        clean.append(best)
        # Suppress reflections in the same image column.
        blobs = [b for b in blobs if abs(b[0] - best[0]) > REFLECTION_TOL]

    cand_L, cand_R = None, None

    if len(clean) >= 2:
        clean.sort(key=lambda p: p[0])
        cand_L, cand_R = clean[0], clean[-1]

    return cand_L, cand_R

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame_buffer = np.zeros((WIN_SIZE, PROC_H, PROC_W), dtype=np.float32)
f_idx = 0
f_count = 0
plot_counter = 0
fft_freqs = np.fft.rfftfreq(WIN_SIZE, 1.0 / CAM_FPS)
target_bin = np.argmin(np.abs(fft_freqs - TARGET_FREQ))

try:
    while RUNNING:
        ret, frame = cap.read()
        if not ret: break
        if FLIP_CODE: frame = cv2.flip(frame, FLIP_CODE)

        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            c = chr(key).lower()
            if c in ['w', 'a', 's', 'd', 'x']:
                current_key = c
                last_key_time = time.time()
            elif c == 'q':
                RUNNING = False
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (PROC_W, PROC_H))
        frame_buffer[f_idx] = small
        f_idx = (f_idx + 1) % WIN_SIZE
        f_count += 1

        cand_L, cand_R = None, None

        if f_count >= WIN_SIZE:
            cand_L, cand_R = detect_beacons(frame_buffer, f_idx)

        tracker_L.update(cand_L)
        tracker_R.update(cand_R)

        if USE_VLP and tracker_L.locked and tracker_R.locked:
            pos_L = tracker_L.get_smoothed_pos()
            pos_R = tracker_R.get_smoothed_pos()

            if pos_L and pos_R:
                uL = pos_L[0]
                uR = pos_R[0]
                calib_midpoint = (CALIB_PIXEL_L + CALIB_PIXEL_R) / 2.0
                cx_im = calib_midpoint

                bearing_L = - math.atan((uL - cx_im) / FOCAL_LENGTH)
                bearing_R = - math.atan((uR - cx_im) / FOCAL_LENGTH)

                with data_lock:
                    ekf_fused.correct_bearing(BEACON_LEFT, bearing_L)
                    ekf_fused.correct_bearing(BEACON_RIGHT, bearing_R)

                cv2.line(frame, pos_L, pos_R, (0, 255, 0), 2)

        for tr in [tracker_L, tracker_R]:
            if tr.current_pos: cv2.circle(frame, tr.current_pos, 20, tr.color, 2)

        cv2.imshow("Robot View", frame)

        plot_counter += 1
        if plot_counter % 10 == 0:
            with data_lock:
                if len(path_odom['x']) > 0:
                    line_odom.set_data(path_odom['x'], path_odom['y'])
                    dot_odom.set_data([path_odom['x'][-1]], [path_odom['y'][-1]])

                if USE_VLP:
                    dot_ekf.set_data([ekf_fused.x[0, 0]], [ekf_fused.x[1, 0]])
                else:
                    dot_ekf.set_data([], [])

            ax.relim()
            ax.autoscale_view()
            plt.pause(0.001)

finally:
    print("\nShutting down...")
    RUNNING = False
    time.sleep(0.5)

    print("FINAL COORDINATES REPORT")

    # Snapshot current estimates; plot history is subsampled.
    with data_lock:
        final_odom_x = ekf_odom.x[0, 0]
        final_odom_y = ekf_odom.x[1, 0]
        final_vlp_x = ekf_fused.x[0, 0]
        final_vlp_y = ekf_fused.x[1, 0]

    if USE_VLP:
        print(f" IMU+Encoder (Odom): X={final_odom_x:.4f} m, Y={final_odom_y:.4f} m")
        print(f" VLP Fused (EKF):    X={final_vlp_x:.4f} m, Y={final_vlp_y:.4f} m")
    else:
        print(f" IMU+Encoder (Odom): X={final_odom_x:.4f} m, Y={final_odom_y:.4f} m")

    if ser: ser.close()
    cap.release()
    cv2.destroyAllWindows()
    plt.close()

