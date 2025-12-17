import cv2
import numpy as np
import collections
import sys

# CONFIGURATION 
CAM_INDEX   = 0
FPS         = 60
WIDTH       = 640
HEIGHT      = 480
FLIP_CODE   = -1    

# VLP SETTINGS 
TARGET_FREQ = 14.1  
WIN_SIZE    = 32
SNR_THRESH  = 5.0   
LOCK_COUNT  = 30    
PROC_W      = 160   
PROC_H      = 120
REFLECTION_TOLERANCE = 30

#  TRACKER CLASS  
class BeaconTracker:
    def __init__(self, label, color):
        self.label = label
        self.color = color
        self.history = collections.deque(maxlen=LOCK_COUNT)
        self.locked = False
        self.miss_count = 0
        self.current_pos = None # Live position

    def update(self, pos):
        if pos:
            self.miss_count = 0
            self.history.append(pos)
            if len(self.history) == LOCK_COUNT:
                self.locked = True
                
            # Average for stability
            avg_x = int(sum(p[0] for p in self.history) / len(self.history))
            avg_y = int(sum(p[1] for p in self.history) / len(self.history))
            self.current_pos = (avg_x, avg_y)
        else:
            if self.locked:
                self.miss_count += 1
                if self.miss_count > 10: # Sticky Tolerance
                    self.locked = False
                    self.history.clear()
                    self.current_pos = None
            else:
                self.history.clear()
                self.current_pos = None

#  MAIN 
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# Buffers
frame_buffer = np.zeros((WIN_SIZE, PROC_H, PROC_W), dtype=np.float32)
frame_idx = 0
frames_collected = 0

fft_freqs = np.fft.rfftfreq(WIN_SIZE, 1.0/FPS)
target_bin = np.argmin(np.abs(fft_freqs - TARGET_FREQ))

tracker_L = BeaconTracker("LEFT", (0, 255, 0))
tracker_R = BeaconTracker("RIGHT", (0, 0, 255))

# Variables to store "Locked" values
saved_L = None
saved_R = None

print("PIXEL EXTRACTION TOOL")
print("1. Place robot at known distance.")
print("2. Wait for Green/Red circles.")
print("3. Press 'Y' to lock and print coordinates.")
print("4. Press 'Q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret: break
    if FLIP_CODE is not None: frame = cv2.flip(frame, FLIP_CODE)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (PROC_W, PROC_H))
    
    frame_buffer[frame_idx] = small
    frame_idx = (frame_idx + 1) % WIN_SIZE
    frames_collected += 1
    
    cand_L = None
    cand_R = None

    if frames_collected >= WIN_SIZE:
        aligned = np.roll(frame_buffer, -frame_idx, axis=0)
        ac = aligned - np.mean(aligned, axis=0)
        fft = np.fft.rfft(ac, axis=0)
        mags = np.abs(fft)
        
        # Energy Map
        mag_map = mags[target_bin]
        snr_map = mag_map / (np.mean(mags) + 0.001)
        
        # Blob Detection
        mask = np.uint8(snr_map > SNR_THRESH) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        raw_blobs = []
        for c in contours:
            if cv2.contourArea(c) < 1: continue
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Signal Strength
                strength = mag_map[cy, cx]
                
                # Scale up to 640x480
                rx = int(cx * (WIDTH/PROC_W))
                ry = int(cy * (HEIGHT/PROC_H))
                
                raw_blobs.append((rx, ry, strength))

        #  REFLECTION KILLER 
        raw_blobs.sort(key=lambda p: p[2], reverse=True) # Brightest first
        clean_blobs = []
        while raw_blobs:
            best = raw_blobs.pop(0)
            clean_blobs.append(best)
            # Remove neighbors in same X column
            raw_blobs = [b for b in raw_blobs if abs(b[0] - best[0]) > REFLECTION_TOLERANCE]

        #  SPATIAL SORTING 
        if len(clean_blobs) >= 2:
            # Sort by strength descending and take top 2 (ignores dimmer reflections)
            clean_blobs.sort(key=lambda p: p[2], reverse=True)
            top_two = clean_blobs[:2]
            # Now sort those by X for left/right assignment
            top_two.sort(key=lambda p: p[0])
            cand_L = (top_two[0][0], top_two[0][1])
            cand_R = (top_two[1][0], top_two[1][1])
        elif len(clean_blobs) == 1:
            x, y, s = clean_blobs[0]
            if x < WIDTH // 2: cand_L = (x, y)
            else:              cand_R = (x, y)

    # Update Trackers
    tracker_L.update(cand_L)
    tracker_R.update(cand_R)

    # Drawing
    for tr in [tracker_L, tracker_R]:
        if tr.current_pos:
            cv2.circle(frame, tr.current_pos, 20, tr.color, 2)
            cv2.putText(frame, f"{tr.current_pos[0]}", (tr.current_pos[0]-20, tr.current_pos[1]-25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, tr.color, 2)
    
    # Show Saved Values 
    if saved_L and saved_R:
        cv2.putText(frame, f"SAVED -> L:{saved_L[0]} R:{saved_R[0]}", (10, 450), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        # Calculate separation
        sep = abs(saved_R[0] - saved_L[0])
        cv2.putText(frame, f"GAP: {sep} px", (10, 420), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Pixel Extractor", frame)
    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('y'):
        #  LOCK LOGIC 
        if tracker_L.current_pos and tracker_R.current_pos:
            saved_L = tracker_L.current_pos
            saved_R = tracker_R.current_pos
            
            print(f"LOCKED: Left_X={saved_L[0]}, Right_X={saved_R[0]} | Separation={abs(saved_R[0]-saved_L[0])} px")
        else:
            print("Cannot Lock: Beacons not stable yet.")

cap.release()
cv2.destroyAllWindows()