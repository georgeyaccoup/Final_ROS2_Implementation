import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math

# --- 1. Camera Setup ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Get camera intrinsics for 2D to 3D mapping
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# --- 2. State Machine Variables ---
is_moving = False
start_time = 0
last_move_time = 0
start_pos = None
last_pos = None

# --- 3. Thresholds ---
ACTIVITY_THRESHOLD = 0.01  # > 1cm frame-to-frame counts as active movement
STOP_DELAY = 0.5           # Wait 0.5 seconds of NO movement before triggering a stop
MIN_TOTAL_DISTANCE = 0.1   # Ignore events under 10cm to filter out noise/wobbling

def get_pear_centroid(color_image):
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
    # Slightly broadened range to catch both the bright green and deep yellow pears
    lower_color = np.array([15, 40, 40])
    upper_color = np.array([45, 255, 255])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # --- NEW: Morphological Filtering ---
    # This "fills in" the holes caused by brown blemishes and removes tiny background noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Removes background noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Closes holes inside the pear
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnts[0] if len(cnts) == 2 else cnts[1]
    
    valid_contours = []
    
    # --- NEW: Shape and Size Constraints ---
    for c in contours:
        area = cv2.contourArea(c)
        if area > 1500:  # Increased minimum size to ignore small background objects
            
            # Calculate Aspect Ratio (Width divided by Height)
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = float(w) / h
            
            # Pears are typically taller than they are wide, or roughly 1:1.
            # This rejects long thin objects (like cables) or wide objects.
            if 0.5 <= aspect_ratio <= 1.3:
                valid_contours.append(c)
                
    # If we found contours that match the shape of a pear
    if valid_contours:
        # Pick the largest one that passed our shape test
        largest_contour = max(valid_contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            return (cx, cy)
            
    return None

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        color_image = np.asanyarray(color_frame.get_data())
        centroid = get_pear_centroid(color_image)
        
        if centroid:
            cx, cy = centroid
            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1) 
            
            depth = depth_frame.get_distance(cx, cy)
            if depth > 0:
                current_pos = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                
                if last_pos is not None:
                    # Calculate frame-to-frame movement
                    frame_dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, last_pos)))
                    
                    # Update activity timer if the pear is actively moving this frame
                    if frame_dist > ACTIVITY_THRESHOLD:
                        last_move_time = time.time()
                        
                        if not is_moving:
                            is_moving = True
                            start_time = time.time()
                            start_pos = current_pos
                            print("Pear started moving...")
                            
                    elif is_moving:
                        # It is not moving THIS frame. Has it been stopped long enough?
                        if time.time() - last_move_time > STOP_DELAY:
                            
                            is_moving = False
                            end_time = last_move_time # Calculate speed up to the moment it actually stopped
                            
                            total_dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(start_pos, current_pos)))
                            total_time = end_time - start_time
                            
                            # Filter out false-starts and noise
                            if total_dist >= MIN_TOTAL_DISTANCE and total_time > 0:
                                speed = total_dist / total_time
                                print(f"--- EVENT COMPLETE ---")
                                print(f"Distance: {total_dist:.2f} meters")
                                print(f"Time: {total_time:.2f} seconds")
                                print(f"Speed: {speed:.2f} m/s")
                                
                last_pos = current_pos

        cv2.imshow('Pear Tracker', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()