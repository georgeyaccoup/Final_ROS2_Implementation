import pyrealsense2 as rs
import cv2
import numpy as np

def process_pear(color_image, depth_image):
    output_img = color_image.copy()

    # 1. DEPTH MASK: Physical extraction
    max_distance_mm = 380 
    depth_mask = cv2.inRange(depth_image, 1, max_distance_mm)

    # 2. DUAL-SPACE SEGMENTATION (HSV + LAB)
    # HSV: Good for Green/Yellow hue
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, np.array([10, 40, 40]), np.array([95, 255, 255]))
    
    # LAB: Good for "pear-skin" characteristics (L channel for intensity, A/B for color tone)
    lab = cv2.cvtColor(color_image, cv2.COLOR_BGR2LAB)
    # A-channel is green-red, B-channel is blue-yellow
    # We focus on the mid-range of lightness and specific pear-colored A/B values
    lab_mask = cv2.inRange(lab, np.array([50, 120, 130]), np.array([255, 150, 200]))

    # Combine masks: Pear must satisfy Depth, HSV, and LAB criteria
    combined_mask = cv2.bitwise_and(depth_mask, cv2.bitwise_and(hsv_mask, lab_mask))

    # 3. ROI EXTRACTION: Focus only on the pear
    contours_info = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
    
    if not contours: return output_img
    
    main_pear = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(main_pear)
    
    if w*h < 500: return output_img

    # 4. MASKED INFECTION ANALYSIS
    # Create a refined mask of ONLY the main pear
    pear_roi_mask = np.zeros_like(combined_mask)
    cv2.drawContours(pear_roi_mask, [main_pear], -1, 255, -1)
    
    # Isolate grayscale pear for analysis
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    masked_gray = cv2.bitwise_and(gray, gray, mask=pear_roi_mask)
    
    # Get pixels strictly inside the pear
    pear_pixels = masked_gray[pear_roi_mask == 255]
    
    # Otsu on the pear pixels only
    otsu_val, _ = cv2.threshold(pear_pixels, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Detect infections (pixels darker than 75% of Otsu's threshold)
    _, infections = cv2.threshold(masked_gray, otsu_val * 0.75, 255, cv2.THRESH_BINARY_INV)
    infections = cv2.bitwise_and(infections, pear_roi_mask)
    
    # Clean up infection spots
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    infections = cv2.morphologyEx(infections, cv2.MORPH_CLOSE, kernel)

    # 5. CLASSIFICATION & ANNOTATION
    inf_contours_info = cv2.findContours(infections, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    inf_contours = inf_contours_info[0] if len(inf_contours_info) == 2 else inf_contours_info[1]
    
    inf_area = sum(cv2.contourArea(c) for c in inf_contours)
    infection_ratio = inf_area / cv2.contourArea(main_pear)
    is_bad = infection_ratio > 0.03

    # Draw result
    color = (0, 0, 255) if is_bad else (0, 255, 0)
    cv2.rectangle(output_img, (x, y), (x + w, y + h), color, 2)
    cv2.putText(output_img, f"{'BAD' if is_bad else 'GOOD'} {infection_ratio*100:.1f}%", 
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.drawContours(output_img, inf_contours, -1, (0, 0, 255), 2)

    return output_img

# ==========================================
# MAIN REALSENSE D455 CAMERA LOOP
# ==========================================
if __name__ == "__main__":
    # 1. Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Request both Color and Depth streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 2. Create an alignment object
    # Depth and Color lenses are physically separated on the D455. 
    # This aligns the 3D depth map to match the 2D color image perfectly.
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # Start streaming
        pipeline.start(config)
        print("✅ Intel RealSense D455 initialized successfully!")
        print("⚠️  Press the 'q' key on your keyboard to quit.")

        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            
            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)
            
            # Extract the aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # Convert RealSense frames to standard NumPy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Send both frames through the pipeline
            processed_frame = process_pear(color_image, depth_image)

            # Display the result
            cv2.imshow("D455 Live Pear Detection", processed_frame)

            # Wait 1 ms and check if the user pressed 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"❌ Camera Error: {e}")
        
    finally:
        # Clean up and stop the camera safely
        pipeline.stop()
        cv2.destroyAllWindows()
        print("🛑 Camera closed safely.")