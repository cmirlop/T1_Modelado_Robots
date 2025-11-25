import sys
import numpy as np
import cv2 as cv
import os
import json

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CALIBRATION_PATH = os.path.join(SCRIPT_DIR, 'calibration.json')

HSV_PARAMS = {}

## VARIABLES DE AJUSTE
MIN_AREA = 1000
MAX_AREA = 50000 # Increased max area (cube is close to camera)
Kp = 1 # TODO ajustar

h = None
w = None

REFERENCE_POINT = {
    'X': w,
    'Y': h
}


error_x = None
error_y = None
# Motores Robot
MOTORS_ANGLES = {
    'Pan': error_x,
    'Tilt': error_y
}

angle_current_pan = None
angle_current_tilt = None
MOTORS_CURRENT_ANGLES = {
    'Pan': angle_current_pan,
    'Tilt': angle_current_tilt
}



def open_camera(camera_index: int) -> cv.VideoCapture:
    capture = cv.VideoCapture(camera_index, cv.CAP_DSHOW)
    if not capture.isOpened():
        capture = cv.VideoCapture(camera_index)
    return capture

def importing_params():
    global HSV_PARAMS
    if not os.path.exists(CALIBRATION_PATH):
        print(f"Error: No se encuentra {CALIBRATION_PATH}")
        sys.exit(1)
    with open(CALIBRATION_PATH, 'r') as json_file:
        calibration_hsv = json.load(json_file)
        HSV_PARAMS = {
            'HSV':{
                'H_MIN': calibration_hsv['HSV']['H']['min'],
                'H_MAX': calibration_hsv['HSV']['H']['max'],
                'S_MIN': calibration_hsv['HSV']['S']['min'],
                'S_MAX': calibration_hsv['HSV']['S']['max'],
                'V_MIN': calibration_hsv['HSV']['V']['min'],
                'V_MAX': calibration_hsv['HSV']['V']['max']
            }
        }

def Object_detection(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Using numpy array is faster and cleaner than splitting channels
    lower = np.array([HSV_PARAMS['HSV']['H_MIN'], HSV_PARAMS['HSV']['S_MIN'], HSV_PARAMS['HSV']['V_MIN']])
    upper = np.array([HSV_PARAMS['HSV']['H_MAX'], HSV_PARAMS['HSV']['S_MAX'], HSV_PARAMS['HSV']['V_MAX']])
    
    mask = cv.inRange(hsv, lower, upper)
    
    # Optional: Clean up noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    return mask

def get_centroid(mask):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    # Find the largest contour that fits our area criteria
    best_c = None
    max_area = 0
    
    for c in contours:
        area = cv.contourArea(c)
        if MIN_AREA < area < MAX_AREA:
            if area > max_area:
                max_area = area
                best_c = c
    
    center = None
    if best_c is not None:
        m = cv.moments(best_c)
        if m['m00'] != 0:
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            center = (cx, cy)
            
    return center, best_c

def frame_dimensions(frame):
    global REFERENCE_POINT
    print(frame.shape)
    h, w = frame.shape[:2]
    REFERENCE_POINT['X'] = w/2
    REFERENCE_POINT['Y'] = h/2

def error_calculation(center_point):
    error = []
    error_x  = REFERENCE_POINT['X'] - center_point[0]
    error_y  = REFERENCE_POINT['Y'] - center_point[1]

    error.append(error_x)
    error.append(error_y)
    return error

def correction_calculation(error):
    motor_corrections = []
    if error[0] > 0:
        angle_pan = MOTORS_CURRENT_ANGLES['Pan'] - (Kp * error[0])
        MOTORS_ANGLES['Pan'] = angle_pan
        motor_corrections.append(angle_pan)

    if error[1] > 0:
        angle_tilt = MOTORS_CURRENT_ANGLES['Tilt'] - (Kp * error[1])
        MOTORS_ANGLES['Tilt'] = angle_tilt
        motor_corrections.append(angle_tilt)

    for angle in motor_corrections:
        if angle > 180:
            print("Angle correction is bigger than 180 ")
            angle = 180
        if angle < 0:
            print("Angle correction is negative")
            angle = 0



def main() -> None:
    importing_params()
    
    # Basic Argument Parsing
    camera_index = 0
    if len(sys.argv) >= 2:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            pass

    capture = open_camera(camera_index)
    if not capture.isOpened():
        sys.exit(1)
    grabbed, frame = capture.read()
    if not grabbed:
        return
    dimensions = frame_dimensions(frame)

    window_title = "Tracking"
    cv.namedWindow(window_title, cv.WINDOW_NORMAL)

    while True:
        grabbed, frame = capture.read()
        if not grabbed:
            break
        
        # 1. Detect Object
        mask = Object_detection(frame)
        
        # 2. Show Mask (CRITICAL FOR DEBUGGING)
        # If this window is black, your calibration.json is wrong
        cv.imshow("Mask Debug", mask) 

        # 3. Calculate Centroid
        center_point, contour = get_centroid(mask)

        # 4. Draw Result
        if center_point is not None:
            # Draw the contour outline
            cv.drawContours(frame, [contour], -1, (255, 0, 0), 2)
            # Draw the center point
            cv.circle(frame, center_point, 8, (0, 255, 0), -1)
            cv.putText(frame, "Tracking", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            error = error_calculation(center_point)
            correction_calculation(error)
        else:
            cv.putText(frame, "Lost", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv.imshow(window_title, frame)
        
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    capture.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()