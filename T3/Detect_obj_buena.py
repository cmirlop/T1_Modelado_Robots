import sys
import numpy as np
import cv2 as cv
import os
import json
import serial
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CALIBRATION_PATH = os.path.join(SCRIPT_DIR, 'calibration.json')

HSV_PARAMS = {}

## VARIABLES DE AJUSTE
MIN_AREA = 1000
MAX_AREA = 50000 # Increased max area (cube is close to camera)
Kp = 2 # TODO ajustar

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

angle_current_pan = 90
angle_current_tilt = 90
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

def correction_calculation(error, arduino):
    # --- 1. PAN (Horizontal) TUNING ---
    # If the robot turns AWAY from the object and hits 0/180,
    # CHANGE THIS SIGN: (Try '+' first, if it fails, change to '-')
    pan_correction = (Kp * error[0]) 
    new_pan = MOTORS_CURRENT_ANGLES['Pan'] - pan_correction  # <--- CHECK THIS SIGN (+ or -)

    # Safety Clamp Pan
    if new_pan > 180: new_pan = 180
    if new_pan < 0: new_pan = 0
    
    MOTORS_CURRENT_ANGLES['Pan'] = new_pan

    # --- 2. TILT (Vertical) - DISABLED FOR NOW ---
    # We keep it fixed at 90 until Pan works perfectly
    # Safety Clamp Pan
    
    tilt_correction = (Kp * error[0])
    new_tilt = MOTORS_CURRENT_ANGLES['Tilt'] - tilt_correction # Reset memory to 90

    if new_tilt > 180: new_tilt = 180
    if new_tilt < 0: new_tilt = 0

    # --- Send Command ---
    command = f"TRACK,{int(new_pan)},{int(new_tilt)}\n"
    arduino.write(command.encode())

    # --- DEBUG PRINT (Crucial!) ---
    # Watch this in the terminal. 
    # If 'Error' is POSITIVE, does 'Pan' INCREASE or DECREASE?
    print(f"Error: {int(error[0])} | Pan Angle: {int(new_pan)} | Tilt Angle: {int(new_tilt)}")


def drawing_diff_err_ref(centroid_point, frame, contour):
    # 1. Create the tuple locally using simple parentheses (x, y)
    # We cast to int() here because OpenCV requires integers
    ref_tuple = (int(REFERENCE_POINT['X']), int(REFERENCE_POINT['Y']))
    
    # 2. Draw using this local tuple
    cv.drawContours(frame, [contour], -1, (255, 0, 0), 2)
    cv.circle(frame, centroid_point, 8, (255, 100, 0), -1)
    
    # Draw the reference (center) point
    cv.circle(frame, ref_tuple, 8, (0, 255, 0), -1)
    
    # Draw the line connecting them
    cv.line(frame, ref_tuple, centroid_point, (0, 255, 0), 2)

def read_arduino_data(arduino):
    # While there is data waiting in the serial buffer...
    while arduino.in_waiting > 0:
        try:
            # Read the line, decode from bytes to string, remove whitespace
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            
            # Print it to the terminal so you can see it
            if line:
                print(f"[Arduino] -> {line}")
        except Exception as e:
            print(f"Serial Read Error: {e}")

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
    frame_dimensions(frame)
    print(f"Dimensions --> [{REFERENCE_POINT['X'],{REFERENCE_POINT['Y']}}]")

    window_title = "Tracking"
    cv.namedWindow(window_title, cv.WINDOW_NORMAL)

    arduino = serial.Serial(port='COM3', baudrate=115200, timeout=0.1)
    time.sleep(2) # Give Arduino time to reboot after connection
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
        centroid_point, contour = get_centroid(mask)

        # 4. Draw Result
        if centroid_point is not None:
            cv.putText(frame, "Tracking", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            error = error_calculation(centroid_point)
            correction_calculation(error, arduino)
            drawing_diff_err_ref(centroid_point, frame, contour)

        else:
            cv.putText(frame, "Lost", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        read_arduino_data(arduino)
        cv.imshow(window_title, frame)
        
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    capture.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()