import sys
import datetime
from pathlib import Path
import numpy as np
import cv2 as cv
import os
import json

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
# Ruta del archivo de calibración
CALIBRATION_PATH = os.path.join(SCRIPT_DIR, 'calibration.json')

HSV_PARAMS = {}

## VARIABLES DE AJUSTE
MIN_AREA = 1000
MAX_AREA = 20000
HSV_DONE = False




def open_camera(camera_index: int) -> cv.VideoCapture:
    capture = cv.VideoCapture(camera_index, cv.CAP_DSHOW)
    if not capture.isOpened():
        # Retry without CAP_DSHOW (cross-platform fallback)
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

def main() -> None:
    importing_params()
    # Camera index from args (default 0)
    camera_index = 0
    if len(sys.argv) >= 2:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print("Índice de cámara inválido. Usa un número (ej. 0, 1).")
            sys.exit(1)

    capture = open_camera(camera_index)
    if not capture.isOpened():
        print(f"No se pudo abrir la cámara con índice {camera_index}.")
        sys.exit(1)

    
    grabbed, prev_frame = capture.read()
    if not grabbed:
        print("No se pudo leer frames de la cámara.")
        capture.release()
        sys.exit(1)
    hsv_done = False
    p0 = None
    window_title = "Tracking"
    cv.namedWindow(window_title, cv.WINDOW_NORMAL)
    while True:
        grabbed, actual_frame = capture.read()
        if not grabbed:
            print("Se perdió la señal de la cámara. Terminando grabación.")
            break
        if actual_frame is not None and prev_frame is not None:
            actual_frame_gray = cv.cvtColor(actual_frame, cv.COLOR_BGR2GRAY)
            prev_frame_gray = cv.cvtColor(prev_frame, cv.COLOR_BGR2GRAY)
            if hsv_done == False:
                mask = Object_detection (actual_frame)
                p0 = centroid_calculation(mask)
                if p0 is not None:
                    hsv_done = True
                    # Dibujamos la línea 
                    a, b = p0.ravel() # pasamos de un punto en 3D a 1D a --> X y b --> Y
                    cv.circle(actual_frame, (int(a), int(b)), 5, (0,255,0), -1)
                else:
                    hsv_done = False
            else:
                if p0 is not None:
                    p1, status, err = cv.calcOpticalFlowPyrLK(prev_frame_gray, actual_frame_gray, p0, None)
                    if status is not None and status[0] == 1: # El punto se ha seguido bien
                        p0 = p1 # actualizamos el punto a seguir

                        # Dibujamos la línea 
                        a, b = p0.ravel() # pasamos de un punto en 3D a 1D a --> X y b --> Y
                        cv.circle(actual_frame, (int(a), int(b)), 5, (0,255,0), -1)

                    else: # No se ha seguido bien el punto
                        hsv_done = False
                        p0 = None
                else:
                    hsv_done = False

        prev_frame = actual_frame.copy()
        cv.imshow(window_title, actual_frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:  # 'q' o ESC
            break

    capture.release()
    cv.destroyAllWindows()
    print("Grabación finalizada.")


def Object_detection (frame):

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    h_channel = hsv[:,:,0]
    s_channel = hsv[:,:,1]
    v_channel = hsv[:,:,2]

    h_min = HSV_PARAMS['HSV']['H_MIN']
    h_max = HSV_PARAMS['HSV']['H_MAX']
    s_min = HSV_PARAMS['HSV']['S_MIN']
    s_max = HSV_PARAMS['HSV']['S_MAX']
    v_min = HSV_PARAMS['HSV']['V_MIN']
    v_max = HSV_PARAMS['HSV']['V_MAX']

    mask_h = cv.inRange(h_channel, int(h_min), int(h_max))
    mask_s = cv.inRange(s_channel, int(s_min), int(s_max))
    mask_v = cv.inRange(v_channel, int(v_min), int(v_max))

    mask = cv.bitwise_and(mask_h, mask_s)
    mask = cv.bitwise_and(mask, mask_v)

    _, mask = cv.threshold(mask, 1, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

    return mask

def centroid_calculation(mask):

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    valid_c = None
    p0 = None

    for c in contours:
        area = cv.contourArea(c)
        if MIN_AREA < area < MAX_AREA:
            valid_c = c
            break
    
    if valid_c is not None:
        m = cv.moments(valid_c)
        if m['m00'] != 0: # Evitamos dividir entre 0
            cx = m['m10'] / m['m00']
            cy = m['m01'] / m['m00']
            # Guardamos como punto inicial para optical flow el centroide
            p0 = np.array([[[cx,cy]]], dtype=np.float32) 
    else:
        cx, cy = 0, 0

    return p0


if __name__ == "__main__":
    main()