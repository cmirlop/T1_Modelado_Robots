import sys
import datetime
from pathlib import Path
import numpy as np
import cv2 as cv


def open_camera(camera_index: int) -> cv.VideoCapture:
    capture = cv.VideoCapture(camera_index, cv.CAP_DSHOW)
    if not capture.isOpened():
        # Retry without CAP_DSHOW (cross-platform fallback)
        capture = cv.VideoCapture(camera_index)
    return capture

def main() -> None:
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

    
    grabbed, frame = capture.read()
    if not grabbed:
        print("No se pudo leer frames de la cámara.")
        capture.release()
        sys.exit(1)

    window_title = "Grabando - presiona 'q' para terminar"
    cv.namedWindow(window_title, cv.WINDOW_NORMAL)

    while True:
        grabbed, frame = capture.read()
        if not grabbed:
            print("Se perdió la señal de la cámara. Terminando grabación.")
            break

        cv.imshow(window_title, frame)
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

    mask_h = cv.inRange(h_channel, int(y_min), int(y_max))
    mask_s = cv.inRange(s_channel, int(s_min), int(s_max))
    mask_v = cv.inRange(v_channel, int(v_min), int(v_max))

    mask = cv.bitwise_and(mask_h, mask_s)
    mask = cv.bitwise_and(mask, mask_v)

    ## Operaciones morfológicas [No creo que haga falta]

    
    #mask = np.zeros(frame.shape, dtype=np.uint8)

def square_detection():
    _, binary = cv.threshold(s_channel, LSD["threshold"], 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    lines = cv.createLineSegmentDetector(0).detect(binary)[0]
    # falta aún

if __name__ == "__main__":
    main()