from ultralytics import YOLO
import cv2
import numpy as np
from mss import mss
import time
import threading
import win32api
import win32con
from win32api import GetAsyncKeyState
from gui import *
import ctypes

model = YOLO("desktop/runs/detect/train13/weights/best.pt")
sct = mss()
monitor_sizeX = 1920
monitor_sizeY = 1080

# move the mouse by dx, dy
def move_mouse(dx, dy, steps, sleep_time):
    for _ in range(steps):
        win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, int(dx), int(dy), 0, 0)
        time.sleep(sleep_time)

# returns the bbox coords closest to the center of the screen
def bbox_closest_to_center(results, bounding_box):
    window_center = bounding_box['width'] / 2, bounding_box['height'] / 2
    min_distance = float('inf')
    center = None  # Initialize center to None

    for result in results:
        for bbox in result.boxes:
            xyxy_array = bbox.xyxy.cpu().numpy()
            x1, y1, x2, y2 = xyxy_array[0]

            # find the center of the bbox, 14% from the top
            bbox_center = ((x1 + x2)/2, y1 + (y2 - y1) * yOffset_slider.get())
            distance = ((bbox_center[0] - window_center[0]) ** 2 + (bbox_center[1] - window_center[1]) ** 2) ** 0.5

            # bbox is new target if closest to center & valid conf score
            if distance < min_distance and bbox.conf >= confidence_slider.get() / 100:  # Check the confidence score
                min_distance = distance
                center = bbox_center

    return center

# run tkinter thread
def start_tkinter_loop():
    root.mainloop()
threading.Thread(target=start_tkinter_loop).start()

while True:
    root.update()
    window_sizeX = inference_windowX.get()
    window_sizeY = inference_windowY.get()
    bounding_box = {'top': int((monitor_sizeY / 2) - (window_sizeY / 2)) , 'left': int((monitor_sizeX / 2) - (window_sizeX / 2)), 'width': window_sizeX, 'height': window_sizeY}
    screenshot = sct.grab(bounding_box)  # capture the screen
    img = np.array(screenshot)  # convert the screenshot to a numpy array
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)  # convert the image from RGBA to RGB

    # show inference window if checkbox is checked
    results = model.predict(img, show=show.get())
    if not show.get():
        try:
            cv2.destroyWindow('image0.jpg')
        except cv2.error:
            # the window is already closed
            pass

    # target is bbox closets to crosshair
    target = bbox_closest_to_center(results, bounding_box)

    if target is not None:
        windowCenterX, windowCenterY = bounding_box['width'] / 2, bounding_box['height'] / 2

        # delta to move the mouse by
        dx = ((round(target[0] - windowCenterX) * speedScaleX_slider.get()) / smoothing_slider.get()) * accelX_slider.get()
        dy = ((round(target[1] - windowCenterY) * speedScaleY_slider.get()) / smoothing_slider.get()) * accelY_slider.get()

        # check if delta is within fov
        if (abs(dx) > fovX_slider.get() or abs(dy) > fovY_slider.get()):
            continue

        # "rage mode" if target is far from crosshair
        # TODO: continue working on this
        if preset.get() == 'rage':
            if (abs(round(target[0] - windowCenterX)) > 20):
                dx = ((round(target[0] - windowCenterX) * speedScaleX_slider.get() * 1.1) / smoothing_slider.get()) * accelX_slider.get() * 1.1

            if (abs(round(target[1] - windowCenterY)) > 20):
                dy = ((round(target[1] - windowCenterY) * speedScaleY_slider.get() * 1.1) / smoothing_slider.get()) * accelY_slider.get() * 1.1


        # holding down aim hotkey
        if len(aimHotkey.get()) == 1:
            # speed this up?
            if GetAsyncKeyState(ctypes.windll.user32.VkKeyScanW(ord(aimHotkey.get()))):
                # new thread for mouse movement
                threading.Thread(target=move_mouse, args=(dx, dy, smoothing_slider.get(), sleepTimer_slider.get())).start()
        

    # listen for exit hotkey
    if len(exitHotkey.get()) == 1:
        # user is interacting with text box
        if exitHotkey.focus_get() == exitHotkey:
            pass
        elif GetAsyncKeyState(ctypes.windll.user32.VkKeyScanW(ord(exitHotkey.get()))):
            break

cv2.destroyAllWindows()