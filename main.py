from ultralytics import YOLO
import cv2
import numpy as np
from mss import mss
import time
import threading
import win32api
import win32con
from win32api import GetAsyncKeyState
import tkinter as tk
import ctypes

model = YOLO("desktop/runs/detect/train8/weights/best.pt")
# offset is size of monitor / 2 - size of window / 2
bounding_box = {'top': 390, 'left': 810, 'width': 300, 'height': 300}

sct = mss()
root = tk.Tk()
root.geometry("610x350")  # Set the width to 500 pixels and the height to 300 pixels
root.title("GUI")

# TODO: break tkinter into a separate file
speedScaleX_slider = tk.Scale(root, from_=1, to=4, resolution=0.1, orient=tk.HORIZONTAL, label='speedScaleX', length=300)
speedScaleX_slider.set(2)
speedScaleX_slider.grid(row=0, column=0, sticky='w')
speedScaleY_slider = tk.Scale(root, from_=1, to=4, resolution=0.1, orient=tk.HORIZONTAL, label='speedScaleY', length=300)
speedScaleY_slider.set(4)
speedScaleY_slider.grid(row=1, column=0, sticky='w')
smoothing_slider = tk.Scale(root, from_=1, to=8, orient=tk.HORIZONTAL, label='steps', length=300)
smoothing_slider.set(3)
smoothing_slider.grid(row=2, column=0, sticky='w')
accelX_slider = tk.Scale(root, from_=0, to=1, resolution=0.05, orient=tk.HORIZONTAL, label='accelX', length=300)
accelX_slider.set(0.5)
accelX_slider.grid(row=3, column=0, sticky='w')
accelY_slider = tk.Scale(root, from_=0, to=1, resolution=0.05, orient=tk.HORIZONTAL, label='accelY', length=300)
accelY_slider.set(0.2)
accelY_slider.grid(row=4, column=0, sticky='w')
fovX_slider = tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, label='fovX', length=300)
fovX_slider.set(50)
fovX_slider.grid(row=0, column=1, sticky='w')
fovY_slider = tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, label='fovY', length=300)
fovY_slider.set(30)
fovY_slider.grid(row=1, column=1, sticky='w')
confidence_slider = tk.Scale(root, from_=50, to=100, orient=tk.HORIZONTAL, label='confidence', length=300)
confidence_slider.set(70)
confidence_slider.grid(row=2, column=1, sticky='w')
yOffset_slider = tk.Scale(root, from_=0, to=1, resolution=0.01, orient=tk.HORIZONTAL, label='Y Offset from top of BB', length=300)
yOffset_slider.set(.14)
yOffset_slider.grid(row=3, column=1, sticky='w')
sleepTimer_slider = tk.Scale(root, from_=0, to=0.1, resolution=0.0005, orient=tk.HORIZONTAL, label='mouse sleep timer', length=300)
sleepTimer_slider.set(.005)
sleepTimer_slider.grid(row=4, column=1, sticky='w')
aimHotkeyLabel = tk.Label(root, text="Aim hotkey:")
aimHotkeyLabel.grid(row=5, column=0, sticky='w')
aimHotkey = tk.Entry(root, width=20)
aimHotkey.insert(0, 'c')
aimHotkey.grid(row=5, column=0, sticky='e')
exitHotkeyLabel = tk.Label(root, text="Exit program hotkey:")
exitHotkeyLabel.grid(row=6, column=0, sticky='w')
exitHotkey = tk.Entry(root, width=20)
exitHotkey.insert(0, 'v')
exitHotkey.grid(row=6, column=0, sticky='e')

# move the mouse by dx, dy
def move_mouse(dx, dy, steps, sleep_time):
    for _ in range(steps):
        win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, int(dx), int(dy), 0, 0)
        time.sleep(sleep_time)

# returns the bbox coords closest to the center of the screen
def bbox_closest_to_center(results):
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
    screenshot = sct.grab(bounding_box)  # capture the screen
    img = np.array(screenshot)  # convert the screenshot to a numpy array
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)  # convert the image from RGBA to RGB

    # run inference
    show = False # change to true if you want to see the bounding boxes
    results = model.predict(img, show=show)

    # target is bbox closets to crosshair
    target = bbox_closest_to_center(results)

    if target is not None:
        windowCenterX, windowCenterY = bounding_box['width'] / 2, bounding_box['height'] / 2

        # delta to move the mouse by
        dx = ((round(target[0] - windowCenterX) * speedScaleX_slider.get()) / smoothing_slider.get()) * accelX_slider.get()
        dy = ((round(target[1] - windowCenterY) * speedScaleY_slider.get()) / smoothing_slider.get()) * accelY_slider.get()

        # check if delta is within fov
        if (abs(dx) > fovX_slider.get() or abs(dy) > fovY_slider.get()):
            continue

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