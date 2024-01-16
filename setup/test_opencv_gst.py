import cv2

gst = "nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' ! queue ! nvvidconv"
cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
breakpoint()
