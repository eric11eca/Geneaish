import time
import os
import sys
import io
import threading
import picamera
import picamera.array
import cv2
import numpy
from adafruit_motorkit import MotorKit
kit = MotorKit()

global running
global Drive
global camera
global processor
running = True

Drive = MotorKit()

# Camera settings
imageWidth  = 320                       # Camera image width
imageHeight = 240                       # Camera image height
frameRate = 3                           # Camera image capture frame rate

# Auto drive settings
autoMaxVel = 1.0                      # Maximum output in automatic mode
autoMinVel = 0.2                      # Minimum output in automatic mode
autoMinArea = 10                      # Smallest target to move towards
autoMaxArea = 10000                   # Largest target to move towards
autoFullSpeedArea = 300               # Target size at which we use the maximum allowed output

def stop_motors():
    global Drive
    Drive.motor1.throttle = 0.0
    Drive.motor2.throttle = 0.0
    Drive.motor3.throttle = 0.0
    Drive.motor4.throttle = 0.0

def forward(vel):
    global Drive
    Drive.motor1.throttle = vel
    Drive.motor2.throttle = vel
    Drive.motor3.throttle = vel
    Drive.motor4.throttle = vel

def backward(vel):
    global Drive
    Drive.motor1.throttle = -vel
    Drive.motor2.throttle = -vel
    Drive.motor3.throttle = -vel
    Drive.motor4.throttle = -vel

def turn_left(vel):
    global Drive
    Drive.motor1.throttle = vel
    Drive.motor2.throttle = 0.0
    Drive.motor3.throttle = vel
    Drive.motor4.throttle = 0.0

def turn_right(vel):
    global Drive
    Drive.motor1.throttle = 0.0
    Drive.motor2.throttle = vel
    Drive.motor3.throttle = 0.0
    Drive.motor4.throttle = vel

def differential(left, right):
    global Drive
    Drive.motor1.throttle = left
    Drive.motor2.throttle = right
    Drive.motor3.throttle = left
    Drive.motor4.throttle = right

# Image stream processing thread
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        while not self.terminated:
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
    
    # Image processing function
    def ProcessImage(self, image):
        image = cv2.medianBlur(image, 5)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV) 
        red = cv2.inRange(image, numpy.array((115, 127, 64)), numpy.array((125, 255, 255)))
        contours,hierarchy = cv2.findContours(red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        foundArea = -1
        foundX = -1
        foundY = -1

        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            cx = x + (w / 2)
            cy = y + (h / 2)
            area = w * h
            if foundArea < area:
                foundArea = area
                foundX = cx
                foundY = cy
        if foundArea > 0:
            ball = [foundX, foundY, foundArea]
        else:
            ball = None
        self.SetSpeedFromBall(ball)

    # Set the motor speed from the ball position
    def SetSpeedFromBall(self, ball):
        global Drive
        driveLeft  = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            area = ball[2]
            if area < autoMinArea:
                print ('Too small / far')
            elif area > autoMaxArea:
                print ('Close enough')
            else:
                if area < autoFullSpeedArea:
                    speed = 1.0
                else:
                    speed = 1.0 / (area / autoFullSpeedArea)
                direction = (x - imageCentreX) / imageCentreX
                if direction < 0.0:
                    # Turn right
                    driveLeft  = speed
                    driveRight = speed * (1.0 + direction)
                else:
                    # Turn left
                    driveLeft  = speed * (1.0 - direction)
                    driveRight = speed
                print ('%.2f, %.2f' % (driveLeft, driveRight))
        else:
            print ('No ball')
        differential(driveLeft, driveRight)

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print ('Start the stream using the video port')
        camera.capture_sequence(0, format='bgr', use_video_port=True)
        print ('Terminating camera processing...')
        processor.terminated = True
        processor.join()
        print ('Processing terminated.')

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()

# Startup sequence
print ('Setup camera')
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate
imageCentreX = imageWidth / 2.0
imageCentreY = imageHeight / 2.0

print ('Setup the stream processing thread')
processor = StreamProcessor()

print ('Wait ...')
time.sleep(2)
captureThread = ImageCapture()

try:
    stop_motors()
    while running:
        time.sleep(1.0)
    stop_motors()
except KeyboardInterrupt:
    print ('User shutdown')
    stop_motors()
except:
    e = sys.exc_info()[0]
    print (e)
    print ('Unexpected error, shutting down!')
    stop_motors()

running = False
captureThread.join()
processor.terminated = True
processor.join()
del camera
stop_motors()
print ('Program terminated.')