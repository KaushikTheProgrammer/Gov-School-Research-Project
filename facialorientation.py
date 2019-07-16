# import the necessary packages
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2
import math
import serial
import time


def calcDist(point1, point2):
    return math.sqrt( math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))

 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
args = vars(ap.parse_args())

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

# smartCar = serial.Serial('/dev/tty.HC-05-DevB', 38400) # Establish the connection on a specific port)

cap = cv2.VideoCapture(0)

# while True:
#     # smartCar.write("Hello SmartCar")
#     time.sleep(0.1)

while True:
    ret, frame = cap.read()

    if ret:
        # load the input image, resize it, and convert it to grayscale
        image = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # detect faces in the grayscale image
        rects = detector(gray, 1)

        maxArea = 0
        maxRect = None

        # Find face with max area (supposed to be closest to the camera)
        for (i, rect) in enumerate(rects):
            if rect.area() > maxArea:
                maxArea = rect.area()
                maxRect = rect
        
        if len(rects) > 0:
            shape = predictor(gray, maxRect)
            shape = face_utils.shape_to_np(shape)

            # convert dlib's rectangle to a OpenCV-style bounding box
            # [i.e., (x, y, w, h)], then draw the face bounding box
            (x, y, w, h) = face_utils.rect_to_bb(maxRect)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
            # show the face number
            cv2.putText(image, "Driver", (x - 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
            # loop over the (x, y)-coordinates for the facial landmarks
            # and draw them on the image
            for (x, y) in shape:
                cv2.circle(image, (x, y), 1, (0, 0, 255), -1)
            
            leftDistance = calcDist(shape[15], shape[55])
            rightDistance = calcDist(shape[3], shape[49])

            rightLeftRatio = rightDistance / leftDistance

            if rightLeftRatio < 0.2:
                print("you are looking right")
            elif rightLeftRatio > 1.6:
                print("you are looking to the left")
            else:
                print("you are looking straight")

        # show the output image with the face detections + facial landmarks
        cv2.imshow("Output", image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# # loop over the face detections
        # for (i, rect) in enumerate(rects):
        #     # determine the facial landmarks for the face region, then
        #     # convert the facial landmark (x, y)-coordinates to a NumPy
        #     # array
        #     shape = predictor(gray, rect)
        #     shape = face_utils.shape_to_np(shape)
        
        #     # convert dlib's rectangle to a OpenCV-style bounding box
        #     # [i.e., (x, y, w, h)], then draw the face bounding box
        #     (x, y, w, h) = face_utils.rect_to_bb(rect)
        #     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        #     # show the face number
        #     cv2.putText(image, "Face #{}".format(i + 1), (x - 10, y - 10),
        #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
        #     # loop over the (x, y)-coordinates for the facial landmarks
        #     # and draw them on the image
        #     for (x, y) in shape:
        #         cv2.circle(image, (x, y), 1, (0, 0, 255), -1)
            
        #     leftDistance = calcDist(shape[15], shape[55])
        #     rightDistance = calcDist(shape[3], shape[49])

        #     rightLeftRatio = rightDistance / leftDistance

        #     if rightLeftRatio < 0.2:
        #         print("you are looking right")
        #     elif rightLeftRatio > 1.6:
        #         print("you are looking to the left")
        #     else:
        #         print("you are looking straight")


