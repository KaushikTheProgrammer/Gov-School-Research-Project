# USAGE
# python detect_blinks.py --shape-predictor shape_predictor_68_face_landmarks.dat --video blink_detection_demo.mp4
# python detect_blinks.py --shape-predictor shape_predictor_68_face_landmarks.dat

# import the necessary packages
from scipy.spatial import distance as dist
from imutils.video import FileVideoStream
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import statistics
import math



#append new value and return new average 
def rollingAvg(new_Val, list2):
    del list2[0]
    list2.append(new_Val)
    return statistics.mean(list2)


def areaTriangle(a,b,c):
    p=(a+b+c)/2
    area=math.sqrt(p*(p-a)*(p-b)*(p-c))
    return area

def eye_Area(eye):
    A = dist.euclidean(eye[0], eye[1])
    B = dist.euclidean(eye[0], eye[5])
    C = dist.euclidean(eye[1], eye[5])
    D = dist.euclidean(eye[1], eye[2])
    E = dist.euclidean(eye[5], eye[2])
    F = dist.euclidean(eye[5], eye[4])
    G = dist.euclidean(eye[2], eye[4])
    H = dist.euclidean(eye[4], eye[3])
    I = dist.euclidean(eye[2], eye[3])
    areaT= areaTriangle(A,B,C)+ areaTriangle(C,D,E)+areaTriangle(E,F,G)+ areaTriangle(G,I,H)
    return areaT

LIST=[]
array_size=50
eyes_opened_area=0 


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
                help="path to facial landmark predictor")
args = vars(ap.parse_args())
# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])
# grab the indexes of the facial landmarks for the left and
# right eye, respectively
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
# start the video stream thread
print("[INFO] starting video stream thread...")
fileStream = True
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
fileStream = False
time.sleep(1.0)


#calib function to tell what opened eye average is
while True:
    if fileStream and not vs.more():
        break
    frame = vs.read()
    if not isinstance(frame, np.ndarray):  # error ocurred on last frame
        break
    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rects = detector(gray, 0)
    # loop over the face detections
    for rect in rects:
        shape = predictor(gray, rect)
        shape = face_utils.shape_to_np(shape)
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]
        
        #calculate area of right and left eye
        leftArea=eye_Area(leftEye) 
        rightArea=eye_Area(rightEye)
        area=(leftArea + rightArea) / 2.0 #average
        print("Area: " + str(area) )      
        time.sleep(0.5)
        if len(LIST) < array_size :
            LIST.append(area)
        else:
            rollingAvg(area, LIST)
            eyes_opened_area = statistics.mean(LIST)
            print("Average: " + str(eyes_opened_area))
        # compute the convex hull for the left and right eye, then
        # visualize each of the eyes
        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
    # show the frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

print("Calibraion Done")
#####################################################################################################
print("Eyes Opened Average Area: " + str(eyes_opened_area))

#####################################################################################################

# loop over frames from the video stream
while True:
    # if this is a file video stream, then we need to check if
    # there any more frames left in the buffer to process
    if fileStream and not vs.more():
        break

    # grab the frame from the threaded video file stream, resize
    # it, and convert it to grayscale
    # channels)
    frame = vs.read()
    if not isinstance(frame, np.ndarray):  # error ocurred on last frame
        break

    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detect faces in the grayscale frame
    rects = detector(gray, 0)

    # loop over the face detections
    for rect in rects:
        # determine the facial landmarks for the face region, then
        # convert the facial landmark (x, y)-coordinates to a NumPy
        # array
        shape = predictor(gray, rect)
        shape = face_utils.shape_to_np(shape)

        # extract the left and right eye coordinates, then use the
        # coordinates to compute the eye aspect ratio for both eyes
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]
        
        #calculate area of right and left eye
        leftArea=eye_Area(leftEye) 
        rightArea=eye_Area(rightEye)
        area=(leftArea + rightArea) / 2.0 #average
        print("Area: " + str(area) )      

        if len(LIST) < array_size :
            LIST.append(area)
            print("gathering data")
        else:
            rollingAvg(area, LIST)
            AVG_XFrames = statistics.mean(LIST)
            print("Avg last ten Frames")
            print(AVG_XFrames)


        # compute the convex hull for the left and right eye, then
        # visualize each of the eyes
        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

        

# reset the eye frame counter

        # draw the total number of blinks on the frame along with
        # the computed eye aspect ratio for the frame
    # show the frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("w"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()