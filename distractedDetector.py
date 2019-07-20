#  RUN WITH: python3 distractedDetector.py --shape-predictor shape_predictor_68_face_landmarks.dat

#  import the necessary packages
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
import time
import serial

#FACE MOVMNT
FACEcounter=0 

#SLEEPY
EYE_AR_THRESH = 0.3
COUNTER = 0   # initialize the frame counters
fps = 30   #frames per second of web cam
firstMaxTime=2 #2 seconds is the max time someone can have their eyes closed
secondMaxTime=5 #after 3 seconds of no response, send second signal


#PERCLOS
#found during calibration
#distances between each eye
initial_baseEYE=[10.441412393625166, 8.23523155338404, 9.811970201585618, 9.956303889709309, 13.767666806987764, 10.030946622353888, 9.570837733193123, 9.065747155170719, 11.207296001343682]
initial_closedEYE=[8.119510199272261, 7.3718767715719995, 3.620601821885923, 7.580515527672495, 8.796532032026711, 7.780692704426537, 3.601086073116023, 8.277990430815217, 8.291176531622051]
baseEYE=[0,0,0,0,0,0,0,0,0]
baseDis=26.319103035695438
LIST=[] #used for avaerage perclos calc over 3 mins
array_size=10     #average calculated over last X array values
preR=False 
postR=False 

stateEYE=0 
statePERCLOS=0
stateFACE=0
finalState=0

#modify measured EYE distance values 
def adjustEYE(baseEYE, coef):
    i = 0
    while i < len(baseEYE):
        baseEYE[i]=baseEYE[i]*coef
        i=1+i
    return baseEYE

# Recalculate the new average
def rollingAvg(new_Val, list2):
    del list2[0]
    list2.append(new_Val)
    return statistics.mean(list2)


def areaTriangle(a,b,c):
    a=abs(a)
    b=abs(b)
    c=abs(c)
    p=(a+b+c)/2
    area=math.sqrt(p*(p-a)*(p-b)*(p-c))
    return area

#adjusted area regardless of distance away from camera. Returned adjusted area of the eye opened
def eye_Area_Adjusted(eye, num):
    
    coef=dist.euclidean(eye[0], eye[3])/baseDis  #currentDistance/baseDistance
    #modify distance values in the list 
    x=0    
    while x<9 :
        if num== 0:
            baseEYE[x]=initial_baseEYE[x]
        else :
            baseEYE[x]=initial_closedEYE[x]
        x+=1

    i = 0
    while i < len(baseEYE):
        baseEYE[i]*=coef
        i=1+i
    
    leftTriangle = areaTriangle(baseEYE[0], baseEYE[1], baseEYE[2])
    topTriangle = areaTriangle(baseEYE[2], baseEYE[3], baseEYE[4])
    bottomTriangle = areaTriangle(baseEYE[4], baseEYE[5], baseEYE[6])
    rightTriangle = areaTriangle(baseEYE[6], baseEYE[7], baseEYE[8])

    #base eye is adjusted
    areaT = leftTriangle + topTriangle + bottomTriangle + rightTriangle
    return areaT #returns area of eye opened - but adjusted to new distance 

#regular area given eye object
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

#percent of eye closed in each frame
def PERCLOS (avg, open, closed):
    perOpen=(avg-closed)/(open-closed)
    return (1-perOpen)

def LIST_Val (perclos_val):
    if perclos_val>=0.8:  #if eyes more than 80 percent closed, counts as 1
        return 1 
    else:       
        return 0

def isSleepy(perclos):
    if perclos>0.15:
        return True
    else:
        return False

def eye_aspect_ratio(eye):
    # compute the euclidean distances between the two sets of
    # vertical eye landmarks (x, y)-coordinates
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])

    # compute the euclidean distance between the horizontal
    # eye landmark (x, y)-coordinates
    C = dist.euclidean(eye[0], eye[3])

    # compute the eye aspect ratio
    ear = (A + B) / (2.0 * C)

    # return the eye aspect ratio
    return ear

def calcDist(point1, point2):
    return math.sqrt( math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))

# def findClosestFace(rects):

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
fileStream = False
time.sleep(1.0)

screenWidth = 600
screenHeight = (screenWidth / 1920) * 1080

smartCar = serial.Serial('/dev/tty.HC-05-DevB', 115200, timeout=0.1) # Establish the connection on a specific port)
time.sleep(2)


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
    
    
    frame = imutils.resize(frame, width=600)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detect faces in the grayscale frame
    rects = detector(gray, 0)

    maxRect = dlib.rectangle(1, 1, 50, 50)
    for rect in rects:
        if rect.area() > maxRect.area():
            maxRect = rect
    
    rightLeftRatio = 0

    if len(rects) == 0:
        rightLeftRatio = 10
    else:
        ##############################################################
        # determine the facial landmarks for the face region, then
        # convert the facial landmark (x, y)-coordinates to a NumPy
        # array
        shape = predictor(gray, maxRect)
        shape = face_utils.shape_to_np(shape)
        ##############################################################
        
        # convert dlib's rectangle to a OpenCV-style bounding box
        # [i.e., (x, y, w, h)], then draw the face bounding box
        (x, y, w, h) = face_utils.rect_to_bb(maxRect)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # show the face number
        # I CHANGED this line
        # cv2.putText(image, "Face #{}".format(i + 1), (x - 10, y - 10),
        cv2.putText(frame, "Face #{}".format(1), (x - 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("x", x, "y", y)
                    
            # loop over the (x, y)-coordinates for the facial landmarks
            # and draw them on the image
        for (x, y) in shape:
            cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
            
        leftDistance = calcDist(shape[14], shape[55])
        rightDistance = calcDist(shape[4], shape[49])

        rightLeftRatio = rightDistance / leftDistance

        ########################################
        ########################################

        # extract the left and right eye coordinates, then use the
        # coordinates to compute the eye aspect ratio for both eyes
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]

        #SLEEPY
        leftEAR = eye_aspect_ratio(leftEye)
        rightEAR = eye_aspect_ratio(rightEye)
        
        # average the eye aspect ratio together for both eyes
        ear = (leftEAR + rightEAR) / 2.0
        if ear < EYE_AR_THRESH:
            COUNTER += 1  #increment number of frames when eyes are closed 
            if COUNTER>fps*firstMaxTime: # eyes closed longer than five seconds
                stateEYE=2 
                if COUNTER>fps*secondMaxTime:
                    stateEYE=3
        else: # otherwise, the eye aspect ratio is not below the blink threshold
            COUNTER = 0
            stateEYE=1

        #PERCLOS
        #calculate area of right and left eye
        leftArea=eye_Area(leftEye) 
        rightArea=eye_Area(rightEye)
        area=(leftArea + rightArea)/2
        
        #find average area of eyes opened - and adjusted
        openedRight=eye_Area_Adjusted(rightEye, 0)
        openedLeft=eye_Area_Adjusted(leftEye, 0)
        avgOpened=(openedRight + openedLeft)/2
        
        #find average area of eyes opened - and adjusted
        closedLeft=eye_Area_Adjusted(rightEye, 1)
        closedRight=eye_Area_Adjusted(leftEye, 1)
        avgClosed=(closedLeft+closedRight)/2
        perclos=PERCLOS(area, avgOpened, avgClosed)  #returns float percent closed value
        
        
        if len(LIST) < array_size :  #modify array_size to take average of last 5400 frames = 3 mins required
            LIST.append(LIST_Val(perclos))
        else:
            rollingAvg(LIST_Val(perclos), LIST)
            AVG_XFrames = statistics.mean(LIST)
            print(AVG_XFrames)
            cv2.putText(frame, "PERCLOS: " + str(AVG_XFrames), ((int(screenWidth*0.725), 300)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if isSleepy(AVG_XFrames):
                postR=True
                statePERCLOS=3
            else:
                postR=False
                statePERCLOS=1
            if preR != postR:
                preR=postR
            
        finalState=max(stateEYE, statePERCLOS, stateFACE)
        # compute the convex hull for the left and right eye, then
        # visualize each of the eyes
        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

        cv2.putText(frame, "EAR: {:.2f}".format(ear), (int(screenWidth*0.8), int(screenHeight * 0.1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "Drowsy?: " + str(postR), (10, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    if rightLeftRatio < 0.55:
        FACEcounter+=1
        cv2.putText(frame, "Head Turn Right   ", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    elif rightLeftRatio > 1.3:
        FACEcounter += 1
        cv2.putText(frame, "Head Turn Left    ", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    elif rightLeftRatio == 10:
        FACEcounter += 1
        cv2.putText(frame, "Distracted     ", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    else:
        FACEcounter=0
        cv2.putText(frame, "Head Turn Straight", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        stateFACE=1
    if FACEcounter>60:
        stateFACE=2
        if FACEcounter>150:
            stateFACE=3
    
    finalState = max(stateFACE, stateEYE, statePERCLOS)

    print(str(finalState).encode())

    # smartCar.write(str(finalState).encode())

    # show the frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("w"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
# smartCar.close()