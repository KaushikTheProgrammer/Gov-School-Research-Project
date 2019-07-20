# import the necessary packages
# python MyVersion.py --shape-predictor shape_predictor_68_face_landmarks.dat 
# --image images/example_01.jpg

from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2
import time
from imutils.video import FileVideoStream
from imutils.video import VideoStream
import statistics
 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
args = vars(ap.parse_args())
 
# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])
 
# # load the input image, resize it, and convert it to grayscale
# image = cv2.imread(args["image"])
# image = imutils.resize(image, width=500)
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def rollingAvg(new_Val, list2):
    del list2[0]
    list2.append(new_Val)
    return statistics.mean(list2)

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
fileStream = True
vs = VideoStream(src=0).start()
fileStream = False
time.sleep(1.0) 

def isHypnotized(pupilCoords, currentPos, checkTime):
    print(len(pupilCoords))
    if len(pupilCoords) == checkTime * 30:
        average_pos = sum(pupilCoords) / checkTime * 30
        if pupil_x - currentPos <= posThreshold:
            return True
        del pupilCoords[0]
    pupilCoords.append(currentPos)
    return False


X_List=[0,0,0,0,0]
Y_List=[0]
W_List=[0]
H_List=[0]
X_Avg_List=[0]
i=0
while i<5 :
    X_List.append(0)
    Y_List.append(0)
    W_List.append(0)
    H_List.append(0)
    i+=1

all_x = []
all_y = []
checkTime = 30 # seconds
posThreshold = 5

while True:
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
    rects = detector(gray, 1)

    # loop over the face detections
    for (i, rect) in enumerate(rects):
        # determine the facial landmarks for the face region, then
        # convert the landmark (x, y)-coordinates to a NumPy array
        shape = predictor(gray, rect)
        shape = face_utils.shape_to_np(shape)
        # loop over the face parts individually
        # clone the original image so we can draw on it, then
        # display the name of the face part on the image
        clone = frame.copy()
        cv2.putText(clone, "right_eye", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2)
        # loop over the subset of facial landmarks, drawing the
        # specific face part
        for (x, y) in shape[36:42]:
            cv2.circle(clone, (x, y), 1, (0, 0, 255), -1)
    # extract the ROI of the face region as a separate image
    #("right_eye", (36, 42)),
    #("left_eye", (42, 48)),
        (x, y, w, h) = cv2.boundingRect(np.array([shape[36:42]]))
        x-=20
        y-=20
        w+=40
        h+=40
        roi = frame[y:y + h, x:x + w]
        roi = imutils.resize(roi, width=250, inter=cv2.INTER_CUBIC)
    
        # show the particular face part
        cv2.imshow("ROI", roi)
        cv2.imshow("Image", clone)

        #pupil movemnt code
        rows, cols, _ = roi.shape
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.GaussianBlur(gray_roi, (7, 7), 0)

        _, threshold = cv2.threshold(gray_roi, 25, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)

            pupil_x = x + w / 2
            pupil_y = y + h / 2

            x_hypnotized = isHypnotized(all_x, pupil_x, 5)
            y_hypnotized = isHypnotized(all_y, pupil_y, 5)
            print("x", x_hypnotized, "y", y_hypnotized)

            if x_hypnotized and y_hypnotized:
                print("HIGHWAY HYPNOSIS DETECTED")


            #cv2.drawContours(roi, [cnt], -1, (0, 0, 255), 3)
            cv2.rectangle(roi, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.line(roi, (x + int(w/2), 0), (x + int(w/2), rows), (0, 255, 0), 2)
            cv2.line(roi, (0, y + int(h/2)), (cols, y + int(h/2)), (0, 255, 0), 2)
            break

        cv2.imshow("Threshold", threshold)
        cv2.imshow("gray roi", gray_roi)
        cv2.imshow("Roi", roi)
        
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("w"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()