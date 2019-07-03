import numpy as np
import cv2
import time

def createLookingAwayDataset():
    
    cap = cv2.VideoCapture(0)

    print("get ready")
    ret, frame = cap.read()

    time.sleep(3)

    print("starting")
    for i in range(100):
        ret, frame = cap.read()
        
        if ret:
            path = "training/lookingaway" + str(i) + ".jpg"
            print("path is", path)
            cv2.imwrite(path, frame)
            print(i, "file saved")
            time.sleep(0.05)
    cap.release()

def createFocusedDataset():
    
    cap = cv2.VideoCapture(0)

    print("get ready")
    ret, frame = cap.read()

    time.sleep(3)

    print("starting")
    for i in range(25):
        ret, frame = cap.read()
        
        if ret:
            path = "testing/focused" + str(i) + ".jpg"
            print("path is", path)
            cv2.imwrite(path, frame)
            print(i, "file saved")
            time.sleep(0.01)
    cap.release()

def oneHotLabel(imgName):
    if imgName.find("focused") != -1:
        label = np.array([1, 0])
    else:
        label = np.array([0, 1])
    return label

def labelDatasets():



