import numpy as np
import cv2
import time
import os
from random import shuffle
from tqdm import tqdm


# Takes numPictures from the webcam and saves each one to savePath, specifying imageType in the name of the file
def takePictures(numPictures, savePath, imageType, **kwargs):
    cap = cv2.VideoCapture(0)
    
    time.sleep(0.05)
    for i in range(numPictures):
        ret, frame = cap.read()
        
        if ret:
            path = savePath + imageType + str(i) + ".jpg"
            cv2.imwrite(path, frame)
            print(i, "file saved")
            time.sleep(0.05)

    cap.release()


def collectData(savePath, **kwargs):
    takePictures(numPictures=45, savePath=savePath, imageType="focused")
    takePictures(numPictures=15, savePath=savePath, imageType="distracted")

def oneHotLabel(imgName):
    if imgName.find("focused") != -1:
        label = np.array([1, 0])
    else:
        label = np.array([0, 1])
    return label

def saveData(dataLoc, dataType, **kwargs):
    arr = []
    for img in tqdm(os.listdir(dataLoc)):
        path = os.path.join(dataLoc, img)
        if path.find("focused") != -1 or path.find("lookingaway") != -1:
            newImg = cv2.resize(cv2.imread(path, cv2.IMREAD_GRAYSCALE), (64, 64))
            arr.append([np.array(newImg), oneHotLabel(img)])
    shuffle(arr)
    np.save(dataType, arr)
    print(dataType, "array saved")


print("Collecting Data")
collectData("dataset/")
print("Data Collection Complete")
print("Getting training and testing")
saveData(dataLoc="dataset/", dataType="training")
saveData(dataLoc="dataset/", dataType="testing")