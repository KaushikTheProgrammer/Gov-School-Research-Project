from random import shuffle
from tqdm import tqdm
from sklearn.model_selection import train_test_split
import os
import numpy as np
import cv2
import time
import time


# Takes numPictures from the webcam and saves each one to savePath, specifying imageType in the name of the file
def takePictures(numPictures, savePath, imageType, **kwargs):
    cap = cv2.VideoCapture(0)
    
    time.sleep(0.05)

    if imageType == "focused":
        os.system("say Look Forward")
        time.sleep(5)
        for i in range(numPictures):
            ret, frame = cap.read()
            
            if ret:
                path = savePath + imageType + str(i) + ".jpg"
                cv2.imwrite(path, frame)
                print(i, "file saved")
                time.sleep(0.05)
        cap.release()
    else:
        os.system("say Look left")
        time.sleep(5)
        imageCounter = 0
        while imageCounter < int(numPictures / 3):
            ret, frame = cap.read()
            if ret:
                imageCounter += 1
                path = savePath + imageType + str(imageCounter) + ".jpg"
                cv2.imwrite(path, frame)
                print(imageCounter, "file saved")
                time.sleep(0.05)
        
        os.system("say Look down")
        time.sleep(5)
        while imageCounter < int(numPictures / 3) * 2:
            ret, frame = cap.read()
            if ret:
                imageCounter += 1
                path = savePath + imageType + str(imageCounter) + ".jpg"
                cv2.imwrite(path, frame)
                print(imageCounter, "file saved")
                time.sleep(0.05)
        
        os.system("say Look right")
        time.sleep(5)
        while imageCounter < numPictures:
            ret, frame = cap.read()
            if ret:
                imageCounter += 1
                path = savePath + imageType + str(imageCounter) + ".jpg"
                cv2.imwrite(path, frame)
                print(imageCounter, "file saved")
                time.sleep(0.05)


def collectData(savePath, **kwargs):
    os.system("say Starting Data Collection")
    takePictures(numPictures=25, savePath="dataset/", imageType="focused")
    takePictures(numPictures=45, savePath="dataset/", imageType="distracted")
    os.system("say image collection complete")

def oneHotLabel(imgName):
    if imgName.find("focused") != -1:
        label = np.array([1, 0])
    else:
        label = np.array([0, 1])
    return label

def getDataset(dataLoc, **kwargs):
    arr = []
    for img in tqdm(os.listdir(dataLoc)):
        path = os.path.join(dataLoc, img)
        if path.find("focused") != -1 or path.find("distracted") != -1:
            newImg = cv2.resize(cv2.imread(path, cv2.IMREAD_GRAYSCALE), (64, 64))
            arr.append([np.array(newImg), oneHotLabel(img)])
    return arr

def getValues(inputArr, firstIndex, **kwargs):
    finalArr = []
    for val in inputArr:
        if firstIndex:
            finalArr.append(val[0])
        else:
            finalArr.append(val[1])
    return finalArr



# print("Collecting Data")
# # collectData("dataset/")
# print("Data Collection Complete")
print("Getting training and testing")
dataset = getDataset(dataLoc="finaldataset/")

print(dataset[0][0].shape)

images = getValues(inputArr=dataset, firstIndex=True)
labels = getValues(inputArr=dataset, firstIndex=False)

Image_train, Image_test, label_train, label_test = train_test_split(images, labels, test_size=0.2)
np.save("xtrain", Image_train)
np.save("xtest", Image_test)
np.save("ytrain", label_train)
np.save("ytest", label_test)