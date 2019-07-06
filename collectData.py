import numpy as np
import cv2
import time
import os
from random import shuffle
from tqdm import tqdm
from keras.models import Sequential
from keras.layers import *
from keras.optimizers import *
from keras.utils.vis_utils import plot_model
from keras.models import load_model
import matplotlib.pyplot as plt

def collectAllData():
    
    cap = cv2.VideoCapture(0)

    print("Look at the screen")
    ret, frame = cap.read()

    time.sleep(3)

    print("taking focused pictures")
    for i in range(30):
        ret, frame = cap.read()
        
        if ret:
            path = "dataset/focused" + str(i) + ".jpg"
            cv2.imwrite(path, frame)
            print(i, "file saved")
            time.sleep(0.05)
    
    print("taking looking away pictures")
    time.sleep(5)
    
    for i in range(30):
        ret, frame = cap.read()
        
        if ret:
            path = "dataset/lookingaway" + str(i) + ".jpg"
            cv2.imwrite(path, frame)
            print(i, "file saved")
            time.sleep(0.05)
    cap.release()

collectAllData()


def oneHotLabel(imgName):
    if imgName.find("focused") != -1:
        label = np.array([1, 0])
    else:
        label = np.array([0, 1])
    return label

def getTraining():
    trainImages = []
    for img in tqdm(os.listdir("training")):
        path = os.path.join("training", img)
        if path.find("focused") != -1 or path.find("lookingaway") != -1:
            newImg = cv2.resize(cv2.imread(path, cv2.IMREAD_GRAYSCALE), (64, 64))
            trainImages.append([np.array(newImg), oneHotLabel(img)])
    shuffle(trainImages)
    return trainImages

def getTesting():
    testImages = []
    for img in tqdm(os.listdir("testing")):
        path = os.path.join("testing", img)
        if path.find("focused") != -1 or path.find("lookingaway") != -1:
            newImg = cv2.resize(cv2.imread(path, cv2.IMREAD_GRAYSCALE), (64, 64))
            testImages.append([np.array(newImg), oneHotLabel(img)])
    shuffle(testImages)
    return testImages






# trainingData = getTraining()
# testingData = getTesting()

# trainingImages = np.array([i[0] for i in trainingData]).reshape(-1,64,64,1)
# testingImages = np.array([i[0] for i in testingData]).reshape(-1,64,64,1)

# trainingLabels = np.array([i[1] for i in trainingData])
# testingLabels = np.array([i[1] for i in testingData])

# model = Sequential()
# model.add(Conv2D(filters=32, kernel_size=5, strides=1, padding='same', activation='relu', input_shape=[64, 64, 1]))
# model.add(MaxPool2D(pool_size=5, padding='same'))

# model.add(Conv2D(filters=50, kernel_size=5, strides=1, padding='same', activation='relu'))
# model.add(MaxPool2D(pool_size=5, padding='same'))

# model.add(Conv2D(filters=80, kernel_size=5, strides=1, padding='same', activation='relu'))
# model.add(MaxPool2D(pool_size=5, padding='same'))

# model.add(Dropout(0.25))
# model.add(Flatten())
# model.add(Dense(512, activation='relu'))
# model.add(Dropout(rate=0.5))
# model.add(Dense(2, activation='softmax'))
# model.summary()
# model.compile(optimizer=Adam(lr=1e-3), loss="categorical_crossentropy", metrics=['accuracy'])
# model.fit(x=trainingImages, y=trainingLabels, validation_split = 0.33, epochs=50, batch_size=10)


# print("************************************************")
# plot_model(model, to_file='model_plot.png', show_shapes=True, show_layer_names=True)

# model.save("finalmodel.h5")

# cap = cv2.VideoCapture(0)

# while True:
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     if ret:
        
#         # Our operations on the frame come here
#         newFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         # Display the resulting frame
#         cv2.imshow('frame',newFrame)

#         # newFrame = np.array(cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 64))).reshape(-1,64,64,1)
#         # prediction = np.rint(model.predict(newFrame))
#         # print(prediction)

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

# model = load_model("finalmodel.h5")

# cap = cv2.VideoCapture(0)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     # Our operations on the frame come here
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     newFrame = np.array(cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 64))).reshape(-1,64,64,1)
#     prediction = np.rint(model.predict(newFrame))
#     print(prediction)

#     # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()