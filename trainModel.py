import cv2
import numpy as np
from keras.models import Sequential
from keras.layers import *
from keras.optimizers import *
from keras.utils.vis_utils import plot_model
from keras.models import load_model
import matplotlib.pyplot as plt


trainingData = np.load("trainingdata")
testingData = np.load("testingdata")

trainingImages = np.array([i[0] for i in trainingData]).reshape(-1,64,64,1)
testingImages = np.array([i[0] for i in testingData]).reshape(-1,64,64,1)

trainingLabels = np.array([i[1] for i in trainingData])
testingLabels = np.array([i[1] for i in testingData])

model = Sequential()
model.add(Conv2D(filters=32, kernel_size=5, strides=1, padding='same', activation='relu', input_shape=[64, 64, 1]))
model.add(MaxPool2D(pool_size=5, padding='same'))

model.add(Conv2D(filters=50, kernel_size=5, strides=1, padding='same', activation='relu'))
model.add(MaxPool2D(pool_size=5, padding='same'))

model.add(Conv2D(filters=80, kernel_size=5, strides=1, padding='same', activation='relu'))
model.add(MaxPool2D(pool_size=5, padding='same'))

model.add(Dropout(0.25))
model.add(Flatten())
model.add(Dense(512, activation='relu'))
model.add(Dropout(rate=0.5))
model.add(Dense(2, activation='softmax'))
model.summary()
model.compile(optimizer=Adam(lr=1e-3), loss="categorical_crossentropy", metrics=['accuracy'])
model.fit(x=trainingImages, y=trainingLabels, validation_split = 0.33, epochs=50, batch_size=10)


print("************************************************")
plot_model(model, to_file='model_plot.png', show_shapes=True, show_layer_names=True)

model.save("finalmodel.h5")