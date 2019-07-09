from keras.models import load_model
import numpy as np
import cv2


model = load_model("finalmodel.h5")

cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()


    newFrame = np.array(cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 64))).reshape(-1,64,64,1)
    prediction = np.rint(model.predict(newFrame))
    
    if prediction == []

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()