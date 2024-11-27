import pickle
from keras.models import load_model
import sim
from time import sleep as delay
import numpy as np
import cv2
import sys

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

lSpeed = 0
rSpeed = 0

if (clientID != -1):
    print('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

# Get handles for the motors
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
if errorCode != sim.simx_return_ok:
    print("Failed to get handle for left motor.")
    sys.exit(1)

errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
if errorCode != sim.simx_return_ok:
    print("Failed to get handle for right motor.")
    sys.exit(1)

# Get handle for the camera
errorCode, camera_handle = sim.simxGetObjectHandle(clientID, 'cam1', sim.simx_opmode_oneshot_wait)
if errorCode != sim.simx_return_ok:
    print("Failed to get handle for camera.")
    sys.exit(1)
delay(2)  # Wait for 2 seconds to ensure the camera is properly initialized

# Start streaming from the vision sensor
print("Attempting to start vision sensor streaming...")
returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_streaming)
delay(0.5)  # Give it a moment to start streaming

# Attempt to get initial image in buffer mode to confirm streaming
returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_buffer)
if returnCode != sim.simx_return_ok:
    print(f"Failed to start streaming from vision sensor. Return Code: {returnCode}")
    sys.exit(1)

model = load_model('../model_creation/Models/model-0.9922680258750916.h5')

# Recompile with metrics
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

dict_file = open("../model_creation/data/ai_car.pkl", "rb")
category_dict = pickle.load(dict_file)

try:
    while True:
        com = cv2.waitKey(1)
        if com == ord('q'):
            print('Exit command received. Exiting loop...')
            break
        print('Entered the loop')  # Ensure the loop is entered
        returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_buffer)
        # Ensure successful image retrieval
        print(f"Return Code: {returnCode}, Resolution: {resolution}, Image Length: {len(image) if image else 'None'}")
        if returnCode != sim.simx_return_ok or not resolution or not image:
            print("Failed to retrieve image from vision sensor, retrying...")
            continue
        
        try:
            im = np.array(image, dtype=np.int16)                    # img is flat list, convert to array, int16 handles negative values, avoids "Python integer -71 out of bounds for uint8" error
            im[im < 0] += 256                                       # fix negative values by wrapping around to fit uint8 range
            im = im.astype(np.uint8)                                # unit8 still needed for proper img representation
            im = im.reshape((resolution[1], resolution[0], 3))      # resolution (height, width, channels)
            im = cv2.flip(im, 0)                                    # flip for orientation
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)                # RGB -> BGR for OpenCV display
            cv2.imshow("data", im)
            cv2.waitKey(1)  # Update window without waiting for user input

            # Preprocess image for model prediction
            test_img = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            test_img = cv2.resize(test_img, (50, 50)) / 255.0
            test_img = test_img.reshape(1, 50, 50, 1)  # Use 3 channels if the model expects RGB

            # Make prediction
            results = model.predict(test_img)
            label = np.argmax(results, axis=1)[0]
            acc = int(np.max(results, axis=1)[0] * 100)
            print(f"Moving : {category_dict[label]} with {acc}% accuracy.")

            # Set motor velocities based on prediction
            if label == 0:
                lSpeed = 2.5
                rSpeed = 2.5
            elif label == 1:
                lSpeed = -1.5
                rSpeed = 2.5
            elif label == 2:
                lSpeed = 2.5
                rSpeed = -1.5
            else:
                lSpeed = 0.2  # Default movement speed to keep moving forward if the label is invalid
                rSpeed = 0.2

            # Set motor velocities in CoppeliaSim
            errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
            errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

        except ValueError as ve:
            print(f'Value error while processing image: {ve}')      # check to handle errors for debugging
            continue

        delay(0.2)  # Small delay to ensure consistent data retrieval

    cv2.destroyAllWindows()
    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
except Exception as e:
    # final error handle
    print(f'An error occurred: {e}')
    cv2.destroyAllWindows()
