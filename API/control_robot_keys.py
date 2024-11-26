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

delay(1)

errorCode, left_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

errorCode, camera_handle = sim.simxGetObjectHandle(
    clientID, 'cam1', sim.simx_opmode_oneshot_wait)
delay(1)

returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, camera_handle, 0, sim.simx_opmode_streaming)
delay(1)

try:
    # Make sure to start with an initial call to simxGetVisionSensorImage in buffer mode
    returnCode, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera_handle, 0, sim.simx_opmode_buffer)
    delay(1)

    while (1):
        print('Entered the loop')  # Debug line to ensure the loop is entered
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera_handle, 0, sim.simx_opmode_buffer)
        if returnCode == sim.simx_return_ok and resolution and image:
            try:
                # Convert the image from a flat list to a numpy array, correcting negative values
                im = np.array(image, dtype=np.int16)
                im[im < 0] += 256  # Fix any negative values by wrapping around to fit uint8 range
                im = im.astype(np.uint8)

                # Reshape the image to its correct resolution (height, width, channels)
                im = im.reshape((resolution[1], resolution[0], 3))

                # Flip the image to correct orientation
                im = cv2.flip(im, 0)

                # Convert RGB to BGR for OpenCV display
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

                # Display the image
                cv2.imshow("data", im)

                # Set motor velocities
                errorCode = sim.simxSetJointTargetVelocity(
                    clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
                errorCode = sim.simxSetJointTargetVelocity(
                    clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

            except ValueError as ve:
                print(f'Value error while processing image: {ve}')
        else:
            print('Failed to get image from vision sensor')

        # Wait for key press
        com = cv2.waitKey(1)
        print(f'Key pressed: {com}')  # Print each key pressed in the terminal
        if (com == ord('q')):
            break
        elif (com == ord('w')):
            lSpeed = 0.8
            rSpeed = 0.8
        elif (com == ord('a')):
            lSpeed = -0.4
            rSpeed = 0.8
        elif (com == ord('d')):
            lSpeed = 0.8
            rSpeed = -0.4
        elif (com == ord('s')):
            lSpeed = -0.8
            rSpeed = -0.8
        else:
            lSpeed = 0
            rSpeed = 0

    cv2.destroyAllWindows()
except Exception as e:
    print(f'An error occurred: {e}')
    cv2.destroyAllWindows()
