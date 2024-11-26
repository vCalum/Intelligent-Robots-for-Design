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

errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

errorCode, camera_handle = sim.simxGetObjectHandle(clientID, 'cam1', sim.simx_opmode_oneshot_wait)
delay(1)

returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_streaming)
delay(1)

try:
    #initial call for simxGetVisionSensorImage buffer mode
    returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_buffer)
    delay(1)

    while (1):
        print('Entered the loop')  #ensure the loop is entered
        returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0, sim.simx_opmode_buffer)
        #checks returned code for succesful image retrieval 
        #ensure res and img data are valid before continuing
        #provided repo code could not handle invalid data
        if returnCode == sim.simx_return_ok and resolution and image:
            try:
                im = np.array(image, dtype=np.int16)                    #img is flat list, convert to array, int16 handles negative values, avoids "Python integer -71 out of bounds for uint8" error
                im[im < 0] += 256                                       #fixnegative values by wrapping around to fit uint8 range
                im = im.astype(np.uint8)                                #unit8 still needed for proper img representation
                im = im.reshape((resolution[1], resolution[0], 3))      #resolution (height, width, channels)
                im = cv2.flip(im, 0)                                    #flip for erientation
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)                #RGB -> BGR for OpenCV display
                cv2.imshow("data", im)

                #set motor velocities
                errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
                errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

            except ValueError as ve:
                print(f'Value error while processing image: {ve}')      #check to handle errors for degugging
        else:
            print('Failed to get image from vision sensor') 

        com = cv2.waitKey(1)
        print(f'Key pressed: {com}')  #print each key press, debugging
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
    #final error handle
    print(f'An error occurred: {e}')
    cv2.destroyAllWindows()
