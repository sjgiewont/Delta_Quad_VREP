# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
    # simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.



import sys
import socket
from threading import Thread
import numpy as np
import pickle

MAX_LENGTH = 95

def recieve_socket_commands(clientsocket, clientID):
  angle = 4.5
  while 1:
    # receive the commands here
    buf = clientsocket.recv(MAX_LENGTH)
    buf_string = buf.decode()
    print(buf_string)
    buf_array = buf_string.split(",", 11)
    print(buf_array)
    # print(int(buf_array[1]) + int(buf_array[0]))
    if buf == '':
        return #client terminated connection
    if buf:
        print('Moving')
        errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_1, np.radians(float(buf_array[0]) - 180), vrep.simx_opmode_oneshot_wait)
        errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_2, np.radians(float(buf_array[1]) - 180), vrep.simx_opmode_oneshot_wait)
        errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_3, np.radians(float(buf_array[2]) - 180), vrep.simx_opmode_oneshot_wait)

        # errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_1, 0.2*numpy.sin(angle) + 0.6, vrep.simx_opmode_oneshot_wait)
        # errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_2, 0.2*numpy.sin(angle) + 0.6, vrep.simx_opmode_oneshot_wait)
        # errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_3, 0.5*numpy.sin(angle) + 0.6, vrep.simx_opmode_oneshot_wait)
        # angle = angle + 0.1
        # print(angle)
        # print(0.5*numpy.sin(angle))
        print('Not Moving')
    # print(buf)

# setup a socket that will recieve commands from Python2 code
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# define port and host address
PORT = 10000
HOST = '127.0.0.1'

serversocket.bind((HOST, PORT))
serversocket.listen(10)

# start initializiton of VREP API
# make sure VREP simulation is started
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Do some tests with the API calls
    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(1)

    # errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    # errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
    # errorCode, pioneer_pos = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)

    errorCode, delta_arm_joint_1 = vrep.simxGetObjectHandle(clientID, 'hip_1_joint', vrep.simx_opmode_oneshot_wait)
    errorCode, delta_arm_joint_2 = vrep.simxGetObjectHandle(clientID, 'hip_2_joint', vrep.simx_opmode_oneshot_wait)
    errorCode, delta_arm_joint_3 = vrep.simxGetObjectHandle(clientID, 'hip_3_joint', vrep.simx_opmode_oneshot_wait)
    print(errorCode)

    if errorCode == -1:
        print('Can not find left or right motor')
        sys.exit()

        errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_1, 0, vrep.simx_opmode_oneshot_wait)

    # errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 1, vrep.simx_opmode_oneshot_wait)
    # errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.5, vrep.simx_opmode_oneshot_wait)

    time.sleep(0.1)

    startTime = time.time()
    # while time.time() - startTime < 1:
    #     # returnCode, data = vrep.simxGetObjectPosition (clientID, pioneer_pos, -1, vrep.simx_opmode_streaming)  # Try to retrieve the streamed data
    #
    #     if returnCode==vrep.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    #         print ('Pioneer Position is on: ',data)

    # errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_oneshot_wait)
    # errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_oneshot_wait)
    # errorCode = vrep.simxSetJointPosition(clientID, delta_arm_joint_1, 45, vrep.simx_opmode_oneshot_wait)


    # start a thread to listen to incomming socket commands from Python2 code
    while 1:
        print('Start Accept connections')
        # accept connections from outside
        (clientsocket, address) = serversocket.accept()
        print('Accept connections')

        # start a thread to continuously check the for commands sent by Python2 script
        ct = Thread(target=recieve_socket_commands, args=(clientsocket,clientID))
        ct.run()

    # Now send some data to V-REP in a non-blocking fashion:
    # vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    # vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    # vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

