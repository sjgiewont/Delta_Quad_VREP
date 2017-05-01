from inverseKinematics import *
from piecewiseMotion import *
from motion import *
from inverseKinematics import loadAnfisNetwork
import timeit
import sys
import time
# import vrep
from subprocess import call
import socket
from threading import Thread
import pickle


def callPython3VREPControl():
    # call the Python3 script that will send commands to VREP
    exit_code = call("py py3_vrep_control\main.py", shell=True)
    return

def main():
    global client_id

    # start a thread call the Python3 script that willl receive commands from this script and control VREP
    vrep_control_thread = Thread(target=callPython3VREPControl, args=())
    vrep_control_thread.start()

    # give the Python3 script some time to establish connection to VREP
    time.sleep(2)
    print "Ready to send commands to Python3 Script"

    HOST = '127.0.0.1'
    PORT = 10000
    global vrep_socket
    vrep_socket = socket.socket()
    vrep_socket.connect((HOST, PORT))

    # print "Start VREP Server"
    # client_id = startVREP()

    print "Open ANFIS"
    anf = loadAnfisNetwork()
    print "ANFIS OPEN"

    # jump(-150, -300, 2)

    step_angle = 90
    step_length = 100
    step_height = 0
    step_precision = 250
    # while True:
    #     print step_angle
        # balanceTest(-100, -260, 150)

    curr_pos = walk_dir(step_length, step_height, step_angle, 1, step_precision)
    print curr_pos
    homePos(curr_pos)
        # time.sleep(.25)
        # step_angle -= 5
        # print "Done Moving"

    # thread to continually check for user input


# walking gait, can walk in a direction and particular number of steps.
# The precision is the number incremental steps between the start and stop motions
def walk_dir(step_length, step_height, degrees, step_num, precision):
    #calculate the walking trajectory of one step
    # walking_trajectory = piecewiseMotion(step_length, step_height, degrees, precision)
    walking_trajectory_R = piecewiseMotion_3(step_length, step_height, degrees, -200, precision)
    walking_trajectory_L = piecewiseMotion_3(step_length, step_height, degrees - 180, -200, precision)

    # initialize the index of each leg, offset all of them
    FL_leg_index = 0
    FR_leg_index = precision / 4
    HL_leg_index = 2 * (precision / 4)
    HR_leg_index = 3 * (precision / 4)

    # FL_leg_index = 2 * (precision / 4)
    # FR_leg_index = 3 * (precision / 4)
    # HL_leg_index = 0
    # HR_leg_index = (precision / 4)

    leg_index = [FL_leg_index, FR_leg_index, HL_leg_index, HR_leg_index]

    steps = 0

    # step a certain amount of times
    while(steps < step_num):
        # print "Front Left:", walking_trajectory[FL_leg_index]
        # print "Front Right:", walking_trajectory[FR_leg_index]
        # print "Hind Left:", walking_trajectory[HL_leg_index]
        # print "Hind Right:", walking_trajectory[HR_leg_index]

        # move_to_pos(walking_trajectory[FL_leg_index], 1)

        moveToPos(walking_trajectory_R, walking_trajectory_L, leg_index)


        # FL_leg_index += 1
        # FR_leg_index += 1
        # HL_leg_index += 1
        # HR_leg_index += 1

        leg_index = [x+1 for x in leg_index]

        # if reached index limit, loop back and restart index count
        if leg_index[0] >= precision:
            leg_index[0] = 0
            steps += 1                  # keep track of the number of steps taken

        if leg_index[1] >= precision:
            leg_index[1] = 0

        if leg_index[2] >= precision:
            leg_index[2] = 0

        if leg_index[3] >= precision:
            leg_index[3] = 0

    return [walking_trajectory_R[leg_index[0]], walking_trajectory_R[leg_index[1]], walking_trajectory_L[leg_index[2]], walking_trajectory_L[leg_index[3]]]


def jump(high_pt, low_pt, precision):
    z_trajectory = np.linspace(high_pt, low_pt, precision)
    x_trajectory = np.zeros((precision, 1))
    y_trajectory = np.zeros((precision, 1))

    final_trajectory = []

    # create matrix of all positions along trajectory
    for i in range(precision):
        final_trajectory.append([x_trajectory[i][0], y_trajectory[i][0], z_trajectory[i]])

    FL_leg_index = 0
    FR_leg_index = 0
    HL_leg_index = 0
    HR_leg_index = 0
    leg_index = [FL_leg_index, FR_leg_index, HL_leg_index, HR_leg_index]

    decrease_index = 0

    while(True):
        print final_trajectory[leg_index[1]]
        leg1 = inverseKinematics(final_trajectory[leg_index[1]])
        leg2 = inverseKinematics(final_trajectory[leg_index[0]])
        leg3 = inverseKinematics(final_trajectory[leg_index[2]])
        leg4 = inverseKinematics(final_trajectory[leg_index[3]])
        msg = '{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f}'.format(leg1[0], leg1[1], leg1[2], leg2[0], leg2[1], leg2[2], leg3[0], leg3[1], leg3[2], leg4[0], leg4[1], leg4[2])
        vrep_socket.send(msg)
        time.sleep(.5)

        if leg_index[0] == precision - 1:
            decrease_index = 1

        if leg_index[0] == 0:
            decrease_index = 0

        if decrease_index == 0:
            leg_index = [x + 1 for x in leg_index]
        elif decrease_index == 1:
            leg_index = [x - 1 for x in leg_index]


def balanceTest(high_pt, low_pt, precision):
    z_trajectory = np.linspace(high_pt, low_pt, precision)
    x_trajectory = np.zeros((precision, 1))
    y_trajectory = np.zeros((precision, 1))

    final_trajectory = []

    # create matrix of all positions along trajectory
    for i in range(precision):
        final_trajectory.append([x_trajectory[i][0], y_trajectory[i][0], z_trajectory[i]])

    FL_leg_index = 0
    FR_leg_index = 0
    HL_leg_index = 0
    HR_leg_index = 0
    leg_index = [FL_leg_index, FR_leg_index, HL_leg_index, HR_leg_index]

    decrease_index = 0

    while(True):
        leg1 = inverseKinematics(final_trajectory[leg_index[0]])
        leg2 = inverseKinematics([-50, -25, low_pt])
        leg3 = inverseKinematics([0, 25, low_pt])
        leg4 = inverseKinematics([0, 25, low_pt])
        msg = '{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f}'.format(leg1[0], leg1[1], leg1[2], leg2[0], leg2[1], leg2[2], leg3[0], leg3[1], leg3[2], leg4[0], leg4[1], leg4[2])
        vrep_socket.send(msg)

        if leg_index[0] == precision - 1:
            decrease_index = 1

        if leg_index[0] == 0:
            decrease_index = 0

        if decrease_index == 0:
            leg_index = [x + 1 for x in leg_index]
        elif decrease_index == 1:
            leg_index = [x - 1 for x in leg_index]

# parabola function between 2 points
def step_to(leg, curr_pos, new_pos, step_height):
    global leg1_q

    # start a timer for benchmarking purposes
    start = timeit.default_timer()

    # convert python array to numpy array to streamline math
    start_pt = np.array([curr_pos[0], curr_pos[1], curr_pos[2]])
    end_pt = np.array([new_pos[0], new_pos[1], new_pos[2]])

    # generate numpy array of numbers 0 to 1, to be used in parametric equations
    t = np.linspace(0, 1, 20)

    # determine the mean of the start and end points
    mid = (start_pt + end_pt) / float(2)

    # determine the actual mid point with the step_height factored in
    mid_pt = np.array([mid[0], mid[1], mid[2] + step_height])

    # create numpy matrix of all x, y z points
    x_pts = np.matrix([[curr_pos[0]], [mid_pt[0]], [new_pos[0]]])
    y_pts = np.matrix([[curr_pos[1]], [mid_pt[1]], [new_pos[1]]])
    z_pts = np.matrix([[curr_pos[2]], [mid_pt[2]], [new_pos[2]]])

    # generate the standard inverse matrix to solve parabolic constraints
    A_1 = np.matrix([[2, -4, 2], [-3, 4, -1], [1, 0, 0]])

    # solve all coefficients by multiplying inverse with points
    x_coeff = A_1 * x_pts
    y_coeff = A_1 * y_pts
    z_coeff = A_1 * z_pts

    # plug in solved coefficents to determine parametric equation for each axis
    x = x_coeff.item(0)*t*t + x_coeff.item(1)*t + x_coeff.item(2)
    y = y_coeff.item(0)*t*t + y_coeff.item(1)*t + y_coeff.item(2)
    z = z_coeff.item(0)*t*t + z_coeff.item(1)*t + z_coeff.item(2)

    pos = []

    # create matrix of all positions along trajectory
    for i in range(len(x)):
        pos.append([x[i], y[i], z[i]])

    # add each row of the matrix to the queue
    map(leg.put, pos)

    # stop the timer to for benchmarking purposes
    stop = timeit.default_timer()
    print "The Time:", stop - start

    # return the final position
    return new_pos


def add_leg1(first_pos, new_pos):
    #args first_pos and new_pos must be arrays

    curr_pos1 = step_to(leg1_q, first_pos, new_pos, 5)
    return


def moveToPos(trajectory_R, trajectory_L, index):
    print trajectory_R[index[0]]
    leg1 = inverseKinematics(trajectory_R[index[0]])
    leg2 = inverseKinematics(trajectory_R[index[1]])
    leg3 = inverseKinematics(trajectory_L[index[2]])
    leg4 = inverseKinematics(trajectory_L[index[3]])

    # leg1 = [180, 180, 180]
    # leg2 = [0.5, 0.5, 0.5]
    # leg3 = [0.5, 0.5, 0.5]
    # leg4 = [0.5, 0.5, 0.5]
    msg = '{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f}'.format(leg1[0], leg1[1], leg1[2], leg2[0], leg2[1], leg2[2], leg3[0], leg3[1], leg3[2], leg4[0], leg4[1], leg4[2])
    #time.sleep(0.05)
    vrep_socket.send(msg)


def homePos(curr_pos):
    home_pos = [0, 0, -200]
    step_height = 75
    precision = 150

    leg_1_pos = curr_pos[0]
    leg2 = curr_pos[1]
    leg3 = curr_pos[2]
    leg4 = curr_pos[3]

    parabola_motion = parabolaStep(leg_1_pos, home_pos, step_height, -200, precision)

    for index in range(0, precision):
        leg1 = inverseKinematics(parabola_motion[index])
        leg2 = inverseKinematics(curr_pos[1])
        leg3 = inverseKinematics(curr_pos[2])
        leg4 = inverseKinematics(curr_pos[3])
        msg = '{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f},{:07.3f}'.format(leg1[0], leg1[1], leg1[2], leg2[0], leg2[1], leg2[2], leg3[0], leg3[1], leg3[2], leg4[0], leg4[1], leg4[2])
        print msg
        vrep_socket.send(msg)


if __name__ == "__main__":
    main()
