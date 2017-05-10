import numpy as np

def angleToServoValue(thetas, leg_num):
    if leg_num == 1:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 60, mapping(thetas[1],270,90,500,2500) + 50, mapping(thetas[2],270,90,500,2500) + 60]))
    elif leg_num == 2:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) - 60, mapping(thetas[1],270,90,500,2500) + 155, mapping(thetas[2],270,90,500,2500) + 220]))
    elif leg_num == 3:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 80, mapping(thetas[1],270,90,500,2500) + 200, mapping(thetas[2],270,90,500,2500) + 350]))
    elif leg_num == 4:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) - 100, mapping(thetas[1],270,90,500,2500) + 100, mapping(thetas[2],270,90,500,2500) + 180]))
    else:
        return ValueError

    if servoValues[0] > 2400:
        print "ERROR: SERVO 0 OUT OF RANGE - HIGH", servoValues[0], leg_num
        servoValues[0] = 2400
    elif servoValues[0] < 600:
        print "ERROR: SERVO 0 OUT OF RANGE - LOW", servoValues[0], leg_num
        servoValues[0] = 600

    if servoValues[1] > 2400:
        print "ERROR: SERVO 1 OUT OF RANGE - HIGH", servoValues[1], leg_num
        servoValues[1] = 2400
    elif servoValues[1] < 600:
        print "ERROR: SERVO 1 OUT OF RANGE - LOW", servoValues[1], leg_num
        servoValues[1] = 600

    if servoValues[2] > 2400:
        print "ERROR: SERVO 2 OUT OF RANGE - HIGH", servoValues[2], leg_num
        servoValues[2] = 2400
    elif servoValues[2] < 600:
        print "ERROR: SERVO 2 OUT OF RANGE - LOW", servoValues[2], leg_num
        servoValues[2] = 600

    return servoValues


def mapping(value, fromLow, fromHigh, toLow, toHigh):
    return (((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow


def serialSend_All(leg_1_thetas, leg_2_thetas, leg_3_thetas, leg_4_thetas):
    cmd1 = "#0 P%d #1 P%d #2 P%d " % (leg_2_thetas[0], leg_2_thetas[1], leg_2_thetas[2])
    cmd2 = "#12 P%d #13 P%d #14 P%d" % (leg_4_thetas[0], leg_4_thetas[1], leg_4_thetas[2])
    cmd3 = "#16 P%d #17 P%d #19 P%d" % (leg_1_thetas[0], leg_1_thetas[1], leg_1_thetas[2])
    cmd4 = "#28 P%d #30 P%d #31 P%d \r" % (leg_3_thetas[0], leg_3_thetas[1], leg_3_thetas[2])

    final_cmd = " ".join((cmd1, cmd2, cmd3, cmd4))

    return final_cmd


