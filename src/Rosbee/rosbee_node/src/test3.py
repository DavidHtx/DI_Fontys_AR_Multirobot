import rbha
import bits
import threading


class error_state(object):
    def __init__(self):
        self.CurrentError = False
        self.EncoderError = False
        self.VoltageLow = False
        self.Voltage3v3Low = False


# Opens serial port, initializes hardware abstraction and executes run once commands
def init_robot():
    rbha.open_serial()
    rbha.enable_robot()



# Get actual move speed and rotational speed of robot in SI units
# Reports calculated speed x from motor encoders and robot rotation based on either encoders of gyro depending on gyrobased being true

def get_movesteer(gyro):
    if gyro:
        vel = rbha.get_movesteer(gyro)
    else:
        vel = rbha.get_movesteer(None)
    dictToList = []
    for key, value in vel.iteritems():
        temp = [value]
        dictToList.append(temp[0])
    if dictToList[0] <= 0:
        dictToList[0] = 0.0

    vel = (dictToList[1], 0, dictToList[0])
    return vel


# Move command. Input in SI units. speed in m/s dir in radians per sec
def do_movesteer(vx, vth):
    """Drive the robot.
@param cmd_vel: velocity setpoint = (vx, vy, vth)
  vx:  linear velocity along the x-axis (m/s)
  vy:  linear velocity along the y-axis (m/s)
  vth: angular velocity about the z-axis (rad/s), also called yaw
returns current state, including velocity, as measured by robot using its encoders
"""
    rbha.do_movesteer_int(vx, vth)


def get_gyro():
    return rbha.rb1.gyroZrad


# Call this routine from the ROS spin loop to uodate the data from Rosbee to ROS
def get_update_from_rosbee():
    if rbha.isportopen():  # request data from embedded controller
        rbha.do_update()


# Disable of robot
def disable_robot():
    rbha.disable_robot()


# Enable of robot
def enable_robot():
    rbha.enable_robot()


# request robot enable status. Returns true if enabled and false if disabled
def request_enable_status():
    return rbha.request_enable_status()


# request alarm bit status of robot. Returns true if any alarm set
def request_alarm():
    return rbha.request_alarm()


def get_avg_current():
    return rbha.rb1.adc_avg[0]


def get_min_current():
    return rbha.rb1.adc_min[0]


def get_max_current():
    return rbha.rb1.adc_max[0]


def get_avg_voltageIn():
    return rbha.rb1.adc_avg[1]


def get_min_voltageIn():
    return rbha.rb1.adc_min[1]


def get_max_voltageIn():
    return rbha.rb1.adc_max[1]


def get_avg_3voltage():
    return rbha.rb1.adc_avg[2]


def get_min_3voltage():
    return rbha.rb1.adc_min[2]


def get_max_3voltage():
    return rbha.rb1.adc_max[2]


def get_avg_5voltage():
    return rbha.rb1.adc_avg[3]


def get_min_5voltage():
    return rbha.rb1.adc_min[3]


def get_max_5voltage():
    return rbha.rb1.adc_max[3]


def get_status():
    return rbha.rb1.version


def get_connection_info():
    return rbha.get_connection_info()


def is_connected():
    return rbha.connected


def get_status():
    error = error_state()
    error.CurrentError = bits.testbit2int(rbha.rb1.status, 11)  # Current error on any axis
    error.EncoderError = bits.testbit2int(rbha.rb1.status, 12)  # MAE encoder alarm
    error.VoltageLow = bits.testbit2int(rbha.rb1.status, 13)  # Main supply volatege too low
    error.Voltage3v3Low = bits.testbit2int(rbha.rb1.status, 14)  # Logic voltage 3V3 too low
    return error


# Stop robot communication
def close_robot():
    rbha.close_serial()
