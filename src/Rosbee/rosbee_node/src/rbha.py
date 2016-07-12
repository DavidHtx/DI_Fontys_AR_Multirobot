#!/usr/bin/env python

# swm data module and command handler
# Implemented by HJ Kiela Opteq mechatronics BV Febr 2016
# This module acts as command interface from any application to the smart wheel module and
# holds a class representing the hardware model and status

# import section
#from __future__ import unicode_literals, absolute_import, print_function, division
from __future__ import unicode_literals, print_function, division
import serial
import threading
import time
import comportconfig_support
import logging
import math
import bits

# ---- constants ----------------
logfilename = 'log.txt'


# hardware abstracted data of smart wheel module. Acts as central memory model.as
# receives regular fresh data from update thread
# Data is used to refresh the GUI or user applications

# noinspection PyPep8Naming
class trb():
    proptime = 0  # internal time in us of Propeller controlle since last power up
    enable = False

    pf_actwheelpos = [0, 0]  # actual wheel encoder position in counts
    pf_actwheelvel = [0, 0]  # Rosbee pf calculated velocity from embedded controller
    pf_gyroX = 0  # measured platform rotation from gyro
    pf_gyroY = 0
    pf_gyroZ = 0
    actplatformspeed_m_per_sec = 0
    # vars for speed and rotation calculations for platform
    gyroZ = 0.0  # gyro calculated angle in degree per second
    gyroZrad = 0.0  # Gyro calculated angle in radians per second

    actsteerpos = 0  # calculated steer from difference in wheel velocity
    actplatformspeed = 0  # calculated average platform speed in center
    prev_actwheelpos = [0, 0]  # previous actual wheel position in counts
    actwheelvel = [0.0, 0.0]  # python calculated velocity
    actwheelvelmeterpersec = [0.0, 0.0]  # python calculated velocity
    actrotvel_radians = 0.0  # Calculated rotational velocity in radians per second
    robot_twist = {'speed_x': 0, 'rot_z': 0}

    lasttime = 0  # Previous propeller time in us
    acttime = 0  #
    deltatime = 0  # last delta time

    # Platform constants
    wheel_base_width = 0.0  # meter
    counts_per_rotation = 0.0
    wheel_diameter = 0.0  # meter
    gyro_sensitivity = 0.0  # gyro sensitivity in mdps

    # Calculated platform constants
    countsperpid_to_meterpersec = 0.0  # conversion to wheel velocity in meters/sec
    meterpersec_to_countsperpid = 0.0  # inverted factor

    dir_to_radianspersec = 0.0  # conversion factor for pf rotational increments to radians per sec
    radianspersec_to_dir = 0.0  # coversion radians per sec to platyform rotational increments

    # Control speed and driection to platform in SI units
    setspeed = 0.0  # new speed in m/sec
    setsteer = 0.0  # new rotational speed in radians/sec
    oldsetspeed = 0.0
    oldsetpos = 0.0

    # status word from platform
    status = 0

    # process counters om embedded controller
    pid_cntr = 0  # pid counter in HW module signalling health in control loops
    plc_cntr = 0  # plc counter
    comm_counter = 0
    # process times in embedded controller in sec
    pid_time = 0.0  # PID cycle time
    PIDtime = 0.0  # actual total PID loop time in ms
    pid_leadtime = 0.0  # PID lead time per cycle
    plc_time = 0.0
    comm_time = 0  # Last command response time for command on rosbee
    max_comm_time = 0  # not used on rosbee

    version = ['0', '0', '0']  # firm ware version in HW module. product, version, subversion
    last_cog = 0  # max number of cogs in use
    configfile = ''

    n_adc_channels = 0  # number of adc channels. generally parsed from incoming data
    adclabels = ['1', '2', '3', '4', '5', '6', '7', '8']  # annotation strings for ADC values. Filled from Power Module
    adc_avg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # actual ADC values
    adc_max = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    adc_min = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    newlabels = False  # Semaphore to signal new labels to refresh gui check boxes
    n_status_labels = 0  # number of status labels received
    status_labels = {}  # empty dictionary for status labels format index, label


# ---------- global variable with initialisation if needed -----
rb1 = trb()  # create instance of wheel module
debug = False  # set debugging info  on and off
ser = None

connected = False
serial_free = True  # semaphore for serialport to avoid collisions from various processes

parselist = ['1', '2', '3', '4', '5', '6', '7', '8']  # list with separated string sections

# ======================= functions ======================
# commands for motion
cmd_disable = '$0'
cmd_enable = '$1'
cmd_move = '$2'

# status commands
cmd_get_adc = '$10'
cmd_get_status = '$11'
cmd_conversionfactors = '$12'
cmd_get_position = '$14'
cmd_get_gyro = '$15'

cmd_version = '$29'

cmd_Watchdog_enabled = '$40'
cmd_Watchdog_disabled = '$41'

cmd_get_times = '$58'
cmd_get_counters = '$59'

cmd_get_adc_labels = '$60'
cmd_get_status_labels = '$61'

# reset commands
cmd_reset_minmax = '$13'
cmd_reset = '$8'

# define constants for command recognition of strings from Power module
get_position = 2
get_adc_values = 10
get_status = 11
report_conversion = 12
report_vel_data = 14
report_gyro = 15
get_version = 29
get_times = 58
get_counters = 59
get_adc_labels = 60
get_status_labels = 61


# +++++++++++++++++++++++ ROS command interface +++++++++++++++++++++++++++++++++++
# ------- Robot control commands ----------------------
# Request Reset command
def do_reset():
    send(cmd_reset)
    print('reset' + cmd_reset)


# request robot enable status. Returns true if enabled and false if disabled
def request_enable_status():
    return rb1.enable


# request alarm bit status of robot. Returns true if any alarm set
def request_alarm():
    return bits.testbit2int(rb1.status, 15)


# Alternative enable of robot for do_enable call
def enable_robot():
    send(cmd_enable)
    rb1.enable = True
    print('RB enable' + cmd_enable)


# Alternative disable of robot for do_enable call
def disable_robot():
    send(cmd_disable)
    print('RB disable' + cmd_disable)
    rb1.enable = False


# Enable/disable robot toggle command
def do_enable():  # enable motion
    if (rb1.enable):
        send(cmd_disable)
        print('RB disable' + cmd_disable)
        rb1.enable = False
    else:
        send(cmd_enable)
        rb1.enable = True
        print('RB enable' + cmd_enable)


# Get actual move speed and rotational speed of robot in SI units
# Reports calculated speed x from motor encoders and robot rotation based on either encoders of gyro depending on gyrobased being true
def get_movesteer(gyrobased):
    #rb1.robot_twist['speed_x'] = rb1.actplatformspeed
    rb1.robot_twist['speed_x'] = rb1.actplatformspeed_m_per_sec
    if gyrobased:
        rb1.robot_twist['rot_z'] = rb1.gyroZrad
    else:
        rb1.robot_twist['rot_z'] = rb1.actrotvel_radians
    return rb1.robot_twist


# Move command. Input in SI units. speed in m/s dir in radians per sec
def do_movesteer(speed, rot_z):
    global rb1
    intspeed = int(speed * rb1.meterpersec_to_countsperpid)
    intdir = int(rot_z * rb1.radianspersec_to_dir)
    send(cmd_move + ',' + str(intspeed) + ',' + str(intdir))
    print ("do movesteer speed")
    print (str(intspeed))
    if debug:
        print('cmd_move speed:' + str(speed) + ',' + str(intspeed) + ' dir:' + str(rot_z) + ',' + str(
            intdir) + ' fact ' + str(rb1.meterpersec_to_countsperpid) + ' ' + str(rb1.radianspersec_to_dir))


# Classic move command with pf dependent values: increments per servo cycle
def do_movesteer_int(speed_int, rot_z_int):
    send(cmd_move + ',' + str(speed_int) + ',' + str(rot_z_int))
    if debug:
        print('cmd_move pf units' + ',' + str(speed_int) + ',' + str(rot_z_int))


# +++++++++++ ------------ END of robot control commands --------------------+++++++++++++++

# reset ADC min max values
def do_reset_minmax():
    send(cmd_reset_minmax)
    if debug:
        print('reset minmax' + cmd_reset_minmax)


# Interpret separated string parts
def parse_strings():
    global parselist

    # first string part indication type of message from Power Module
    if parselist[0].isdecimal():
        received_command = int(float(parselist[0]))
        if debug:
            print("command : {0:d}".format(received_command))

            # switch case to detect the kind of packet from wheel Module
        if received_command == get_adc_values:
            handle_get_adc_values()
        if received_command == get_version:
            handle_get_version()
        if received_command == get_counters:
            handle_get_counters()
        if received_command == get_status:
            handle_get_status()
        if received_command == get_times:
            handle_get_times()
        if received_command == get_counters:
            handle_get_counters()
        if received_command == get_adc_labels:
            handle_get_adc_labels()
        # if received_command == get_status_labels:
        #     handle_get_status_labels()
        if received_command == report_conversion:
            handle_report_conversion()
        # if received_command == get_position:
        #     handle_get_position()
        if received_command == report_vel_data:
            handle_get_position()
        if received_command == report_gyro:
            handle_report_gyro()


# -------- routines to further process parse list depending on received response string --------
# expects a data string with: command, n, ch0 avg, ch0 min, ch0, max, ch1..chn
def handle_get_adc_values():  # parse all adc values
    global parselist, rb1
    rb1.n_adc_channels = int(parselist[1])  # get number of channels from data
    if debug:
        print('get ' + str(rb1.n_adc_channels) + ' adc values')
    for index in range(0, rb1.n_adc_channels):
        rb1.adc_avg[index] = float(parselist[index + 2]) / 1000
        rb1.adc_min[index] = float(parselist[index + 2 + rb1.n_adc_channels]) / 1000
        rb1.adc_max[index] = float(parselist[index + 2 + rb1.n_adc_channels * 2]) / 1000
    if debug:
        print('get adc values')


def handle_get_status():  # parse output status
    global parselist, rb1
    rb1.status = int((parselist[1]))
    #    rb1.errors = int(float(parselist[2]))
    #    rb1.UDPerrorCnt = int(float(parselist[3]))
    # print('status '+ wm1.status + ' error : ' + wm1.errors)
    if debug:
        print('Status ' + str(rb1.status))
    if debug:
        print('processed status and errors :')


def handle_get_version():  # parse version
    global parselist, rb1
    rb1.version[0] = parselist[1]
    rb1.version[1] = parselist[2]
    # rb1.version[2] = parselist[3]
    if debug:
        print('got version info')


def handle_report_gyro():  # parse gyro data
    global parselist, rb1
    rb1.pf_gyroX = int(float(parselist[1]))
    rb1.pf_gyroY = int(float(parselist[2]))
    rb1.pf_gyroZ = int(float(parselist[3]))

    rb1.gyroZ = rb1.pf_gyroZ * rb1.gyro_sensitivity / 1000  # Calculate degree/second
    rb1.gyroZrad = math.radians(rb1.gyroZ)  # calculate radians per second
    if debug:
        print('get gyro')


def handle_get_times():  # parse process times
    global parselist, rb1
    rb1.PIDtime = int(float(parselist[1])) / 1000  # comes as ms
    rb1.pid_leadtime = int(float(parselist[2])) / 1000000  # comes as us
    rb1.plc_time = int(float(parselist[3])) / 1000000  # comes as us
    rb1.comm_time = int(float(parselist[4])) / 1000000  # comes as us
    if debug:
        print('get process times')


def handle_get_counters():  # parse process counters
    global parselist, rb1
    rb1.pid_cntr = int(float(parselist[1]))
    rb1.plc_cntr = int(float(parselist[2]))
    rb1.comm_counter = int(float(parselist[3]))
    rb1.last_cog = int(float(parselist[4]))
    if debug:
        print('get cntrs')


def handle_get_adc_labels():  # parse adc labels
    global parselist, rb1
    rb1.n_adc_channels = int(parselist[1])  # get number of channels from data
    print('get ' + str(rb1.n_adc_channels) + ' adc values')
    for index in range(0, rb1.n_adc_channels):
        rb1.adclabels[index] = parselist[2 + index]
        print("label " + rb1.adclabels[index])
    if debug:
        print('got adc labels')


# def handle_get_status_labels():    # parse status and error labels
#     global parselist, rb1
#
#     rb1.n_adc_channels = int(parselist[1])    # get number of channels from data
#     print('get ' +str(rb1. n_status_labels) + ' adc values')
#     for index in range(0, rb1. n_status_labels):
#       rb1.status_labels[int(parselist[0])] = parselist[2+ index]
#     rb1.status_labels[int(parselist[2])] = parselist[3]
#     rb1.status_labels[int(parselist[4])] = parselist[5]
#     rb1.status_labels[int(parselist[6])] = parselist[7]
#     rb1.status_labels[int(parselist[8])] = parselist[9]
#     rb1.status_labels[int(parselist[10])] = parselist[11]
#     rb1.status_labels[int(parselist[12])] = parselist[13]
#     rb1.status_labels[int(parselist[14])] = parselist[15]
#     rb1.status_labels[int(parselist[16])] = parselist[17]
#     if debug:
#         print('got status labels')
#         print(rb1.status_labels)
#
#     rb1.newlabels = True

# Handle packet with conversion info from Rosbee platform and calculate conversion factors
# for counts per PID cycle to velocity in m/s and rotation in Rad per sec and inverse.
def handle_report_conversion():
    global parselist, rb1
    rb1.wheel_base_width = float(parselist[1]) / 1000  # sent in mm converted to m
    rb1.counts_per_rotation = int(parselist[2])
    rb1.wheel_diameter = float(parselist[3]) / 1000  # sent in mm converted to m
    rb1.wheel_circimref = rb1.wheel_diameter * math.pi
    rb1.gyro_sensitivity = float(
        parselist[4]) / 1000  # sent in milli degree per sec, converted to degree per sec per bit
    rb1.pid_time = int(parselist[5]) / 1000  # sent in ms converted to sec

    # calculate conversion factor from counts per pid cycle into meters per second
    rb1.countsperpid_to_meterpersec = rb1.wheel_circimref / (rb1.pid_time * rb1.counts_per_rotation)
    rb1.meterpersec_to_countsperpid = 1 / rb1.countsperpid_to_meterpersec

    counts_per_pf_rev = rb1.counts_per_rotation * rb1.wheel_base_width / rb1.wheel_diameter
    counts_per_radian = counts_per_pf_rev / (2 * math.pi)
    rb1.radianspersec_to_dir = counts_per_radian * rb1.pid_time * 2  # convert dir in counts per servo cycly to m/s into radians
    rb1.dir_to_radianspersec = 1 / rb1.radianspersec_to_dir  # The inverse radians to dir conversion
    if True:  # debug:
        print('rb1.counts_per_rotation ' + str(rb1.counts_per_rotation))
        print('Wheel diam ' + str(rb1.wheel_diameter))
        print('rb1.PIDtime ' + str(rb1.pid_time))
        print('rb1.wheel_circimref ' + str(rb1.wheel_circimref))
        print('rb1.dir_to_radianspersec ' + str(rb1.dir_to_radianspersec) + ' rb1.radianspersec_to_dir ' + str(
            rb1.radianspersec_to_dir))
        print('counts_per_pf_rev ' + str(counts_per_pf_rev) + ' counts_per_radian ' + str(counts_per_radian))
        print('Conversions countsperpid_to_meterpersec ' + str(
            rb1.countsperpid_to_meterpersec) + ' rb1.meterpersec_to_countsperpid ' + str(
            rb1.meterpersec_to_countsperpid))


# END handle_report_conversion()

# parse actual speed and directions and calculate SI units velocities
def handle_get_position():
    global parselist, rb1
    rb1.pf_actwheelpos[0] = int(float(parselist[1]))
    rb1.pf_actwheelpos[1] = int(float(parselist[2]))
    rb1.pf_actwheelvel[0] = int(float(parselist[3]))
    rb1.pf_actwheelvel[1] = int(float(parselist[4]))
    # rb1.gyroZ = int(float(parselist[3]))
    # tempstatusstatus = int(float(parselist[4]))
    # rb1.status = int(float(parselist[4]))
    rb1.proptime = int(float(parselist[5]))
    if debug:
        print('Got new positions. proptime ' + str(rb1.proptime))

    # calculate velocities
    rb1.acttime = rb1.proptime
    rb1.deltatime = rb1.acttime - rb1.lasttime
    if rb1.deltatime > 0:
        # calculate encoder velocity based on travelled distance and time between samples
        rb1.actwheelvel[0] = 10000 * (rb1.prev_actwheelpos[0] - rb1.pf_actwheelpos[0]) / (
            rb1.acttime - rb1.lasttime)  # calc wheel vel
        rb1.actwheelvel[1] = -10000 * (rb1.prev_actwheelpos[1] - rb1.pf_actwheelpos[1]) / (rb1.acttime - rb1.lasttime)

        rb1.prev_actwheelpos[0] = rb1.pf_actwheelpos[0]  # store act into prev
        rb1.prev_actwheelpos[1] = rb1.pf_actwheelpos[1]

        # calculate rotational speed of robot in wheel encoders counts per PID cycle
        rb1.actsteerpos = rb1.actwheelvel[1] - rb1.actwheelvel[0]

        # calculate average platform speed at center of platform in encoder counts per PID cycle
        rb1.actplatformspeed = (rb1.actwheelvel[1] + rb1.actwheelvel[1]) / 2

        # calculate wheel velocity in m/sec
        rb1.actwheelvelmeterpersec[0] = rb1.countsperpid_to_meterpersec * rb1.actwheelvel[0]
        rb1.actwheelvelmeterpersec[1] = rb1.countsperpid_to_meterpersec * rb1.actwheelvel[1]
        rb1.actplatformspeed_m_per_sec = (rb1.actwheelvelmeterpersec[0] + rb1.actwheelvelmeterpersec[
            1]) / 2  # avg forward speed in m/sec
        # calculate platform rotationals speed from wheel speeds in radians per sec
        rb1.actrotvel_radians = (rb1.actwheelvel[1] - rb1.actwheelvel[0]) * rb1.dir_to_radianspersec
        if debug:
            print('rb1.actrotvel_radians ' + str(rb1.actrotvel_radians))

    rb1.lasttime = rb1.acttime  # store time for next cycle
    if debug:
        print('got velocities')


# end: handle_get_actpos_speed()

# ---------- Check and parse incoming string into separated parts --------------------
def separate_string(instr):
    global parselist
    # search for $.
    dollarpos = instr.find("$")
    if debug:
        print(instr)
        print("$ found at: ", dollarpos)
    if dollarpos > -1:
        instr = instr[dollarpos + 1:]  # strip left part of string including dollar sign
        parselist = instr.split(",")
        if debug:
            print('Separated strings: ')
            print(parselist)
        for item in range(len(parselist)):  # strip spaces from string items
            parselist[item] = parselist[item].strip(" ")
        parse_strings()  # interpret data in string parts
    else:
        print("!!! $ char missing in response : " + instr)


# end: separate_string(instr)

# ------- return string with connection info  ----
def get_connection_info():
    connectioninfo = comportconfig_support.connection_info()
    print('connection info : ' + connectioninfo['baudrate'] + ' ' + connectioninfo['comport'])
    return connectioninfo


# end: get_connection

#
# --------------- initialize program and variables --------------------
def init_vars():
    global connected #, logger
    # logging.basicConfig(format='%(asctime)s %(message)s')           # initialize logging
    # logging.basicConfig(filename=logfilename,level=logging.INFO)     # open log file for debug
    # logger = logging.getLogger('start log')

    t = threading.Thread(target=do_update)  # create update thread to read data from wheel module
    t.setDaemon(1)
    t.start()


#    com.init()    # init UDP port module
# end: init_vars()

# initialize program and variables
def init_serial():
    #    global received_command
    #    global connected, logger
    global connected, r, logger
    logging.basicConfig(format='%(asctime)s %(message)s')  # initialize logging
    logging.basicConfig(filename=logfilename, level=logging.INFO)  # open log file for debug
    logger = logging.getLogger('start log')

    connected = False
    r = threading.Thread(target=reader)  # create read thread
    r.setDaemon(1)
    r.start()
    print('init serialport' + ' logging to: ' + logfilename)

    t = threading.Thread(target=do_update)  # create update thread to read data from wheel module
    t.setDaemon(1)
    t.start()


def receive():
    global ser
    data = ser.readline()
    if len(data) > 0:
        print('in raw:' + str(data))
        if (data[0] == 255):
            data[0] = 32
        data_decoded = data.decode('utf-8')  # decode to ascii
        # print('in:' + data_decoded)
        separate_string(data_decoded)  # parse incoming string

# ----------------- Serial port handling routines ---------------------------------
# thread function reading lines from comport
def reader():
    global connected, ser
    # loop forever and copy serial->console
    while 1:
        if (connected):
            receive()



def open_serial():
    global logger, ser, connected
    #init_serial()  # init serial port
    ''' open serial port with timeout '''
    serialconfig = comportconfig_support.Readconfig()
    print(serialconfig)

    ser = serial.Serial(port=serialconfig['comport'], baudrate=serialconfig['baudrate'], parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
                        timeout=0.1)
    connected = True
    #    logger.info('serial port open ' + serialconfig['comport'] + ' ' + serialconfig['baudrate'] )
    print('serial port open ' + serialconfig['comport'] + ' ' + serialconfig['baudrate'])
    #run_once()  # run some commands at the start of communication to get version, labels, etc from power module


def close_serial():
    global ser, connected
    print('close serialport')
    connected = False
    ser.close()  # close port


def send(linput):
    global serial_free
    # while serial_free:        # wait for release of serial port
    #     continue
    outstr = linput + '\r\n'  # add cr lf to string
    serial_free = False
    str = linput + '\r\n'  # add cr lf to string
    # print('ser ' + linput)
    #ser.write(outstr.encode('utf-8'))
    ser.write(outstr)
    print ("sending data")
    print (outstr)
    serial_free = True

    # print("out> " + outstr )


# Check if port is open
def isportopen():
    global connected
    if debug:
        if connected:
            print('connected status True')
        else:
            print('connected status False')
    return connected


# ------------- Send new setpoints if anything changed and enabled ----------------
def sendnewsetpoints():
    if debug:
        print('new steer speed ' + str(rb1.enable))
    if rb1.enable:
        print ("sendnewsetpoints")
        print (rb1.setspeed)
        do_movesteer(rb1.setspeed, rb1.setsteer)
        rb1.oldsetspeed = rb1.setspeed
        rb1.oldsetpos = rb1.setsteer


# ------------- Run once after initialisation -----------------
def run_once():
    if isportopen():
        send(cmd_version)
        send(cmd_get_adc_labels)
        # send(cmd_get_status_labels)
        send(cmd_conversionfactors)
        send(cmd_reset_minmax)
        # send(cmd_Watchdog_enabled)


#        print(' test once')

# ------------- Run periodic functions-----------------
# send periodic command to power module for status update
def do_update():
    wm_refreshtime = 0.1  # sec

    time.sleep(wm_refreshtime)  # wait until run once commands have been completed
    while 1:
        if debug:
            print('Thread update RB status')

        if isportopen():  # request data from embedded controller at regular intervals
            send(cmd_get_adc)  # get adc values
            send(cmd_get_status)  # get status and errors
            send(cmd_get_counters)  # get process counters
            send(cmd_get_times)  # get process times
            send(cmd_get_position)  # get wheel encoder positions
            send(cmd_get_gyro)  # get gyro data
            sendnewsetpoints()  # send new setpoints to wheels if port open
        #            print('test')
        time.sleep(wm_refreshtime)  # wait refresh time