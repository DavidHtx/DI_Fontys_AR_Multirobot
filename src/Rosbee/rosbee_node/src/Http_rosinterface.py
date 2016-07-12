import json
from flask import Flask, url_for, request, Response
import rosinterface

robot = rosinterface
app = Flask(__name__)

@app.route("/")
def hello():
    return "Hello World!"

@app.route("/robot/enable")
def robot_enable():
    robot.enable_robot()
    return robot.request_enable_status()

@app.route("/robot/disable")
def robot_disable():
    robot.disable_robot()
    return robot.request_enable_status()

@app.route("/robot/enable_status")
def robot_status():
    return robot.request_enable_status()

@app.route("/robot/alarm")
def robot_alarm():
    return robot.request_alarm()

@app.route("/robot/init")
def robot_init():
    robot.init_robot()

@app.route("/robot/get_gyro")
def robot_get_gyro():
    return robot.get_gyro()

@app.route("/robot/speed/<int:vx>/<int:vth>")
def robot_setspeed(vx, vth):
    return vx + vth
    #robot.set_movesteer()


if __name__ == '__main__':
    app.run(debug=True)