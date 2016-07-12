#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from math import sin, cos
from covariances import \
    ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2
import rosinterface



PACKAGE = 'rosbee_node'  # this package name
NAME = 'robot_node'  # this node name

class RobotNode(object):
    def __init__(self, name):
        self.robot = rosinterface
        self.robot.init_robot()
        rospy.init_node(name, anonymous=True)
        self.req_cmd_vel = None
        self.robot.enable_robot()
        self._pos2d = Pose2D()
        self.diag_msg = DiagnosticArray()
        self._init_params()
        self._init_pubsub()
        self.spin()

    def _init_params(self):

        # node general
        self.update_rate = rospy.get_param('~update_rate', 10)
        self.verbose = rospy.get_param('~verbose', True)

        # fake serial connection to a robot
        self.fake_connection = rospy.get_param('~fake_connection', False)

        # fake ideal status respons from the robot
        self.fake_respons = rospy.get_param('~fake_respons', False)

        # serial connection parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.connection_timeout = rospy.Duration(rospy.get_param('~connection_timeout', 60))

        # cmd_vel
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
        self.speedLimit = rospy.get_param('~speed_limit', 2)
        # odom: correction factors from calibration
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)

        # tf
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

        # print
        if self.fake_connection:
            rospy.loginfo("fake robot connection")
        else:
            rospy.loginfo("connect to real robot")
            rospy.loginfo("serial port: %s" % (self.port))
            rospy.loginfo("baudrate: %s" % (self.baudrate))

        if self.fake_respons:
            rospy.loginfo("fake robot respons")

        rospy.loginfo("update_rate: %s" % (self.update_rate))

        if self.publish_tf:
            rospy.loginfo("publish tf")
        else:
            rospy.loginfo("no tf published")

    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel)
        self.pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf.TransformBroadcaster()

    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and \
                        msg.angular.z != 0.0 and \
                        abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        # Limit maximum yaw to avoid saturating the gyro
        if self.max_abs_yaw_vel is not None and \
                        self.max_abs_yaw_vel > 0.0 and \
                        msg.angular.z != 0.0 and \
                        abs(msg.angular.z) > self.max_abs_yaw_vel:
            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel

            # Convert twist to velocity setpoint
        # Assume robot drives in 2-dimensional world
        self.req_cmd_vel = msg.linear.x, msg.linear.y, msg.angular.z
        # self.robot.drive(msg.linear.x, msg.angular.z)

    def average_vel(self, old_vel, current_vel):
        avg_vel = tuple((float(x) + float(y)) / 2 for x, y in zip(old_vel, current_vel))
        return avg_vel


    def compute_imu(self, velocities, last_time, current_time, imu):
        dt = (current_time - last_time).to_sec()
        vx, vy, vth = velocities
        imu.header.stamp = current_time
        imu.linear_acceleration.x = 0
        imu.linear_acceleration.y = 0
        imu.linear_acceleration.z = 0
        imu.angular_velocity.x = vth
        imu.angular_velocity.x = 0
        imu.angular_velocity.x = 0

    def compute_odom(self, velocities, last_time, current_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.
        @param velocities: linear and angular velocities
          @type  velocities: (vx, vy, vth)
        @param last_time: time of last calculation
          @type  last_time: rospy.Time
        @param current_time: current time
          @type  current_time: rospy.Time
        @param odom: Odometry instance to update.
          @type  odom: nav_msgs.msg.Odometry
        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        dt = (current_time - last_time).to_sec()
        vx, vy, vth = velocities

        # Calculating delta distance (d) and delta_angle (angle) from velocities
        d = (vx * dt) * self.odom_linear_scale_correction  # correction factor from calibration
        angle = (vth * dt) * self.odom_angular_scale_correction  # correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle) * x - sin(last_angle) * y
        self._pos2d.y += sin(last_angle) * x + cos(last_angle) * y
        self._pos2d.theta += angle

        # Rosbee quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta / 2.), cos(self._pos2d.theta / 2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(d / dt, 0, 0), Vector3(0, 0, angle / dt))
        """
        # old code did not apply correction to velocities
        # odom calculation from velocities
        th = self._pos2d.theta
        self._pos2d.x += (vx * cos(th) - vy * sin(th)) * self.odom_linear_scale_correction * dt;
        self._pos2d.y += (vx * sin(th) + vy * cos(th)) * self.odom_linear_scale_correction* dt;
        self._pos2d.theta += vth * self.odom_angular_scale_correction * dt;

        # Quaternion from yaw
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))
        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat
        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        """

        if vx == 0.0 and \
                        vy == 0.0 and \
                        vth == 0.0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # return the transform
        return transform


    def publish_odometry_transform(self, odometry):
        self.transform_broadcaster.sendTransform(
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
            (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w),
            odometry.header.stamp, odometry.child_frame_id, odometry.header.frame_id)


    def publish_diagnostic(self):
        self.diag_msg.header.stamp = rospy.Time.now()
        self.diag_msg.status.append(self.battery_status())
        #diag_msg.status.append(self.info_status())
        # diag_msg.status.append(self.network_status())
        self.diag_msg.status.append(self.connection_status())
        self.diag_msg.status.append(self.speed_status())
        self.pub_diagnostics.publish(self.diag_msg)
        print (self.robot.request_enable_status())


    def battery_status(self):
        stat = DiagnosticStatus(name="Battery", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("Voltage avg(Vin)", str(self.robot.get_avg_voltageIn())),
            KeyValue("Voltage max(Vin)", str(self.robot.get_max_voltageIn())),
            KeyValue("Voltage min(Vin)", str(self.robot.get_min_voltageIn())),
            KeyValue("Current avg(A)", str(self.robot.get_avg_current())),
            KeyValue("Current max(A)", str(self.robot.get_avg_current())),
            KeyValue("Current min(A)", str(self.robot.get_avg_current())),
            KeyValue("Voltage avg(V5V)", str(self.robot.get_avg_5voltage())),
            KeyValue("Voltage max(V5V)", str(self.robot.get_max_5voltage())),
            KeyValue("Voltage min(V5V)", str(self.robot.get_min_5voltage())),
            KeyValue("Voltage avg(V3.3)", str(self.robot.get_avg_3voltage())),
            KeyValue("Voltage max(V3.3)", str(self.robot.get_max_3voltage())),
            KeyValue("Voltage min(V3.3)", str(self.robot.get_min_3voltage()))]
        if self.robot.get_status().VoltageLow == True:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Voltage too low"
        if self.robot.get_status().CurrentError == True:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Current error"
        if self.robot.get_status().Voltage3v3Low == True:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Voltage3.3 too low"
        return stat

    def network_status(self):
        stat = DiagnosticStatus(name="Network", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("Baudrate", str(self.robot.get_connection_info()[1])),
            KeyValue("Comport", str(self.robot.get_connection_info()[3]))]
        if self.robot.is_connected() == False:
            stat.message = "disconnected"
            stat.level = DiagnosticStatus.ERROR
        return stat

    def connection_status(self):
        stat = DiagnosticStatus(name="Connection", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("Baudrate", str(self.robot.get_connection_info()["baudrate"])),
            KeyValue("Comport", str(self.robot.get_connection_info()["comport"]))]
        if self.robot.is_connected() == False:
            stat.message = "disconnected"
            stat.level = DiagnosticStatus.ERROR
        return stat

    def speed_status(self):
        stat = DiagnosticStatus(name="Speed", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("Linear speed (Vx)", str(self.robot.get_movesteer(None)[0])),
            KeyValue("Angular speed (Vth)", str(self.robot.get_movesteer(None)[2]))]
        if self.robot.get_movesteer(None)[0] > self.speedLimit:
            stat.level = DiagnosticStatus.WARN
            stat.message = "speed is too high"
        return stat

    def info_status(self):
        stat = DiagnosticStatus(name="Info_Platform", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("", str(self.robot.get_status()[0])),
            KeyValue("", str(self.robot.get_status()[1])),
            KeyValue("", str(self.robot.get_status()[2]))]
        return stat

    def spin(self):
        odom = Odometry(header=rospy.Header(frame_id=self.odom_frame), child_frame_id=self.base_frame)
        imu = Imu(header=rospy.Header(frame_id=self.odom_frame))
        self.robot.get_update_from_rosbee()
        transform = None
        last_state_time = rospy.get_rostime()
        last_vel_state = (0.0, 0.0, 0.0)
        last_gyro_state = (0.0, 0.0, 0.0)
        req_cmd_vel = (0.0, 0.0, 0.0)
        last_cmd_vel_time = rospy.get_rostime()
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            self.robot.get_update_from_rosbee()
            # ACT & SENSE
            if self.req_cmd_vel is not None:
                req_cmd_vel = self.req_cmd_vel
                # set to None to signal that command is handled
                self.req_cmd_vel = None
                # reset time for timeout
                last_cmd_vel_time = current_time
            else:
                # stop on timeout
                if current_time - last_cmd_vel_time > self.cmd_vel_timeout:
                    req_cmd_vel = (0.0, 0.0, 0.0)
                    if self.verbose:
                        rospy.loginfo('timeout')

            # send velocity command & receive state
            old_state_time = last_state_time
            old_vel_state = last_vel_state
            old_vel_state_gyro = last_gyro_state
            self.robot.set_movesteer(req_cmd_vel[0], req_cmd_vel[2])
            last_gyro_state = self.robot.get_movesteer(self.robot.get_gyro())
            last_state = self.robot.get_movesteer(None)
            last_state_time = current_time
            last_vel_state = last_state
            # COMPUTE ODOMETRY
            # use average velocity, i.e. assume constant acceleration
            avg_vel_state = self.average_vel(old_vel_state, last_state)
            avg_vel_state_gyro = self.average_vel(old_vel_state_gyro, last_gyro_state)
            self.compute_imu(avg_vel_state_gyro, old_state_time, last_state_time, imu)
            transform = self.compute_odom(avg_vel_state, old_state_time, last_state_time, odom)
            # PUBLISH ODOMETRY
            self.odom_pub.publish(odom)
            self.imu_pub.publish(imu)
            self.publish_diagnostic()
            if self.publish_tf:
                self.publish_odometry_transform(odom)

            if self.verbose:
                rospy.loginfo("velocity setpoint: %s", str(req_cmd_vel))
                rospy.loginfo("velocity measured: %s", str(last_vel_state))
                rospy.loginfo("pose: %s", str(transform))
            r.sleep()


if __name__ == '__main__':
    rospy.loginfo("%s started...", NAME)
    try:
        RobotNode(NAME)
    except rospy.ROSInterruptException, err:
        rospy.loginfo(str(err))
        rospy.loginfo("%s stopped working...", NAME)
