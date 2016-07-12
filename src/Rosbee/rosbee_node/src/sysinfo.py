import rospy
# from rosbee_node.msg import BatteryStatus, NetworkStatus, SysInfo, SystemStatus
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
import rosinterface


class SystemInfo(object):
    def __init__(self):
        self.robot = rosinterface
        self.robot.init_robot()
        self.robot.enable_robot()
        rospy.init_node("sysinfo")
        self.pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.diag_msg = DiagnosticArray()
        self.update_rate = rospy.get_param('~update_rate', 50)
        self.speedLimit = rospy.get_param('~speed_limit', 10)

    def publish_diagnostic(self):
        self.robot.get_update_from_rosbee()
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


if __name__ == '__main__':
    try:
        obj = SystemInfo()
    except rospy.ROSInterruptException:
        pass
