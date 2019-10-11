#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import roslib; roslib.load_manifest('dfall_pkg')
# import rospy
# from dfall_pkg.msg import ControlCommand
# from std_msgs.msg import Int32


# General import
import time, sys
import struct
import logging

# import rosbag
# from rospkg import RosPack
from std_msgs.msg import Float32
# from std_msgs.msg import String

# Add library
#sys.path.append("lib")

# CrazyFlie client imports
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

import cflib.drivers.crazyradio

# Logging import(*
from cflib.crazyflie.log import LogConfig

# Logging settings
logging.basicConfig(level=logging.ERROR)

# Types:

CF_COMMAND_TYPE_MOTORS = 6
CF_COMMAND_TYPE_RATE   = 7
CF_COMMAND_TYPE_ANGLE =  8

CONTROLLER_MOTOR = 2
CONTROLLER_ANGLE = 1
CONTROLLER_RATE = 0
RAD_TO_DEG = 57.296

# CrazyRadio states:
CONNECTED = 0
CONNECTING = 1
DISCONNECTED = 2

# Commands coming
CMD_RECONNECT = 0
CMD_DISCONNECT = 1


# rp = RosPack()
# record_file = rp.get_path('dfall_pkg') + '/LoggingOnboard.bag'
# rospy.loginfo('afdsasdfasdfsadfasdfasdfasdfasdfasdfasdf')
# rospy.loginfo(record_file)
# bag = rosbag.Bag(record_file, 'w')

class CrazyRadioClient:
    """
       CrazyRadio client that recieves the commands from the controller and
       sends them in a CRTP package to the crazyflie with the specified
       address.
    """
    def __init__(self):

        # Setpoints to be sent to the CrazyFlie
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.motor1cmd = 0.0
        self.motor2cmd = 0.0
        self.motor3cmd = 0.0
        self.motor4cmd = 0.0
        self._status = DISCONNECTED
        self.link_uri = ""

        # self.status_pub = rospy.Publisher(node_name + '/CrazyRadioStatus', Int32, queue_size=1)
        # self.FlyingAgentClient_command_pub = rospy.Publisher('FlyingAgentClient/Command', Int32, queue_size=1)
        time.sleep(1.0)

        # Initialize the CrazyFlie and add callbacks
        self._init_cf()

        # Connect to the Crazyflie
        self.connect()

    def _init_cf(self):
        self._cf = Crazyflie()

        # Add callbacks that get executed depending on the connection _status.
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)


    def get_status(self):
        return self._status

    def update_link_uri(self):
        self.link_uri = "radio://0/79/2M"

    def connect(self):
        # update link from ros params
        self.update_link_uri()

        print "Connecting to %s" % self.link_uri
        self._cf.open_link(self.link_uri)

    def disconnect(self):
        print "Motors OFF"
        self._send_to_commander(0, 0, 0, 0, 0, 0, 0, 0, CONTROLLER_MOTOR)
        print "Disconnecting from %s" % self.link_uri
        self._cf.close_link()

    def _data_received_callback(self, timestamp, data, logconf):
        #print "log of stabilizer and battery: [%d][%s]: %s" % (timestamp, logconf.name, data)
        batteryVolt = Float32()
        stabilizerYaw = Float32()
        stabilizerPitch = Float32()
        stabilizerRoll = Float32()
        batteryVolt.data = data["pm.vbat"]
        stabilizerYaw.data = data["stabilizer.yaw"]
        stabilizerPitch.data = data["stabilizer.pitch"]

    def _logging_error(self, logconf, msg):
        print "Error when logging %s" % logconf.name

    # def _init_logging(self):

    def _start_logging(self):
        self.logconf = LogConfig("LoggingTest", 100) # second variable is freq in ms
        self.logconf.add_variable("stabilizer.roll", "float");
        self.logconf.add_variable("stabilizer.pitch", "float");
        self.logconf.add_variable("stabilizer.yaw", "float");
        self.logconf.add_variable("pm.vbat", "float");

        self._cf.log.add_config(self.logconf)
        if self.logconf.valid:
            self.logconf.data_received_cb.add_callback(self._data_received_callback)
            self.logconf.error_cb.add_callback(self._logging_error)
            print "logconf valid"
        else:
            print "logconf invalid"

        self.logconf.start()
        print "logconf start"

    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        # cf_client._send_to_commander(15000, 15000, 15000, 15000);
        # cf_client._send_to_commander_rate(1000, 0, 1000, 0, 1, 1, 1)
        cf_client._send_to_commander_angle(1000, 0, 1000, 0, 1, 1, 1)
        print "sent command to commander"
        # Config for Logging
        self._start_logging()


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        rospy.logerr("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.logerr("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.logconf.stop()
        rospy.loginfo("logconf stopped")
        self.logconf.delete()
        rospy.loginfo("logconf deleted")


    def _send_to_commander_motor(self, cmd1, cmd2, cmd3, cmd4):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHH', CF_COMMAND_TYPE_MOTORS, cmd1, cmd2, cmd3, cmd4)
        self._cf.send_packet(pk)

    def _send_to_commander_rate(self, cmd1, cmd2, cmd3, cmd4, roll_rate, pitch_rate, yaw_rate):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', CF_COMMAND_TYPE_RATE, cmd1, cmd2, cmd3, cmd4, roll_rate, pitch_rate, yaw_rate)
        self._cf.send_packet(pk)

    def _send_to_commander_angle(self, cmd1, cmd2, cmd3, cmd4, roll, pitch, yaw):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', CF_COMMAND_TYPE_ANGLE, cmd1, cmd2, cmd3, cmd4, roll, pitch, yaw)
        self._cf.send_packet(pk)

if __name__ == '__main__':
    # global node_name
    # node_name = "CrazyRadio"
    # rospy.init_node(node_name, anonymous=True)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #wait until address parameter is set by FlyingAgentClient
    # while not rospy.has_param("~crazyFlieAddress"):
    #     time.sleep(0.05)

    #use this following two lines to connect without data from CentralManager
    # radio_address = "radio://0/72/2M"
    # rospy.loginfo("manual address loaded")

    # global cfbattery_pub
    # cfbattery_pub = rospy.Publisher(node_name + '/CFBattery', Float32, queue_size=10)

    global cf_client

    cf_client = CrazyRadioClient()
    # rospy.Subscriber("FlyingAgentClient/crazyRadioCommand", Int32, cf_client.crazyRadioCommandCallback) # allows commands from scripts

    # time.sleep(1.0)

    # rospy.Subscriber("FlyingAgentClient/ControlCommand", ControlCommand, controlCommandCallback)

    # rospy.spin()
    # rospy.loginfo("Turning off crazyflie")


    #wait for client to send its commands
    # time.sleep(1.0)


    # bag.close()
    # rospy.loginfo("bag closed")

    # cf_client._cf.close_link()
    # rospy.loginfo("Link closed")
