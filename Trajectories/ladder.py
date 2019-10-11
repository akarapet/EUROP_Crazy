#!/usr/bin/env python

import rospy
from crazy_pkg.msg import Setpoint
from crazy_pkg.msg import CrazyflieData
from std_msgs.msg import Int32
import time
import rosbag

bag = rosbag.Bag('pd2.bag', 'w')

pubState = rospy.Publisher('/DefaultControllerService/requestStateChange', Int32, queue_size=10, latch = True)
pub = rospy.Publisher('DefaultControllerService/RequestSetpointChange', Setpoint, queue_size=10)
rospy.init_node('generator', anonymous=True)

x = float(0.0)
y = float(0.0)
z = float(0.3)
yaw = float(0.0)


msg = Setpoint()
msg.x = x
msg.y = y
msg.z = z
msg.yaw = yaw

recording = True

def callback(data):
    if (recording):
        bag.write('CrazyRadio/fullState', data)


rospy.Subscriber("CrazyRadio/fullState", CrazyflieData, callback)

pubState.publish(1)

print ("published")

time.sleep(5)

msg.z = float(0.5)
pub.publish(msg)

time.sleep(5)

msg.z = float(0.7)
pub.publish(msg)

time.sleep(5)

msg.z = float(0.5)
pub.publish(msg)

time.sleep(5)

msg.z = float(0.3)
pub.publish(msg)

time.sleep (5)

recording = False

time.sleep(1)
bag.close()
print("Bag Saved!")