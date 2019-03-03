#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_from_euler

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

rospy.init_node("razor_node")
pub = rospy.Publisher('imu', Imu, queue_size=1)
rpy_pub = rospy.Publisher('/imu/rpy/filtered',Vector3Stamped,queue_size=1)

imuMsg = Imu()
rpyMsg = Vector3Stamped()
port='/dev/ttyACM0'
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=5)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

roll=pitch=yaw=seq=0

rospy.loginfo("Flushing first 100 IMU entries...")
for i in range(100):
    line = ser.readline()
    print line
rospy.loginfo("Publishing IMU data...")

ser.write('t\n')
ser.write('m\n')
ser.write('e\n')
ser.write('q\n')
rospy.loginfo("format parase data")

def restore_com_config():
    global ser
    ser.write('t\n')
    ser.write('m\n')
    ser.write('q\n')
    ser.write('e\n')
rospy.on_shutdown(restore_com_config)

while not rospy.is_shutdown():
    line = ser.readline()
    words = string.split(line.rstrip(),",")    # Fields split
#    print words
    if len(words) == 13:
        try:
#            print words[0],words[1],words[2]
            pitch = float(words[10])*degrees2rad
            roll = float(words[11])*degrees2rad
            yaw = float(words[12])*degrees2rad
            imuMsg.orientation.x = float(words[7])
            imuMsg.orientation.y = float(words[8])
            imuMsg.orientation.z = float(words[9])
            imuMsg.orientation.w = float(words[6])
            imuMsg.linear_acceleration.x = float(words[0])
            imuMsg.linear_acceleration.y = float(words[1])
            imuMsg.linear_acceleration.z = float(words[2])
            imuMsg.angular_velocity.x = float(words[3])
            imuMsg.angular_velocity.y = float(words[4])
            imuMsg.angular_velocity.z = float(words[5])
        except:
            continue
        rpyMsg.vector.x=roll
        rpyMsg.vector.y=pitch
        rpyMsg.vector.z=yaw 
        rpy_pub.publish(rpyMsg)
        q = quaternion_from_euler(roll,pitch,yaw)
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'imu_link'
        imuMsg.header.seq = seq
        seq = seq + 1
        pub.publish(imuMsg)
ser.close

