#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights re#ser ved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
#import serial
import socket
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
rpy_pub = rospy.Publisher('/imu/rpy/filtered',Vector3Stamped,queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()
rpyMsg = Vector3Stamped()
# Orientation covariance estimation:
# Ob#ser ved orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Ob#ser ved gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# ob#ser ved acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

#default_port='/dev/ttyUSB0'
default_port='/dev/ttyUSB0'
port = rospy.get_param('~port', default_port)

#read calibration parameters
port = rospy.get_param('~port', default_port)
#port = "/dev/pts/9"
#accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

# Check your COM port and baud rate
ip = "192.168.0.61"
#rospy.loginfo("Opening %s...", port)
rospy.loginfo("Opening socket...")
try:
    #ser  = #ser ial.#ser ial(port=port, baudrate=57600, timeout=5)
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_address = ('192.168.0.61',10001)
    sock.connect(server_address)
    f = sock.makefile()
except socket.error,e:
    sock.close()
    sys.exit(0)
#except #ser ial.#ser ialutil.#ser ialexception:
#    rospy.logerr("imu not found at port "+port + ". did you specify the correct port in the launch file?")
#    #exit
#    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1g (9.8m/s^2). convert to m/s^2.
rospy.loginfo("giving the razor imu board 3 seconds to boot...")
rospy.sleep(3) # sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
#ser .write('#o0' + '\r')
sock.send('#o0\r');
print 'sock.send("#o0\\r")'
#discard old input
#automatic flush - NOT WORKING
##ser .flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
#discard = #ser .readlines() 

#discard = f.readlines()

#set output mode
#ser .write('#ox' + '\r') # To start display angle and sensor reading in text
sock.send('#ox\r')
print 'sock.send("#ox\\r")'
rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
#ser .write('#caxm' + str(accel_x_min) + '\r')
sock.send('#caxm' + str(accel_x_min) + '\r')
#ser .write('#caxM' + str(accel_x_max) + '\r')
sock.send('#caxM' + str(accel_x_max) + '\r')
#ser .write('#caym' + str(accel_y_min) + '\r')
sock.send('#caym' + str(accel_y_min) + '\r')
#ser .write('#cayM' + str(accel_y_max) + '\r')
sock.send('#cayM' + str(accel_y_max) + '\r')
#ser .write('#cazm' + str(accel_z_min) + '\r')
sock.send('#cazm' + str(accel_z_min) + '\r')
#ser .write('#cazM' + str(accel_z_max) + '\r')
sock.send('#cazM' + str(accel_z_max) + '\r')

if (not calibration_magn_use_extended):
    #ser .write('#cmxm' + str(magn_x_min) + '\r')
    #ser .write('#cmxM' + str(magn_x_max) + '\r')
    #ser .write('#cmym' + str(magn_y_min) + '\r')
    #ser .write('#cmyM' + str(magn_y_max) + '\r')
    #ser .write('#cmzm' + str(magn_z_min) + '\r')
    #ser .write('#cmzM' + str(magn_z_max) + '\r')
    
    sock.send('#cmxm' + str(magn_x_min) + '\r')
    sock.send('#cmxM' + str(magn_x_max) + '\r')
    sock.send('#cmym' + str(magn_y_min) + '\r')
    sock.send('#cmyM' + str(magn_y_max) + '\r')
    sock.send('#cmzm' + str(magn_z_min) + '\r')
    sock.send('#cmzM' + str(magn_z_max) + '\r')
else:
    #ser .write('#ccx' + str(magn_ellipsoid_center[0]) + chr(13))
    #ser .write('#ccy' + str(magn_ellipsoid_center[1]) + chr(13))
    #ser .write('#ccz' + str(magn_ellipsoid_center[2]) + chr(13))
    #ser .write('#ctxX' + str(magn_ellipsoid_transform[0][0]) + chr(13))
    #ser .write('#ctxY' + str(magn_ellipsoid_transform[0][1]) + chr(13))
    #ser .write('#ctxZ' + str(magn_ellipsoid_transform[0][2]) + chr(13))
    #ser .write('#ctyX' + str(magn_ellipsoid_transform[1][0]) + chr(13))
    #ser .write('#ctyY' + str(magn_ellipsoid_transform[1][1]) + chr(13))
    #ser .write('#ctyZ' + str(magn_ellipsoid_transform[1][2]) + chr(13))
    #ser .write('#ctzX' + str(magn_ellipsoid_transform[2][0]) + chr(13))
    #ser .write('#ctzY' + str(magn_ellipsoid_transform[2][1]) + chr(13))
    #ser .write('#ctzZ' + str(magn_ellipsoid_transform[2][2]) + chr(13))

    sock.send('#ccx' + str(magn_ellipsoid_center[0]) + '\r')
    sock.send('#ccy' + str(magn_ellipsoid_center[1]) + '\r')
    sock.send('#ccz' + str(magn_ellipsoid_center[2]) + '\r')
    sock.send('#ctxX' + str(magn_ellipsoid_transform[0][0]) + '\r')
    sock.send('#ctxY' + str(magn_ellipsoid_transform[0][1]) + '\r')
    sock.send('#ctxZ' + str(magn_ellipsoid_transform[0][2]) + '\r')
    sock.send('#ctyX' + str(magn_ellipsoid_transform[1][0]) + '\r')
    sock.send('#ctyY' + str(magn_ellipsoid_transform[1][1]) + '\r')
    sock.send('#ctyZ' + str(magn_ellipsoid_transform[1][2]) + '\r')
    sock.send('#ctzX' + str(magn_ellipsoid_transform[2][0]) + '\r')
    sock.send('#ctzY' + str(magn_ellipsoid_transform[2][1]) + '\r')
    sock.send('#ctzZ' + str(magn_ellipsoid_transform[2][2]) + '\r')

#ser .write('#cgx' + str(gyro_average_offset_x) + chr(13))
#ser .write('#cgy' + str(gyro_average_offset_y) + chr(13))
#ser .write('#cgz' + str(gyro_average_offset_z) + chr(13))

sock.send('#cgx' + str(gyro_average_offset_x) + '\r')
sock.send('#cgy' + str(gyro_average_offset_y) + '\r')
sock.send('#cgz' + str(gyro_average_offset_z) + '\r')

#print calibration values for verification by u#ser 
#ser .flushInput()

#ser .write('#p' + chr(13))
sock.send('#p' + '\r')

#calib_data = #ser .readlines()
#calib_data = sock.readlines()
#calib_data = f.readlines()
calib_data = sock.recv(1024)
calib_data_print = "Printing set calibration values:\r\n"
for line in calib_data:
    calib_data_print += line
rospy.loginfo(calib_data_print)

#start datastream
#ser .write('#o1' + chr(13))
sock.send('#o1' + '\r')

#automatic flush - NOT WORKING
##ser .flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the #ser ial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
#    line = #ser .readline()
    #line = sock.readline()
    #line = f.readline()
    line = sock.recv(1024)
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

while not rospy.is_shutdown():
 #   line = #ser .readline()
#    line = f.readline()
    line = sock.recv(1024)
    #print line
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        try:
            yaw_deg = -float(words[0])
        except:
            continue
        yaw_deg = yaw_deg + imu_yaw_calibration

        #if yaw_deg > 360 or yaw_deg < -360:
        #    print words[0],yaw_deg
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0

        #calibration the degree in foure area modigy by cjtong
        #if yaw_deg >= -91 and yaw_deg < -20:
        #    yaw_deg = yaw_deg + (yaw_deg + 94.7368)*0.2676
        if yaw_deg >= -85 and yaw_deg < -20:
            yaw_deg = yaw_deg + (yaw_deg + 57.142857142857)*0.53846153846
        elif yaw_deg >= -20 and yaw_deg < 80:
            yaw_deg = yaw_deg + (yaw_deg -180)*-0.1
        elif yaw_deg >= 80 and yaw_deg < 180:
            yaw_deg = yaw_deg + (180 - yaw_deg)*0.1
        else:
            yaw_deg = yaw_deg + (yaw_deg + 180)*-0.052631578947
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        try:
            pitch = -float(words[1])*degrees2rad
            roll = float(words[2])*degrees2rad
        except:
            continue
        rpyMsg.vector.x=roll
        rpyMsg.vector.y=pitch
        rpyMsg.vector.z=yaw
        rpy_pub.publish(rpyMsg)
        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        try:
            imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
            imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
            imuMsg.linear_acceleration.z = float(words[5]) * accel_factor
    
            imuMsg.angular_velocity.x = float(words[6])
            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            imuMsg.angular_velocity.y = -float(words[7])
            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
            imuMsg.angular_velocity.z = -float(words[8])
        except:
            continue
    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)

    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
#ser .close
sock.close()
#f.close
