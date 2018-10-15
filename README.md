1.目录介绍：
src/Razor_SHRS/目录下为imu3.0的arduino单片机程序
nodes/imu3_0.py为ros驱动程序（板子输出原始串口数据，转换为ros imu message）
2.常见问题：
出现图像不动，echo /imu没有数据，检查一下imu串口输出数据是否和imu3_0.py文件解析对应。imu3_0.py解析格式为时间 pitch roll yaw ,但是出厂可能串口输出不是。自己检查一下。
*以下为控制输出格式指令*：
The format of this default string is:
<timeMS>, <accelX>, <accelY>, <accelZ>, <gyroX>, <gyroY>, <gyroZ>, <magX>, <magY>, <magZ>
The string can be modified by sending any of the following commands:
(SPACE) – Pause/resume serial port printing
t – Turn time readings on or off
a – Turn accelerometer readings on or off
g – Turn gyroscope readings on or off
m – Turn magnetometer readings on or off
c – Switch to/from calculated values from/to raw readings
q – Turn quaternion readings on or off (qw, qx, qy, and qz are printed after mag readings)
e – Turn Euler angle calculations (pitch, roll, yaw) on or off (printed after quaternions)
h – Turn heading readings on or off
r – Adjust log rate in 10Hz increments between 1-100Hz (1, 10, 20, …, 100)
A – Adjust accelerometer full-scale range. Cycles between ± 2, 4, 8, and 16g.
G – Adjust gyroscope full-scale range. Cycles between ± 250, 500, 1000, 2000 dps.
s – Enable/disable SD card logging

/**************************完美分割线×××××××××××××××××××××××××××××××××××××××××××/

Official ROS Documentation
--------------------------
A much more extensive and standard ROS-style version of this documentation can be found on the ROS wiki at:

http://wiki.ros.org/razor_imu_9dof


Install and Configure ROS Package
---------------------------------
1) Install dependencies:

	$ sudo apt-get install python-visual

2) Download code:

	$ cd ~/catkin_workspace/src
	$ git clone https://github.com/KristofRobot/razor_imu_9dof.git
	$ cd ..
	$ catkin_make


Install Arduino firmware
-------------------------
1) Open ``src/Razor_AHRS/Razor_AHRS.ino`` in Arduino IDE. Note: this is a modified version
of Peter Bartz' original Arduino code (see https://github.com/ptrbrtz/razor-9dof-ahrs). 
Use this version - it emits linear acceleration and angular velocity data required by the ROS Imu message

2) Select your hardware here by uncommenting the right line in ``src/Razor_AHRS/Razor_AHRS.ino``, e.g.

<pre>
// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
</pre>

3) Upload Arduino sketch to the Sparkfun 9DOF Razor IMU board


Configure
---------
In its default configuration, ``razor_imu_9dof`` expects a yaml config file ``my_razor.yaml`` with:
* USB port to use
* Calibration parameters

An example``razor.yaml`` file is provided.
Copy that file to ``my_razor.yaml`` as follows:

    $ roscd razor_imu_9dof/config
    $ cp razor.yaml my_razor.yaml

Then, edit ``my_razor.yaml`` as needed

Launch
------
Publisher and 3D visualization:

	$ roslaunch razor_imu_9dof razor-pub-and-display.launch

Publisher only:

	$ roslaunch razor_imu_9dof razor-pub.launch

Publisher only with diagnostics:

	$ roslaunch razor_imu_9dof razor-pub-diags.launch

3D visualization only:

	$ roslaunch razor_imu_9dof razor-display.launch


Calibrate
---------
For best accuracy, follow the tutorial to calibrate the sensors:

http://wiki.ros.org/razor_imu_9dof

A copy of Peter Bartz's magnetometer calibration scripts from https://github.com/ptrbrtz/razor-9dof-ahrs is provided in the ``magnetometer_calibration`` directory.

Update ``my_razor.yaml`` with the new calibration parameters.

Dynamic Reconfigure
-------------------
After having launched the publisher with one of the launch commands listed above, 
it is possible to dynamically reconfigure the yaw calibration.

1) Run:

    $ rosrun rqt_reconfigure rqt_reconfigure 
    
2) Select ``imu_node``. 

3) Change the slider to move the calibration +/- 10 degrees. 
If you are running the 3D visualization you'll see the display jump when the new calibration takes effect.

The intent of this feature is to let you tune the alignment of the AHRS to the direction of the robot driving direction, so that if you can determine that, for example, the AHRS reads 30 degrees when the robot is actually going at 35 degrees as shown by e.g. GPS, you can tune the calibration to make it read 35. It's the compass-equivalent of bore-sighting a camera.
