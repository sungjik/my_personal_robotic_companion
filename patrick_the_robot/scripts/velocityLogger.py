#!/usr/bin/env python
'''
Created January, 2012

@author: Dr. Rainer Hessmer

  velocityLogger.py - This ROS node subscribes to the /odom topic and logs
  the linear and angular velocities with associated time stamps. This faciliates
  measuring the acceleration limits of the robot.

  Copyright (c) 2012 Dr. Rainer Hessmer.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rospy
import sys
import time

from nav_msgs.msg import Odometry

startTime = time.time()
lastTimestamp = startTime

class VelocityLogger(object):
       
        def __init__(self, outputFilePath, odomTopic = '/odom'):
                self._OutputFilePath = outputFilePath
                self._OdomTopic = odomTopic

        def start(self):
                self._OutputFile = open(self._OutputFilePath, 'w')
                rospy.loginfo('opened file %s', self._OutputFilePath)
                self._StartTime = time.time()
                self._LastTimestamp = startTime
                self._twist_x_prev = 0.0
                self._twist_theta_prev = 0.0
                self._accel_x_max = 0.0
                self._accel_theta_max = 0.0
                rospy.Subscriber(self._OdomTopic, Odometry, self._onOdomMessageReceived)

        def _onOdomMessageReceived(self, data):
                twist = data.twist.twist
                currentTime = time.time()
                secondsSinceStart = currentTime - self._StartTime
                deltaT = currentTime - self._LastTimestamp
                self._LastTimestamp = currentTime
                #rospy.loginfo(str(deltaT)+', '+str(twist.linear.x)+', '+str(twist.angular.z))
                accel_x = twist.linear.x - self._twist_x_prev
                accel_theta = twist.angular.z - self._twist_theta_prev
                self._twist_x_prev = twist.linear.x
                self._twist_theta_prev = twist.angular.z
                self._accel_x_max = max(accel_x/deltaT, self._accel_x_max)
                self._accel_theta_max = max(accel_theta/deltaT, self._accel_theta_max)
                if not self._OutputFile.closed:
                    self._OutputFile.write(str(self._accel_x_max)+', '+str(self._accel_theta_max)+'\n')

        def close(self):
                print("Closing")
                self._OutputFile.close()

if __name__ == '__main__':
        rospy.init_node('velocityLogger')
        velocityLogger = VelocityLogger('/home/sung/catkin_ws/src/patrick_the_robot/archive/OdomOutput.txt')
        velocityLogger.start()
        rospy.spin()
        velocityLogger.close()
