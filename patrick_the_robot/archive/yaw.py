#!/usr/bin/env python
#credits : http://forum.arduino.cc/index.php/topic,8573.0.html

import rospy
import socket
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import sqrt,atan2,cos,sin,pi

def imu_publisher(sock):
    host="192.168.42.185"
    port=5555
    yaw_offset = 0.0
    roll_offset = 0.0
    pitch_offset = 0.0
    pub_freq = 10
    alpha = 0.9
    count = 0
    num_callibration_itrs = 60
    debug = False

    pub = rospy.Publisher('imu', Imu, queue_size=50)
    rpy_pub = rospy.Publisher('rpy', Vector3, queue_size=50)

    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(pub_freq)
    if rospy.has_param('~num_callibration_itrs'):
        num_callibration_itrs = rospy.get_param('~num_callibration_itrs')
    if rospy.has_param('~host'):
        host = rospy.get_param('~host')
    if rospy.has_param('~debug'):
        debug = rospy.get_param('~debug')

    sock.bind((host,port))

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rospy.loginfo("waiting for device...")
    while not rospy.is_shutdown():
        data,addr = sock.recvfrom(1024)
        line = data.split(',')
        if len(line) == 4:  #received complete packet
            current_time = rospy.Time.now()
            yaw = float(line[0])
            roll = float(line[1])
            pitch = float(line[2])
            if count < num_callibration_itrs:
                yaw_offset += yaw
                roll_offset += roll
                pitch_offset += pitch
                count += 1
            elif count == num_callibration_itrs:
                yaw_offset /= num_callibration_itrs
                roll_offset /= num_callibration_itrs
                pitch_offset /= num_callibration_itrs
                rospy.loginfo("finished callibrating yaw")
                count += 1

            #publish ros Imu message
            else:
                yaw -= yaw_offset
                roll -= roll_offset
                pitch -= pitch_offset
                if yaw > pi:
                    yaw -= 2*pi
                if yaw < -pi:
                    yaw += 2*pi
                if roll > pi:
                    roll -= 2*pi
                if roll < -pi:
                    roll += 2*pi
                if pitch > pi:
                    pitch -= 2*pi
                if pitch < -pi:
                    pitch += 2*pi
                if debug:
                    rospy.loginfo('roll %s pitch %s yaw %s', int(roll), int(pitch), int(yaw))
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = '/base_link'
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                #imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
                imu_msg.angular_velocity_covariance[0] = -1
                imu_msg.linear_acceleration_covariance[0] = -1
                pub.publish(imu_msg)
                rpy_msg = Vector3()
                rpy_msg.x = roll
                rpy_msg.y = pitch
                rpy_msg.z = yaw
                rpy_pub.publish(rpy_msg)
            last_time = current_time
            #rate.sleep()
        else:
            rospy.loginfo("received incomplete UDP packet from android IMU")
            continue
             

if __name__ == '__main__':
    try:
        sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        imu_publisher(sock)
    except rospy.ROSInterruptException:
        pass
