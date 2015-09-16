#!/usr/bin/env python
#credits : http://forum.arduino.cc/index.php/topic,8573.0.html

import rospy
import socket
import tf
from sensor_msgs.msg import Imu
from math import sqrt,atan2,cos,sin,pi

host="192.168.42.7"
port=5555

def map_min_max(x, in_min, in_max, out_min, out_max):
    return (long(x)-long(in_min))*(long(out_max)-long(out_min))/(long(in_max)-long(in_min))+long(out_min)

def imu_publisher(sock):
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0
    accel_x_offset = 0.0
    accel_y_offset = 0.0
    accel_z_offset = 0.0
    mag_x_min = 10000
    mag_y_min = 10000
    mag_z_min = 10000
    mag_x_max = 0
    mag_y_max = 0
    mag_z_max = 0
    mag_x_map = 0.0
    mag_y_map = 0.0
    mag_z_map = 0.0
    pub_freq = 10
    alpha = 0.5
    beta = 0.9
    yaw_mag = 0.0
    yaw_cf = 0.0
    yawU = 0.0
    accel_x_filt = 0.0
    accel_y_filt = 0.0
    accel_z_filt = 0.0
    mag_x_filt = 0.0
    mag_y_filt = 0.0
    mag_z_filt = 0.0

    pub = rospy.Publisher('imu', Imu, queue_size=50)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(pub_freq)

    num_itrs = 60
    rospy.loginfo("callibrating accelerometer and gyroscope readings...")
    rospy.loginfo("waiting for device...")
    count = 0
    while count < 60:
        data,addr = sock.recvfrom(1024)
        line = data.split(',')
        if len(line) != 17:
            continue
        line = data.split(',')
        gyro_x_offset = gyro_x_offset + float(line[6])
        gyro_y_offset = gyro_y_offset + float(line[7])
        gyro_z_offset = gyro_z_offset + float(line[8])
        accel_x_offset = accel_x_offset + float(line[2])
        accel_y_offset = accel_y_offset + float(line[3])
        accel_z_offset = accel_z_offset +  float(line[4])
        count = count + 1

    gyro_x_offset = gyro_x_offset / num_itrs
    gyro_y_offset = gyro_y_offset / num_itrs
    gyro_z_offset = gyro_z_offset / num_itrs
    accel_x_offset = accel_x_offset / num_itrs
    accel_y_offset = accel_y_offset / num_itrs
    accel_z_offset = accel_z_offset / num_itrs
    rospy.loginfo("finished callibrating")

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        data,addr = sock.recvfrom(1024)
        line = data.split(',')
        if len(line) == 17:
            #received complete packet
            current_time = rospy.Time.now()
            accel_x = float(line[2])# - accel_x_offset #m/s^2
            accel_y = float(line[3])# - accel_y_offset
            accel_z = float(line[4])# - accel_z_offset
            gyro_x = float(line[6]) #- gyro_x_offset  #rad/s
            gyro_y = float(line[7])# - gyro_y_offset
            gyro_z = float(line[8])# - gyro_z_offset
            mag_x = float(line[10])
            mag_y = float(line[11])
            mag_z = float(line[12])

            #filter grav and mag readings with low pass filter
            accel_x_filt = accel_x_filt + alpha * (accel_x - accel_x_filt)
            accel_y_filt = accel_y_filt + alpha * (accel_y - accel_y_filt)
            accel_z_filt = accel_z_filt + alpha * (accel_z - accel_z_filt)
            
            mag_x_filt = mag_x_filt + alpha * (mag_x - mag_x_filt)
            mag_y_filt = mag_y_filt + alpha * (mag_y - mag_y_filt)
            mag_z_filt = mag_z_filt + alpha * (mag_z - mag_z_filt)

            #get azimuth for magnetometer readings
            mag_x_max = max(mag_x_max, long(mag_x_filt))
            mag_y_max = max(mag_y_max, long(mag_y_filt))
            mag_z_max = max(mag_z_max, long(mag_z_filt))

            mag_x_min = min(mag_x_min, long(mag_x_filt))
            mag_y_min = min(mag_y_min, long(mag_y_filt))
            mag_z_min = min(mag_z_min, long(mag_z_filt))
            
            mag_x_map = map_min_max(mag_x_filt, mag_x_min, mag_x_max, -10000, 10000)/10000.0
            mag_y_map = map_min_max(mag_y_filt, mag_y_min, mag_y_max, -10000, 10000)/10000.0
            mag_z_map = map_min_max(mag_z_filt, mag_z_min, mag_z_max, -10000, 10000)/10000.0

            #normalize grav and mag readings
            accel_norm = sqrt(pow(accel_x_filt,2)+pow(accel_y_filt,2)+pow(accel_z_filt,2))
          #  accel_x_filt = accel_x_filt/accel_norm
          #  accel_y_filt = accel_y_filt/accel_norm
          #  accel_z_filt = accel_z_filt/accel_norm 

            mag_norm = sqrt(pow(mag_x_map,2)+pow(mag_y_map,2)+pow(mag_z_map,2))
            mag_x_map = mag_x_map/mag_norm
            mag_y_map = mag_y_map/mag_norm
            mag_z_map = mag_z_map/mag_norm

            #calculate roll pitch yaw
            pitch = atan2(-accel_x_filt, sqrt(pow(accel_y_filt,2) + pow(accel_z_filt,2)))
            pitch2 = atan2(-accel_x, accel_z)
            roll  = atan2(accel_y_filt, sqrt(pow(accel_x_filt,2) + pow(accel_z_filt,2)))
            roll2 = atan2(accel_y, accel_z)
            yaw_mag   = atan2(-mag_y_map*cos(roll)+mag_z_map*sin(roll), 
                           mag_x_map*cos(pitch)+mag_z_map*sin(pitch)*sin(roll)+mag_z_map*sin(pitch)*cos(roll))
            yawU = atan2(-mag_y_map, mag_x_map)
            dt = current_time.to_sec() - last_time.to_sec()
            yaw_cf = beta*(yaw_cf + dt*gyro_z) + (1-beta)*yaw_mag
            print roll*180/pi, pitch*180/pi, yaw_mag*180/pi, yawU*180/pi, yaw_cf*180/pi
            #publish ros Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = 'base_link'
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw_cf)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            pub.publish(imu_msg)
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
        sock.bind((host,port))
        imu_publisher(sock)
    except rospy.ROSInterruptException:
        pass
