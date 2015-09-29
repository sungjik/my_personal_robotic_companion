#!/usr/bin/env python
#credits : http://forum.arduino.cc/index.php/topic,8573.0.html

import rospy
import socket
import tf
from sensor_msgs.msg import Imu
from math import sqrt,atan2,cos,sin,pi

def map_min_max(x, in_min, in_max, out_min, out_max):
    return (long(x)-long(in_min))*(long(out_max)-long(out_min))/(long(in_max)-long(in_min))+long(out_min)

def imu_publisher(sock):
    host="192.168.42.7"
    port=5555
    roll_offset = 0.0
    pitch_offset = 0.0
    yaw_offset = 0.0
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0
    pub_freq = 10
    alpha = 0.9
    yaw_mag = 0.0
    yaw_cf = 0.0
    yawU = 0.0
    roll_gyro = 0.0
    pitch_gyro = 0.0
    yaw_gyro = 0.0
    gyro_x_filt = 0.0
    gyro_y_filt = 0.0
    gyro_z_filt = 0.0
    accel_x_filt = 0.0
    accel_y_filt = 0.0
    accel_z_filt = 0.0
    mag_x_filt = 0.0
    mag_y_filt = 0.0
    mag_z_filt = 0.0
    count = 0
    num_callibration_itrs = 60
    debug = False

    pub = rospy.Publisher('imu', Imu, queue_size=50)
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
        if len(line) == 13:  #received complete packet
            current_time = rospy.Time.now()
            accel_x = float(line[2])    ##m/s^2
            accel_y = float(line[3])
            accel_z = float(line[4])
            gyro_x = float(line[6])     #rad/s
            gyro_y = float(line[7])
            gyro_z = float(line[8])
            mag_x = float(line[10])
            mag_y = float(line[11])
            mag_z = float(line[12])

            if count == 0:
                rospy.loginfo("callibrating accelerometer and gyroscope readings for %s itrs...", num_callibration_itrs)
                for i in range(0, num_callibration_itrs):
                    gyro_x_offset += gyro_x
                    gyro_y_offset += gyro_y
                    gyro_z_offset += gyro_z
                gyro_x_offset /= num_callibration_itrs
                gyro_y_offset /= num_callibration_itrs
                gyro_z_offset /= num_callibration_itrs

            #filter accel and mag readings with low pass filter
            accel_x_filt += alpha * (accel_x - accel_x_filt)
            accel_y_filt += alpha * (accel_y - accel_y_filt)
            accel_z_filt += alpha * (accel_z - accel_z_filt)
            
            mag_x_filt += alpha * (mag_x - mag_x_filt)
            mag_y_filt += alpha * (mag_y - mag_y_filt)
            mag_z_filt += alpha * (mag_z - mag_z_filt)

            gyro_x_filt += alpha * (gyro_x - gyro_x_filt)
            gyro_y_filt += alpha * (gyro_y - gyro_y_filt)
            gyro_z_filt += alpha * (gyro_z - gyro_z_filt)

            #normalize accel and mag readings
            accel_norm = sqrt(pow(accel_x_filt,2)+pow(accel_y_filt,2)+pow(accel_z_filt,2))
            accel_x_filt /= accel_norm
            accel_y_filt /= accel_norm
            accel_z_filt /= accel_norm 

            mag_norm = sqrt(pow(mag_x_filt,2)+pow(mag_y_filt,2)+pow(mag_z_filt,2))
            mag_x_filt /= mag_norm
            mag_y_filt /= mag_norm
            mag_z_filt /= mag_norm

            #discretize gyro readings
            #gyro_z = float(int((gyro_z) * 100 / 20))/5
            #print gyro_z
            #calculate roll pitch yaw
            pitch   = atan2(accel_x_filt, sqrt(pow(accel_y_filt,2) + pow(accel_z_filt,2)))
            roll    = atan2(-accel_y_filt, sqrt(pow(accel_x_filt,2) + pow(accel_z_filt,2)))
            yaw_mag = atan2(-mag_y_filt*cos(roll)+mag_z_filt*sin(roll),
                             mag_x_filt*cos(pitch)+mag_z_filt*sin(pitch)*sin(roll)+mag_z_filt*sin(pitch)*cos(roll))
            yawU = atan2(mag_x_filt, mag_y_filt)
            dt = current_time.to_sec() - last_time.to_sec()
            yaw_cf = alpha*(yaw_cf + dt*(gyro_z-gyro_z_offset)) + (1-alpha)*yawU

            roll_gyro += dt*(gyro_x_filt - gyro_x_offset)
            pitch_gyro += dt*(gyro_y_filt - gyro_y_offset)
            yaw_gyro += dt*(gyro_z - gyro_z_offset)

            #callibrate roll pitch yaw
            #roll = roll_gyro
            #pitch = pitch_gyro
            yaw = yaw_gyro
            if count < num_callibration_itrs:
                roll_offset += roll
                pitch_offset += pitch
                yaw_offset += yaw
                count += 1
            elif count == num_callibration_itrs:
                roll_offset /= num_callibration_itrs
                pitch_offset /= num_callibration_itrs
                yaw_offset /= num_callibration_itrs
                rospy.loginfo("finished callibrating rpy")
                count += 1

            #publish ros Imu message
            else:
                roll -= roll_offset
                pitch -= pitch_offset
                yaw -= yaw_offset
                if roll < -pi:
                    roll += 2*pi
                if pitch < -pi:
                    pitch += 2*pi
                if yaw > pi:
                    yaw -= 2*pi
                if yaw < -pi:
                    yaw += 2*pi
                if debug:
                    rospy.loginfo('roll %s pitch %s yaw %s', int(roll*180/pi), int(pitch*180/pi), int(yaw*180/pi))
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = '/base_link'
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
                imu_msg.angular_velocity_covariance[0] = -1
                imu_msg.linear_acceleration_covariance[0] = -1
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
        imu_publisher(sock)
    except rospy.ROSInterruptException:
        pass
