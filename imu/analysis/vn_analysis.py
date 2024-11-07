import rosbag
import numpy as np
import matplotlib.pyplot as plt


"""convert to quaternions in the driver code returns [qx,qy,qz,qw] but msg saves it seperately.
individual parameters required"""
def quaternion_to_euler(qx, qy, qz, qw):
    #qx,qy,qz,qw=quaternion
    roll=np.degrees(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)))
    pitch=np.degrees(2*(qw*qy-qz*qx)/(1-2*(qy*qy+qz*qz)))
    yaw=np.degrees(2 *(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)))
    return [roll, pitch, yaw]

time_data=[]
gyro_x=[]
gyro_y=[]
gyro_z=[]
accel_x=[]
accel_y=[]
accel_z=[]
rot_x=[]
rot_y=[]
rot_z=[]

imu_topic='/imu'
with rosbag.Bag('/home/skompe/vn_driver_stat.bag', 'r') as bag:
    start_time=None

    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        if start_time is None:
            start_time=t.to_sec()

        time_data.append(t.to_sec()-start_time)
        #fig0
        gyro_x.append(msg.imu.angular_velocity.x*180/np.pi) #will be in radians per second. converted to degrees per second. lesgoo
        gyro_y.append(msg.imu.angular_velocity.y*180/np.pi)
        gyro_z.append(msg.imu.angular_velocity.z*180/np.pi)
        #fig1
        accel_x.append(msg.imu.linear_acceleration.x)
        accel_y.append(msg.imu.linear_acceleration.y)
        accel_z.append(msg.imu.linear_acceleration.z)
        #fig2
        roll, pitch, yaw=quaternion_to_euler(msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w)
        rot_x.append(roll)
        rot_y.append(pitch)
        rot_z.append(yaw)



"""print(time_data)
print(gyro_x)
print(gyro_y)
print(gyro_z)"""

##################################################################
#Rotational eate
plt.figure(figsize=(14, 10))

#X axis
plt.subplot(3, 1, 1)
plt.plot(time_data, gyro_x,linestyle='-', linewidth=0.5, color='b', label='Gyro X(deg/s)')
plt.xlabel('Time(s)')
plt.ylabel('X Rotation(deg/s)')
plt.legend()
plt.grid(True)

#Y axis
plt.subplot(3, 1, 2)
plt.plot(time_data, gyro_y, linestyle='-', linewidth=0.5, color='r', label='Gyro Y(deg/s)')
plt.xlabel('Time(s)')
plt.ylabel('Y Rotation(deg/s)')
plt.legend()
plt.grid(True)

#Z axis
plt.subplot(3, 1, 3)
plt.plot(time_data, gyro_z, linestyle='-', linewidth=0.5, color='g', label='Gyro Z(deg/s)')
plt.xlabel('Time(s)')
plt.ylabel('Z Rotation(deg/s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
##############################################################
#Acceleration
plt.figure(figsize=(14, 10))

#X axis
plt.subplot(3, 1, 1)
plt.plot(time_data, accel_x, linestyle='-', linewidth=0.5, color='b', label='Accel X(m/s²)')
plt.xlabel('Time(s)')
plt.ylabel('X Acceleration(m/s²)')
plt.legend()
plt.grid(True)

#Y axis
plt.subplot(3, 1, 2)
plt.plot(time_data, accel_y, linestyle='-', linewidth=0.5, color='r', label='Accel Y(m/s²)')
plt.xlabel('Time(s)')
plt.ylabel('Y Acceleration(m/s²)')
plt.legend()
plt.grid(True)

#Z axis
plt.subplot(3, 1, 3)
plt.plot(time_data, accel_z, linestyle='-', linewidth=0.5, color='g', label='Accel Z(m/s²)')
plt.xlabel('Time(s)')
plt.ylabel('Z Acceleration(m/s²)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
##########################################
#VN  estimation
plt.figure(figsize=(14, 10))

#  X axis
plt.subplot(3, 1, 1)
plt.plot(time_data, rot_x, linestyle='-', linewidth=0.5, color='b', label='Rotation X')
plt.xlabel('Time(s)')
plt.ylabel('X Rotation(degrees)')
plt.legend()
plt.grid(True)

#Y axis 
plt.subplot(3, 1, 2)
plt.plot(time_data, rot_y, linestyle='-', linewidth=0.5, color='r', label='Rotation Y')
plt.xlabel('Time(s)')
plt.ylabel('Y Rotation(degrees)')
plt.legend()
plt.grid(True)

#Z axis
plt.subplot(3, 1, 3)
plt.plot(time_data, rot_z, linestyle='-', linewidth=0.5, color='g', label='Rotation Z')
plt.xlabel('Time(s)')
plt.ylabel('Z Rotation(degrees)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
##########################################################
#histograms
plt.figure(figsize=(14, 10))

#X axis histogram
plt.subplot(3, 1, 1)
plt.hist(rot_x, bins=50, color='blue', edgecolor='black')
plt.title('Histogram of Rotation in X')
plt.xlabel('Roll(degrees)')
plt.ylabel('Frequency')

#Y axis histogram
plt.subplot(3, 1, 2)
plt.hist(rot_y, bins=50, color='red', edgecolor='black')
plt.title('Histogram of Rotation in Y')
plt.xlabel('Pitch(degrees)')
plt.ylabel('Frequency')

#Z axis histogram
plt.subplot(3, 1, 3)
plt.hist(rot_z, bins=50, color='green', edgecolor='black')
plt.title('Histogram of Rotation in Z')
plt.xlabel('Yaw(degrees)')
plt.ylabel('Frequency')

plt.tight_layout()
plt.show()

