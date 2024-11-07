"""vnymr_read='$VNYMR,45.0,-10.0,5.0,0.35,-0.21,0.12,9.81,0.01,-9.80,0.05,0.02,-0.03*7A'
#print(vnymr_read)
vnymr_split1=str(vnymr_read).split('*')
print(vnymr_split1)
vnymr_join1=','.join(vnymr_split1)
vnymr=str(vnymr_join1).split(',')



#vnymr_split=vnymr_read.split(',|*',vnymr_read)
print(vnymr)
print(len(vnymr))
#$VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ*Checksum
yaw=float(vnymr[1])
pitch=float(vnymr[2])
roll=float(vnymr[3])
magx=vnymr[4]
magy=[5]
magz=[6]
accx=[7]
accy=[8]
accz=[9]
gyrox=[10]
gyroy=[11]
gyroz=[12]
check_sum=[13]

import numpy as np

def convert_to_quaternion(r1, p1, y1):
    r=np.radians(r1)
    p=np.radians(p1)
    y=np.radians(y1)

    cosr=np.cos(r*0.5)
    cosp=np.cos(p*0.5)
    cosy=np.cos(y*0.5)
    sinr=np.sin(r*0.5)
    sinp=np.sin(p*0.5)
    siny=np.sin(y*0.5)

    qw=cosr*cosp*cosy+sinr*sinp*siny
    qx=sinr*cosp*cosy-cosr*sinp*siny
    qy=cosr*sinp*cosy+sinr*cosp*siny
    qz=cosr*cosp*siny-sinr*sinp*cosy

    return float(qw),float(qx),float(qy),float(qz)
x=convert_to_quaternion(roll, pitch, yaw)
print(x)"""


#libraries to be used 
import rospy
import serial
import math
import sys
#sys.path.append('/home/catkin_ws/src/vn_driver')
#import vn_driver
from vn_driver.msg import Vectornav
import numpy as np


#checks whether the string contains $VNMYR string
def isVNYMRinString(inputString):
    return "$VNMYR" in inputString

#read data
def read_data(port):
    rawdata=port.readline().decode('utf-8').rstrip()
    imudata=rawdata.split(',')
    temp=imudata[-1].split('*')
    rospy.logdebug(imudata)
    data=np.ones(13)
    for i in range(1,12):
        data[i]=float(imudata[i])
    data[12]=float(temp[0])
    rospy.logdebug(data)
    print(data)
    return data, rawdata

#converting euler angles to quaternions
def convert_to_quaternion(data):
    r=np.radians(data[3])
    p=np.radians(data[2])
    y=np.radians(data[1])

    cosr=np.cos(r*0.5)
    cosp=np.cos(p*0.5)
    cosy=np.cos(y*0.5)
    sinr=np.sin(r*0.5)
    sinp=np.sin(p*0.5)
    siny=np.sin(y*0.5)

    qw=cosr*cosp*cosy+sinr*sinp*siny
    qx=sinr*cosp*cosy-cosr*sinp*siny
    qy=cosr*sinp*cosy+sinr*cosp*siny
    qz=cosr*cosp*siny-sinr*sinp*cosy

    #return float(qw),float(qx),float(qy),float(qz)
    return [qx,qy,qz,qw]

#configuring output
def configure_vectornav(port,freq,output_mode):
    try:
        port.write(b"$VNWRG,07,%d*xx\r"%freq)
        port.write(b"$VNWRG,06,%d*xx\r"%output_mode)
        check=0
        while check==0:
            aodf_freq=port.write(b"VNRRG,07*xx\r")
            AODF=port.readline()
            if str(freq) in str(aodf_freq) or str(freq) in str(AODF):
                check=1
        
        while check==1:
            aodf_freq1=port.write(b"VNRRH,06*xx\r")
            AODF=port.readline()
            if str(output_mode) in str(aodf_freq1) or str(output_mode) in str(AODF):
                check=0
        rospy.loginfo('IMU Configured')
    except Exception as e:
        rospy.logerr('IMU not configured: %s'%str(e))

if __name__=="__main__":
    rospy.init_node('imu_driver', anonymous=True)
    args=rospy.myargv(argv=sys.argv)
    if len(args)<2:
        print('Usage: IMU driver.py <port>')
        sys.exit(1)
    
    port_param=args[1]

    port=serial.Serial(port_param,115200)
    freq=40
    output_mode=14
    configure_vectornav(port,freq,output_mode)
    rate=rospy.Rate(10)

    imu_pub=rospy.Publisher('imu',Vectornav,queue_size=10)

    try:
        while not rospy.is_shutdown():
            try:
                data, rawdata=read_data(port)
                quaternions=convert_to_quaternion(data)
            
                msg=Vectornav()
                msg.header.stamp=rospy.Time.now()
                msg.header.frame_id="imu1_frame"
                msg.imu.header.frame_id="imu1_frame"
                msg.imu.header.stamp=rospy.Time.now()
                msg.imu.orientation.x=quaternions[0]
                msg.imu.orientation.y=quaternions[1]
                msg.imu.orientation.z=quaternions[2]
                msg.imu.orientation.w=quaternions[3]
                msg.mag_field.header.frame_id="imu1_frame"
                msg.mag_field.header.stamp=rospy.Time.now()
                msg.mag_field.magnetic_field.x=data[4]
                msg.mag_field.magnetic_field.y=data[5]
                msg.mag_field.magnetic_field.z=data[6]
                msg.imu.linear_acceleration.x=data[7]
                msg.imu.linear_acceleration.y=data[8]
                msg.imu.linear_acceleration.z=data[9]
                msg.imu.angular_velocity.x=data[10]
                msg.imu.angular_velocity.y=data[11]
                msg.imu.angular_velocity.z=data[12]
                msg.raw_imudata=rawdata
                rospy.loginfo('Publishing info')
                rospy.loginfo(msg)
                imu_pub.publish(msg)
            except ValueError as e:
                rospy.logwarn('ValueError: Could not convert data'+str(e))
            except IndexError as e:
                rospy.logwarn('IndexError: data out of range:'+str(e))
    except rospy.ROSInterruptException:
        pass

    port.close()



