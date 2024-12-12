import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from scipy import signal
from scipy import integrate

time_data = []
gyro_x = []
gyro_y = []
gyro_z = []
accel_x = []
accel_y = []
accel_z = []
rot_x = []
rot_y = []
rot_z = []
mag_x = []
mag_y = []
mag_z = []
imu_heading = []
latitude = []
longitude = []

imu_topic='/imu'
gps_topic='/gps_data'
with rosbag.Bag('/home/skompe/EECE5554/lab5/src/data/circles2.bag', 'r') as bag:
    start_time=None

    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        if start_time is None:
            start_time=t.to_sec()

        time_data.append(t.to_sec()-start_time)
        mag_x.append(msg.magnetic_field.x)
        mag_y.append(msg.magnetic_field.y)
        mag_z.append(msg.magnetic_field.z)
        #r = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z)
        gyro_x.append(msg.angular_velocity.x) 
        gyro_y.append(msg.angular_velocity.y)
        gyro_z.append(msg.angular_velocity.z)
        accel_x.append(msg.linear_acceleration.x)
        accel_y.append(msg.linear_acceleration.y)
        accel_z.append(msg.linear_acceleration.z)
        imu_heading.append(msg.orientation.z)

    for topic, msg, t in bag.read_messages(topics=[gps_topic]):
        latitude.append(msg.latitude)
        longitude.append(msg.longitude)

bag.close()

mag_x = np.array(mag_x)
mag_y = np.array(mag_y)
mag_z = np.array(mag_z)
gyro_x = np.array(gyro_x)
gyro_y = np.array(gyro_y)
gyro_z = np.array(gyro_z)
time_data = np.array(time_data)
accel_x = np.array(accel_x)
accel_y = np.array(accel_y)
accel_z = np.array(accel_z)
latitude = np.array(latitude)
longitude = np.array(longitude)
mag_data = np.array([mag_x, mag_y, mag_z]).T
#print("-----")
print(latitude[0:10])
#calculating hard iron and soft iron corrections
def calibrate(data):
    off_hi = np.mean(data, axis=0)
    centered = data - off_hi
    covariance = np.cov(centered.T)
    eigen_val, eigen_vec = np.linalg.eig(covariance)

    si = np.linalg.inv(np.sqrt(np.diag(eigen_val)))@eigen_vec.T
    calibrated_data = centered@si.T

    return calibrated_data, off_hi, si

mag_data_calibrated, hardIron_offset, softIron = calibrate(mag_data)

def yaw_calculation(data):
    yaw = np.arctan2(data[:, 1], data[:, 0])
    return yaw

yaw_raw = yaw_calculation(mag_data)
yaw_raw_deg = np.degrees(yaw_raw)
yaw_corrected = yaw_calculation(mag_data_calibrated)
yaw_corrected_deg = np.degrees(yaw_corrected)

yaw_gyro_angle = cumulative_trapezoid(gyro_z, time_data, initial=0)
yaw_gyro_angle_deg = np.degrees(yaw_gyro_angle)

def lowpass(cutoff, fs, order=4):
    nyquist = 0.5*fs
    normal_cutoff = cutoff/nyquist
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def highpass(cutoff, fs, order=4):
    nyquist = 0.5*fs
    normal_cutoff = cutoff/nyquist
    b, a = signal.butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def applying_filter(data, b, a):
    return signal.filtfilt(b, a, data)

mag_yaw = np.arctan2(mag_y, mag_x)
#lowpass
fs=1/np.mean(np.diff(time_data))
lowpass_cutoff = 0.5
b_lp, a_lp = lowpass(lowpass_cutoff, fs)
mag_yaw_lowpass = applying_filter(mag_yaw, b_lp, a_lp)

#highpass
highpass_cutoff = 0.01
b_hp, a_hp = highpass(highpass_cutoff, fs)
gyro_highpass = applying_filter(gyro_z, b_hp, a_hp)

gyro_highpass = integrate.cumulative_trapezoid(gyro_highpass, time_data, initial=0)
alpha = 0.98
comp_yaw = alpha*gyro_highpass + (1-alpha)*mag_yaw_lowpass

raw_vel = cumulative_trapezoid(accel_x, time_data, initial = 0)
bias = np.mean(raw_vel[0:50])
adjusted_vel = raw_vel-bias
"""
R = 6371000
lat_rad= np.radians(latitude)
lon_rad = np.radians(longitude)
x = R*np.cos(lat_rad)*np.cos(lon_rad)
y = R*np.cos(lat_rad)*np.sin(lon_rad)

dx = np.diff(x)
dy = np.diff(y)
dt = np.diff(time_data)

dt[dt==0] = np.nan
velo = np.sqrt(dx**2+dy**2) /dt
velo = np.nan_to_num(velo)
accel_x_obs = accel_x  
yaw_rate_integral = cumulative_trapezoid(gyro_z, time_data, initial=0)  

velocity_x = cumulative_trapezoid(accel_x, time_data, initial=0)

omega_x_prime = gyro_z * velocity_x
y_double_prime_obs = np.zeros_like(velocity_x)

heading = yaw_rate_integral
ve = velocity_x * np.cos(heading)  
vn = velocity_x * np.sin(heading)  

xe = cumulative_trapezoid(ve, time_data, initial=0)
xn = cumulative_trapezoid(vn, time_data, initial=0)

x_normalized = x - x[0]
y_normalized = y - y[0]

xe_normalized = xe - xe[0]
xn_normalized = xn - xn[0]

#print(hardIron_offset)
#print(softIron)
"""
#plotting results
plt.figure(figsize=(12, 6))
#uncorrected
plt.subplot(1, 2, 1)
plt.scatter(mag_data[:, 0], mag_data[:, 1], color='red', alpha=0.5, label='Uncorrected')
plt.title('Uncorrected Magnatometer Data')
plt.xlabel('X')
plt.ylabel('Y')
plt.subplot(1, 2, 2)
plt.scatter(mag_data_calibrated[:, 0], mag_data_calibrated[:, 1], color='blue', alpha=0.5, label='Corrected')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Corrected Magnatometer Data')
plt.tight_layout()
plt.show()
"""
plt.figure(figsize=(10, 6))
plt.plot(yaw_raw_deg, label='Raw Yaw(degrees)', color='blue', alpha=0.7)
plt.plot(yaw_corrected_deg, label='Corrected Yaw(degrees)', color='red', alpha=0.7)
plt.title('Yaw Angles: Raw vs Corrected')
plt.xlabel('Sample Index')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_data, yaw_gyro_angle_deg, label='Gyro Yaw Angle (degrees)', color='blue')
plt.title('Gyroscope Yaw Angle Estimation vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(time_data, np.degrees(mag_yaw_lowpass), label='Magnetometer Yaw (Low-pass)')
plt.title('Low-pass Filtered Magnetometer Yaw')
plt.xlabel('Time(s)')
plt.ylabel('Yaw(degrees)')
plt.grid()
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time_data, np.degrees(gyro_highpass), label='Gyroscope Yaw (High-pass)', color='red')
plt.title('High-pass Filtered Gyroscope Yaw')
plt.xlabel('Time(s)')
plt.ylabel('Yaw(degrees)')
plt.grid()
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time_data, np.degrees(comp_yaw), label='Complementary Filter Yaw', color='blue')
plt.title('Complementary Filter Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (degrees)')
plt.grid()
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_data, imu_heading, label='IMU Heading Estimate', color='purple')
plt.title('IMU Heading Estimate')
plt.xlabel('Time (s)')
plt.ylabel('Heading (unitless)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))

plt.plot(time_data, raw_vel, label='Raw Velocity', color='blue', alpha=0.7)

plt.plot(time_data, adjusted_vel, label='Adjusted Velocity', color='red', alpha=0.7)

plt.title('Forward Velocity from Accelerometer')
plt.xlabel('Time(seconds)')
plt.ylabel('Velocity(m/s)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))

plt.plot(time_data[1:], velo, label='GPS Velocity', color='blue')
plt.title('Forward Velocity from GPS')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(x_normalized, y_normalized, label='GPS Trajectory', color='blue')
plt.title('GPS Trajectory')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 2)
plt.plot(xe_normalized, xn_normalized, label='IMU Trajectory (Dead Reckoning)', color='red')
plt.title('IMU Trajectory (Dead Reckoning)')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(time_data, omega_x_prime, label='ωx\'', color='green')
plt.plot(time_data, y_double_prime_obs, label='y\'\'obs (Assumed 0)', color='purple', linestyle='--')
plt.title('Comparison of ωx\' and y\'\'obs')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(x_normalized, y_normalized, label='GPS Trajectory', color='blue')
plt.plot(xe_normalized, xn_normalized, label='IMU Trajectory (Dead Reckoning)', color='red')
plt.title('GPS vs IMU Trajectory')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()


scaling_factor = 1
print(f"Scaling Factor for IMU trajectory: {scaling_factor}")
"""