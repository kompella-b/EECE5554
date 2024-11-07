import numpy as np
import matplotlib.pyplot as plt
import rosbag
import allantools as allan

time_data=[]
gyro_x=[]
gyro_y=[]
gyro_z=[]

imu_topic='/vectornav'
with rosbag.Bag('/home/skompe/Downloads/LocationA.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        data_values=msg.data.split(',')
        if len(data_values)==13:
            try:
                gyro_x.append(float(data_values[7]))
                gyro_y.append(float(data_values[8]))
                gyro_z.append(float(data_values[9]))
                time_data.append(t.to_sec())
            except ValueError:
                print("Failed to parse gyroscope data")
                continue
        else:
            print("Unexpected data format")
            continue
#converting array to numpy arrays for easy processing
gyro_x=np.array(gyro_x)
gyro_y=np.array(gyro_y)
gyro_z=np.array(gyro_z)
time_data=np.array(time_data)

#calculating ts
if len(time_data)>1:
    ts=np.mean(np.diff(time_data))
    #print(ts)
else:
    print("Insufficient data")
    ts=0

if ts>0:
    tx=np.cumsum(gyro_x)*ts
    ty=np.cumsum(gyro_y)*ts
    tz=np.cumsum(gyro_z)*ts

#tx = np.zeros(len(gyro_x))
#ty = np.zeros(len(gyro_y))
#tz = np.zeros(len(gyro_z))
"""
for i in range(1, len(gyro_x)):
    dt = time_data[i] - time_data[i - 1]  # Use the actual time interval
    tx[i] = tx[i - 1] + (gyro_x[i] + gyro_x[i - 1]) * dt / 2
    ty[i] = ty[i - 1] + (gyro_y[i] + gyro_y[i - 1]) * dt / 2
    tz[i] = tz[i - 1] + (gyro_z[i] + gyro_z[i - 1]) * dt / 2
"""
#tx=np.cumsum(gyro_x)*ts
#ty=np.cumsum(gyro_y)*ts
#tz=np.cumsum(gyro_z)*ts

#print(tx[:10])
#print(ty[:10])
#print(tz[:10])
"""
def AllanDeviation(arr,ts,maxnum=100):
    n=len(arr)
    m_max=2**np.floor(np.log2(n/2))
    m=np.logspace(np.log10(1),np.log10(m_max),num=maxnum)
    m=np.ceil(m).astype(int)
    m=np.unique(m)
    tau=m*ts

    allanvar=np.zeros(len(m))
    for i,mi in enumerate(m):
        twomi=2*mi
        mi=int(mi)
        allanvar[i]=np.sum(arr[twomi:]-(2*arr[mi:-mi]+arr[0:-twomi])**2)

        allanvar/=(2*tau*2(n-2*m))
    return tau,np.sqrt(allanvar)

(tau_x,ad_x)=AllanDeviation(tx,ts,maxnum=200)
(tau_y,ad_y)=AllanDeviation(ty,ts,maxnum=200)
(tau_z,ad_z)=AllanDeviation(tz,ts,maxnum=200)
"""

rate=1/ts
tau_x,ad_x,_,_=allan.oadev(tx,rate=rate,data_type='phase',taus='all')
tau_y,ad_y,_,_=allan.oadev(ty,rate=rate,data_type='phase',taus='all')
tau_z,ad_z,_,_=allan.oadev(tz,rate=rate,data_type='phase',taus='all')

#calculating bias stability, angle random walk and rate random walk
#for bias stability B=minimum point in data before it increases again



print(tau_x[:10])
print(ad_x[:10])

plt.figure()
plt.title('Allan Deviation for Accelerometer')
plt.plot(tau_x, ad_x, label='gx')
plt.plot(tau_y, ad_y, label='gy')
plt.plot(tau_z, ad_z, label='gz')
plt.xlabel(r'$\tau$(sec)')
plt.ylabel('Deviation(deg/sec)')
plt.grid(True, which='both', ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()