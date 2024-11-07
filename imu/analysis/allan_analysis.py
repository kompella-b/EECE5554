import numpy as np
import matplotlib.pyplot as plt
import rosbag
import allantools as allan
import re

time_data=[]
gyro_x=[]
gyro_y=[]
gyro_z=[]

imu_topic='/vectornav'
with rosbag.Bag('/home/skompe/Downloads/LocationC.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        data_values=msg.data.split(',')
        if len(data_values)==13:
            try:
                #using re to clean and parse gyro data
                gyro_x_val = re.sub(r'[^\d\.\-]', '', data_values[10].strip())
                gyro_y_val = re.sub(r'[^\d\.\-]', '', data_values[11].strip())
                gyro_z_val = re.sub(r'[^\d\.\-]', '', data_values[12].split('*')[0].strip())
                try:
                    gyro_x.append(float(gyro_x_val))
                except ValueError:
                    print(f"Invalid gyro_x value '{gyro_x_val}'; setting to 0")
                    gyro_x.append(0)

                try:
                    gyro_y.append(float(gyro_y_val))
                except ValueError:
                    print(f"Invalid gyro_y value '{gyro_y_val}'; setting to 0")
                    gyro_y.append(0)

                try:
                    angular_rate_z = float(gyro_z_val)
                    gyro_z.append(angular_rate_z)
                except ValueError:
                    print(f"Invalid gyro_z value '{gyro_z_val}'; setting to 0")
                    gyro_z.append(0)

                time_data.append(t.to_sec())
            except ValueError:
                print("Failed to parse gyroscope data in gyro_x or gyro_y")
                continue
        else:
            print("Unexpected data format")
            continue

#converting array to numpy arrays for easy processing
gyro_x=np.array(gyro_x)
gyro_y=np.array(gyro_y)
gyro_z=np.array(gyro_z,dtype=float)
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

        allanvar/=(2*tau**2*(n-2*m))
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
def calculations(tau,ad):
    base_index=np.argmin(ad)
    b=ad[base_index]
    base_tau=tau[base_index]

    #for angle random walk, N=value at tau=1
    one_index=np.argmin(np.abs(tau-1))
    N=ad[one_index]

    #for rate random walk, K=slope of +0.5 region
    log_tau,log_ad=np.log10(tau),np.log10(ad)
    slope=np.diff(log_ad)/np.diff(log_tau)
    slope_range=(0.3, 0.7)  #widened range for flexibility
    k_index=np.where((slope>=slope_range[0])&(slope<=slope_range[1]))[0]
    if k_index.size>0:
        K=ad[k_index[0]]*np.sqrt(tau[k_index[0]])
    else:
        print("No +0.5 slope")
        K=None  # No region with +0.5 slope found
    #k_index=np.where((slope>=0.4)&(slope<=0.6))[0]
    #K=ad[k_index[0]]*np.sqrt(tau[k_index[0]]) if k_index.size>0 else None
    return N,b,K,base_tau


print(tau_x[:10])
print(ad_x[:10])
fig, axs = plt.subplots(3, 1, figsize=(10, 12))
noise_parameters=[calculations(tau_x,ad_x),calculations(tau_y,ad_y),calculations(tau_z,ad_z)]
for i, (tau,ad,params,color,axis) in enumerate(zip([tau_x, tau_y, tau_z], 
                                                        [ad_x, ad_y, ad_z], 
                                                        noise_parameters,
                                                        ["red", "green", "blue"], 
                                                        ["X", "Y", "Z"])):
    #N, B, K, BI_tau = params
    N, B, K, BI_tau = params
    axs[i].loglog(tau, ad, color=color, label=f'Gyro Allan deviation Angle{axis}\nN={N:.2e}, B={B:.2e}, K={K}')

    axs[i].plot(BI_tau, B, 'o', label="Bias Instability(B)")
    axs[i].legend()
    axs[i].grid(True)
    axs[i].set_xlabel("Time(tau)")
    axs[i].set_ylabel("Allan Deviation")
#fig.suptitle("Allan Deviation Plot for Gyroscope Angles")
plt.show()
"""
plt.figure()

plt.title('Gyro Allan Deviation')
plt.plot(tau_x, ad_x, label='gx')
plt.plot(tau_y, ad_y, label='gy')
plt.plot(tau_z, ad_z, label='gz')
plt.xlabel(r'$\tau$(sec)')
plt.ylabel('Deviation(deg/sec)')
plt.grid(True, which='both', ls="-", color='0.65')
#plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()
"""







