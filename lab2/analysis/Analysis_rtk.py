import bagpy
import pandas as pd
import statistics
import math
import matplotlib.pyplot as plt
import numpy as np
from bagpy import bagreader
Data1_open = bagreader('/home/skompe/EECE5554/rtk_gnss/data/Stat_open_rename.bag')
Data2_oc = bagreader('/home/skompe/EECE5554/rtk_gnss/data/Stat_occluded_rename.bag')
Data3_mopen =bagreader('/home/skompe/EECE5554/rtk_gnss/data/Square_open_sree.bag')
Data4_moc =bagreader('/home/skompe/EECE5554/rtk_gnss/data/Square_occluded_sree.bag')
#Data5_mov_oc=bagreader('/home/.../..../...bagfile.bag')
print(Data1_open.topic_table)
print(Data2_oc.topic_table)
print(Data3_mopen.topic_table) 
print(Data4_moc.topic_table)
Data_Convert_open=Data1_open.message_by_topic('/gps')
Data_Convert_oc=Data2_oc.message_by_topic('/gps')
Data_Convert_D3_mopen = Data3_mopen.message_by_topic('/gps')
Data_Convert_D4_moc = Data4_moc.message_by_topic('/gps')
#Data_Convert_D5_mov_oc = Data5_mov_oc.message_by_topic('/gps_data')

DCSV_open = pd.read_csv(Data_Convert_open)
DCSV_oc = pd.read_csv(Data_Convert_oc)
DCSV_mo = pd.read_csv(Data_Convert_D3_mopen)
DCSV_moc = pd.read_csv(Data_Convert_D4_moc)
#DCSV_D5 = pd.read_csv(Data_Convert_D5_mov_oc) 
#DCSV_open.head()
#DCSV_oc.head()
#DCSV_mo.head()
DCSV_moc.head()

def deviation(data):
    offset = data - data[0] #removing 1st point
    mean_offset = offset.mean() #finding centeroid
    deviation = offset - mean_offset #deviation from mean
    return (deviation, mean_offset)
    
def calculate(data):
     deviation_n, mean_n = deviation(data['utm_northing'])
     deviation_e, mean_e = deviation(data['utm_easting'])
     centroid = [mean_n, mean_e]
     euclidian = np.sqrt(np.square(deviation_n - mean_n) + np.square(deviation_e - mean_e))
     return deviation_n, mean_n,deviation_e, mean_e,centroid , euclidian

Dev_n,Mean_n,Dev_e,Mean_e,centroid_open, Euclidian_open = calculate(DCSV_open)
Dev_No,Mean_no,Dev_Eo,Mean_Eo,centroid_occ, Euclidian_occ = calculate(DCSV_oc)
print(DCSV_mo['utm_northing'])
# dev_nmo,mean_nmo = (DCSV_mo['utm_northing'])
# dev_emo,mean_emo = (DCSV_mo['utm_easting'])
#Dev_nmo,Mean_nmo,Dev_emo,Mean_emo,centroid_Mopen, _ = calculate(DCSV_mo)
#Dev_nmoc,Mean_nmoc,Dev_emoc,Mean_emoc,centroid_Mocc, _ = calculate(DCSV_moc)

#Stationary northing vs. easting scatterplots (open and occluded on same fig with different markers)
#plt.figure(figsize=(8, 6))
plt.scatter(Dev_e,Dev_n, label='Open', marker='o')
plt.scatter(Dev_Eo,Dev_No, label='Occuluded', marker='x')
plt.scatter(centroid_open[1], centroid_open[0], color='red', label='Centroid Open', marker='*')
plt.scatter(centroid_occ[1], centroid_occ[0], color='blue', label='Centroid Occluded', marker='*')
plt.xlabel('Easting Offset (m)')
plt.ylabel('Northinging Offset (m)')
plt.title('Northing vs. Easting (Open vs. Occluded)')
plt.text(centroid_open[0], centroid_open[1],({centroid_open[0]},{centroid_open[1]}), color='red')
plt.text(centroid_occ[0], centroid_occ[1],({centroid_occ[0]},{centroid_occ[1]}), color='blue')
plt.legend()
plt.savefig('/home/skompe/EECE5554/lab2/Static_OpenvsOccluded.png')
plt.show()

#Stationary Altitude and Time graph
plt.scatter(DCSV_open['Time'],DCSV_open['altitude'],label = 'Stationary Open' , marker = 'o')
plt.scatter(DCSV_oc['Time'],DCSV_oc['altitude'] , label = 'Stationary Occluded' , marker = 'x')
plt.xlabel('Time')
plt.ylabel('Altitude')
plt.title('Altitude and Time')
plt.legend()
plt.savefig('/home/skompe/EECE554/lab2/Static_Altitude_Time.png')
plt.show()

#Stationary histogram plots for position - Open
plt.hist(Euclidian_open, color='blue', edgecolor='black',label = 'Open')
plt.xlabel('Frequency')
plt.ylabel('Euclidian Distance')
plt.title('Histogram Open')
plt.legend()
plt.savefig('/home/skompe/EECE5554/lab2/Static_hist_open.png')
plt.show()

#Stationary histogram plots for position - Occluded
plt.hist(Euclidian_occ, color='blue', edgecolor='black',label = 'occluded')
plt.xlabel('Frequency')
plt.ylabel('Euclidian Distance')
plt.title('Histogram Occluded')
plt.legend()
plt.savefig('/home/skompe/EECE5554/lab2/Static_hist_occ.png')
plt.show()

#Moving data northing vs. easting scatterplot with line of best fit 
plt.scatter(DCSV_mo['utm_northing'], DCSV_mo['utm_easting'], label='Moving Open', marker='o')
plt.scatter(DCSV_moc['utm_northing'],DCSV_moc['utm_easting'],label='Moving Occluded',marker='*')
#x,y = np.polyfit(DCSV_mo['utm_northing'], DCSV_mo['utm_easting'],1) #x, y = np.polyfit(a, b, 1)
#plt.plot(DCSV_mo['utm_northing'],x*DCSV_mo['utm_northing']+y,color='black',linestyle = '-',linewidth = 2) #plt.plot(a, x*a+y)
#a,b = np.polyfit(DCSV_moc['utm_northing'],DCSV_moc['utm_easting'],1)
#plt.plot(DCSV_moc['utm_northing'],a*DCSV_moc['utm_northing']+b,color='black',linestyle = '-',linewidth = 2) 
plt.xlabel('Northing Offset (m)')
plt.ylabel('Easting Offset (m)')
plt.title('Moving Northing vs. Easting (Moving Open vs. Occluded)')
plt.legend(loc=4)
plt.savefig('/home/skompe/EECE5554/lab2/Moving_OpVsOc_Plot.png')
plt.show()

#Moving data altitude vs. time plot (open and occluded on same fig with different markers) 
plt.scatter(DCSV_mo['Time'],DCSV_mo['altitude'],label = 'Moving Open' , marker = 'o') 
plt.scatter(DCSV_moc['Time'],DCSV_moc['altitude'] , label = 'Moving Occluded' , marker = 'x')
plt.xlabel('Altitude') 
plt.ylabel('Time') 
plt.title('Altitude and Time') 
plt.legend()
plt.savefig('/home/skompe/EECE5554/lab2/Moving_AltivsTime_Plot.png') 
plt.show()

