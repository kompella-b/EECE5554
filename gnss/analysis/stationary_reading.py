import datetime
import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_rosbag(bag_path, topic):
    bag = rosbag.Bag(bag_path)

    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'easting': [], 'northing': []}

    for _, msg, t in bag.read_messages(topics=[topic]):
        data['timestamp'].append(t.to_sec())
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        data['easting'].append(msg.utm_easting)
        data['northing'].append(msg.utm_northing)

    bag.close()

    return pd.DataFrame(data)

def analyze_stationary_data(open_df, occluded_df):
    # Subtract the first point from each dataset (remove offset)
    open_easting_offset = open_df['easting'].iloc[0]
    open_northing_offset = open_df['northing'].iloc[0]
    open_df['easting'] -= open_easting_offset
    open_df['northing'] -= open_northing_offset

    occluded_easting_offset = occluded_df['easting'].iloc[0]
    occluded_northing_offset = occluded_df['northing'].iloc[0]
    occluded_df['easting'] -= occluded_easting_offset
    occluded_df['northing'] -= occluded_northing_offset

    # Calculate centroids
    open_centroid = {
        'easting': np.mean(open_df['easting']),
        'northing': np.mean(open_df['northing'])
    }

    occluded_centroid = {
        'easting': np.mean(occluded_df['easting']),
        'northing': np.mean(occluded_df['northing'])
    }

    # Subtract centroid from each point
    open_df['easting'] -= open_centroid['easting']
    open_df['northing'] -= open_centroid['northing']
    occluded_df['easting'] -= occluded_centroid['easting']
    occluded_df['northing'] -= occluded_centroid['northing']

    # Convert to NumPy arrays
    open_timestamps = open_df['timestamp'].to_numpy()
    open_altitude = open_df['altitude'].to_numpy()
    occluded_timestamps = occluded_df['timestamp'].to_numpy()
    occluded_altitude = occluded_df['altitude'].to_numpy()

    # Plot 1: Stationary Northing vs. Easting Scatterplots
    plt.figure()
    plt.scatter(open_df['easting'], open_df['northing'], label='Open Spot', marker='o', color='blue')
    plt.scatter(occluded_df['easting'], occluded_df['northing'], label='Occluded Spot', marker='x', color='red')
    plt.title('Stationary Northing vs. Easting (After Subtracting Centroid)')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()
    #plt.text(0.05, 0.95, f"Open Centroid: {open_centroid}\nOccluded Centroid: {occluded_centroid}", 
             #transform=plt.gca().transAxes, fontsize=8, verticalalignment='top')

    # Plot 2: Stationary Altitude vs. Time
    plt.figure()
    plt.plot(open_timestamps, open_altitude, label='Open Spot', marker='o', color='blue')
    plt.plot(occluded_timestamps, occluded_altitude, label='Occluded Spot', marker='x', color='red')
    plt.title('Stationary Altitude vs. Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    # Calculate Euclidean distance to centroids
    open_distances = np.sqrt((open_df['easting'])**2 + (open_df['northing'])**2)
    occluded_distances = np.sqrt((occluded_df['easting'])**2 + (occluded_df['northing'])**2)

    # Plot 3: Histogram for Open Spot distances
    plt.figure()
    plt.hist(open_distances, bins=20, alpha=0.7, label='Open Spot - Distance from Centroid', color='blue')
    plt.title('Open Spot Distance from Centroid Histogram')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.legend()

    # Plot 4: Histogram for Occluded Spot distances
    plt.figure()
    plt.hist(occluded_distances, bins=20, alpha=0.7, label='Occluded Spot - Distance from Centroid', color='red')
    plt.title('Occluded Spot Distance from Centroid Histogram')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.legend()

def analyze_moving_data(open_df, occluded_df):
    # Scatter plot with line of best fit
    plt.figure()
    plt.scatter(open_df['easting'], open_df['northing'], label='Open Spot', marker='o', color='blue')
    plt.scatter(occluded_df['easting'], occluded_df['northing'], label='Occluded Spot', marker='x', color='red')

    # Fit lines
    open_fit = np.polyfit(open_df['easting'], open_df['northing'], 1)
    occluded_fit = np.polyfit(occluded_df['easting'], occluded_df['northing'], 1)

    plt.plot(open_df['easting'], np.polyval(open_fit, open_df['easting']), color='blue', linestyle='--', label='Open Spot Best Fit')
    plt.plot(occluded_df['easting'], np.polyval(occluded_fit, occluded_df['easting']), color='red', linestyle='--', label='Occluded Spot Best Fit')

    plt.title('Moving Northing vs. Easting with Best Fit Line')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    # Convert to NumPy arrays
    open_timestamps = open_df['timestamp'].to_numpy()
    open_altitude = open_df['altitude'].to_numpy()
    occluded_timestamps = occluded_df['timestamp'].to_numpy()
    occluded_altitude = occluded_df['altitude'].to_numpy()

    # Moving Altitude vs. Time plot
    plt.figure()
    plt.plot(open_timestamps, open_altitude, label='Open Spot', marker='o', color='blue')
    plt.plot(occluded_timestamps, occluded_altitude, label='Occluded Spot', marker='x', color='red')
    plt.title('Moving Altitude vs. Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Altitude (meters)')
    plt.legend()

if __name__ == '__main__':
    open_spot_df = load_rosbag('/home/skompe/catkin_ws/src/gps_driver/data/no_building_stationary.bag', '/gps')
    occluded_spot_df = load_rosbag('/home/skompe/catkin_ws/src/gps_driver/data/building_stationary.bag', '/gps')

    # Analyze stationary data
    analyze_stationary_data(open_spot_df, occluded_spot_df)

    # Analyze moving data (use moving data if available)
    # analyze_moving_data(open_spot_df, occluded_spot_df)

    # Show the plots
    plt.show()