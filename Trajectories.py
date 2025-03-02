import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import savgol_filter

# Load the image

# Load the image
image = cv2.imread('ankle_angle_r.png')  # Replace with your image file path
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image to create a binary mask
_, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

# Detect edges using Canny edge detection
edges = cv2.Canny(binary, threshold1=50, threshold2=150)

# Find the coordinates of the white pixels (data points)
coordinates = np.column_stack(np.where(edges > 0))

# Flip the Y-axis (since image origin is top-left, not bottom-left)
height, width = image.shape[:2]
coordinates[:, 0] = height - coordinates[:, 0]

# Normalize coordinates to the plot's axis scale (manual calibration needed)
# Update these based on your image's axis range
x_min, x_max = 0.0, 1.0  # X-axis range in the plot
y_min, y_max = -15.0, 10.0  # Y-axis range in the plot

# Scale pixel coordinates to the actual data range
x = coordinates[:, 1] / width * (x_max - x_min) + x_min
y = coordinates[:, 0] / height * (y_max - y_min) + y_min

# Plot the digitized data to verify
plt.figure(figsize=(8, 6))
plt.scatter(x, y, s=1, c='red', label='Digitized Points')
plt.title('Digitized Data from Plot')
plt.xlabel('Time')
plt.ylabel('Ankle Angle (degrees)')
plt.legend()
plt.grid()
plt.show()


# Load digitalized data
data = pd.read_csv('digitized_data_AL.csv')
time = data['Time'].values
angle = data['Ankle_Angle'].values

# Define the valid range for the signal
valid_range = (-15, 10)
cleaned_time = time[(angle >= valid_range[0]) & (angle <= valid_range[1])]
cleaned_angle = angle[(angle >= valid_range[0]) & (angle <= valid_range[1])]

# Smooth the signal using Savitzky-Golay Filter
window_length = 15
polyorder = 2
smoothed_angle = savgol_filter(cleaned_angle,window_length, polyorder)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time, angle, 'o', markersize=3, label='Original Data', alpha=0.6)
plt.plot(cleaned_time, smoothed_angle, '-', linewidth=2, label='Cleaned and Smoothed Signal')
plt.title('Signal Cleaning and Smoothing')
plt.xlabel('Time')
plt.ylabel('Ankle Angle')
plt.legend()
plt.grid()
plt.show()


# Save the digitized data to a CSV file
#data = pd.DataFrame({'Time': x, 'Ankle_Angle': y})
#data.to_csv('digitized_data_AR.csv', index=False)
#print("Digitized data saved to 'digitized_data.csv'")



