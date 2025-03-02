import pandas as pd
import matplotlib.pyplot as plt

# Load the dataset
df = pd.read_csv("filtered_data.csv")
print(df.head())

# Count duplicate time values
duplicate_counts = df['Time'].value_counts()

# Filter only duplicate time values
duplicate_times = duplicate_counts[duplicate_counts>1]

# Remove duplicates by averafing the Ankle_Angle for each unique Time
df_cleaned = df.groupby("Time", as_index=False).mean()

# Plot cleaned trajectory
plt.figure(figsize=(10, 6))
plt.plot(df_cleaned["Time"], df_cleaned["Ankle_Angle"], color="blue", label="Ankle Angle Trajectory")
plt.xlabel("Time (s)")
plt.ylabel("Ankle Angle (degrees)")
plt.title("Cleaned Ankle Angle Trajectory Over Time")
plt.legend()
plt.grid()
plt.show()

#df_cleaned.to_csv("cleaned_trajectory.csv", index=False)