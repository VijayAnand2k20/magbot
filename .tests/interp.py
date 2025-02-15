import pandas as pd
import numpy as np
from transforms3d.euler import quat2euler  # Or use ROS 2's tf_transformations

# Load data
odom_df = pd.read_csv("/home/jarvis/magic/ROS/magbot_ws/odom_export.csv")
gt_df = pd.read_csv("/home/jarvis/magic/ROS/magbot_ws/ground_truth_export.csv")

# 1. Set 'timestamp' as index for BOTH DataFrames
odom_df = odom_df.set_index('timestamp')
gt_df = gt_df.set_index('timestamp')

# 2. Reindex one DataFrame to match the other (important!)
#    This ensures both DataFrames have the SAME timestamps
common_timestamps = odom_df.index.intersection(gt_df.index)  # Get common timestamps
odom_df = odom_df.loc[common_timestamps]
gt_df = gt_df.loc[common_timestamps]

# 3. Interpolate (if necessary, after reindexing) - MOST IMPORTANT
odom_df = odom_df.interpolate(method='linear', axis=0)
gt_df = gt_df.interpolate(method='linear', axis=0)

# 4. Convert quaternions (if needed, after reindexing and interpolation)
gt_quaternions = gt_df[['qx', 'qy', 'qz', 'qw']].values
gt_euler = []
for q in gt_quaternions:
    q_reordered = [q[3], q[0], q[1], q[2]]
    euler = quat2euler(q_reordered)
    gt_euler.append(euler)
gt_euler = np.array(gt_euler)

# 5. Extract data (AFTER reindexing and interpolation)
odom_data = odom_df[['x', 'y']].values
gt_data = gt_df[['x', 'y']].values

# 6. Calculate errors (shapes should now match!)
errors = np.sqrt((odom_data[:, 0] - gt_data[:, 0])**2 + (odom_data[:, 1] - gt_data[:, 1])**2)

odom_df.to_csv("odom_interpolated.csv")
gt_df.to_csv("ground_truth_interpolated.csv")

# Now plot the aligned data and the errors
import matplotlib.pyplot as plt
plt.plot(gt_data[:, 0], gt_data[:, 1], label="Ground Truth")
plt.plot(odom_data[:, 0], odom_data[:, 1], label="Odom (Aligned)")
plt.legend()
plt.show()

plt.plot(gt_df.index, errors)
plt.xlabel("Timestamp")
plt.ylabel("Error")
plt.show()

print(f"Mean Error: {np.mean(errors)}")