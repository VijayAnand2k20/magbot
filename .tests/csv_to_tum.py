import pandas as pd

def convert_csv_to_tum(csv_file, tum_file):
    df = pd.read_csv(csv_file)
    print("Columns in CSV:", df.columns)  # This will print: Index(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'], dtype='object')
    
    with open(tum_file, "w") as f:
        f.write("# timestamp x y z qx qy qz qw\n")
        
        for _, row in df.iterrows():
            timestamp = row["timestamp"]
            x, y, z = row["x"], row["y"], row["z"]
            qx, qy, qz, qw = row["qx"], row["qy"], row["qz"], row["qw"]
            f.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

# Convert odom and ground truth CSV files to TUM format
convert_csv_to_tum("/home/jarvis/magic/ROS/magbot_ws/.tests/odom_interpolated.csv", "odom.tum")
convert_csv_to_tum("/home/jarvis/magic/ROS/magbot_ws/.tests/ground_truth_interpolated.csv", "ground_truth.tum")

print("Conversion complete! You can now use evo.")
