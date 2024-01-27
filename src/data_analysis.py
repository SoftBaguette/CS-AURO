import pandas as pd

# Define the filenames
filenames = [f"data_log_{i}.csv" for i in range(10)]

# Initialize lists to store data
red_counts = []
green_counts = []
blue_counts = []
total_counts = []
red_values = []
green_values = []
blue_values = []
total_values = []

# Read and analyze each file
for filename in filenames:
    try:
        # Read the CSV file
        df = pd.read_csv(filename)

        # Append the last value of each column to the lists (assuming the last value represents the final count/value)
        red_counts.append(df['red_count'].iloc[-1])
        green_counts.append(df['green_count'].iloc[-1])
        blue_counts.append(df['blue_count'].iloc[-1])
        total_counts.append(df['total_count'].iloc[-1])
        red_values.append(df['red_value'].iloc[-1])
        green_values.append(df['green_value'].iloc[-1])
        blue_values.append(df['blue_value'].iloc[-1])
        total_values.append(df['total_value'].iloc[-1])
    except Exception as e:
        print(f"Error reading {filename}: {e}")

# Calculate averages
avg_red_count = sum(red_counts) / len(red_counts) if red_counts else 0
avg_green_count = sum(green_counts) / len(green_counts) if green_counts else 0
avg_blue_count = sum(blue_counts) / len(blue_counts) if blue_counts else 0
avg_total_count = sum(total_counts) / len(total_counts) if total_counts else 0
avg_red_value = sum(red_values) / len(red_values) if red_values else 0
avg_green_value = sum(green_values) / len(green_values) if green_values else 0
avg_blue_value = sum(blue_values) / len(blue_values) if blue_values else 0
avg_total_value = sum(total_values) / len(total_values) if total_values else 0

# Display the results
results = {
    "Average Red Count": avg_red_count,
    "Average Green Count": avg_green_count,
    "Average Blue Count": avg_blue_count,
    "Average Total Count": avg_total_count,
    "Average Red Value": avg_red_value,
    "Average Green Value": avg_green_value,
    "Average Blue Value": avg_blue_value,
    "Average Total Value": avg_total_value
}

print(results)
