import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Define the filenames
filenames = [f"data_log_{i}.csv" for i in range(10)]

# Initialize lists to store data
red_counts, green_counts, blue_counts, total_counts = [], [], [], []

# Read and analyze each file
for filename in filenames:
    try:
        df = pd.read_csv(filename)

        # Append the last value of each column to the lists
        red_counts.append(df['red_count'].iloc[-1])
        green_counts.append(df['green_count'].iloc[-1])
        blue_counts.append(df['blue_count'].iloc[-1])
        total_counts.append(df['total_count'].iloc[-1])
    except Exception as e:
        print(f"Error reading {filename}: {e}")

print(max(total_counts))

# Function to calculate average and standard deviation
def calculate_metrics(data_list):
    avg = np.mean(data_list)
    std = np.std(data_list)
    return avg, std

# Calculate averages and standard deviations
avg_red_count, std_red_count = calculate_metrics(red_counts)
avg_green_count, std_green_count = calculate_metrics(green_counts)
avg_blue_count, std_blue_count = calculate_metrics(blue_counts)
avg_total_count, std_total_count = calculate_metrics(total_counts)

# Prepare data for LaTeX
latex_results = pd.DataFrame({
    'Metric': ['Red Count', 'Green Count', 'Blue Count', 'Total Count'],
    'Average': [avg_red_count, avg_green_count, avg_blue_count, avg_total_count],
    'Standard Deviation': [std_red_count, std_green_count, std_blue_count, std_total_count]
})

# Save results to CSV for LaTeX
latex_results.to_csv('performance_results_for_latex2.csv', index=False)

# Plotting
metrics = ['Red Count', 'Green Count', 'Blue Count', 'Total Count']
averages = [avg_red_count, avg_green_count, avg_blue_count, avg_total_count]
std_devs = [std_red_count, std_green_count, std_blue_count, std_total_count]

plt.figure(figsize=(8, 6))
plt.bar(metrics, averages, yerr=std_devs, capsize=5, color='skyblue')
plt.ylabel('Average Counts')
plt.title('Performance Metrics of Autonomous Robots')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('performance_metrics2.png')
