import matplotlib.pyplot as plt
import csv
import sys
import os

def plot_shooter_data(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found")
        return

    times = []
    target_velocities = []
    current_velocities = []
    errors = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time']))
            target_velocities.append(float(row['target_angular_velocity']))
            current_velocities.append(float(row['current_angular_velocity']))
            errors.append(float(row['error']))

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(times, target_velocities, 'b-', label='Target Angular Velocity', linewidth=2)
    axes[0].plot(times, current_velocities, 'r-', label='Current Angular Velocity', linewidth=1.5)
    axes[0].set_ylabel('Angular Velocity (rad/s)')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_title('Shooter Angular Velocity')

    axes[1].plot(times, errors, 'g-', label='Error', linewidth=1.5)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    axes[1].set_ylabel('Error (rad/s)')
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title('Error (Target - Current)')

    axes[2].plot(times, target_velocities, 'b-', label='Target', linewidth=2)
    axes[2].plot(times, current_velocities, 'r-', label='Current', linewidth=1.5)
    axes[2].fill_between(times, target_velocities, current_velocities, alpha=0.3, color='gray', label='Error Area')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Angular Velocity (rad/s)')
    axes[2].legend()
    axes[2].grid(True)
    axes[2].set_title('Target vs Current with Error Area')

    plt.tight_layout()

    output_dir = os.path.dirname(csv_file)
    output_file = os.path.join(output_dir, 'shooter_plot.png')
    plt.savefig(output_file, dpi=150)
    print(f"Plot saved to: {output_file}")

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_shooter_data.py <csv_file>")
        print("Example: python plot_shooter_data.py shooter_data_20240224_120000.csv")
        sys.exit(1)

    plot_shooter_data(sys.argv[1])
