import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv
import sys
import os
import numpy as np
import re

def detect_model_name(filename):
    if 'int8' in filename.lower():
        return 'int8'
    elif 'int16' in filename.lower():
        return 'int16'
    elif 'int32' in filename.lower():
        return 'int32'
    return os.path.basename(filename)

def load_single_csv(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found")
        return None

    frame_times = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if 'frame_time_ms' in row and row['frame_time_ms']:
                try:
                    frame_times.append(float(row['frame_time_ms']))
                except ValueError:
                    pass
    return np.array(frame_times) if frame_times else None

def plot_multiple_models(csv_files):
    model_names = ['int8', 'int16', 'int32']
    model_colors = {'int8': 'lightblue', 'int16': 'lightgreen', 'int32': 'lightcoral'}
    model_data = {}

    for csv_file in csv_files:
        name = detect_model_name(csv_file)
        data = load_single_csv(csv_file)
        if data is not None and len(data) > 0:
            model_data[name] = data
            print(f"Loaded {name}: {len(data)} frames, mean={np.mean(data):.2f}ms")

    if not model_data:
        print("Error: No valid data found in any CSV file")
        return

    sorted_names = sorted(model_data.keys(), key=lambda x: model_names.index(x) if x in model_names else 99)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    box_data = []
    box_labels = []
    box_colors = []
    for name in sorted_names:
        box_data.append(model_data[name])
        box_labels.append(name)
        box_colors.append(model_colors.get(name, 'gray'))

    bp = axes[0, 0].boxplot(box_data, vert=True, patch_artist=True, tick_labels=box_labels)
    for patch, color in zip(bp['boxes'], box_colors):
        patch.set_facecolor(color)
    for median in bp['medians']:
        median.set_color('red')
        median.set_linewidth(2)
    axes[0, 0].set_ylabel('Frame Time (ms)')
    axes[0, 0].set_xlabel('Model')
    axes[0, 0].set_title('YOLO Model Runtime Comparison\n(Box and Whisker Plot)')
    axes[0, 0].grid(True, axis='y', linestyle='--', alpha=0.7)

    for i, name in enumerate(sorted_names):
        data = model_data[name]
        stats_text = f"n={len(data)}\nμ={np.mean(data):.1f}ms\nσ={np.std(data):.1f}ms\nFPS={1000/np.mean(data):.1f}"
        axes[0, 0].text(i + 1, axes[0, 0].get_ylim()[1] * 0.95, stats_text,
                    fontsize=8, ha='center', va='top',
                    bbox=dict(boxstyle='round', facecolor=box_colors[i], alpha=0.3))

    bar_means = [np.mean(model_data[name]) for name in sorted_names]
    bar_stds = [np.std(model_data[name]) for name in sorted_names]
    x_pos = np.arange(len(sorted_names))
    bars = axes[0, 1].bar(x_pos, bar_means, yerr=bar_stds, capsize=5,
                          color=box_colors, edgecolor='black', alpha=0.8)
    axes[0, 1].set_ylabel('Frame Time (ms)')
    axes[0, 1].set_xlabel('Model')
    axes[0, 1].set_title('Mean Frame Time with Std Dev\n(Bar Chart)')
    axes[0, 1].set_xticks(x_pos)
    axes[0, 1].set_xticklabels(sorted_names)
    axes[0, 1].grid(True, axis='y', linestyle='--', alpha=0.7)

    for bar, mean, std in zip(bars, bar_means, bar_stds):
        axes[0, 1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.5,
                    f'{mean:.1f}ms\n({1000/mean:.1f} FPS)',
                    ha='center', va='bottom', fontsize=9)

    fps_values = [1000 / np.mean(model_data[name]) for name in sorted_names]
    bars = axes[1, 0].bar(x_pos, fps_values, color=box_colors, edgecolor='black', alpha=0.8)
    axes[1, 0].set_ylabel('FPS')
    axes[1, 0].set_xlabel('Model')
    axes[1, 0].set_title('Frames Per Second\n(Higher is Better)')
    axes[1, 0].set_xticks(x_pos)
    axes[1, 0].set_xticklabels(sorted_names)
    axes[1, 0].grid(True, axis='y', linestyle='--', alpha=0.7)

    for bar, fps in zip(bars, fps_values):
        axes[1, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{fps:.1f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

    bin_width = 5
    all_data = np.concatenate([model_data[name] for name in sorted_names])
    bins = np.arange(0, np.max(all_data) + bin_width, bin_width)

    for name, color in zip(sorted_names, box_colors):
        axes[1, 1].hist(model_data[name], bins=bins, alpha=0.5, label=name, color=color, edgecolor='black')
    axes[1, 1].set_xlabel('Frame Time (ms)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Distribution of Frame Times\n(Overlaid Histogram)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()

    output_dir = os.path.dirname(csv_files[0]) if os.path.dirname(csv_files[0]) else '.'
    output_file = os.path.join(output_dir, 'yolo_model_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_file}")

    print("\n" + "="*80)
    print("MODEL COMPARISON SUMMARY")
    print("="*80)
    print(f"{'Model':<10} {'Count':>8} {'Mean (ms)':>12} {'Median (ms)':>12} {'Std (ms)':>10} {'Min (ms)':>10} {'Max (ms)':>10} {'FPS':>8}")
    print("-"*80)
    for name in sorted_names:
        data = model_data[name]
        print(f"{name:<10} {len(data):>8} {np.mean(data):>12.2f} {np.median(data):>12.2f} "
              f"{np.std(data):>10.2f} {np.min(data):>10.2f} {np.max(data):>10.2f} {1000/np.mean(data):>8.1f}")
    print("="*80)

    print("\nRelative Performance (vs int8):")
    if 'int8' in model_data:
        int8_mean = np.mean(model_data['int8'])
        for name in sorted_names:
            if name != 'int8':
                ratio = np.mean(model_data[name]) / int8_mean
                print(f"  {name} is {ratio:.2f}x {'slower' if ratio > 1 else 'faster'} than int8")

def plot_comparison(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found")
        return

    model_names = ['int8', 'int16', 'int32']
    model_colors = {'int8': 'lightblue', 'int16': 'lightgreen', 'int32': 'lightcoral'}
    model_data = {}

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for name in model_names:
                key = f"{name}_ms"
                if key in row and row[key]:
                    if name not in model_data:
                        model_data[name] = []
                    try:
                        model_data[name].append(float(row[key]))
                    except ValueError:
                        pass

    for name in model_names:
        if name in model_data:
            model_data[name] = np.array(model_data[name])

    has_data = any(name in model_data and len(model_data[name]) > 0 for name in model_names)
    if not has_data:
        print("Error: No data found in CSV file")
        return

    sorted_names = [name for name in model_names if name in model_data and len(model_data[name]) > 0]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    box_data = [model_data[name] for name in sorted_names]
    box_colors = [model_colors[name] for name in sorted_names]

    bp = axes[0, 0].boxplot(box_data, vert=True, patch_artist=True, tick_labels=sorted_names)
    for patch, color in zip(bp['boxes'], box_colors):
        patch.set_facecolor(color)
    for median in bp['medians']:
        median.set_color('red')
        median.set_linewidth(2)
    axes[0, 0].set_ylabel('Frame Time (ms)')
    axes[0, 0].set_xlabel('Model')
    axes[0, 0].set_title('YOLO Model Runtime Comparison\n(Box and Whisker Plot)')
    axes[0, 0].grid(True, axis='y', linestyle='--', alpha=0.7)

    for i, name in enumerate(sorted_names):
        data = model_data[name]
        stats_text = f"n={len(data)}\nμ={np.mean(data):.1f}ms\nσ={np.std(data):.1f}ms\nFPS={1000/np.mean(data):.1f}"
        axes[0, 0].text(i + 1, axes[0, 0].get_ylim()[1] * 0.95, stats_text,
                    fontsize=8, ha='center', va='top',
                    bbox=dict(boxstyle='round', facecolor=box_colors[i], alpha=0.3))

    bar_means = [np.mean(model_data[name]) for name in sorted_names]
    bar_stds = [np.std(model_data[name]) for name in sorted_names]
    x_pos = np.arange(len(sorted_names))
    bars = axes[0, 1].bar(x_pos, bar_means, yerr=bar_stds, capsize=5,
                          color=box_colors, edgecolor='black', alpha=0.8)
    axes[0, 1].set_ylabel('Frame Time (ms)')
    axes[0, 1].set_xlabel('Model')
    axes[0, 1].set_title('Mean Frame Time with Std Dev\n(Bar Chart)')
    axes[0, 1].set_xticks(x_pos)
    axes[0, 1].set_xticklabels(sorted_names)
    axes[0, 1].grid(True, axis='y', linestyle='--', alpha=0.7)

    for bar, mean, std in zip(bars, bar_means, bar_stds):
        axes[0, 1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.5,
                    f'{mean:.1f}ms\n({1000/mean:.1f} FPS)',
                    ha='center', va='bottom', fontsize=9)

    fps_values = [1000 / np.mean(model_data[name]) for name in sorted_names]
    bars = axes[1, 0].bar(x_pos, fps_values, color=box_colors, edgecolor='black', alpha=0.8)
    axes[1, 0].set_ylabel('FPS')
    axes[1, 0].set_xlabel('Model')
    axes[1, 0].set_title('Frames Per Second\n(Higher is Better)')
    axes[1, 0].set_xticks(x_pos)
    axes[1, 0].set_xticklabels(sorted_names)
    axes[1, 0].grid(True, axis='y', linestyle='--', alpha=0.7)

    for bar, fps in zip(bars, fps_values):
        axes[1, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{fps:.1f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

    bin_width = 5
    all_data = np.concatenate([model_data[name] for name in sorted_names])
    bins = np.arange(0, np.max(all_data) + bin_width, bin_width)

    for name, color in zip(sorted_names, box_colors):
        axes[1, 1].hist(model_data[name], bins=bins, alpha=0.5, label=name, color=color, edgecolor='black')
    axes[1, 1].set_xlabel('Frame Time (ms)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Distribution of Frame Times\n(Overlaid Histogram)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()

    output_dir = os.path.dirname(csv_file) if os.path.dirname(csv_file) else '.'
    output_file = os.path.join(output_dir, 'yolo_model_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")

    print("\n" + "="*80)
    print("MODEL COMPARISON SUMMARY")
    print("="*80)
    print(f"{'Model':<10} {'Count':>8} {'Mean (ms)':>12} {'Median (ms)':>12} {'Std (ms)':>10} {'Min (ms)':>10} {'Max (ms)':>10} {'FPS':>8}")
    print("-"*80)
    for name in sorted_names:
        data = model_data[name]
        print(f"{name:<10} {len(data):>8} {np.mean(data):>12.2f} {np.median(data):>12.2f} "
              f"{np.std(data):>10.2f} {np.min(data):>10.2f} {np.max(data):>10.2f} {1000/np.mean(data):>8.1f}")
    print("="*80)

def plot_single_model(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found")
        return

    frame_times = load_single_csv(csv_file)
    if frame_times is None or len(frame_times) == 0:
        print("Error: No data found in CSV file")
        return

    model_name = detect_model_name(csv_file)
    color = 'lightblue' if model_name == 'int8' else ('lightgreen' if model_name == 'int16' else 'lightcoral')

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].boxplot(frame_times, vert=True, patch_artist=True,
                    boxprops=dict(facecolor=color, color='blue'),
                    medianprops=dict(color='red', linewidth=2),
                    whiskerprops=dict(color='blue'),
                    capprops=dict(color='blue'),
                    flierprops=dict(marker='o', markerfacecolor='red', markersize=5, alpha=0.5))
    axes[0].set_ylabel('Frame Time (ms)')
    axes[0].set_title(f'YOLO Model Runtime per Frame ({model_name})\n(Box and Whisker Plot)')
    axes[0].set_xticklabels([model_name])
    axes[0].grid(True, axis='y', linestyle='--', alpha=0.7)

    stats_text = f"Statistics:\n"
    stats_text += f"Count: {len(frame_times)}\n"
    stats_text += f"Mean: {np.mean(frame_times):.2f} ms\n"
    stats_text += f"Median: {np.median(frame_times):.2f} ms\n"
    stats_text += f"Std Dev: {np.std(frame_times):.2f} ms\n"
    stats_text += f"Min: {np.min(frame_times):.2f} ms\n"
    stats_text += f"Max: {np.max(frame_times):.2f} ms\n"
    stats_text += f"Q1 (25%): {np.percentile(frame_times, 25):.2f} ms\n"
    stats_text += f"Q3 (75%): {np.percentile(frame_times, 75):.2f} ms\n"
    stats_text += f"IQR: {np.percentile(frame_times, 75) - np.percentile(frame_times, 25):.2f} ms\n"
    stats_text += f"FPS: {1000.0 / np.mean(frame_times):.1f}"

    axes[0].text(1.3, np.median(frame_times), stats_text, 
                 fontsize=10, verticalalignment='center',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                 family='monospace')

    axes[1].hist(frame_times, bins=50, color=color, edgecolor='black', alpha=0.7)
    axes[1].axvline(np.mean(frame_times), color='red', linestyle='--', linewidth=2, label=f'Mean: {np.mean(frame_times):.2f} ms')
    axes[1].axvline(np.median(frame_times), color='green', linestyle='-', linewidth=2, label=f'Median: {np.median(frame_times):.2f} ms')
    axes[1].set_xlabel('Frame Time (ms)')
    axes[1].set_ylabel('Frequency')
    axes[1].set_title(f'Distribution of Frame Times ({model_name})\n(Histogram)')
    axes[1].legend()
    axes[1].grid(True, axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()

    output_dir = os.path.dirname(csv_file) if os.path.dirname(csv_file) else '.'
    output_file = os.path.join(output_dir, f'yolo_timing_{model_name}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")

    print("\n" + "="*50)
    print(f"Summary Statistics ({model_name})")
    print("="*50)
    print(f"Total frames: {len(frame_times)}")
    print(f"Mean:   {np.mean(frame_times):.2f} ms")
    print(f"Median: {np.median(frame_times):.2f} ms")
    print(f"Std:    {np.std(frame_times):.2f} ms")
    print(f"Min:    {np.min(frame_times):.2f} ms")
    print(f"Max:    {np.max(frame_times):.2f} ms")
    print(f"Q1:     {np.percentile(frame_times, 25):.2f} ms")
    print(f"Q3:     {np.percentile(frame_times, 75):.2f} ms")
    print(f"IQR:    {np.percentile(frame_times, 75) - np.percentile(frame_times, 25):.2f} ms")
    print(f"FPS (approx): {1000.0 / np.mean(frame_times):.1f}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_yolo_timing.py <csv_file> [csv_file2] [csv_file3] ...")
        print("")
        print("Modes:")
        print("  1. Single CSV (comparison format): python plot_yolo_timing.py yolo_comparison.csv")
        print("  2. Multiple CSVs (one per model):  python plot_yolo_timing.py yolo_int8.csv yolo_int16.csv yolo_int32.csv")
        print("  3. Single CSV (single model):      python plot_yolo_timing.py yolo_frame_times.csv")
        print("")
        print("Examples:")
        print("  python plot_yolo_timing.py yolo_comparison_20240224_120000.csv")
        print("  python plot_yolo_timing.py yolo_int8_20240224.csv yolo_int16_20240224.csv yolo_int32_20240224.csv")
        sys.exit(1)

    csv_files = sys.argv[1:]

    if len(csv_files) == 1:
        csv_file = csv_files[0]
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)

        if 'int8_ms' in header or 'int16_ms' in header or 'int32_ms' in header:
            print("Detected comparison CSV format - plotting model comparison...")
            plot_comparison(csv_file)
        else:
            print("Detected single model CSV format - plotting single model...")
            plot_single_model(csv_file)
    else:
        print(f"Detected multiple CSV files ({len(csv_files)}) - plotting model comparison...")
        plot_multiple_models(csv_files)
