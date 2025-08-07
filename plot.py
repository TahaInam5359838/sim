import matplotlib.pyplot as plt
import csv
import sys

def load_motion_segments_with_time(file_path):
    segments = []
    current_segment = []

    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0] == 'timestamp':
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
                continue
            try:
                timestamp = float(row[0])
                vmag = float(row[-1])
                current_segment.append((timestamp, vmag))
            except ValueError:
                continue

    if current_segment:
        segments.append(current_segment)

    return segments

def moving_average(values, window_size=3):
    if len(values) < window_size:
        return values
    averaged = []
    for i in range(len(values)):
        start = max(0, i - window_size + 1)
        window = values[start:i+1]
        avg = sum(window) / len(window)
        averaged.append(avg)
    return averaged

def plot_segments_with_time(segments, window_size=3):
    plt.figure(figsize=(10, 6))

    for i, segment in enumerate(segments):
        # Normalize time to start from 0
        base_time = segment[0][0]
        times = [t - base_time for t, _ in segment]
        vmags = [v for _, v in segment]

        smoothed_vmags = moving_average(vmags, window_size)
        plt.plot(times, smoothed_vmags, label=f'Motion {i+1}')

    plt.xlabel('Time (s from segment start)')
    plt.ylabel('Smoothed Velocity Magnitude (vmag)')
    plt.title(f'Overlay of Smoothed vmag vs Time (Moving Avg Window = {window_size})')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    file_path = sys.argv[1]
    motion_segments = load_motion_segments_with_time(file_path)
    plot_segments_with_time(motion_segments, window_size=5)