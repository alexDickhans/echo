import argparse
import matplotlib.pyplot as plt

'''
Simple plotting tool to analyze log files as a graph quickly
Helps with PID tuning, FF Constant Calibration, and data analysis for the notebook
'''

def read_terminal_output(file_path, names):
    data = {name: ([], []) for name in names}

    with open(file_path, 'r') as file:
        for line in file:
            for name in names:
                if name in line:
                    parts = line.split(':')
                    if len(parts) == 2:
                        try:
                            x = float(parts[0].split(" ")[-1])
                            y = float(parts[1].split(" ")[1])

                            data[name][0].append(x)
                            data[name][1].append(y)
                        except ValueError:
                            continue

    return data

def plot_data(data):
    fig, ax1 = plt.subplots()

    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    ax2 = None

    for i, (name, (x, y)) in enumerate(data.items()):
        if i == 0:
            ax1.plot(x, y, color=colors[i % len(colors)], label=name)
            ax1.set_xlabel('X values')
            ax1.set_ylabel(f'Y values ({name})', color=colors[i % len(colors)])
            ax1.tick_params(axis='y', labelcolor=colors[i % len(colors)])
        else:
            if ax2 is None:
                ax2 = ax1.twinx()
            else:
                ax2 = ax2.twinx()
                ax2.spines['right'].set_position(('outward', 60 * (i - 1)))

            ax2.plot(x, y, color=colors[i % len(colors)], label=name)
            ax2.set_ylabel(f'Y values ({name})', color=colors[i % len(colors)])
            ax2.tick_params(axis='y', labelcolor=colors[i % len(colors)])
            ax2.set_ylim(ax1.get_ylim())  # Align the secondary y-axis with the primary y-axis

    fig.tight_layout()
    plt.title('Plot for multiple variables')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot x, y relationship from terminal output.')
    parser.add_argument('file_path', type=str, help='Path to the terminal output file')
    parser.add_argument('names', type=str, nargs='+', help='Names to filter by')

    args = parser.parse_args()

    data = read_terminal_output(args.file_path, args.names)
    if any(data.values()):
        plot_data(data)
    else:
        print(f"No data found for names: {', '.join(args.names)}")