import argparse
import matplotlib.pyplot as plt

def read_terminal_output(file_path, name):
    x_values = []
    y_values = []

    with open(file_path, 'r') as file:
        for line in file:
            if name in line:
                parts = line.split(':')
                if len(parts) == 2:
                    try:
                        x = float(parts[0].split(" ")[-1])
                        y = float(parts[1].split(" ")[1])
                        x_values.append(x)
                        y_values.append(y)
                    except ValueError:
                        continue
    return x_values, y_values

def plot_data(x, y, name):
    plt.plot(x, y, marker='')
    plt.title(f'Plot for {name}')
    plt.xlabel('X values')
    plt.ylabel('Y values')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot x, y relationship from terminal output.')
    parser.add_argument('file_path', type=str, help='Path to the terminal output file')
    parser.add_argument('name', type=str, help='Name to filter by')

    args = parser.parse_args()

    x, y = read_terminal_output(args.file_path, args.name)
    if x and y:
        plot_data(x, y, args.name)
    else:
        print(f"No data found for name: {args.name}")