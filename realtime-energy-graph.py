import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def main():
    parser = argparse.ArgumentParser(description="Live plot CSV data as it grows")
    parser.add_argument("csv_file", help="Path to the CSV file")
    parser.add_argument("--interval", type=int, default=1000,
                        help="Update interval in milliseconds (default: 1000)")
    parser.add_argument("--tail", type=int, default=None,
                        help="Show only the last N rows (default: all rows)")
    args = parser.parse_args()

    fig = plt.figure()

    def animate(i):
        try:
            df = pd.read_csv(args.csv_file)
            if args.tail:
                df = df.tail(args.tail)

            plt.cla()  # clear previous plot
            # ⚠️ adjust columns to match your CSV structure
            plt.plot(df.iloc[:, 0], df.iloc[:, 1], marker="o")
            plt.xlabel(df.columns[0])
            plt.ylabel(df.columns[1])
            plt.title(f"Live Plot: {args.csv_file}")
        except Exception as e:
            print(f"Error reading {args.csv_file}: {e}")

    ani = animation.FuncAnimation(fig, animate, interval=args.interval)
    plt.show()

if __name__ == "__main__":
    main()
