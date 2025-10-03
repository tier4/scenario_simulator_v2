#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from matplotlib.animation import FuncAnimation
import os

class TrajectoryPlotter:
    def __init__(self, csv_file=None):
        if csv_file is None:
            # Get the directory where this script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            csv_file = os.path.join(script_dir, 'src', 'trajectory_data.csv')
        self.csv_file = csv_file

        # Use default style
        plt.style.use('default')

        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, figsize=(14, 14))
        self.fig.suptitle('FollowTrajectoryAction: Kinematic Analysis', fontsize=18, fontweight='bold', y=0.995)

        # Initialize empty lists for data
        self.times = []
        self.velocities = []
        self.accelerations = []
        self.positions_x = []
        self.positions_y = []
        self.remaining_distances = []

        # Set up plots
        self.setup_plots()

    def setup_plots(self):
        # Velocity plot
        self.ax1.set_title('Linear Velocity Profile', fontsize=14, fontweight='bold', pad=10)
        self.ax1.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('Velocity [m/s]', fontsize=12, fontweight='bold')
        self.ax1.grid(True, which='major', alpha=0.5, linestyle='-', linewidth=0.8)
        self.ax1.grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
        self.ax1.minorticks_on()
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=2.5, label='Linear Velocity', alpha=0.9)
        self.ax1.legend(loc='best', fontsize=11, framealpha=0.95)
        self.ax1.tick_params(labelsize=10)

        # Acceleration plot
        self.ax2.set_title('Linear Acceleration Profile', fontsize=14, fontweight='bold', pad=10)
        self.ax2.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('Acceleration [m/s²]', fontsize=12, fontweight='bold')
        self.ax2.grid(True, which='major', alpha=0.5, linestyle='-', linewidth=0.8)
        self.ax2.grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
        self.ax2.minorticks_on()
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=2.5, label='Linear Acceleration', alpha=0.9)
        self.ax2.legend(loc='best', fontsize=11, framealpha=0.95)
        self.ax2.tick_params(labelsize=10)

        # Jerk plot (derivative of acceleration)
        self.ax3.set_title('Jerk Profile (Rate of Change of Acceleration)', fontsize=14, fontweight='bold', pad=10)
        self.ax3.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
        self.ax3.set_ylabel('Jerk [m/s³]', fontsize=12, fontweight='bold')
        self.ax3.grid(True, which='major', alpha=0.5, linestyle='-', linewidth=0.8)
        self.ax3.grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
        self.ax3.minorticks_on()
        self.line3, = self.ax3.plot([], [], 'g-', linewidth=2.5, label='Jerk', alpha=0.9)
        self.ax3.legend(loc='best', fontsize=11, framealpha=0.95)
        self.ax3.tick_params(labelsize=10)

        # Remaining distance plot
        self.ax4.set_title('Remaining Distance to Final Waypoint', fontsize=14, fontweight='bold', pad=10)
        self.ax4.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
        self.ax4.set_ylabel('Distance [m]', fontsize=12, fontweight='bold')
        self.ax4.grid(True, which='major', alpha=0.5, linestyle='-', linewidth=0.8)
        self.ax4.grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
        self.ax4.minorticks_on()
        self.line4, = self.ax4.plot([], [], 'm-', linewidth=2.5, label='Distance to Final Waypoint', alpha=0.9)
        self.ax4.legend(loc='best', fontsize=11, framealpha=0.95)
        self.ax4.tick_params(labelsize=10)

        plt.tight_layout()

    def update_plots(self, frame):
        if not os.path.exists(self.csv_file):
            return self.line1, self.line2, self.line3, self.line4

        try:
            # Read CSV file
            df = pd.read_csv(self.csv_file)

            if len(df) == 0:
                return self.line1, self.line2, self.line3, self.line4

            # Extract data
            times = df['time'].values
            velocities = df['velocity'].values
            accelerations = df['acceleration'].values
            remaining_distances = df['remaining_distance'].values

            # Calculate jerk (derivative of acceleration)
            jerk = np.zeros_like(accelerations)
            if len(accelerations) > 1:
                jerk[1:] = np.diff(accelerations) / np.diff(times)

            # Update velocity plot
            self.line1.set_data(times, velocities)
            self.ax1.relim()
            self.ax1.autoscale_view()

            # Update acceleration plot
            self.line2.set_data(times, accelerations)
            self.ax2.relim()
            self.ax2.autoscale_view()

            # Update jerk plot
            self.line3.set_data(times, jerk)
            self.ax3.relim()
            self.ax3.autoscale_view()

            # Update remaining distance plot
            self.line4.set_data(times, remaining_distances)
            self.ax4.relim()
            self.ax4.autoscale_view()

        except Exception as e:
            print(f"Error reading CSV: {e}")

        return self.line1, self.line2, self.line3, self.line4

    def start_plotting(self):
        # Animation function
        ani = FuncAnimation(self.fig, self.update_plots, interval=100, blit=True, cache_frame_data=False)

        # Show the plot
        plt.show()
        return ani

if __name__ == "__main__":
    plotter = TrajectoryPlotter()
    print("Starting real-time trajectory plotting...")
    print("Make sure trajectory_data.csv is being generated by the C++ program.")
    print("Close the plot window to exit.")

    ani = plotter.start_plotting()