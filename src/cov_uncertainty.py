#!/usr/bin/env python3
import rospy
import sys
import argparse
import numpy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque
import os

class UncertaintyPlotter:
    def __init__(self, max_points=1000):
        self.times = deque(maxlen=max_points)
        self.absolute_uncertainty = deque(maxlen=max_points)
        self.start_time = None

    def add_point(self, time, std_x, std_y, std_z):
        if self.start_time is None:
            self.start_time = time

        # Calculate absolute uncertainty (Euclidean norm)
        abs_uncertainty = numpy.sqrt(std_x**2 + std_y**2 + std_z**2)
        self.times.append((time - self.start_time).to_sec())
        self.absolute_uncertainty.append(abs_uncertainty * 1e3)  # Convert to mm

    def save_plot(self, filename):
        plt.figure()
        plt.plot(self.times, self.absolute_uncertainty, label='Uncertainty', color='b')
        plt.xlabel('Time (s)')
        plt.ylabel('Uncertainty (mm)')
        plt.title('Position Uncertainty over Time')
        plt.legend()
        plt.grid()
        plt.savefig(filename)
        plt.close()  # Close the figure to free memory

def extract_position_uncertainty(covariance_matrix):
    # Convert the covariance list into a 6x6 numpy array
    cov_matrix = numpy.array(covariance_matrix).reshape(6, 6)

    # Extract the standard deviations for position (x, y, z) from the diagonal elements
    std_x = numpy.sqrt(cov_matrix[0, 0])  # Standard deviation in x
    std_y = numpy.sqrt(cov_matrix[1, 1])  # Standard deviation in y
    std_z = numpy.sqrt(cov_matrix[2, 2])  # Standard deviation in z

    return std_x, std_y, std_z

def odom_callback(msg, plotter):
    # Extract the covariance matrix from the message
    covariance = msg.pose.covariance

    # Get position uncertainties (standard deviations)
    std_x, std_y, std_z = extract_position_uncertainty(covariance)

    # Add point to the plotter
    current_time = rospy.Time.now()
    plotter.add_point(current_time, std_x, std_y, std_z)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', help='Odometry topic to subscribe to', default='/odometry/filtered')
    parser.add_argument('--output_folder', help='Folder to save the plot', default='./')
    args = parser.parse_args()

    rospy.init_node('uncertainty_plotter_node')

    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(1)

    rospy.loginfo('Waiting for odometry topic')
    rospy.sleep(0.00001)

    plotter = UncertaintyPlotter()

    # Subscribe to the odometry topic
    rospy.Subscriber(args.topic, Odometry, lambda msg: odom_callback(msg, plotter))

    rospy.loginfo('Listening to odometry data. Press Ctrl-C to stop.')

    try:
        rospy.spin()  # Keep the ROS node alive
    except rospy.ROSInterruptException:
        pass

    # Save the plot to the specified output folder
    output_path = os.path.join(args.output_folder, 'uncertainty_plot.png')
    plotter.save_plot(output_path)
    rospy.loginfo(f'Plot saved to {output_path}')

if __name__ == '__main__':
    main()
