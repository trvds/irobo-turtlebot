#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque

class UncertaintyPlotter:
    def __init__(self, max_points=1000):
        self.times = deque(maxlen=max_points)
        self.uncertainties = deque(maxlen=max_points)
        self.start_time = None

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label="Uncertainty (mm)")
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Uncertainty (mm)')
        self.ax.set_title('Uncertainty over Time')
        self.ax.legend()

    def add_point(self, time, uncertainty):
        if self.start_time is None:
            self.start_time = time
        self.times.append((time - self.start_time).to_sec())
        self.uncertainties.append(uncertainty * 1e3)  # Convert to mm

    def update_plot(self, event=None):
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.uncertainties)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def odom_callback(odom_msg):
    # Extract positional uncertainty (covariance matrix diagonal for x and y)
    covariance = np.array(odom_msg.pose.covariance).reshape(6, 6)
    uncertainty = np.sqrt(covariance[0, 0] + covariance[1, 1])  # 2D positional uncertainty

    rospy.loginfo('Uncertainty (mm): {:.2f}'.format(uncertainty * 1e3))
    
    # Add the uncertainty data point to the plotter
    plotter.add_point(rospy.Time.now(), uncertainty)

rospy.init_node('uncertainty_plotter_node')

# Initialize plotter
plotter = UncertaintyPlotter()

# Subscribe to /odometry/filtered for uncertainty data
rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

# Set up a timer to update the plot on the main thread
rospy.Timer(rospy.Duration(1), plotter.update_plot)

try:
    rospy.loginfo('Listening to /odometry/filtered for uncertainty data')
    rospy.spin()
except rospy.ROSInterruptException:
    pass

plt.ioff()
plt.show()
