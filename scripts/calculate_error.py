#!/usr/bin/env python3
import rospy
import rosbag
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
from collections import deque

class TransformHandler():
    def __init__(self, gt_frame, est_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]
        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))

def get_errors(transform):
    tr = transform.transform.translation
    return numpy.linalg.norm([tr.x, tr.y])

class ErrorPlotter:
    def __init__(self, max_points=1000):
        self.times = deque(maxlen=max_points)
        self.errors = deque(maxlen=max_points)
        self.start_time = None
        
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Error (mm)')
        self.ax.set_title('Error over Time')
        
    def add_point(self, time, error):
        if self.start_time is None:
            self.start_time = time
        
        self.times.append((time - self.start_time).to_sec())
        self.errors.append(error * 1e3)  # Convert to mm
        
    def update_plot(self):
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.errors)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

parser = argparse.ArgumentParser()
parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')
args = parser.parse_args()

gt_frame = args.gt_frame
est_frame = args.est_frame

rospy.init_node('evaluation_node')

if rospy.rostime.is_wallclock():
    rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
    sys.exit(1)

rospy.loginfo('Waiting for clock')
rospy.sleep(0.00001)

handler = TransformHandler(gt_frame, est_frame, max_time_between=20)  # 20s
plotter = ErrorPlotter()

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
sleeper = rospy.Rate(10)  # Reduce rate to 10 Hz for better visualization

try:
    while not rospy.is_shutdown():
        try:
            t = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(e)
        else:
            eucl = get_errors(t)
            rospy.loginfo('Error (in mm): {:.2f}'.format(eucl * 1e3))
            
            # Add point to the plot
            plotter.add_point(rospy.Time.now(), eucl)
            plotter.update_plot()
        
        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)
except rospy.exceptions.ROSInterruptException:
    pass

plt.ioff()
plt.show()