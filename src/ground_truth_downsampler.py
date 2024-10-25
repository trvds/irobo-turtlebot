#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class GroundTruthDownsampler:
    def __init__(self, input_topic, output_topic):
        self.pub = rospy.Publisher(output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.sub = rospy.Subscriber(input_topic, Odometry, self.callback)
        self.last_publish_time = rospy.Time.now()

    def callback(self, msg):
        current_time = rospy.Time.now()
        # Publish only if 1 second has passed (1 Hz)
        if (current_time - self.last_publish_time).to_sec() >= 1.0:
            self.last_publish_time = current_time
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose  # Copy the pose and covariance from the ground truth

            self.pub.publish(pose_msg)
            rospy.loginfo("Published downsampled ground truth data.")

if __name__ == '__main__':
    rospy.init_node('ground_truth_downsampler')

    input_topic = rospy.get_param('~input_topic', '/tf')
    output_topic = rospy.get_param('~output_topic', '/downsampled_ground_truth')

    GroundTruthDownsampler(input_topic, output_topic)
    rospy.spin()
