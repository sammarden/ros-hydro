
import rospy

import tf

from geometry_msgs.msg import Quaternion, Pose, PoseArray

class ROSLeapMotionDriver:
    """Class to convert Leap data to ROS and publish."""
    
    def __init__(self):
        """Constructor."""
        self.hands_pub = rospy.Publisher('hands', PoseArray)

    def publish_hands(self, hands):
        """
        Publish hand data.
        @param hands: Hands detected by the Leap.
        """
        hand_array = PoseArray()

        hand_array.header.stamp = rospy.Time.now()

        for hand in hands:

            hand_pose = Pose()

            hand_pose.position.x = hand.palm_position.x
            hand_pose.position.y = hand.palm_position.y
            hand_pose.position.z = hand.palm_position.z

            quat = tf.transformations.quaternion_from_euler(hand.direction.yaw, hand.direction.pitch, hand.palm_normal.roll, 'rzyx')

            hand_pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

            hand_array.poses.append(hand_pose)

        self.hands_pub.publish(hand_array)

