#!/usr/bin/env python
import rospy
import tf.transformations
import time
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray


def pose_to_msg(x, y, theta):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose


def pose_callback(pose_with_covariance):
    # print(pose_with_covariance)
    pose = pose_with_covariance.pose.pose
    print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.z, pose.position.y, pose.orientation.z))


def move_base_status(status):
    pass
    # print(status)


def move_base_result(result):
    print("move_base_result = ", result)


# Main program
def main():
    rospy.init_node('RAI1', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result)

    # TODO
    goal_pose_msg = pose_to_msg(2, 2, 0)
    print(goal_pose_msg)
    # publisher_goal.publish(  )
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

