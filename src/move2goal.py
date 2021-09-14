#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib


def pose_callback(pose_with_covariance):
    # print(pose_with_covariance)
    pose = pose_with_covariance.pose.pose
    print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))


def move_base_status_callback(status):
    pass


def move_base_result_callback(result):
    pass


class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, theta):
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    def moveToPoint(self, x, y, theta):
        target_point = self.createGoal(x, y, theta)
        self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        print ("Move to %f, %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.z
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False


# Main program
def main():
    rospy.init_node('RAI1', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # TODO
    mba = moveBaseAction()
    while not rospy.is_shutdown():
        mba.moveToPoint(0.57014, 0.32602, -0.6654)
        rospy.sleep(1)

        mba.moveToPoint(0.10218, 0, 0.9966)
        rospy.sleep(1)

    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
