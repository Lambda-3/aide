import actionlib
import move_base_msgs.msg
import rospy
from rospy import loginfo
from apis import tag


def move(x, y):
    client = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)
    success = client.wait_for_result(rospy.Duration(60))
    loginfo(success)


def move_to_place(name):
    coords = tag.get_tagged_coordinates(name)
    if coords:
        move(*coords)
