#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from math import pi
from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionResult
from moveit_commander.conversions import pose_to_list
from aruco_msgs.msg import MarkerArray
class Manipulator():
    def __init__(self):
        rospy.init_node('group_python_interface_tutorial',anonymous=True)

        self.moveit =  moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.display_pub = rospy.Publisher('/group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        self.gripper_pub = rospy.Publisher('/gripper_command',String,queue_size=10)
        self.gripper_pub.publish("close")
        rospy.sleep(15)
        self.group.allow_replanning(True)
        self.group.set_num_planning_attempts(5)
        self.group.set_planning_time(5)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.joint_goal = self.group.get_current_joint_values()
        self.tagPose = geometry_msgs.msg.Pose()
        self.tagPose.position.x = 0
        self.tagPose.position.y = 0
        self.tagPose.position.z = 0
        self.tagPose.orientation.w = 0
        self.plan = 0
        self.fraction = 0
        self.waypoints = []
        self.tags = {}
        self.refLocation = {}
        self.wpose = 0
        self.tagID = 0
        self.zOffset = 0.1
        self.yOffset = 0.1
        self.xOffset = 0.005
        self.gripperState = False
        self.execute = False
        self.home_joint_positions = [-0.22388934701633367,-2.3322009193961972, 1.959136798652601, 0.3733129866667584,1.346910642842957, -1.5708751286915055]
        self.group.set_joint_value_target(self.home_joint_positions)
        self.group.go(wait=True)
        self.detectedTag = []
        self.group.set_goal_tolerance(0.003)  # Set the tolerance for the goal position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def callback(self, data):

        self.tagPose.position.x = data.markers[0].pose.pose.position.x
        self.tagPose.position.y = data.markers[0].pose.pose.position.y
        self.tagPose.position.z =  data.markers[0].pose.pose.position.z
        self.tagPose.orientation.w = -0.13062443840305
        self.tagID = data.markers[0].id
        position_orientation = (self.tagPose.position.x, self.tagPose.position.y, self.tagPose.position.z, self.tagPose.orientation.w)
        if self.tagID in self.tags:
            pass
        else:
            self.tags[self.tagID] = [position_orientation]
            print(f"tag{self.tagID} {self.tagPose} ")



    def girpperState_callback(self,data):

        self.gripperState = data.result.reached_goal


    def sub(self):
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray,self.callback)
        rospy.Subscriber("/gripper/gripper_cmd/result", GripperCommandActionResult,self.girpperState_callback)

    def pub(self):
        self.gripper_pub.publish("close")

    def plan_execute(self): ## self,x,y,z
        self.waypoints = []
        self.wpose = self.group.get_current_pose().pose
        self.wpose.position.x =  0.46478635408628305
        self.wpose.position.y =  -0.058791090580734964
        self.wpose.position.z = 0.22409526718234624
        # self.wpose.orientation.x =  0.4983631031601795
        self.waypoints.append(copy.deepcopy(self.wpose))
        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        print(f"plan{self.plan}")
        self.group.execute(self.plan,wait=True)

    def button1_press(self, size=0.12):
        scale = 10
        self.waypoints = []
        self.wpose = self.group.get_current_pose().pose
        # Move forward
        self.wpose.position.x += size*1.1
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Move backward
        self.wpose.position.x -= size*1.1
        self.waypoints.append(copy.deepcopy(self.wpose))
        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        self.group.execute(self.plan,wait=True)

    def go_to_home_position(self):
        self.group.set_joint_value_target(self.home_joint_positions)
        self.group.go(wait=True)
    def plan_first_btn(self, size=0.12):
        scale = 10
        self.waypoints = []
        self.wpose = self.group.get_current_pose().pose

        # Move forward
        self.wpose.position.x += size*0.6
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Move up
        self.wpose.position.z +=  0.01
        self.waypoints.append(copy.deepcopy(self.wpose))
        # Move right
        self.wpose.position.y -= 0.049
        self.waypoints.append(copy.deepcopy(self.wpose))


        # Move forward
        self.wpose.position.x += size*0.4
        self.waypoints.append(copy.deepcopy(self.wpose))

        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        self.group.execute(self.plan,wait=True)

    def plan_second_btn(self, size=0.12):
        scale = 10
        self.waypoints = []
        self.wpose = self.group.get_current_pose().pose

        # Move forward
        self.wpose.position.x += size*0.6
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Move up
        self.wpose.position.z +=  0.01
        self.waypoints.append(copy.deepcopy(self.wpose))
        # Move right
        self.wpose.position.y -= 0.17
        self.waypoints.append(copy.deepcopy(self.wpose))


        # Move forward
        self.wpose.position.x += size*0.4
        self.waypoints.append(copy.deepcopy(self.wpose))

        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        self.group.execute(self.plan,wait=True)
    def plan_third_btn(self, size=0.12):
        scale = 10
        self.waypoints = []
        self.wpose = self.group.get_current_pose().pose

        # Move forward
        self.wpose.position.x += size*0.6
        self.waypoints.append(copy.deepcopy(self.wpose))
        # Move down
        self.wpose.position.z -= size + 0.04
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Move forward
        self.wpose.position.x += size*0.1
        self.waypoints.append(copy.deepcopy(self.wpose))
        # Move right
        self.wpose.position.y -= 0.16

        # Move forward
        self.wpose.position.x += size*1
        self.waypoints.append(copy.deepcopy(self.wpose))



        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        self.group.execute(self.plan,wait=True)


if __name__ == "__main__":
    man1 = Manipulator()
    # man1.sub()
    
    man1.plan_execute()
    while not rospy.is_shutdown():
        try:
            pass


        except rospy.ROSInterruptException:
            pass
