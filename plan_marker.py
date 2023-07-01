#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from aruco_msgs.msg import MarkerArray 

class Manipulator():
    def __init__(self):
        self.moveit =  moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.pub = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.joint_goal = self.group.get_current_joint_values()
        self.pose_goal = geometry_msgs.msg.Pose()
        self.pose_goal.position.x = 0
        self.pose_goal.position.y = 0
        self.pose_goal.position.z = 0
        self.pose_goal.orientation.w = 0
        self.plan = 0
        self.fraction = 0
        self.waypoints = []
        self.wpose = 0

    def output(self):
        print ("============ Reference frame:" + self.planning_frame)
        print ("============ End effector: %s" + self.eef_link)
        print ("============ Robot Groups:"), self.robot.get_group_names()
        print ("============ Printing robot state")
        print (self.robot.get_current_state())
        print ("")

    def reset(self):
        self.joint_goal = self.group.get_current_joint_values()
        self.joint_goal[0] = 0
        self.joint_goal[1] = -pi/4
        self.joint_goal[2] = 0
        self.joint_goal[3] = -pi/2
        self.joint_goal[4] = 0
        self.joint_goal[5] = pi/3
        self.group.go(self.joint_goal, wait=True)
        self.group.stop()

    def stop(self):
        self.group.stop()


    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.pose_goal.position.z) 
        self.pose_goal.position.x = data.markers[0].pose.pose.position.x
        self.pose_goal.position.y = data.markers[0].pose.pose.position.y  
        self.pose_goal.position.z = data.markers[0].pose.pose.position.z  
        self.pose_goal.orientation.w = data.markers[0].pose.pose.orientation.w


    def plan_cart(self):
        scale = 10
        self.wpose = self.group.get_current_pose().pose
        self.wpose.position.x += scale * self.pose_goal.position.x   # Y of arm --> (-ve) X of marker
        self.wpose.position.y += scale * self.pose_goal.position.y   # X of arm --> (+ve) Y of marker
        self.wpose.position.z += scale * self.pose_goal.position.z  # Z of arm --> (+ve) Z of marker
        self.waypoints.append(copy.deepcopy(self.wpose))
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.pose_goal) 
        # self.waypoints.append(copy.deepcopy(self.wpose))
        (self.plan,self.fraction) = self.group.compute_cartesian_path(self.waypoints,0.01,0.0)
        return  self.plan,self.fraction
   
    def move(self):
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray,self.callback)
        rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.waypoints) 
        self.plan_cart()
        # self.group.execute(self.plan,wait=True)
            # self.group.set_pose_target(self.pose_goal)
            # self.plan = self.group.go(self.pose_goal,wait=True)
            # self.group.clear_pose_targets()
            # self.group.stop()
            # self.group.clear_pose_targets()
            # rate.sleep()
if __name__ == "__main__":
    man = Manipulator()
    while not rospy.is_shutdown():
        try:
            man.move()
        except rospy.ROSInterruptException:
            pass
