#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import rospkg
import os
from math import radians
from std_msgs.msg import String


def main():
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=10,
    )
    rospack = rospkg.RosPack()

    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = "world"
    mesh_pose.pose.orientation.w = 1.0
    mesh_pose.pose.position.x = 0.8  # above the panda_hand frame
    mesh_pose.pose.position.z = 1.0  # above the panda_hand frame
    mesh_scale = (0.5,0.5,0.5)
    mesh_name = "biw"
    mesh_path = os.path.join(rospack.get_path("test_environment"), "meshes", "car_BIW.stl")
    scene.add_mesh(mesh_name, mesh_pose, mesh_path, mesh_scale)

    start_state: moveit_msgs.msg.RobotState = move_group.get_current_state()
    
    start_state.joint_state.position = (
        radians(45),
        radians(101),
        radians(-108),
        radians(84),
        radians(-169),
        radians(-31),
        radians(-13),
        *start_state.joint_state.position[7:],
    )
    
    move_group.set_start_state(start_state)
    move_group.set_goal_joint_tolerance(1e-6)
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.6297555433686766
    joint_goal[1] = 0.7004650602444055
    joint_goal[2] = 0.08393550285275565
    joint_goal[3] = 1.620558404639375
    joint_goal[4] = -2.9109985961500664
    joint_goal[5] = -0.9425748749512245
    joint_goal[6] = -0.47314676349031026
        
    planners = ["RRTConnect", "BiTRRT", "ABITstar", "AITstar", "EITstar", "EIRMstar"]
    
    for p in planners:
        move_group.set_planner_id(p)
        move_group.set_planning_time(120)
        success, traj, planning_time, error_code = move_group.plan(joint_goal)
        traj_time = traj.joint_trajectory.points[-1].time_from_start.to_sec() if success else 0
        print(f"[{p}] Success: {success}, Planning time: {planning_time}, trajectory time: {traj_time}")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(traj)
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    main()
    rospy.spin()


        # radians(-68),
        # radians(78),
        # radians(55),
        # radians(81),
        # radians(-156),
        # radians(-88),
        # radians(144),
    # joint_goal[0] = radians(-34)
    # joint_goal[1] = radians(39)
    # joint_goal[2] = radians(1)
    # joint_goal[3] = radians(93)
    # joint_goal[4] = radians(-166)
    # joint_goal[5] = radians(-55)
    # joint_goal[6] = radians(-27)
