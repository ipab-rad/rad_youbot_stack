/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

 #include <moveit/robot_state/robot_state.h>

 //tf
 #include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_demo");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  
  // BEGIN_DEMO
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("arm_1");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // GO CANDLE

  // Setting Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat ;
  quat = tf::createQuaternionMsgFromRollPitchYaw(0.00,0.00,0.017);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.190;
  target_pose.pose.position.y = 0.000;
  target_pose.pose.position.z = 0.580;
  group.setPoseTarget(target_pose, group.getEndEffectorLink());
  group.setGoalTolerance(0.1);
  group.setGoalOrientationTolerance(0.01);

  // Moving to pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Moving group");
  group.move();
  /* Sleep to give time to execute movement. */
  sleep(30.0);


  // GO FRONT GRASP


  // Setting Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.1329,-0.0390,0.1853);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.4018;
  target_pose.pose.position.y = -0.0077;
  target_pose.pose.position.z = -0.00157;
  group.setPoseTarget(target_pose, group.getEndEffectorLink());
  group.setGoalTolerance(0.1);
  group.setGoalOrientationTolerance(0.01);

  // Moving to pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Moving group");
  group.move();
  /* Sleep to give time to execute movement. */
  sleep(30.0);


  // GO RIGHT SIDE GRASP

  // Setting Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  quat = tf::createQuaternionMsgFromRollPitchYaw(3.1174,-0.1087,-1.6135);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.1003;
  target_pose.pose.position.y = -0.2483;
  target_pose.pose.position.z = 0.03079;
  group.setPoseTarget(target_pose, group.getEndEffectorLink());
  group.setGoalTolerance(0.1);
  group.setGoalOrientationTolerance(0.01);

  // Moving to pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Moving group");
  group.move();
  /* Sleep to give time to execute movement. */
  sleep(30.0);

  // GO BACK DROP

  // Setting Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  quat = tf::createQuaternionMsgFromRollPitchYaw(3.0215,-0.4952,-3.0676);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.09906;
  target_pose.pose.position.y = 0.04693;
  target_pose.pose.position.z = 0.1643;
  group.setPoseTarget(target_pose, group.getEndEffectorLink());
  group.setGoalTolerance(0.1);
  group.setGoalOrientationTolerance(0.01);

  // Moving to pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Moving group");
  group.move();
  /* Sleep to give time to execute movement. */
  sleep(30.0);

  // GO FOLDED

  // Setting Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  quat = tf::createQuaternionMsgFromRollPitchYaw(-0.0874,0.34516,-0.4479);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.1867;
  target_pose.pose.position.y = -0.00388;
  target_pose.pose.position.z = 0.3713;
  group.setPoseTarget(target_pose, group.getEndEffectorLink());
  group.setGoalTolerance(0.1);
  group.setGoalOrientationTolerance(0.01);

  // Moving to pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Moving group");
  group.move();
  /* Sleep to give time to execute movement. */
  sleep(10.0);

  // END_DEMO


  ros::shutdown();  
  return 0;
}