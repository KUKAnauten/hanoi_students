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
 *   * Neither the name of the author nor the names of its
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

/* Author: Marcus Ebner */

#include <cassert>
#include <iimoveit/robot_interface.h>
#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>

using namespace robotiq;

namespace hanoi {

class HanoiRobot : public iimoveit::RobotInterface {
private:
  boost::shared_ptr<robotiq_s_model_control::SModelMsgClient> sModelMsgClient_;
  std::vector<double> tower_poses_jointSpace_[3];
  geometry_msgs::PoseStamped tower_poses_[3];
  int tower_nSlices_[3];
  double slice_height_;
  bool grapping_;


public:
  robotiq_s_model_control::SModelAPI gripper;


  HanoiRobot(ros::NodeHandle* node_handle, const std::string& planning_group, const std::vector<double> base_pose, int tow1_nSlices, double slice_height)
      :  RobotInterface(node_handle, planning_group, base_pose),
         tower_poses_jointSpace_{base_pose, base_pose, base_pose},
         tower_nSlices_{tow1_nSlices, 0, 0},
         slice_height_(slice_height),
         sModelMsgClient_(new robotiq_s_model_control::SModelMsgClient(*node_handle)),
         gripper(sModelMsgClient_) {
    tower_poses_[0] = poseFromJointAngles(base_pose);
    tower_poses_[1] = poseFromJointAngles(base_pose);
    tower_poses_[2] = poseFromJointAngles(base_pose);

    std::string movegroup_name ="manipulator";
    move_group_.setPlannerId(movegroup_name+"[RRTConnectkConfigDefault]");
  }

  void setTowerPose(int index, const std::vector<double>& pose) {
    assert(index >= 0 && index <= 2);
    tower_poses_jointSpace_[index] = pose;
    tower_poses_[index] = poseFromJointAngles(pose);
    std::cout << tower_poses_[index] << std::endl;

    if (index == 1) {
      base_pose_ = tower_poses_[1];
      base_pose_.pose.position.z += 0.3;
    }
  }

  void setTowerPose(int index, const geometry_msgs::PoseStamped& pose) {
    assert(index >= 0 && index <= 2);
    tower_poses_[index] = pose;

    if (index == 1) {
      base_pose_ = tower_poses_[1];
      base_pose_.pose.position.z += 0.3;
    }
  }

  void planAndMoveAboveTower(bool approvalRequired = true) {
    geometry_msgs::PoseStamped current_pose = getPose();
    double difference = current_pose.pose.position.z - tower_poses_[0].pose.position.z - 0.133;
    if (difference < 0) {
      current_pose.pose.position.z -= difference;
      planAndMove(current_pose, approvalRequired);
    }
  }

  void planAndMoveToBasePose(bool approvalRequired = true) {
    planAndMoveAboveTower(approvalRequired);
    RobotInterface::planAndMoveToBasePose(approvalRequired);
  }

  geometry_msgs::PoseStamped getTowerPose(int index) {
    assert(index >= 0 && index <= 2);
    return tower_poses_[index];
  }

  void checkPoses() {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::PoseStamped pose = tower_poses_[i];
      pose.pose.position.z += 0.13;
      planAndMove(pose, true);

      pose.pose.position.z -= 0.13;
      planAndMove(pose, true);

      pose.pose.position.z += 0.13;
      planAndMove(pose, true);
    }
  }
  
  void gripperInit() {
    ROS_INFO("Initializing gripper...");
    gripper.setInitialization(INIT_ACTIVATION);
    gripper.setGraspingMode(GRASP_PINCH);
    gripper.setActionMode(ACTION_GO);
    gripper.setRawVelocity(255);
    gripper.setRawForce(1);
    gripper.setRawPosition(0);
    gripper.write();
    waitForGripper();
  }

  void waitForGripper() {
    ros::Duration(0.1).sleep();
    do {
      //if (!gripper.isInitialized()) std::cout <<  "Not initialized!" << std::endl;
      //if (!gripper.isReady()) std::cout << "Not ready!" << std::endl;
      //if (gripper.isMoving()) std::cout << "Still Moving!" << std::endl;
      gripper.read();
      ros::Duration(0.1).sleep();
    } while (ros::ok() && (!gripper.isInitialized() || !gripper.isReady() || gripper.isMoving()));
    ros::Duration(0.1).sleep();
  }

  void gripperClose() {
    gripper.setRawPosition(90);
    gripper.write();
    waitForGripper();
  }

  void gripperOpen() {
    gripper.setRawPosition(30);
    gripper.write();
    waitForGripper();
  }

  void moveSlice(int from, int to, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose_above_from = tower_poses_[from].pose;
    pose_above_from.position.z += 0.133;
    for (int i = 0; i < (13 - tower_nSlices_[from] + 1); ++i) {
      pose_above_from.position.z -= 0.01;
      waypoints.push_back(pose_above_from);
    }
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0, true, approvalRequired);

    gripperClose();
    tower_nSlices_[from]--;

    std::reverse(waypoints.begin(), waypoints.end());

    geometry_msgs::Pose pose_to = tower_poses_[to].pose;
    pose_to.position.z += 0.133;
    waypoints.push_back(pose_to);
    for (int i = 0; i < 3; ++i) {
      pose_to.position.z -= 0.01;
      waypoints.push_back(pose_to);
    }
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0, true, approvalRequired);
    //move_group_.clearPathConstraints();

    gripperOpen();

    pose_to.position.z += 0.03;
    planAndMove(pose_to, approvalRequired);

    tower_nSlices_[to]++;
  }

  void moveTower(int height, int from, int to, int with, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);
    assert(with >= 0 && with <= 2);

    if (height >= 1) {
      moveTower(height-1, from, with, to, approvalRequired);
      moveSlice(from, to, approvalRequired);
      moveTower(height-1, with, to, from, approvalRequired);
    }
  }
};
} // namespace hanoi


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> base_pose_jointSpace{-1.38016748428, -0.778217494488, 2.38215756416, -1.00167918205, -1.14128601551, 1.17012655735, 1.29470527172};

  /**
  geometry_msgs::PoseStamped tow0_pose;
  tow0_pose.header.frame_id = "world";
  tow0_pose.pose.position.x = 0.64;
  tow0_pose.pose.position.y = -0.283443;
  tow0_pose.pose.position.z = 0.96;
  tow0_pose.pose.orientation.x = 0.963151;
  tow0_pose.pose.orientation.y = 0.26867;
  tow0_pose.pose.orientation.z = 0.00855197;
  tow0_pose.pose.orientation.w = 0.0091728;

  //geometry_msgs::PoseStamped tow1_pose;
  //tow1_pose.pose.position.XYZ = ???

  //geometry_msgs::PoseStamped tow2_pose = tow0_pose;
  //tow2_pose.pose.position.XYZ = ???
  */

  geometry_msgs::PoseStamped tow0_pose;
  tow0_pose.header.frame_id = "world";
  tow0_pose.pose.position.x = 0.709;
  tow0_pose.pose.position.y = 0.3545;
  tow0_pose.pose.position.z = 1.009;
  
  tow0_pose.pose.orientation.x = 0.0;
  tow0_pose.pose.orientation.y = 1.0;
  tow0_pose.pose.orientation.z = 0.0;
  tow0_pose.pose.orientation.w = 0.0;

  geometry_msgs::PoseStamped tow1_pose = tow0_pose;
  tow1_pose.pose.position.x -= 0.045;
  tow1_pose.pose.position.y += 0.285;

  geometry_msgs::PoseStamped tow2_pose = tow1_pose;
  tow2_pose.pose.position.x -= 0.313;
  tow2_pose.pose.position.y -= 0.001;

  hanoi::HanoiRobot hanoi_robot(&node_handle, "manipulator", base_pose_jointSpace, 3, 0.01);
  hanoi_robot.setTowerPose(0, tow0_pose);
  hanoi_robot.setTowerPose(1, tow1_pose);
  hanoi_robot.setTowerPose(2, tow2_pose);

  hanoi_robot.planAndMoveToBasePose();
  hanoi_robot.gripperInit();
  hanoi_robot.waitForApproval();

  hanoi_robot.moveTower(3, 0, 2, 1, true);
  hanoi_robot.planAndMoveToBasePose(true);
  
  ros::shutdown();
  return 0;
}
