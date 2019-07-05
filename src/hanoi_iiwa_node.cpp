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
    double sicherheitsabstand=0.05;
    double stabhoehe=0.10;
    double schwammhoehe=0.00;
    geometry_msgs::PoseStamped fromPose = getTowerPose(from);
    fromPose.pose.position.z+=sicherheitsabstand+ stabhoehe;
    geometry_msgs::PoseStamped fromPoseDown = getTowerPose(from);
    fromPoseDown.pose.position.z+=(tower_nSlices_[from]*slice_height_) + schwammhoehe-0.005;
    geometry_msgs::PoseStamped toPose = getTowerPose(to);
    toPose.pose.position.z+=sicherheitsabstand+ stabhoehe;

    gripperOpen();
    planAndMove(fromPose,approvalRequired);
    planAndMove(fromPoseDown,approvalRequired);
    gripperClose();
    tower_nSlices_[from]-=1;
    waitForApproval();
    publishPoseGoalLinear(fromPose);
    planAndMove(toPose,approvalRequired);
    gripperOpen();
    tower_nSlices_[to]+=1;

    
  }

  void moveTower(int height, int from, int to, int with, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);
    assert(with >= 0 && with <= 2);

    if(height<=0){ //nix machen, wenn kein Turm bewegt werden soll
      return;
    }
    moveTower(height-1,from, with, to, approvalRequired); //erst den Turm eins kleiner auf das Hilfsfeld bewegen
    moveSlice(from, to, approvalRequired); // dann den untersten Stein auf das Zielfeld bewegen
    moveTower(height-1, with, to, from, approvalRequired); // und den Turm eins kleiner nachholen


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
  tow1_pose.pose.position.x = 0.709-0.04;
  tow1_pose.pose.position.y = 0.3545+0.28;

  geometry_msgs::PoseStamped tow2_pose = tow1_pose;
  tow2_pose.pose.position.x = 0.709-0.04-0.32;
  tow2_pose.pose.position.y = 0.3545+0.28;

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
