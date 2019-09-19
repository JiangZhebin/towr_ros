/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>
#include <towr/Robot_arm_model.h>


#include <ur10/ur10_inverse_kinematic.h>
#include <ur10/inverse_kinematics_ur10.h>
#include <ur10/inverse_kinematics_ur10_2.h>
#include <ur10/inverse_kinematics_ur10_4.h>

#include <xpp_states/endeffectors.h>
namespace towr {


TowrRosInterface::TowrRosInterface ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  solver_ = std::make_shared<ifopt::IpoptSolver>();

  visualization_dt_ = 0.01;
}

BaseState
TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  return goal;
}

void
TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
  // robot model
  formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
//  formulation_.model_.kinematic_model_=std::make_shared<RobotArmKinematicModel>();
//  formulation_.model_.dynamic_model_=std::make_shared<ObjectDynamicModel>();
  auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // terrain
  //auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
  //formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

  int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation_.params_ = GetTowrParameters(n_ee, msg);
  formulation_.final_base_ = GetGoalState(msg);

  SetTowrInitialState();

  // solver parameters
  SetIpoptParameters(msg);

  // visualization
  PublishInitialState();

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize || msg.play_initialization) {
    nlp_ = ifopt::Problem();
    for (auto c : formulation_.GetVariableSets(solution))
      nlp_.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
      nlp_.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
      nlp_.AddCostSet(c);

    solver_->Solve(nlp_);


    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + "/xpp/joint_ur10_des_1" + " "
        + "/xpp/joint_ur10_des_2" + " "
        + "/xpp/joint_ur10_des_3" + " "
        + "/xpp/joint_ur10_des_4" + " "
        + xpp_msgs::terrain_info
        + " -r " + std::to_string(msg.replay_speed)
        + " --quiet " + bag_file).c_str());
  }

  if (msg.plot_trajectory) {
    int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // to publish entire trajectory (e.g. to send to controller)
  // xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
}

void
TowrRosInterface::PublishInitialState()
{
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());
  switch (n_ee) {
  case 1:
    xpp.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
    break;
  case 2:
    xpp.robot_base_vec.at(0) = Vector3d(-0.3,0,0);
    xpp.robot_base_vec.at(1) = Vector3d(0.3,0,0);
    break;
  case 4:
    xpp.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
    xpp.robot_base_vec.at(1)=Vector3d(0.3,0,0);
    xpp.robot_base_vec.at(2)=Vector3d(0,-0.3,0);
    xpp.robot_base_vec.at(3)=Vector3d(0,0.3,0);
    break;
  default:
    xpp.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
  }
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
    //xpp.robot_base_vec.at(ee_xpp).setZero();

  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec
TowrRosInterface::GetTrajectory () const
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    //set the center of blue box
    switch (n_ee) {
    case 1:
      state.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
      break;
    case 2:
      state.robot_base_vec.at(0) = Vector3d(-0.3,0,0);
      state.robot_base_vec.at(1) = Vector3d(0.3,0,0);
      break;
    case 4:
      state.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
      state.robot_base_vec.at(1)=Vector3d(0.3,0,0);
      state.robot_base_vec.at(2)=Vector3d(0,-0.3,0);
      state.robot_base_vec.at(3)=Vector3d(0,0.3,0);
      break;
    default:
      state.robot_base_vec.at(0)=Vector3d(-0.3,0,0);
    }
    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();

  auto final_joint_value = GetJointValue();

  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  SaveJointValueInRosbag(bag, final_joint_value,"/xpp/joint_ur10_des_1","/xpp/joint_ur10_des_2",
                         "/xpp/joint_ur10_des_3","/xpp/joint_ur10_des_4");

  bag.close();
}

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n =Vector3d(0,0,-1); //formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = 0.5;//formulation_.terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}


//TODO!! write joint space in the rosbag file
std::vector<xpp::RobotStateJoint>
TowrRosInterface::GetJointValue() const
{
  std::vector<xpp::RobotStateJoint> JointValue;
  double t=0.0;
  double T = solution.base_linear_->GetTotalTime();
  EulerConverter base_angular(solution.base_angular_);
  while (t<=T+1e-5) {
    int n_ee =solution.ee_motion_.size();
    xpp::EndeffectorsPos ee_W(n_ee);
    xpp::RobotStateCartesian state(n_ee);
    state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);

    Eigen::VectorXd q;
    for(auto ee : ee_W.GetEEsOrdered())
      ee_W.at(ee) = solution.ee_motion_.at(ee)->GetPoint(t).p();

    xpp::RobotStateJoint JointState(n_ee,6);

    switch(n_ee){
    case 1:{
      xpp::InverseKinematicsUR10 ik;
      JointState.q_=ik.GetAllJointAngles(ee_W, state.base_.ang.q);
      break;}
    case 2:{
      xpp::InverseKinematicsUR10_2 ik_2;
      JointState.q_ = ik_2.GetAllJointAngles(ee_W,state.base_.ang.q);
      break;}
    case 4:{
      xpp::InverseKinematicsUR10_4 ik_4;
      JointState.q_ = ik_4.GetAllJointAngles(ee_W,state.base_.ang.q);
      break;
    }
    }

    JointState.t_global_ = t;
    JointValue.push_back(JointState);
    t += visualization_dt_;
  }
  return JointValue;
}



void
TowrRosInterface::SaveJointValueInRosbag(rosbag::Bag& bag, const std::vector<xpp::RobotStateJoint> &joint,
                            const std::string& topic1,const std::string& topic2,
                            const std::string& topic3,const std::string& topic4) const
{
  for(const auto JointValue : joint){
    auto timestamp = ::ros::Time(JointValue.t_global_ + 1e-6);
    xpp_msgs::RobotStateJoint msg;
    msg.time_from_start = ros::Duration(JointValue.t_global_);
    Eigen::VectorXd q = JointValue.q_.ToVec();
    switch (q.size()) {
    case 6:{
      msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());

      bag.write(topic1, timestamp, msg);

      break;
    }
    case 12:{
      msg.joint_state.position =  std::vector<double>(q.data(), q.data()+6);
      bag.write(topic1, timestamp, msg);
      msg.joint_state.position = std::vector<double>(q.data()+6, q.data()+q.size());
      bag.write(topic2, timestamp, msg);
      break;}
    case 24:{
        msg.joint_state.position =  std::vector<double>(q.data(), q.data()+6);
        bag.write(topic1, timestamp, msg);
        msg.joint_state.position = std::vector<double>(q.data()+6, q.data()+12);
        bag.write(topic2, timestamp, msg);
        msg.joint_state.position =  std::vector<double>(q.data()+12, q.data()+18);
        bag.write(topic3, timestamp, msg);
        msg.joint_state.position = std::vector<double>(q.data()+18, q.data()+q.size());
        bag.write(topic4, timestamp, msg);
        break;
      }
    default:
       ROS_ERROR("wrong number of joint number");

    }

  }
}

} /* namespace towr */

