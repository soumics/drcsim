/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "drcsim_gazebo_ros_plugins/RobotiqHandPlugin.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <sdf/JointAxis.hh>

using drcsim_gazebo_ros_plugins::RobotiqHandPlugin;

//////////////////////////////////////////////////
RobotiqHandPlugin::RobotiqHandPlugin()
{
}

//////////////////////////////////////////////////
RobotiqHandPlugin::~RobotiqHandPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "RobotiqHandPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    // Deferred from Configure(): a model's child joint entities are not
    // guaranteed to exist yet when Configure() runs.
    this->Load(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm);
  }
}

//////////////////////////////////////////////////
double RobotiqHandPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::Load(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->sdfConfig->HasElement("side")) {
    gzerr << "Failed to determine which hand we're controlling; "
          << "aborting plugin load." << std::endl;
    return;
  }
  this->side = this->sdfConfig->Get<std::string>("side");
  if (this->side != "left" && this->side != "right") {
    gzerr << "Failed to determine which hand we're controlling; "
          << "aborting plugin load." << std::endl;
    return;
  }

  gzmsg << "RobotiqHandPlugin loading for " << this->side << " hand."
        << std::endl;

  if (!this->FindJoints(_ecm)) {
    return;
  }

  this->jointStates.name = this->jointNames;
  this->jointStates.position.assign(this->jointNames.size(), 0.0);
  this->jointStates.velocity.assign(this->jointNames.size(), 0.0);
  this->jointStates.effort.assign(this->jointNames.size(), 0.0);

  std::string controlTopicName = "/left_hand/command";
  std::string stateTopicName = "/left_hand/state";
  if (this->side == "right") {
    controlTopicName = "/right_hand/command";
    stateTopicName = "/right_hand/state";
  }

  for (int i = 0; i < kNumJoints; ++i) {
    gz::sim::Joint fingerJoint(this->fingerJoints[i]);
    double effortLimit = 60.0;
    const auto axes = fingerJoint.Axis(_ecm);
    if (axes && !axes->empty() && std::isfinite((*axes)[0].Effort())) {
      effortLimit = (*axes)[0].Effort();
    }
    this->posePID[i].Init(1.0, 0, 0.5, 0.0, 0.0, effortLimit, -effortLimit);

    if (this->sdfConfig->HasElement("kp_position")) {
      this->posePID[i].SetPGain(this->sdfConfig->Get<double>("kp_position"));
    }
    if (this->sdfConfig->HasElement("ki_position")) {
      this->posePID[i].SetIGain(this->sdfConfig->Get<double>("ki_position"));
    }
    if (this->sdfConfig->HasElement("kd_position")) {
      this->posePID[i].SetDGain(this->sdfConfig->Get<double>("kd_position"));
    }
    if (this->sdfConfig->HasElement("position_effort_min")) {
      this->posePID[i].SetCmdMin(
        this->sdfConfig->Get<double>("position_effort_min"));
    }
    if (this->sdfConfig->HasElement("position_effort_max")) {
      this->posePID[i].SetCmdMax(
        this->sdfConfig->Get<double>("position_effort_max"));
    }
  }

  if (this->sdfConfig->HasElement("topic_command")) {
    controlTopicName = this->sdfConfig->Get<std::string>("topic_command");
  }
  if (this->sdfConfig->HasElement("topic_state")) {
    stateTopicName = this->sdfConfig->Get<std::string>("topic_state");
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "robotiq_hand_plugin_" + this->side);

  this->pubHandleState =
    this->rosNode->create_publisher<atlas_msgs::msg::SModelRobotInput>(
    stateTopicName, rclcpp::QoS(100).transient_local());

  this->pubJointStates =
    this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    "robotiq_hands/" + this->side + "_hand/joint_states", 10);

  this->subHandleCommand =
    this->rosNode->create_subscription<atlas_msgs::msg::SModelRobotOutput>(
    controlTopicName, rclcpp::QoS(100),
    std::bind(&RobotiqHandPlugin::SetHandleCommand, this, std::placeholders::_1));

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->rosSpinThread = std::thread(
    [this]()
    {
      this->executor->spin();
    });

  gzmsg << "Topic for sending hand commands: [" << controlTopicName
        << "]. Topic for receiving hand state: [" << stateTopicName << "]"
        << std::endl;

  this->validConfig = true;
}

//////////////////////////////////////////////////
bool RobotiqHandPlugin::GetAndPushBackJoint(
  gz::sim::EntityComponentManager & _ecm,
  const std::string & _jointName,
  std::vector<gz::sim::Entity> & _joints)
{
  const gz::sim::Entity joint = this->model.JointByName(_ecm, _jointName);
  if (joint == gz::sim::kNullEntity) {
    gzerr << "Failed to find joint [" << _jointName
          << "]; aborting plugin load." << std::endl;
    return false;
  }
  gz::sim::Joint jointWrapper(joint);
  jointWrapper.EnablePositionCheck(_ecm);
  jointWrapper.EnableVelocityCheck(_ecm);
  _joints.push_back(joint);
  gzmsg << "RobotiqHandPlugin found joint [" << _jointName << "]" << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool RobotiqHandPlugin::FindJoints(gz::sim::EntityComponentManager & _ecm)
{
  const std::string prefix = (this->side == "left") ? "l_" : "r_";

  std::string suffix = "palm_finger_1_joint";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->joints)) {
    return false;
  }
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->fingerJoints)) {
    return false;
  }
  this->jointNames.push_back(prefix + suffix);

  suffix = "palm_finger_2_joint";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->joints)) {
    return false;
  }
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->fingerJoints)) {
    return false;
  }
  this->jointNames.push_back(prefix + suffix);

  // We read the joint state from finger_1_joint_1 but actuate
  // finger_1_joint_proximal_actuating_hinge (see the class-level design
  // note on the two joint vectors).
  suffix = "finger_1_joint_proximal_actuating_hinge";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->fingerJoints)) {
    return false;
  }
  suffix = "finger_1_joint_1";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->joints)) {
    return false;
  }
  this->jointNames.push_back(prefix + suffix);

  suffix = "finger_2_joint_proximal_actuating_hinge";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->fingerJoints)) {
    return false;
  }
  suffix = "finger_2_joint_1";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->joints)) {
    return false;
  }
  this->jointNames.push_back(prefix + suffix);

  suffix = "finger_middle_joint_proximal_actuating_hinge";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->fingerJoints)) {
    return false;
  }
  suffix = "finger_middle_joint_1";
  if (!this->GetAndPushBackJoint(_ecm, prefix + suffix, this->joints)) {
    return false;
  }
  this->jointNames.push_back(prefix + suffix);

  // Remaining underactuated joints: informative only, never driven directly.
  static const char * const kUnderactuated[] = {
    "finger_1_joint_2", "finger_1_joint_3",
    "finger_2_joint_2", "finger_2_joint_3",
    "palm_finger_middle_joint",
    "finger_middle_joint_2", "finger_middle_joint_3"
  };
  for (const char * s : kUnderactuated) {
    if (!this->GetAndPushBackJoint(_ecm, prefix + s, this->joints)) {
      return false;
    }
    this->jointNames.push_back(prefix + s);
  }

  gzmsg << "RobotiqHandPlugin found all joints for " << this->side
        << " hand." << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool RobotiqHandPlugin::VerifyField(
  const std::string & _label, int _min, int _max, int _v)
{
  if (_v < _min || _v > _max) {
    RCLCPP_ERROR(
      this->rosNode->get_logger(),
      "Illegal %s value: [%d]. The correct range is [%d,%d]",
      _label.c_str(), _v, _min, _max);
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool RobotiqHandPlugin::VerifyCommand(
  const atlas_msgs::msg::SModelRobotOutput & _command)
{
  return this->VerifyField("r_act", 0, 1, _command.r_act) &&
         this->VerifyField("r_mod", 0, 3, _command.r_mod) &&
         this->VerifyField("r_gto", 0, 1, _command.r_gto) &&
         this->VerifyField("r_atr", 0, 1, _command.r_atr) &&
         this->VerifyField("r_icf", 0, 1, _command.r_icf) &&
         this->VerifyField("r_ics", 0, 1, _command.r_ics) &&
         this->VerifyField("r_pra", 0, 255, _command.r_pra) &&
         this->VerifyField("r_spa", 0, 255, _command.r_spa) &&
         this->VerifyField("r_fra", 0, 255, _command.r_fra) &&
         this->VerifyField("r_prb", 0, 255, _command.r_prb) &&
         this->VerifyField("r_spb", 0, 255, _command.r_spb) &&
         this->VerifyField("r_frb", 0, 255, _command.r_frb) &&
         this->VerifyField("r_prc", 0, 255, _command.r_prc) &&
         this->VerifyField("r_spc", 0, 255, _command.r_spc) &&
         this->VerifyField("r_frc", 0, 255, _command.r_frc) &&
         this->VerifyField("r_prs", 0, 255, _command.r_prs) &&
         this->VerifyField("r_sps", 0, 255, _command.r_sps) &&
         this->VerifyField("r_frs", 0, 255, _command.r_frs);
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::SetHandleCommand(
  const atlas_msgs::msg::SModelRobotOutput::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  if (!this->VerifyCommand(*_msg)) {
    RCLCPP_ERROR(this->rosNode->get_logger(), "Ignoring command");
    return;
  }

  this->prevCommand = this->handleCommand;
  this->handleCommand = *_msg;
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::ReleaseHand()
{
  this->handleCommand.r_pra = 0;
  this->handleCommand.r_prb = 0;
  this->handleCommand.r_prc = 0;

  this->handleCommand.r_spa = 127;
  this->handleCommand.r_spb = 127;
  this->handleCommand.r_spc = 127;
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::StopHand()
{
  this->handleCommand.r_pra = this->handleState.g_pra;
  this->handleCommand.r_prb = this->handleState.g_prb;
  this->handleCommand.r_prc = this->handleState.g_prc;
}

//////////////////////////////////////////////////
bool RobotiqHandPlugin::IsHandFullyOpen(gz::sim::EntityComponentManager & _ecm)
{
  const double toleranceRad = 1.0 * M_PI / 180.0;

  for (int i = 2; i < kNumJoints; ++i) {
    gz::sim::Joint joint(this->joints[i]);
    double lower = 0.0;
    const auto axes = joint.Axis(_ecm);
    if (axes && !axes->empty()) {
      lower = (*axes)[0].Lower();
    }
    if (!(FirstOrZero(joint.Position(_ecm)) < lower + toleranceRad)) {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  if (_info.simTime <= this->lastControllerUpdateTime) {
    return;
  }

  this->userHandleCommand = this->handleCommand;

  // Step 1: state transitions.
  if (this->handleCommand.r_act == 0) {
    this->handState = Disabled;
  } else if (this->handleCommand.r_atr == 1) {
    this->handState = Emergency;
  } else if (this->handleCommand.r_ics == 1) {
    this->handState = ICS;
  } else if (this->handleCommand.r_icf == 1) {
    this->handState = ICF;
  } else {
    if (static_cast<int>(this->handleCommand.r_mod) != this->graspingMode) {
      this->handState = ChangeModeInProgress;
      this->lastHandleCommand = this->handleCommand;
      this->graspingMode =
        static_cast<GraspingMode>(this->handleCommand.r_mod);
    } else if (this->handState != ChangeModeInProgress) {
      this->handState = Simplified;
    }

    if (this->handState == ChangeModeInProgress && this->IsHandFullyOpen(_ecm)) {
      this->prevCommand = this->handleCommand;
      this->handleCommand = this->lastHandleCommand;
      this->handState = Simplified;
    }
  }

  // Step 2: actions in each state.
  switch (this->handState) {
    case Disabled:
      break;

    case Emergency:
      if (this->IsHandFullyOpen(_ecm)) {
        this->StopHand();
      } else {
        this->ReleaseHand();
      }
      break;

    case ICS:
      RCLCPP_ERROR_THROTTLE(
        this->rosNode->get_logger(), *this->rosNode->get_clock(), 5000,
        "Individual Control of Scissor not supported");
      break;

    case ICF:
      if (this->handleCommand.r_gto == 0) {
        this->StopHand();
      }
      break;

    case ChangeModeInProgress:
      this->ReleaseHand();
      break;

    case Simplified:
      // All fingers follow finger A.
      this->handleCommand.r_prb = this->handleCommand.r_pra;
      this->handleCommand.r_prc = this->handleCommand.r_pra;
      this->handleCommand.r_spb = this->handleCommand.r_spa;
      this->handleCommand.r_spc = this->handleCommand.r_spa;
      this->handleCommand.r_frb = this->handleCommand.r_fra;
      this->handleCommand.r_frc = this->handleCommand.r_fra;

      if (this->handleCommand.r_gto == 0) {
        this->StopHand();
      }
      break;

    default:
      RCLCPP_ERROR(
        this->rosNode->get_logger(), "Unrecognized state [%d]", this->handState);
  }

  const double dt = std::chrono::duration<double>(
    _info.simTime - this->lastControllerUpdateTime).count();
  this->UpdatePIDControl(_ecm, dt);

  this->GetAndPublishHandleState(_ecm);

  const rclcpp::Time stamp(
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());
  this->GetAndPublishJointState(_ecm, stamp);

  this->lastControllerUpdateTime = _info.simTime;
}

//////////////////////////////////////////////////
uint8_t RobotiqHandPlugin::GetObjectDetection(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _jointEntity,
  int _index, uint8_t _rPR, uint8_t _prevRPR)
{
  gz::sim::Joint joint(_jointEntity);
  const bool isMoving = FirstOrZero(joint.Velocity(_ecm)) > kVelTolerance;

  double pe, ie, de;
  this->posePID[_index].Errors(pe, ie, de);
  const bool reachPosition = pe < kPoseTolerance;

  if (isMoving) {
    // Finger is in motion.
    return 0;
  }
  if (reachPosition) {
    // Finger is at the requested position.
    return 3;
  }
  if (static_cast<int>(_rPR) - static_cast<int>(_prevRPR) > 0) {
    // Finger has stopped due to a contact while closing.
    return 2;
  }
  // Finger has stopped due to a contact while opening.
  return 1;
}

//////////////////////////////////////////////////
uint8_t RobotiqHandPlugin::GetCurrentPosition(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _jointEntity)
{
  gz::sim::Joint joint(_jointEntity);
  double lower = 0.0;
  double upper = 0.0;
  const auto axes = joint.Axis(_ecm);
  if (axes && !axes->empty()) {
    lower = (*axes)[0].Lower();
    upper = (*axes)[0].Upper();
  }

  // Full range of motion.
  double range = upper - lower;

  // The maximum value in pinch mode is 177.
  if (this->graspingMode == Pinch) {
    range *= 177.0 / 255.0;
  }

  const double relAngle = FirstOrZero(joint.Position(_ecm)) - lower;

  return static_cast<uint8_t>(std::round(255.0 * relAngle / range));
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::GetAndPublishHandleState(
  gz::sim::EntityComponentManager & _ecm)
{
  this->handleState.g_act = this->userHandleCommand.r_act;
  this->handleState.g_mod = this->userHandleCommand.r_mod;
  this->handleState.g_gto = this->userHandleCommand.r_gto;

  if (this->handState == Emergency) {
    this->handleState.g_imc = 0;
  } else if (this->handState == ChangeModeInProgress) {
    this->handleState.g_imc = 2;
  } else {
    this->handleState.g_imc = 3;
  }

  gz::sim::Joint jointA(this->joints[2]);
  gz::sim::Joint jointB(this->joints[3]);
  gz::sim::Joint jointC(this->joints[4]);
  const bool isMovingA = FirstOrZero(jointA.Velocity(_ecm)) > kVelTolerance;
  const bool isMovingB = FirstOrZero(jointB.Velocity(_ecm)) > kVelTolerance;
  const bool isMovingC = FirstOrZero(jointC.Velocity(_ecm)) > kVelTolerance;

  double pe, ie, de;
  this->posePID[2].Errors(pe, ie, de);
  const bool reachPositionA = pe < kPoseTolerance;
  this->posePID[3].Errors(pe, ie, de);
  const bool reachPositionB = pe < kPoseTolerance;
  this->posePID[4].Errors(pe, ie, de);
  const bool reachPositionC = pe < kPoseTolerance;

  if (isMovingA || isMovingB || isMovingC) {
    this->handleState.g_sta = 0;
  } else if (reachPositionA && reachPositionB && reachPositionC) {
    this->handleState.g_sta = 3;
  } else if (!reachPositionA && !reachPositionB && !reachPositionC) {
    this->handleState.g_sta = 2;
  } else {
    this->handleState.g_sta = 1;
  }

  this->handleState.g_dta = this->GetObjectDetection(
    _ecm, this->joints[2], 2, this->handleCommand.r_pra, this->prevCommand.r_pra);
  this->handleState.g_dtb = this->GetObjectDetection(
    _ecm, this->joints[3], 3, this->handleCommand.r_prb, this->prevCommand.r_prb);
  this->handleState.g_dtc = this->GetObjectDetection(
    _ecm, this->joints[4], 4, this->handleCommand.r_prc, this->prevCommand.r_prc);
  this->handleState.g_dts = this->GetObjectDetection(
    _ecm, this->joints[0], 0, this->handleCommand.r_prs, this->prevCommand.r_prs);

  if (this->handState == ChangeModeInProgress) {
    this->handleState.g_flt = 6;
  } else if (this->handState == Disabled) {
    this->handleState.g_flt = 7;
  } else if (this->handState == Emergency) {
    this->handleState.g_flt = 11;
  } else {
    this->handleState.g_flt = 0;
  }

  this->handleState.g_pra = this->userHandleCommand.r_pra;
  this->handleState.g_poa = this->GetCurrentPosition(_ecm, this->joints[2]);
  this->handleState.g_cua = 0;

  this->handleState.g_prb = this->userHandleCommand.r_prb;
  this->handleState.g_pob = this->GetCurrentPosition(_ecm, this->joints[3]);
  this->handleState.g_cub = 0;

  this->handleState.g_prc = this->userHandleCommand.r_prc;
  this->handleState.g_poc = this->GetCurrentPosition(_ecm, this->joints[4]);
  this->handleState.g_cuc = 0;

  this->handleState.g_prs = this->userHandleCommand.r_prs;
  this->handleState.g_pos = this->GetCurrentPosition(_ecm, this->joints[1]);
  this->handleState.g_cus = 0;

  this->pubHandleState->publish(this->handleState);
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::GetAndPublishJointState(
  const gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp)
{
  this->jointStates.header.stamp = _stamp;
  for (std::size_t i = 0; i < this->joints.size(); ++i) {
    gz::sim::Joint joint(this->joints[i]);
    this->jointStates.position[i] = FirstOrZero(joint.Position(_ecm));
    this->jointStates.velocity[i] = FirstOrZero(joint.Velocity(_ecm));
  }
  this->pubJointStates->publish(this->jointStates);
}

//////////////////////////////////////////////////
void RobotiqHandPlugin::UpdatePIDControl(
  gz::sim::EntityComponentManager & _ecm, double _dt)
{
  if (this->handState == Disabled) {
    for (int i = 0; i < kNumJoints; ++i) {
      gz::sim::Joint(this->fingerJoints[i]).SetForce(_ecm, {0.0});
    }
    return;
  }

  const std::chrono::duration<double> dtDuration(_dt);

  for (int i = 0; i < kNumJoints; ++i) {
    gz::sim::Joint informativeJoint(this->joints[i]);
    double lower = 0.0;
    double upper = 0.0;
    const auto axes = informativeJoint.Axis(_ecm);
    if (axes && !axes->empty()) {
      lower = (*axes)[0].Lower();
      upper = (*axes)[0].Upper();
    }

    double targetPose = 0.0;
    // Only ever assigned in the Scissor branches below, and -- same as the
    // original -- never actually read afterward. Preserved as-is: it looks
    // like vestigial support for a velocity-mode Scissor control path that
    // was never wired up to the PID, not a deliberate design choice worth
    // "fixing" during a port.
    double targetSpeed = (kMinVelocity + kMaxVelocity) / 2.0;

    if (i == 0) {
      switch (this->graspingMode) {
        case Wide:
          targetPose = upper;
          break;
        case Pinch:
          // -11 degrees.
          targetPose = -0.1919;
          break;
        case Scissor:
          // Max position is reached at value 215.
          targetPose = upper -
            (upper - lower) * (215.0 / 255.0) * this->handleCommand.r_pra / 255.0;
          break;
        case Basic:
        default:
          break;
      }
    } else if (i == 1) {
      switch (this->graspingMode) {
        case Wide:
          targetPose = lower;
          break;
        case Pinch:
          // 11 degrees.
          targetPose = 0.1919;
          break;
        case Scissor:
          // Max position is reached at value 215.
          targetPose = lower +
            (upper - lower) * (215.0 / 255.0) * this->handleCommand.r_pra / 255.0;
          break;
        case Basic:
        default:
          break;
      }
    } else {
      if (this->graspingMode == Pinch) {
        // Max position is reached at value 177.
        targetPose = lower +
          (upper - lower) * (177.0 / 255.0) * this->handleCommand.r_pra / 255.0;
      } else if (this->graspingMode == Scissor) {
        targetSpeed = kMinVelocity +
          (kMaxVelocity - kMinVelocity) * this->handleCommand.r_spa / 255.0;
      } else {
        targetPose = lower +
          (upper - lower) * this->handleCommand.r_pra / 255.0;
      }
    }
    static_cast<void>(targetSpeed);

    const double currentPose = FirstOrZero(informativeJoint.Position(_ecm));
    const double poseError = currentPose - targetPose;
    const double torque = this->posePID[i].Update(poseError, dtDuration);
    gz::sim::Joint(this->fingerJoints[i]).SetForce(_ecm, {torque});
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(RobotiqHandPlugin,
              gz::sim::System,
              RobotiqHandPlugin::ISystemConfigure,
              RobotiqHandPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RobotiqHandPlugin,
    "drcsim_gazebo_ros_plugins::RobotiqHandPlugin")
