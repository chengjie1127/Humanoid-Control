//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_controllers/humanoidController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <humanoid_dummy/gait/GaitReceiver.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>
#include <humanoid_interface/common/utils.h>
#include <humanoid_wbc/WeightedWbc.h>
#include <array>
#include <limits>
#include <sstream>



namespace humanoid_controller{
using namespace ocs2;
using namespace humanoid;

namespace {
// Stance PD gains (used during warmup, stance hold, and start of walking blend)
// Stand gains tuned for MuJoCo: higher stiffness/damping on knee + ankle pitch to prevent
// gradual forward tip and joint collapse before the whole-body controller fully stabilizes.
constexpr std::array<float, 12> kStandKp = {250.0F, 250.0F, 120.0F, 250.0F, 120.0F, 80.0F,
                                           250.0F, 250.0F, 120.0F, 250.0F, 120.0F, 80.0F};
constexpr std::array<float, 12> kStandKd = {6.0F, 6.0F, 2.0F, 6.0F, 4.0F, 2.5F,
                                           6.0F, 6.0F, 2.0F, 6.0F, 4.0F, 2.5F};
// Walk PD gains: increased to provide robust tracking against modeling errors
constexpr std::array<float, 12> kWalkKp = {60.0F, 60.0F, 40.0F, 60.0F, 40.0F, 30.0F, 60.0F, 60.0F, 40.0F, 60.0F, 40.0F, 30.0F};
constexpr std::array<float, 12> kWalkKd = {8.0F, 8.0F, 8.0F, 8.0F, 4.0F, 3.2F, 8.0F, 8.0F, 8.0F, 8.0F, 4.0F, 3.2F};
}

bool humanoidController::init(std::shared_ptr<rclcpp::Node> controller_nh) {
  controllerNh_ = controller_nh;
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
    controllerNh_->declare_parameter<std::string>("urdfFile", "");
    controllerNh_->get_parameter("urdfFile", urdfFile);
    controllerNh_->declare_parameter<std::string>("taskFile", "");
    controllerNh_->get_parameter("taskFile", taskFile);
    controllerNh_->declare_parameter<std::string>("referenceFile", "");
    controllerNh_->get_parameter("referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);

  setupHumanoidInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      HumanoidInterface_->getPinocchioInterface(), pinocchioMapping, HumanoidInterface_->modelSettings().contactNames3DoF);

  // Visualization
  robotVisualizer_ = std::make_shared<HumanoidVisualizer>(HumanoidInterface_->getPinocchioInterface(),
                                                             HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);

  defalutJointPos_.resize(jointNum_);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  // Hardware interface
  //TODO: setup hardware controller interface
  //create a ROS subscriber to receive the joint pos and vel
  jointPos_ = vector_t::Zero(jointNum_);
  jointPos_ << -0.30, 0.0, 0.0, 0.70, -0.40, 0, -0.30, 0.0, 0.0, 0.70, -0.40, 0;
  jointVel_ = vector_t::Zero(jointNum_);
  quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
  angularVel_.setZero();
  linearAccel_.setZero();
  orientationCovariance_.setZero();
  angularVelCovariance_.setZero();
  linearAccelCovariance_.setZero();
  lastJointStateWallTime_ = std::chrono::steady_clock::now();
  lastImuWallTime_ = std::chrono::steady_clock::now();
  lastDiagnosticWallTime_ = std::chrono::steady_clock::now();
  jointPosVelSub_ = controllerNh_->create_subscription<std_msgs::msg::Float32MultiArray>("/jointsPosVel", 10, std::bind(&humanoidController::jointStateCallback, this, std::placeholders::_1));
  imuSub_ = controllerNh_->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&humanoidController::ImuCallback, this, std::placeholders::_1));
  targetTorquePub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetTorque", 10);
  targetPosPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetPos", 10);
  targetVelPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetVel", 10);
  targetKpPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetKp", 10);
  targetKdPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetKd", 10);

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(HumanoidInterface_->getPinocchioInterface(), HumanoidInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(HumanoidInterface_->getCentroidalModelInfo());

  return true;
}

void humanoidController::jointStateCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg) {
  if (msg->data.size() != 2 * jointNum_) {
    RCLCPP_ERROR_STREAM(controllerNh_->get_logger(), "Received joint state message with wrong size: " << msg->data.size());
    return;
  }
  for (size_t i = 0; i < jointNum_; ++i) {
      jointPos_(i) = msg->data[i];
      jointVel_(i) = msg->data[i + jointNum_];
  }
  receivedJointState_ = true;
  lastJointStateWallTime_ = std::chrono::steady_clock::now();
}

void humanoidController::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    quat_.coeffs().w() = msg->orientation.w;
    quat_.coeffs().x() = msg->orientation.x;
    quat_.coeffs().y() = msg->orientation.y;
    quat_.coeffs().z() = msg->orientation.z;
    angularVel_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    linearAccel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    orientationCovariance_ << msg->orientation_covariance[0], msg->orientation_covariance[1], msg->orientation_covariance[2],
            msg->orientation_covariance[3], msg->orientation_covariance[4], msg->orientation_covariance[5],
            msg->orientation_covariance[6], msg->orientation_covariance[7], msg->orientation_covariance[8];
    angularVelCovariance_ << msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
            msg->angular_velocity_covariance[3], msg->angular_velocity_covariance[4], msg->angular_velocity_covariance[5],
            msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], msg->angular_velocity_covariance[8];
    linearAccelCovariance_ << msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
            msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
            msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8];
    receivedImu_ = true;
    lastImuWallTime_ = std::chrono::steady_clock::now();
}

void humanoidController::starting(const rclcpp::Time& time) {
  while (!(receivedJointState_ && receivedImu_) && rclcpp::ok()) {
    publishDiagnosticStatus(true);
    publishStandCommand();
    rclcpp::sleep_for(std::chrono::milliseconds(2));
  }

  // Use a controller-local time base: OCS2 typically assumes time starts at 0.
  // We derive it from ROS time (/clock in simulation) by subtracting an offset.
  rosTimeZeroSec_ = time.seconds();
  lastRosTimeSec_ = rosTimeZeroSec_;
  rosTimeZeroValid_ = true;

  // Initial state
  currentObservation_.state = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.state(8) = 0.75;
  currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;

  updateStateEstimation(time, rclcpp::Duration::from_seconds(0.002));
  currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  RCLCPP_INFO_STREAM(controllerNh_->get_logger(), "Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
    publishDiagnosticStatus(true);
    publishStandCommand();
    try {
      mpcMrtInterface_->advanceMpc();
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(controllerNh_->get_logger(), "[humanoid Controller] advanceMpc exception during starting: " << e.what());
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / HumanoidInterface_->mpcSettings().mrtDesiredFrequency_)));
  }
  RCLCPP_INFO_STREAM(controllerNh_->get_logger(), "Initial policy has been received.");

  mpcRunning_ = true;
  controlBlendStartInitialized_ = true;
  controlBlendElapsedTime_ = 0.0;
  initialStanceHoldActive_ = true;
}

void humanoidController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {  
  if (!(receivedJointState_ && receivedImu_)) {
    publishDiagnosticStatus(true);
    publishStandCommand();
    return;
  }

  // MPC thread watchdog: helps diagnose policy staleness (thread stopped vs blocked vs slow).
  if (mpcRunning_) {
    const auto wallNow = std::chrono::steady_clock::now();
    const std::int64_t wallNowNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(wallNow.time_since_epoch()).count();
    const std::int64_t startNs = mpcAdvanceStartWallNs_.load(std::memory_order_relaxed);
    const std::int64_t endNs = mpcAdvanceEndWallNs_.load(std::memory_order_relaxed);
    const std::uint64_t count = mpcAdvanceCounter_.load(std::memory_order_relaxed);

    if (startNs > 0 && endNs > 0 && startNs > endNs) {
      const double inProgressSec = static_cast<double>(wallNowNs - startNs) * 1e-9;
      if (inProgressSec > 1.0) {
        RCLCPP_WARN_THROTTLE(controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
                             "[MpcWatchdog] advanceMpc appears in-progress for %.3fs (count=%lu). Possible hang.",
                             inProgressSec, static_cast<unsigned long>(count));
      }
    } else {
      const double sinceEndSec = (endNs > 0) ? static_cast<double>(wallNowNs - endNs) * 1e-9 : std::numeric_limits<double>::infinity();
      if (sinceEndSec > 2.0) {
        RCLCPP_WARN_THROTTLE(controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
                             "[MpcWatchdog] no advanceMpc completion for %.3fs (count=%lu).",
                             sinceEndSec, static_cast<unsigned long>(count));
      }
    }
  }

  // MPC warm-up: run full MPC+WBC pipeline but gradually ramp up WBC torque feedforward.
  // This lets the MPC converge on real sensor data while smoothly transitioning from PD-only to full WBC control.
  if (mpcWarmupActive_ && mpcWarmupMinIterations_ > 0) {
    mpcWarmupIterations_++;
    try {
      updateStateEstimation(time, period);
      mpcMrtInterface_->setCurrentObservation(currentObservation_);
      mpcMrtInterface_->updatePolicy();

      vector_t optimizedState, optimizedInput;
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode_);
      currentObservation_.input = optimizedInput;

      vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode_, period.seconds());
      const vector_t& torque = x.tail(jointNum_);

      // Ramp torque blend from 0 to 1 over the warm-up period
      scalar_t blend = std::clamp(static_cast<scalar_t>(mpcWarmupIterations_) / static_cast<scalar_t>(mpcWarmupMinIterations_), 0.0, 1.0);
      vector_t blendedTorque = blend * torque;
      publishStandCommand(blendedTorque);
      publishDiagnosticStatus(true, true, torque(0), torque(1), torque(2));
    } catch (const std::exception& e) {
      RCLCPP_WARN_STREAM(controllerNh_->get_logger(), "[humanoid Controller] MPC warmup exception: " << e.what());
      publishDiagnosticStatus(true);
      publishStandCommand();
    }

    if (mpcWarmupIterations_ >= mpcWarmupMinIterations_) {
      mpcWarmupActive_ = false;
      RCLCPP_INFO_STREAM(controllerNh_->get_logger(), 
          "MPC warm-up complete after " << mpcWarmupIterations_ << " iterations.");
    }
    return;
  }

  try {
    // State Estimate
    updateStateEstimation(time, period);

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy.
    // Guard against requesting a time beyond the last available policy point;
    // this can happen when the MPC thread lags and otherwise spams warnings and
    // forces extrapolation.
    vector_t optimizedState, optimizedInput;
    scalar_t policyEvalTime = currentObservation_.time;
    {
      const auto& policy = mpcMrtInterface_->getPolicy();
      if (!policy.timeTrajectory_.empty()) {
        const scalar_t policyEndTime = policy.timeTrajectory_.back();
        if (policyEvalTime > policyEndTime) {
          const scalar_t policyLagSec = policyEvalTime - policyEndTime;

          // If the MPC thread is wedged (advanceMpc stalled), the policy can become
          // arbitrarily stale. Continuing to apply WBC feedforward based on a stale
          // policy often destabilizes the robot. In that case, fall back to the
          // conservative PD stand command until MPC recovers.
          constexpr scalar_t kMaxPolicyLagSec = 0.05;  // 50ms
          if (policyLagSec > kMaxPolicyLagSec) {
            RCLCPP_WARN_THROTTLE(
                controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
                "MPC policy too stale (lag=%.3fs, now=%.3f, policy_end=%.3f). Falling back to PD stand.",
                policyLagSec, currentObservation_.time, policyEndTime);
            publishDiagnosticStatus(true);
            publishStandCommand();
            return;
          }

          policyEvalTime = policyEndTime;
          RCLCPP_WARN_THROTTLE(
              controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
              "MPC policy is stale (now=%.3f > policy_end=%.3f). Clamping evaluation time.",
              currentObservation_.time, policyEndTime);
        }
      }
    }
    mpcMrtInterface_->evaluatePolicy(policyEvalTime, currentObservation_.state, optimizedState, optimizedInput, plannedMode_);

    // Lightweight debug to confirm that a STANCE gait actually stays in STANCE,
    // and to catch any unexpected policy schedules that would release stance hold.
    {
      const auto& modeSequence = mpcMrtInterface_->getPolicy().modeSchedule_.modeSequence;
      bool upcomingNonStance = false;
      for (const auto& mode : modeSequence) {
        if (mode != ModeNumber::STANCE) {
          upcomingNonStance = true;
          break;
        }
      }

      if (initialStanceHoldActive_) {
        std::ostringstream modeSeqStream;
        modeSeqStream << "[";
        const size_t maxPrint = 10;
        for (size_t i = 0; i < modeSequence.size() && i < maxPrint; ++i) {
          modeSeqStream << modeSequence[i];
          if (i + 1 < modeSequence.size() && i + 1 < maxPrint) {
            modeSeqStream << ",";
          }
        }
        if (modeSequence.size() > maxPrint) {
          modeSeqStream << ",...";
        }
        modeSeqStream << "]";

        RCLCPP_INFO_THROTTLE(
            controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
            "[StanceHold] active=1 plannedMode=%d upcomingNonStance=%d policyModeSeq=%s",
            static_cast<int>(plannedMode_), static_cast<int>(upcomingNonStance), modeSeqStream.str().c_str());
      } else {
        RCLCPP_INFO_THROTTLE(
            controllerNh_->get_logger(), *controllerNh_->get_clock(), 1000,
            "[StanceHold] active=0 plannedMode=%d upcomingNonStance=%d",
            static_cast<int>(plannedMode_), static_cast<int>(upcomingNonStance));
      }
    }

    // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode_, period.seconds());
    wbcTimer_.endTimer();

    const vector_t& torque = x.tail(jointNum_);
    const vector_t& wbc_planned_joint_acc = x.segment(6, jointNum_);
    const vector_t& wbc_planned_body_acc = x.head(6);
    const vector_t& wbc_planned_contact_force = x.segment(6 + jointNum_, wbc_->getContactForceSize());


    vector_t posDes = centroidal_model::getJointAngles(optimizedState, HumanoidInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, HumanoidInterface_->getCentroidalModelInfo());

    scalar_t dt = period.seconds();
    posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
    velDes = velDes + wbc_planned_joint_acc * dt;

    // Release stance hold when first non-STANCE mode is planned or upcoming in the policy
    if (initialStanceHoldActive_) {
      bool upcomingNonStance = false;
      const auto& modeSequence = mpcMrtInterface_->getPolicy().modeSchedule_.modeSequence;
      for (const auto& mode : modeSequence) {
        if (mode != ModeNumber::STANCE) {
          upcomingNonStance = true;
          break;
        }
      }

      if (plannedMode_ != ModeNumber::STANCE || upcomingNonStance) {
        RCLCPP_INFO_STREAM(controllerNh_->get_logger(),
            "Initial stance hold released (mode=" << plannedMode_ << ", upcomingNonStance=" << upcomingNonStance << ")");
        initialStanceHoldActive_ = false;
        walkingBlendActive_ = true;
        walkingBlendIterations_ = 0;
        filteredWalkTorque_ = torque;
        walkTorqueFilterInitialized_ = true;
      }
    }

    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
      RCLCPP_ERROR_STREAM(controllerNh_->get_logger(), "[humanoid Controller] Safety check failed, stopping the controller.");
      publishDiagnosticStatus(true, true, torque(0), torque(1), torque(2));
      // Safety trips typically mean the robot is already in an extreme orientation.
      // Do not keep applying WBC feedforward torque (it can thrash / amplify the fall).
      // Fall back to a conservative joint-space PD stand command.
      publishStandCommand();
      //TODO: send the stop command to hardware interface
      return;
    }

    // During stance hold: WBC torque + PD to default joint positions (proven stable)
    if (initialStanceHoldActive_ && plannedMode_ == ModeNumber::STANCE) {
      publishDiagnosticStatus(true, true, torque(0), torque(1), torque(2));

      // Hold current joint angles (no posture correction) while applying WBC feedforward torque.
      // This avoids injecting large PD torques that can fight the WBC solution.
      publishHoldCommand(torque);

      robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
      observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
      return;
    }

    publishDiagnosticStatus(false, true, torque(0), torque(1), torque(2));

    // Walking diagnostics: log orientation and torques at 20Hz
    {
      const auto wallNow = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(wallNow - lastDiagnosticWallTime_).count() >= 50) {
        scalar_t actual_pitch = 2.0 * std::asin(quat_.y());
        scalar_t actual_roll = 2.0 * std::asin(quat_.x());
        RCLCPP_INFO_STREAM(controllerNh_->get_logger(),
            "[WalkDiag] mode=" << plannedMode_
            << " pitch=" << actual_pitch << " roll=" << actual_roll
            << " t0=" << torque(0) << " t6=" << torque(6));
      }
    }

    // Walking blend: smooth 500ms transition from stance to walk PD gains.
    // During the blend, stance-leg joints are smoothly blended from default
    // positions to MPC-desired positions. Swing-leg joints always get full MPC
    // positions so the foot can follow the swing trajectory.
    constexpr int kWalkingBlendDuration = 250;  // 500ms at 500Hz
    std::array<float, 12> activeKp, activeKd;
    if (walkingBlendActive_) {
      walkingBlendIterations_++;
      float alpha = std::min(1.0f, static_cast<float>(walkingBlendIterations_) / static_cast<float>(kWalkingBlendDuration));

      // Determine which joints are on the stance leg vs swing leg
      // Joints 0-5 = left leg, joints 6-11 = right leg
      bool leftIsSwinging = (plannedMode_ == ModeNumber::RCONTACT);  // right foot on ground => left swings
      bool rightIsSwinging = (plannedMode_ == ModeNumber::LCONTACT); // left foot on ground => right swings

      // Blend stance-leg positions; pass through swing-leg positions fully
      for (int i = 0; i < static_cast<int>(jointNum_); i++) {
        bool isSwingJoint = (i < 6 && leftIsSwinging) || (i >= 6 && rightIsSwinging);
        if (!isSwingJoint) {
          // Stance leg (or double-support): blend from default to MPC
          posDes(i) = (1.0 - alpha) * defalutJointPos_(i) + alpha * posDes(i);
          velDes(i) = alpha * velDes(i);
        }
        // Swing leg: keep full MPC posDes/velDes for trajectory tracking
      }

      // Always blend PD gains smoothly
      for (int i = 0; i < 12; i++) {
        activeKp[i] = kStandKp[i] * (1.0f - alpha) + kWalkKp[i] * alpha;
        activeKd[i] = kStandKd[i] * (1.0f - alpha) + kWalkKd[i] * alpha;
      }
      if (walkingBlendIterations_ >= kWalkingBlendDuration) {
        walkingBlendActive_ = false;
        RCLCPP_INFO(controllerNh_->get_logger(), "Walking blend complete");
      }
    } else {
      activeKp = kWalkKp;
      activeKd = kWalkKd;
    }

    // Mode-dependent PD: reduce PD gains for swing legs to avoid fighting WBC
    // The WBC already computes optimal torques for swing trajectory tracking;
    // but moderate PD overlay is required in imperfect physics simulation.
    constexpr float kSwingKp = 40.0f;   // Provided moderate position PD for swing legs
    constexpr float kSwingKd = 4.0f;    // Increased damping
    if (plannedMode_ == ModeNumber::LCONTACT) {
      // Left foot on ground → right leg (joints 6-11) is swinging
      for (int i = 6; i < 12; i++) {
        activeKp[i] = kSwingKp;
        activeKd[i] = kSwingKd;
      }
    } else if (plannedMode_ == ModeNumber::RCONTACT) {
      // Right foot on ground → left leg (joints 0-5) is swinging
      for (int i = 0; i < 6; i++) {
        activeKp[i] = kSwingKp;
        activeKd[i] = kSwingKd;
      }
    }

    // (Pitch drift compensation removed — constant offsets fight the MPC)

    // EMA filter to smooth WBC torque chattering from high-authority QP
    constexpr float kTorqueFilterAlpha = 0.3f;
    vector_t walkTorque = torque;
    if (walkTorqueFilterInitialized_) {
      for (int i = 0; i < static_cast<int>(jointNum_); i++) {
        filteredWalkTorque_(i) = kTorqueFilterAlpha * torque(i) + (1.0f - kTorqueFilterAlpha) * filteredWalkTorque_(i);
      }
      walkTorque = filteredWalkTorque_;
    } else {
      filteredWalkTorque_ = torque;
      walkTorqueFilterInitialized_ = true;
    }

    // (Pitch feedforward torque removed — constant offsets fight the MPC)

    // Publish WBC torque + blended PD gains
    {
      std_msgs::msg::Float32MultiArray targetTorqueMsg;
      for (int i1 = 0; i1 < 12; ++i1) {
          targetTorqueMsg.data.push_back(static_cast<float>(walkTorque(i1)));
      }
      std_msgs::msg::Float32MultiArray targetPosMsg;
      for (int i1 = 0; i1 < 12; ++i1) {
          targetPosMsg.data.push_back(static_cast<float>(posDes(i1)));
      }
      std_msgs::msg::Float32MultiArray targetVelMsg;
      for (int i1 = 0; i1 < 12; ++i1) {
          targetVelMsg.data.push_back(static_cast<float>(velDes(i1)));
      }
      targetTorquePub_->publish(targetTorqueMsg);
      targetPosPub_->publish(targetPosMsg);
      targetVelPub_->publish(targetVelMsg);
      std_msgs::msg::Float32MultiArray targetKp;
      std_msgs::msg::Float32MultiArray targetKd;
      targetKp.data.assign(activeKp.begin(), activeKp.end());
      targetKd.data.assign(activeKd.begin(), activeKd.end());
      targetKpPub_->publish(targetKp);
      targetKdPub_->publish(targetKd);
    }

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(controllerNh_->get_logger(), "[humanoid Controller] update exception: " << e.what());
    publishDiagnosticStatus(true);
    publishStandCommand();
    return;
  }
}

void humanoidController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(jointNum_), jointVel(jointNum_);
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;


  jointPos = jointPos_;
    jointVel = jointVel_;
  //TODO: get contactFlag from hardware interface
  //暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
  contactFlag = modeNumber2StanceLeg(plannedMode_);

  quat = quat_;
  angularVel = angularVel_;
  linearAccel = linearAccel_;
  orientationCovariance = orientationCovariance_;
  angularVelCovariance = angularVelCovariance_;
  linearAccelCovariance = linearAccelCovariance_;

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  // OCS2 assumes a local timebase starting at 0. Use ROS time (/clock) but convert
  // it to controller-relative time to avoid huge absolute timestamps.
  const double rosNowSec = time.seconds();
  if (!rosTimeZeroValid_) {
    rosTimeZeroSec_ = rosNowSec;
    lastRosTimeSec_ = rosNowSec;
    rosTimeZeroValid_ = true;
  }
  // Handle simulation time resets (e.g. restarting simulator without restarting all nodes).
  if (rosNowSec + 1e-6 < lastRosTimeSec_) {
    RCLCPP_WARN_STREAM(controllerNh_->get_logger(),
                       "ROS time jumped back (" << lastRosTimeSec_ << " -> " << rosNowSec
                                                << "). Resetting controller time base.");
    rosTimeZeroSec_ = rosNowSec;
  }
  lastRosTimeSec_ = rosNowSec;
  currentObservation_.time = std::max(0.0, rosNowSec - rosTimeZeroSec_);
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  //  currentObservation_.mode = stateEstimate_->getMode();
  //TODO: 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
  currentObservation_.mode =  plannedMode_;
}

void humanoidController::publishStandCommand() {
  std_msgs::msg::Float32MultiArray targetTorqueMsg;
  targetTorqueMsg.data.assign(jointNum_, 0.0F);

  std_msgs::msg::Float32MultiArray targetPosMsg;
  targetPosMsg.data.reserve(jointNum_);
  for (size_t i = 0; i < jointNum_; ++i) {
    targetPosMsg.data.push_back(static_cast<float>(defalutJointPos_(i)));
  }

  std_msgs::msg::Float32MultiArray targetVelMsg;
  targetVelMsg.data.assign(jointNum_, 0.0F);

  std_msgs::msg::Float32MultiArray targetKpMsg;
  targetKpMsg.data.assign(kStandKp.begin(), kStandKp.end());
  std_msgs::msg::Float32MultiArray targetKdMsg;
  targetKdMsg.data.assign(kStandKd.begin(), kStandKd.end());

  targetTorquePub_->publish(targetTorqueMsg);
  targetPosPub_->publish(targetPosMsg);
  targetVelPub_->publish(targetVelMsg);
  targetKpPub_->publish(targetKpMsg);
  targetKdPub_->publish(targetKdMsg);
}

void humanoidController::publishStandCommand(const vector_t& torqueFeedforward) {
  std_msgs::msg::Float32MultiArray targetTorqueMsg;
  targetTorqueMsg.data.reserve(jointNum_);
  for (size_t i = 0; i < jointNum_; ++i) {
    targetTorqueMsg.data.push_back(static_cast<float>(torqueFeedforward(i)));
  }

  std_msgs::msg::Float32MultiArray targetPosMsg;
  targetPosMsg.data.reserve(jointNum_);
  for (size_t i = 0; i < jointNum_; ++i) {
    targetPosMsg.data.push_back(static_cast<float>(defalutJointPos_(i)));
  }

  std_msgs::msg::Float32MultiArray targetVelMsg;
  targetVelMsg.data.assign(jointNum_, 0.0F);

  std_msgs::msg::Float32MultiArray targetKpMsg;
  targetKpMsg.data.assign(kStandKp.begin(), kStandKp.end());
  std_msgs::msg::Float32MultiArray targetKdMsg;
  targetKdMsg.data.assign(kStandKd.begin(), kStandKd.end());

  targetTorquePub_->publish(targetTorqueMsg);
  targetPosPub_->publish(targetPosMsg);
  targetVelPub_->publish(targetVelMsg);
  targetKpPub_->publish(targetKpMsg);
  targetKdPub_->publish(targetKdMsg);
}

void humanoidController::publishHoldCommand(const vector_t& torqueFeedforward) {
  std_msgs::msg::Float32MultiArray targetTorqueMsg;
  targetTorqueMsg.data.reserve(jointNum_);
  for (size_t i = 0; i < jointNum_; ++i) {
    targetTorqueMsg.data.push_back(static_cast<float>(torqueFeedforward(i)));
  }

  std_msgs::msg::Float32MultiArray targetPosMsg;
  targetPosMsg.data.reserve(jointNum_);
  for (size_t i = 0; i < jointNum_; ++i) {
    targetPosMsg.data.push_back(static_cast<float>(jointPos_(i)));
  }

  std_msgs::msg::Float32MultiArray targetVelMsg;
  targetVelMsg.data.assign(jointNum_, 0.0F);

  std_msgs::msg::Float32MultiArray targetKpMsg;
  targetKpMsg.data.assign(kStandKp.begin(), kStandKp.end());
  std_msgs::msg::Float32MultiArray targetKdMsg;
  targetKdMsg.data.assign(kStandKd.begin(), kStandKd.end());

  targetTorquePub_->publish(targetTorqueMsg);
  targetPosPub_->publish(targetPosMsg);
  targetVelPub_->publish(targetVelMsg);
  targetKpPub_->publish(targetKpMsg);
  targetKdPub_->publish(targetKdMsg);
}

void humanoidController::publishDiagnosticStatus(bool fallbackMode, bool hasTorque, scalar_t torque0, scalar_t torque1, scalar_t torque2) {
  const auto wallNow = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(wallNow - lastDiagnosticWallTime_).count() < 200) {
    return;
  }
  lastDiagnosticWallTime_ = wallNow;

  const auto contactFlag = modeNumber2StanceLeg(plannedMode_);
  const auto closedContacts = numberOfClosedContacts(contactFlag);
  const auto jointAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(wallNow - lastJointStateWallTime_).count();
  const auto imuAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(wallNow - lastImuWallTime_).count();

  std::ostringstream contactFlagsStream;
  contactFlagsStream << "[";
  for (size_t i = 0; i < contactFlag.size(); ++i) {
    contactFlagsStream << (contactFlag[i] ? 1 : 0);
    if (i + 1 < contactFlag.size()) {
      contactFlagsStream << ",";
    }
  }
  contactFlagsStream << "]";

  std::ostringstream torqueStream;
  if (hasTorque) {
    torqueStream << "[" << torque0 << ", " << torque1 << ", " << torque2 << "]";
  } else {
    torqueStream << "[n/a, n/a, n/a]";
  }

  RCLCPP_INFO_STREAM(
      controllerNh_->get_logger(),
      "[CtrlDiag] fallback=" << (fallbackMode ? "1" : "0")
                             << " sensors_ready=" << ((receivedJointState_ && receivedImu_) ? "1" : "0")
                             << " joint_age_ms=" << jointAgeMs
                             << " imu_age_ms=" << imuAgeMs
                             << " quat_wxyz=[" << quat_.w() << ", " << quat_.x() << ", " << quat_.y() << ", " << quat_.z() << "]"
                             << " torque012=" << torqueStream.str()
                             << " planned_mode=" << plannedMode_
                             << " contacts_closed=" << closedContacts
                             << " contact_flags=" << contactFlagsStream.str());
}

humanoidController::~humanoidController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void humanoidController::setupHumanoidInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile);
  HumanoidInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void humanoidController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->sqpSettings(),
                                  HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                    HumanoidInterface_->getCentroidalModelInfo());

  const std::string robotName = "humanoid";
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(controllerNh_, HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, HumanoidInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(controllerNh_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = controllerNh_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void humanoidCheaterController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->sqpSettings(),
                                 HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                   HumanoidInterface_->getCentroidalModelInfo());

  const std::string robotName = "humanoid";

  // For MuJoCo cheat-control STANCE stabilization we don't rely on live gait schedule updates.
  // Avoid registering synchronized modules (e.g., GaitReceiver) to reduce the risk of blocking
  // inside advanceMpc().

  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, HumanoidInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(controllerNh_);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  observationPublisher_ = controllerNh_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void humanoidController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&HumanoidInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    RCLCPP_INFO_STREAM(controllerNh_->get_logger(), "[MpcThread] started");
    auto lastHeartbeat = std::chrono::steady_clock::now();
    size_t advanceCounter = 0;
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                const auto startWall = std::chrono::steady_clock::now();
                mpcAdvanceStartWallNs_.store(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(startWall.time_since_epoch()).count(),
                    std::memory_order_relaxed);
                mpcMrtInterface_->advanceMpc();
                const auto endWall = std::chrono::steady_clock::now();
                mpcAdvanceEndWallNs_.store(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(endWall.time_since_epoch()).count(),
                    std::memory_order_relaxed);
                mpcTimer_.endTimer();
                ++advanceCounter;
                mpcAdvanceCounter_.store(static_cast<std::uint64_t>(advanceCounter), std::memory_order_relaxed);

                const auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastHeartbeat).count() >= 1000) {
                  lastHeartbeat = now;
                  RCLCPP_INFO_STREAM(controllerNh_->get_logger(),
                                     "[MpcThread] advanceMpc_ok count=" << advanceCounter
                                                                    << " now=" << controllerNh_->now().seconds());
                }
              }
            },
            HumanoidInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR_STREAM(controllerNh_->get_logger(), "[Ocs2 MPC thread] Error : " << e.what());
        //TODO: send the stop command to hardware interface
      }
    }
  });
  setThreadPriority(HumanoidInterface_->sqpSettings().threadPriority, mpcThread_);
}

void humanoidController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                          HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void humanoidCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);

  // In MuJoCo cheat-control, joint-space PD alone cannot stabilize the floating base.
  // The generic MPC warm-up ramps WBC feedforward torque from 0 -> 100% over a few seconds,
  // which causes the robot to tip before sufficient stabilizing torque is applied.
  // Disable the warm-up so full WBC torque is available immediately after unpausing.
  disableMpcWarmup();
}

}  // namespace humanoid_controller