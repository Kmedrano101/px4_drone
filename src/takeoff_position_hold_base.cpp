#include "px4_drone/takeoff_position_hold_base.hpp"

#include <limits>

#include "px4_drone/frame_transforms.hpp"

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

TakeoffPositionHoldBase::TakeoffPositionHoldBase(
  const std::string & node_name, float default_takeoff_height_m, float default_hold_seconds)
: Node(node_name),
  takeoff_height_m_(static_cast<float>(
      declare_parameter<double>("takeoff_height_m", default_takeoff_height_m))),
  hold_seconds_(static_cast<float>(
      declare_parameter<double>("hold_seconds", default_hold_seconds)))
{
  const bool confirm_takeoff = declare_parameter<bool>("confirm_takeoff", false);
  // Ver comentario equivalente en offboard_control.cpp: "" para firmware
  // v1.14 (default, FC actual), "_v1" para v1.17 (HKUST_NXT_DUAL).
  const std::string v = declare_parameter<std::string>("topic_version_suffix", "");

  if (!confirm_takeoff) {
    RCLCPP_FATAL(
      get_logger(),
      "Este nodo va a DESPEGAR de verdad. Por seguridad no arranca sin la confirmacion "
      "explicita: relanzar con --ros-args -p confirm_takeoff:=true. Nodo detenido.");
    ok_to_run_ = false;
    return;
  }

  if (takeoff_height_m_ <= 0.0f) {
    RCLCPP_FATAL(
      get_logger(), "takeoff_height_m debe ser positivo (recibido %.2f). Nodo detenido.",
      takeoff_height_m_);
    ok_to_run_ = false;
    return;
  }

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.transient_local();
  qos.keep_last(1);

  rclcpp::QoS status_qos(5);
  status_qos.best_effort();

  offboard_control_mode_pub_ = create_publisher<OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);
  trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);
  vehicle_command_pub_ = create_publisher<VehicleCommand>(
    "/fmu/in/vehicle_command", qos);

  vehicle_status_sub_ = create_subscription<VehicleStatus>(
    "/fmu/out/vehicle_status" + v, status_qos,
    std::bind(&TakeoffPositionHoldBase::vehicleStatusCallback, this, std::placeholders::_1));

  vehicle_local_position_sub_ = create_subscription<VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position" + v, status_qos,
    std::bind(&TakeoffPositionHoldBase::vehicleLocalPositionCallback, this, std::placeholders::_1));

  vehicle_command_ack_sub_ = create_subscription<VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", status_qos,
    std::bind(&TakeoffPositionHoldBase::vehicleCommandAckCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / kLoopRateHz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&TakeoffPositionHoldBase::onTimer, this));

  RCLCPP_WARN(
    get_logger(),
    "%s iniciado: VA A DESPEGAR %.1f m y mantener posicion %.0f s.",
    node_name.c_str(), takeoff_height_m_, hold_seconds_);
}

void TakeoffPositionHoldBase::onTimer()
{
  switch (state_) {
    case State::kWaitPositionSource: {
        if (cycle_count_ == 0) {
          RCLCPP_INFO(
            get_logger(), "Esperando fuente de posicion: %s...", positionSourceName().c_str());
        }
        if (positionSourceReady()) {
          RCLCPP_INFO(get_logger(), "Fuente de posicion (%s) lista.", positionSourceName().c_str());
          state_ = State::kWarmup;
          cycle_count_ = 0;
        } else if (cycle_count_ >= kPositionSourceTimeoutCycles) {
          RCLCPP_ERROR(
            get_logger(),
            "Fuente de posicion (%s) no disponible tras %.0f s. Abortando (no se toca OFFBOARD/ARM).",
            positionSourceName().c_str(), kPositionSourceTimeoutCycles / kLoopRateHz);
          state_ = State::kFinished;
        }
        break;
      }

    case State::kWarmup: {
        publishOffboardControlMode();
        publishHoldCurrentPosition();
        if (cycle_count_ >= kWarmupCycles) {
          state_ = State::kRequestOffboard;
        }
        break;
      }

    case State::kRequestOffboard: {
        publishOffboardControlMode();
        publishHoldCurrentPosition();
        publishVehicleCommand(
          VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        RCLCPP_INFO(get_logger(), "Solicitando modo OFFBOARD...");
        state_ = State::kArm;
        cycle_count_ = 0;
        break;
      }

    case State::kArm: {
        publishOffboardControlMode();
        publishHoldCurrentPosition();

        if (!offboard_confirmed_) {
          if (cycle_count_ >= kOffboardConfirmTimeoutCycles) {
            RCLCPP_WARN(
              get_logger(),
              "No se confirmo el cambio a OFFBOARD (nav_state) en %.0f s. Abortando intento de ARM.",
              kOffboardConfirmTimeoutCycles / kLoopRateHz);
            state_ = State::kFinished;
          }
          break;
        }

        if (!arm_wait_started_) {
          arm_wait_started_ = true;
          arm_wait_start_cycle_ = cycle_count_;
        }

        if (!arm_command_sent_ && cycle_count_ >= arm_wait_start_cycle_ + kArmDelayCycles) {
          publishVehicleCommand(
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand::ARMING_ACTION_ARM);
          RCLCPP_INFO(get_logger(), "Solicitando ARM...");
          arm_command_sent_ = true;
          arm_sent_cycle_ = cycle_count_;
        }

        if (arm_command_sent_) {
          if (last_arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
            const auto * lp = localPosition();
            origin_x_ned_ = lp->x;
            origin_y_ned_ = lp->y;
            origin_z_ned_ = lp->z;

            const px4_drone::frames::Vec3 climb_enu{0.0f, 0.0f, takeoff_height_m_};
            const auto climb_ned = px4_drone::frames::enuNedSwap(climb_enu);
            target_x_ned_ = origin_x_ned_ + climb_ned[0];
            target_y_ned_ = origin_y_ned_ + climb_ned[1];
            target_z_ned_ = origin_z_ned_ + climb_ned[2];
            target_yaw_ned_ = lp->heading;

            RCLCPP_INFO(
              get_logger(),
              "ARMADO. Despegando %.1f m desde origen NED (%.2f, %.2f, %.2f) "
              "hacia (%.2f, %.2f, %.2f)...",
              takeoff_height_m_, origin_x_ned_, origin_y_ned_, origin_z_ned_,
              target_x_ned_, target_y_ned_, target_z_ned_);

            state_ = State::kTakeoff;
            cycle_count_ = 0;
          } else if (cycle_count_ >= arm_sent_cycle_ + kArmConfirmTimeoutCycles) {
            RCLCPP_ERROR(
              get_logger(),
              "El FC no confirmo ARMED tras %.0f s de enviado el comando. Abortando.",
              kArmConfirmTimeoutCycles / kLoopRateHz);
            state_ = State::kFinished;
          }
        }
        break;
      }

    case State::kTakeoff: {
        publishOffboardControlMode();
        publishTrajectorySetpoint(target_x_ned_, target_y_ned_, target_z_ned_, target_yaw_ned_);
        if (cycle_count_ == 1) {
          RCLCPP_INFO(get_logger(), "Subiendo a la altura objetivo...");
        }

        const auto * lp = localPosition();
        const bool height_reached = lp != nullptr &&
          std::fabs(lp->z - target_z_ned_) < kAltitudeToleranceM;

        if (height_reached || cycle_count_ >= kTakeoffTimeoutCycles) {
          if (height_reached) {
            RCLCPP_INFO(get_logger(), "Altura objetivo alcanzada.");
          } else {
            RCLCPP_WARN(
              get_logger(), "No se confirmo la altura objetivo tras %.0f s, se continua igual.",
              kTakeoffTimeoutCycles / kLoopRateHz);
          }
          state_ = State::kHold;
          cycle_count_ = 0;
        }
        break;
      }

    case State::kHold: {
        publishOffboardControlMode();
        publishTrajectorySetpoint(target_x_ned_, target_y_ned_, target_z_ned_, target_yaw_ned_);
        if (cycle_count_ == 1) {
          RCLCPP_INFO(get_logger(), "Manteniendo posicion por %.0f s...", hold_seconds_);
        }
        if (cycle_count_ >= static_cast<uint64_t>(hold_seconds_ * kLoopRateHz)) {
          RCLCPP_INFO(get_logger(), "Fin del hold. Solicitando aterrizaje (VEHICLE_CMD_NAV_LAND)...");
          publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          state_ = State::kLand;
          cycle_count_ = 0;
        }
        break;
      }

    case State::kLand: {
        // A partir de aqui el FC controla el descenso (AUTO_LAND); no seguimos
        // publicando setpoints de offboard, igual que un GCS que suelta el
        // control tras pedir LAND.
        if (last_arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Aterrizaje confirmado: el FC se desarmo.");
          state_ = State::kFinished;
        } else if (cycle_count_ >= kLandTimeoutCycles) {
          RCLCPP_WARN(
            get_logger(), "No se confirmo el desarme tras el aterrizaje en %.0f s.",
            kLandTimeoutCycles / kLoopRateHz);
          state_ = State::kFinished;
        }
        break;
      }

    case State::kFinished:
      timer_->cancel();
      break;
  }

  ++cycle_count_;
}

void TakeoffPositionHoldBase::publishOffboardControlMode()
{
  OffboardControlMode msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  offboard_control_mode_pub_->publish(msg);
}

void TakeoffPositionHoldBase::publishTrajectorySetpoint(
  float ned_x, float ned_y, float ned_z, float yaw_ned)
{
  TrajectorySetpoint msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  msg.position = {ned_x, ned_y, ned_z};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.yaw = yaw_ned;
  trajectory_setpoint_pub_->publish(msg);
}

void TakeoffPositionHoldBase::publishHoldCurrentPosition()
{
  const auto * lp = localPosition();
  if (lp != nullptr) {
    publishTrajectorySetpoint(lp->x, lp->y, lp->z, lp->heading);
  } else {
    // No deberia pasar: kWaitPositionSource ya exigio datos validos de
    // posicion antes de llegar aqui.
    publishTrajectorySetpoint(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void TakeoffPositionHoldBase::publishVehicleCommand(
  uint16_t command, float param1, float param2, float param4, float param5, float param6,
  float param7)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  vehicle_command_pub_->publish(msg);
}

void TakeoffPositionHoldBase::vehicleStatusCallback(const VehicleStatus::SharedPtr msg)
{
  if (msg->nav_state != last_nav_state_) {
    RCLCPP_INFO(
      get_logger(), "nav_state: %s -> %s",
      navStateToString(last_nav_state_).c_str(), navStateToString(msg->nav_state).c_str());

    if (state_ == State::kArm && !offboard_confirmed_ &&
      msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      offboard_confirmed_ = true;
      RCLCPP_INFO(
        get_logger(), "OFFBOARD confirmado por el FC, esperando %.1f s antes de armar...",
        kArmDelayCycles / kLoopRateHz);
    }

    last_nav_state_ = msg->nav_state;
  }

  if (msg->arming_state != last_arming_state_) {
    RCLCPP_INFO(
      get_logger(), "arming_state: %s",
      msg->arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMED" : "DISARMED");
    last_arming_state_ = msg->arming_state;
  }
}

void TakeoffPositionHoldBase::vehicleLocalPositionCallback(
  const VehicleLocalPosition::SharedPtr msg)
{
  last_local_position_ = msg;
}

void TakeoffPositionHoldBase::vehicleCommandAckCallback(const VehicleCommandAck::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "vehicle_command_ack: command=%u result=%s (result_param1=%u)",
    msg->command, commandResultToString(msg->result).c_str(), msg->result_param1);
}

std::string TakeoffPositionHoldBase::commandResultToString(uint8_t result)
{
  using Ack = px4_msgs::msg::VehicleCommandAck;
  switch (result) {
    case Ack::VEHICLE_CMD_RESULT_ACCEPTED: return "ACCEPTED";
    case Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
    case Ack::VEHICLE_CMD_RESULT_DENIED: return "DENIED";
    case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED: return "UNSUPPORTED";
    case Ack::VEHICLE_CMD_RESULT_FAILED: return "FAILED";
    case Ack::VEHICLE_CMD_RESULT_IN_PROGRESS: return "IN_PROGRESS";
    case Ack::VEHICLE_CMD_RESULT_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN(" + std::to_string(result) + ")";
  }
}

std::string TakeoffPositionHoldBase::navStateToString(uint8_t nav_state)
{
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL: return "MANUAL";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
    case VehicleStatus::NAVIGATION_STATE_POSCTL: return "POSCTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "AUTO_MISSION";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "AUTO_LOITER";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "AUTO_RTL";
    case VehicleStatus::NAVIGATION_STATE_ACRO: return "ACRO";
    case VehicleStatus::NAVIGATION_STATE_DESCEND: return "DESCEND";
    case VehicleStatus::NAVIGATION_STATE_TERMINATION: return "TERMINATION";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
    case VehicleStatus::NAVIGATION_STATE_STAB: return "STAB";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "AUTO_TAKEOFF";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "AUTO_LAND";
    case VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: return "AUTO_FOLLOW_TARGET";
    case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND: return "AUTO_PRECLAND";
    case VehicleStatus::NAVIGATION_STATE_ORBIT: return "ORBIT";
    case VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: return "AUTO_VTOL_TAKEOFF";
    default: return "UNKNOWN(" + std::to_string(nav_state) + ")";
  }
}
