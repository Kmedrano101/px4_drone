#include "px4_drone/offboard_control.hpp"

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleStatus;

OffboardControl::OffboardControl()
: Node("offboard_control")
{
  // El firmware PX4 solo agrega el sufijo "_vN" a un topic si el mensaje
  // correspondiente tiene MESSAGE_VERSION != 0 (ver px4_msgs commit
  // alineado). En firmwares viejos (ej. release/1.14, anteriores al
  // versionado de mensajes) estos mismos topics van SIN sufijo. Parametro
  // para poder apuntar el mismo nodo a cualquiera de los dos sin recompilar
  // codigo distinto: "" (default, firmware v1.14, FC actual) o "_v1"
  // (firmware v1.17 HKUST_NXT_DUAL, branch px4_msgs fc-v17-82e3322e).
  const std::string v = declare_parameter<std::string>("topic_version_suffix", "");

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.transient_local();
  qos.keep_last(1);

  rclcpp::QoS status_qos(5);
  status_qos.best_effort();

  offboard_control_mode_pub_ = create_publisher<OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);
  vehicle_attitude_setpoint_pub_ = create_publisher<VehicleAttitudeSetpoint>(
    "/fmu/in/vehicle_attitude_setpoint" + v, qos);
  vehicle_command_pub_ = create_publisher<VehicleCommand>(
    "/fmu/in/vehicle_command", qos);

  vehicle_status_sub_ = create_subscription<VehicleStatus>(
    "/fmu/out/vehicle_status" + v, status_qos,
    std::bind(&OffboardControl::vehicleStatusCallback, this, std::placeholders::_1));

  vehicle_command_ack_sub_ = create_subscription<VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", status_qos,
    std::bind(&OffboardControl::vehicleCommandAckCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / kLoopRateHz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&OffboardControl::onTimer, this));

  RCLCPP_INFO(get_logger(), "offboard_control iniciado a %.0f Hz", kLoopRateHz);
}

void OffboardControl::onTimer()
{
  switch (state_) {
    case State::kWarmup: {
        publishOffboardControlMode();
        publishVehicleAttitudeSetpoint();
        if (cycle_count_ >= kWarmupCycles) {
          state_ = State::kRequestOffboard;
        }
        break;
      }

    case State::kRequestOffboard: {
        publishOffboardControlMode();
        publishVehicleAttitudeSetpoint();
        publishVehicleCommand(
          VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        RCLCPP_INFO(get_logger(), "Solicitando modo OFFBOARD...");
        state_ = State::kArm;
        cycle_count_ = 0;
        break;
      }

    case State::kArm: {
        publishOffboardControlMode();
        publishVehicleAttitudeSetpoint();

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

        if (cycle_count_ == arm_wait_start_cycle_ + kArmDelayCycles) {
          publishVehicleCommand(
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand::ARMING_ACTION_ARM);
          RCLCPP_INFO(get_logger(), "Solicitando ARM...");
        }
        if (cycle_count_ >= arm_wait_start_cycle_ + kArmDelayCycles) {
          state_ = State::kOffboardActive;
          cycle_count_ = 0;
        }
        break;
      }

    case State::kOffboardActive: {
        publishOffboardControlMode();
        publishVehicleAttitudeSetpoint();
        if (cycle_count_ == 1) {
          RCLCPP_INFO(
            get_logger(),
            "En OFFBOARD activo (armado, actitud nivelada, empuje cero, sin despegue), esperando %.0f s antes de simular perdida de heartbeat...",
            kActiveCyclesBeforeFailsafeTest / kLoopRateHz);
        }
        if (cycle_count_ >= kActiveCyclesBeforeFailsafeTest) {
          state_ = State::kSimulateFailsafe;
          cycle_count_ = 0;
          RCLCPP_WARN(
            get_logger(),
            ">>> Simulando perdida de heartbeat: se deja de publicar OffboardControlMode/VehicleAttitudeSetpoint <<<");
        }
        break;
      }

    case State::kSimulateFailsafe: {
        // No publicamos nada: esto es lo que hace PX4 pensar que perdimos el enlace.
        if (cycle_count_ >= kFailsafeMonitorCycles) {
          state_ = State::kFinished;
          if (failsafe_confirmed_) {
            RCLCPP_INFO(
              get_logger(),
              "Test de failsafe FINALIZADO: el FC reacciono correctamente a la perdida de heartbeat "
              "(cambio a Position mode o se desarmo).");
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Test de failsafe FINALIZADO: no se detecto ni POSCTL ni desarme en %.0f s. "
              "Revisa COM_OBL_ACT / COM_OF_LOSS_T en el FC.",
              kFailsafeMonitorCycles / kLoopRateHz);
          }
        }
        break;
      }

    case State::kFinished:
      timer_->cancel();
      break;
  }

  ++cycle_count_;
}

void OffboardControl::publishOffboardControlMode()
{
  OffboardControlMode msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = true;
  msg.body_rate = false;
  offboard_control_mode_pub_->publish(msg);
}

void OffboardControl::publishVehicleAttitudeSetpoint()
{
  // Control por actitud (no requiere estimador de posicion/velocidad, a
  // diferencia de position/velocity): actitud nivelada, empuje cero,
  // sin comandar despegue. Props deben estar retirados o el vehiculo asegurado,
  // ya que al armar en este modo los motores giran aunque el empuje sea cero.
  VehicleAttitudeSetpoint msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.q_d = {1.0f, 0.0f, 0.0f, 0.0f};
  msg.thrust_body = {0.0f, 0.0f, 0.0f};
  vehicle_attitude_setpoint_pub_->publish(msg);
}

void OffboardControl::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  vehicle_command_pub_->publish(msg);
}

void OffboardControl::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  if (msg->nav_state != last_nav_state_) {
    RCLCPP_INFO(
      get_logger(), "nav_state: %s -> %s",
      navStateToString(last_nav_state_).c_str(), navStateToString(msg->nav_state).c_str());

    if (state_ == State::kSimulateFailsafe &&
      last_nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
      (msg->nav_state == VehicleStatus::NAVIGATION_STATE_POSCTL ||
      msg->nav_state == VehicleStatus::NAVIGATION_STATE_ALTCTL))
    {
      failsafe_confirmed_ = true;
      RCLCPP_INFO(
        get_logger(), "*** FAILSAFE OK: el FC cambio a %s al perder el heartbeat ***",
        navStateToString(msg->nav_state).c_str());
    }

    if (state_ == State::kArm && !offboard_confirmed_ &&
      msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      offboard_confirmed_ = true;
      RCLCPP_INFO(get_logger(), "OFFBOARD confirmado por el FC, esperando %.1f s antes de armar...",
        kArmDelayCycles / kLoopRateHz);
    }

    last_nav_state_ = msg->nav_state;
  }

  if (msg->arming_state != last_arming_state_) {
    RCLCPP_INFO(
      get_logger(), "arming_state: %s",
      msg->arming_state == VehicleStatus::ARMING_STATE_ARMED ? "ARMED" : "DISARMED");

    if (state_ == State::kSimulateFailsafe &&
      last_arming_state_ == VehicleStatus::ARMING_STATE_ARMED &&
      msg->arming_state != VehicleStatus::ARMING_STATE_ARMED)
    {
      failsafe_confirmed_ = true;
      RCLCPP_INFO(
        get_logger(),
        "*** FAILSAFE OK: el FC se desarmo al perder el heartbeat "
        "(sin estimador de posicion, no puede caer a Position mode) ***");
    }

    last_arming_state_ = msg->arming_state;
  }
}

void OffboardControl::vehicleCommandAckCallback(
  const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "vehicle_command_ack: command=%u result=%s (result_param1=%u)",
    msg->command, commandResultToString(msg->result).c_str(), msg->result_param1);
}

std::string OffboardControl::commandResultToString(uint8_t result)
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

std::string OffboardControl::navStateToString(uint8_t nav_state)
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
