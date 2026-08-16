#include <chrono>
#include <memory>
#include <string>

#include "dronoto_core/safety_state_machine.hpp"
#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "dronoto_msgs/msg/safety_state.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "dronoto_msgs/srv/trigger_failsafe.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using dronoto_core::SafetyState;
using dronoto_core::SafetyStateMachine;
using dronoto_core::SystemConditions;

/// Superviseur de securite. Autorite de veto absolue sur la chaine de controle.
///
/// Entree  : control/setpoint_safe (sortie de l'evitement reactif ; en P1, le
///           suivi de trajectoire directement, via remappage dans le lancement)
/// Sortie  : control/setpoint_final (seule entree de px4_interface)
///
/// Le superviseur peut supprimer ou remplacer toute consigne sans cooperation
/// de l'amont : un composant de securite qui a besoin que le composant fautif
/// coopere n'est pas un composant de securite.
class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode()
  : Node("safety_supervisor"), fsm_(SafetyState::BOOT)
  {
    declare_parameter<double>("telemetry_timeout_s", 1.0);
    declare_parameter<double>("setpoint_timeout_s", 0.5);
    declare_parameter<double>("battery_critical_ratio", 0.10);
    declare_parameter<double>("battery_return_ratio", 0.25);
    telemetry_timeout_s_ = get_parameter("telemetry_timeout_s").as_double();
    setpoint_timeout_s_ = get_parameter("setpoint_timeout_s").as_double();
    battery_critical_ratio_ = get_parameter("battery_critical_ratio").as_double();
    battery_return_ratio_ = get_parameter("battery_return_ratio").as_double();

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        last_telemetry_time_ = now();
        have_telemetry_ = true;
      });

    // Le superviseur observe directement le statut PX4 : c'est la seule
    // exception documentee a la regle « seul px4_interface parle a PX4 »
    // (docs/architecture/05-integration-px4.md). Il doit voir ce que PX4 a
    // detecte sans dependre d'un intermediaire qui pourrait defaillir.
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { px4_status_ = *msg; });

    setpoint_sub_ = create_subscription<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_safe", rclcpp::QoS(1),
      [this](dronoto_msgs::msg::ControlSetpoint::UniquePtr msg) {
        incoming_setpoint_ = *msg;
        last_setpoint_time_ = now();
        have_setpoint_ = true;
      });

    // TRANSIENT_LOCAL : tout noeud qui demarre recoit immediatement l'etat
    // courant sans attendre la publication suivante.
    state_pub_ = create_publisher<dronoto_msgs::msg::SafetyState>(
      "safety/state", rclcpp::QoS(1).reliable().transient_local());
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());
    setpoint_pub_ = create_publisher<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_final", rclcpp::QoS(1));

    transition_srv_ = create_service<dronoto_msgs::srv::TriggerFailsafe>(
      "safety/trigger_failsafe",
      [this](
        const dronoto_msgs::srv::TriggerFailsafe::Request::SharedPtr req,
        dronoto_msgs::srv::TriggerFailsafe::Response::SharedPtr res) {
        const auto target = static_cast<SafetyState>(req->target_state);
        const bool ok = fsm_.requestTransition(target, buildConditions());
        res->accepted = ok;
        res->message = ok
          ? std::string("transition acceptee vers ") + SafetyStateMachine::toString(target)
          : std::string("transition refusee depuis ") +
              SafetyStateMachine::toString(fsm_.state());
        if (ok) {
          publishEvent(
            dronoto_msgs::msg::SafetyEvent::INFO, "STATE_TRANSITION",
            std::string(SafetyStateMachine::toString(target)) + " (" + req->reason + ")");
          publishState();
        }
      });

    timer_ = create_wall_timer(100ms, [this] { tick(); });  // 10 Hz
    publishState();
    RCLCPP_INFO(get_logger(), "safety_supervisor demarre en etat BOOT");
  }

private:
  SystemConditions buildConditions() const
  {
    SystemConditions c;
    const auto t = now();

    c.telemetry_fresh =
      have_telemetry_ && (t - last_telemetry_time_).seconds() < telemetry_timeout_s_;

    if (c.telemetry_fresh) {
      c.armed = telemetry_.armed;
      c.preflight_ok = telemetry_.preflight_ok;
      c.landed = telemetry_.altitude_relative_m < 0.5f;
      // battery_remaining vaut -1 quand PX4 ne sait pas : ne pas confondre
      // « inconnu » avec « vide », sinon le drone atterrit d'urgence au sol.
      const bool battery_known = telemetry_.battery_remaining >= 0.0f;
      c.battery_critical =
        battery_known && telemetry_.battery_remaining < battery_critical_ratio_;
      c.battery_below_return =
        battery_known && telemetry_.battery_remaining < battery_return_ratio_;
      c.state_estimate_lost = !telemetry_.ekf_position_valid;
    } else {
      // Aucune telemetrie : on suppose le pire cas, jamais le meilleur.
      c.state_estimate_lost = true;
    }

    // VehicleStatus.failsafe est le drapeau que PX4 leve lui-meme lorsqu'il
    // prend la main. On le lit directement plutot que de le reconstituer a
    // partir de conditions individuelles.
    c.px4_failsafe_active = px4_status_.failsafe;

    const bool setpoint_fresh =
      have_setpoint_ && (t - last_setpoint_time_).seconds() < setpoint_timeout_s_;
    c.setpoint_stale = SafetyStateMachine::isInFlight(fsm_.state()) && !setpoint_fresh;

    // Conditions cablees en P6 : liaison, SLAM, perception, geofence.
    return c;
  }

  void tick()
  {
    const auto decision = fsm_.update(buildConditions());

    if (decision.changed) {
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::WARNING, decision.reason,
        std::string("etat -> ") + SafetyStateMachine::toString(decision.state));
    }

    publishState();
    forwardOrVetoSetpoint(decision);
  }

  /// Le point de veto. Trois cas, dans cet ordre.
  void forwardOrVetoSetpoint(const dronoto_core::SafetyDecision & decision)
  {
    if (!decision.allow_setpoints) {
      // 1. L'etat interdit toute consigne externe : on n'emet RIEN.
      //    px4_interface appliquera sa politique de consigne perimee
      //    (maintien de position), qui est le comportement sur.
      return;
    }

    const bool setpoint_fresh =
      have_setpoint_ && (now() - last_setpoint_time_).seconds() < setpoint_timeout_s_;
    if (!setpoint_fresh) {
      return;  // 2. Amont muet : meme raisonnement.
    }

    // 3. Consigne fraiche et etat permissif : transmission.
    auto out = incoming_setpoint_;
    out.header.stamp = now();
    out.source = incoming_setpoint_.source + "|supervisor";
    setpoint_pub_->publish(out);
  }

  void publishState()
  {
    dronoto_msgs::msg::SafetyState msg;
    msg.header.stamp = now();
    msg.state = static_cast<uint8_t>(fsm_.state());
    msg.reason = fsm_.reason();
    msg.allow_setpoints = SafetyStateMachine::allowsSetpoints(fsm_.state());
    msg.setpoint_vetoed = !msg.allow_setpoints;
    state_pub_->publish(msg);
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
    RCLCPP_INFO(get_logger(), "[%s] %s", code.c_str(), message.c_str());
  }

  SafetyStateMachine fsm_;

  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  dronoto_msgs::msg::ControlSetpoint incoming_setpoint_{};
  px4_msgs::msg::VehicleStatus px4_status_{};

  rclcpp::Time last_telemetry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  bool have_telemetry_{false};
  bool have_setpoint_{false};

  double telemetry_timeout_s_{1.0};
  double setpoint_timeout_s_{0.5};
  double battery_critical_ratio_{0.10};
  double battery_return_ratio_{0.25};

  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyState>::SharedPtr state_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Service<dronoto_msgs::srv::TriggerFailsafe>::SharedPtr transition_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
