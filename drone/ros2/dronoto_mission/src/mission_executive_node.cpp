#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "dronoto_core/safety_state_machine.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "dronoto_msgs/msg/safety_state.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "dronoto_msgs/srv/trigger_failsafe.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using dronoto_core::SafetyState;

/// Phases de la mission P1. Strictement sequentielles.
enum class Phase
{
  WAIT_TELEMETRY,
  PREFLIGHT,
  TAKEOFF_MODE,    ///< demander AUTO_TAKEOFF, AVANT d'armer
  ARMING,
  CLIMBING,        ///< PX4 execute le decollage jusqu'a MIS_TAKEOFF_ALT
  ENTER_OFFBOARD,
  CLIMB_TO_MISSION,///< en offboard, monter a l'altitude de mission
  WAYPOINTS,
  LANDING,
  DONE,
  ABORTED,
};

/// Executif de mission : decollage, waypoints, atterrissage.
///
/// Ne pilote JAMAIS directement : il demande des transitions au superviseur et
/// publie des objectifs pour le suivi de trajectoire. Le superviseur peut
/// refuser ou outrepasser a tout moment.
///
/// P1 : sequenceur a etats. BehaviorTree.CPP arrive en P4, quand la boucle
/// d'exploration a besoin de reevaluation reactive.
class MissionExecutiveNode : public rclcpp::Node
{
public:
  MissionExecutiveNode()
  : Node("mission_executive")
  {
    declare_parameter<double>("takeoff_altitude_m", 5.0);
    declare_parameter<double>("waypoint_tolerance_m", 0.8);
    declare_parameter<double>("phase_timeout_s", 60.0);
    declare_parameter<bool>("autostart", false);
    // Waypoints aplatis : [x1, y1, z1, x2, y2, z2, ...] en repere ENU local.
    //
    // Une liste vide en YAML (« waypoints: [] ») n'a pas de type : ROS 2 ne
    // peut pas l'inferer et declare_parameter leve une exception qui tue le
    // noeud. On la rattrape pour produire un message actionnable plutot qu'un
    // abort brutal, et on retombe sur une mission sans waypoint.
    try {
      declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
      RCLCPP_ERROR(
        get_logger(),
        "parametre 'waypoints' non typable (%s). Ne pas ecrire « waypoints: [] » "
        "dans le YAML : retirer la ligne, ou donner des valeurs reelles. "
        "Mission demarree sans waypoint.",
        e.what());
    }

    takeoff_altitude_ = get_parameter("takeoff_altitude_m").as_double();
    waypoint_tolerance_ = get_parameter("waypoint_tolerance_m").as_double();
    phase_timeout_s_ = get_parameter("phase_timeout_s").as_double();
    autostart_ = get_parameter("autostart").as_bool();

    const auto waypoints_param = get_parameter("waypoints");
    if (waypoints_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      loadWaypoints(waypoints_param.as_double_array());
    }

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        have_telemetry_ = true;
      });

    safety_sub_ = create_subscription<dronoto_msgs::msg::SafetyState>(
      "safety/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](dronoto_msgs::msg::SafetyState::UniquePtr msg) { safety_ = *msg; });

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      "navigation/distance_to_goal", rclcpp::QoS(10),
      [this](std_msgs::msg::Float32::UniquePtr msg) {
        distance_to_goal_ = msg->data;
        have_distance_ = true;
      });

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "navigation/goal", rclcpp::QoS(1).reliable().transient_local());
    phase_pub_ = create_publisher<std_msgs::msg::String>(
      "mission/phase", rclcpp::QoS(1).reliable().transient_local());
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());

    arm_client_ = create_client<std_srvs::srv::Trigger>("px4/arm");
    takeoff_client_ = create_client<std_srvs::srv::Trigger>("px4/takeoff");
    offboard_client_ = create_client<std_srvs::srv::Trigger>("px4/set_offboard");
    land_client_ = create_client<std_srvs::srv::Trigger>("px4/land");
    safety_client_ = create_client<dronoto_msgs::srv::TriggerFailsafe>(
      "safety/trigger_failsafe");

    timer_ = create_wall_timer(200ms, [this] { tick(); });  // 5 Hz
    phase_start_ = now();
    RCLCPP_INFO(
      get_logger(), "mission_executive demarre : %zu waypoints, decollage a %.1f m, autostart=%s",
      waypoints_.size(), takeoff_altitude_, autostart_ ? "true" : "false");
  }

private:
  void loadWaypoints(const std::vector<double> & flat)
  {
    if (flat.size() % 3 != 0) {
      RCLCPP_ERROR(
        get_logger(), "parametre 'waypoints' invalide : %zu valeurs, multiple de 3 attendu",
        flat.size());
      return;
    }
    for (size_t i = 0; i + 2 < flat.size(); i += 3) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "odom";
      wp.pose.position.x = flat[i];
      wp.pose.position.y = flat[i + 1];
      wp.pose.position.z = flat[i + 2];
      wp.pose.orientation.w = 1.0;
      waypoints_.push_back(wp);
    }
  }

  void setPhase(Phase p, const std::string & label)
  {
    phase_ = p;
    phase_start_ = now();
    std_msgs::msg::String msg;
    msg.data = label;
    phase_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "phase -> %s", label.c_str());
    publishEvent(dronoto_msgs::msg::SafetyEvent::INFO, "MISSION_PHASE", label);
  }

  bool phaseTimedOut() const { return (now() - phase_start_).seconds() > phase_timeout_s_; }

  void abort(const std::string & why)
  {
    publishEvent(dronoto_msgs::msg::SafetyEvent::ERROR, "MISSION_ABORTED", why);
    RCLCPP_ERROR(get_logger(), "mission interrompue : %s", why.c_str());
    callTrigger(land_client_);
    setPhase(Phase::ABORTED, "ABORTED");
  }

  void tick()
  {
    if (!autostart_ || phase_ == Phase::DONE || phase_ == Phase::ABORTED) {
      return;
    }

    // Le superviseur a l'autorite : s'il n'est plus dans un etat compatible
    // avec la poursuite de la mission, on abandonne.
    if (static_cast<SafetyState>(safety_.state) == SafetyState::EMERGENCY_LAND) {
      abort("le superviseur a declenche un atterrissage d'urgence");
      return;
    }

    if (phaseTimedOut()) {
      abort("delai de phase depasse");
      return;
    }

    switch (phase_) {
      case Phase::WAIT_TELEMETRY:
        if (have_telemetry_) {
          requestSafety(SafetyState::IDLE, "demarrage de mission");
          requestSafety(SafetyState::PREFLIGHT, "verifications prealables");
          setPhase(Phase::PREFLIGHT, "PREFLIGHT");
        }
        break;

      case Phase::PREFLIGHT:
        // Verifications prealables de 05-integration-px4.md, sous-ensemble P1.
        if (telemetry_.preflight_ok && telemetry_.ekf_position_valid &&
            telemetry_.battery_remaining > 0.30f) {
          requestSafety(SafetyState::READY, "verifications passees");
          // ORDRE CRITIQUE : demander le mode decollage AVANT d'armer, comme le
          // fait PX4 lui-meme (Commander.cpp, commande « takeoff »). Armer en
          // AUTO_LOITER au sol laisse l'integrateur d'altitude du controleur de
          // position se charger, et le drone part pleine poussee des qu'il
          // quitte le sol — constate : montee incontrolee jusqu'a 14 km.
          callTrigger(takeoff_client_);
          setPhase(Phase::TAKEOFF_MODE, "TAKEOFF_MODE");
        }
        break;

      case Phase::TAKEOFF_MODE:
        if (telemetry_.nav_state ==
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
          callTrigger(arm_client_);
          setPhase(Phase::ARMING, "ARMING");
        }
        break;

      case Phase::ARMING:
        if (telemetry_.armed) {
          requestSafety(SafetyState::ARMED, "arme");
          requestSafety(SafetyState::TAKEOFF, "decollage");
          setPhase(Phase::CLIMBING, "CLIMBING");
        }
        break;

      case Phase::CLIMBING:
        // On attend que PX4 declare le decollage TERMINE (il quitte
        // AUTO_TAKEOFF pour AUTO_LOITER), et non un seuil d'altitude : PX4
        // monte a MIS_TAKEOFF_ALT, qui n'a aucune raison d'egaler l'altitude
        // demandee par la mission. Attendre un seuil qu'il n'atteindra jamais
        // bloque la mission jusqu'au delai de phase.
        if (telemetry_.nav_state !=
              px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF &&
            telemetry_.altitude_relative_m > 1.0f) {
          publishFirstGoalAtCurrentPosition();
          callTrigger(offboard_client_);
          setPhase(Phase::ENTER_OFFBOARD, "ENTER_OFFBOARD");
        }
        break;

      case Phase::ENTER_OFFBOARD:
        if (telemetry_.offboard_active) {
          requestSafety(SafetyState::NOMINAL, "offboard actif");
          publishClimbGoal();
          setPhase(Phase::CLIMB_TO_MISSION, "CLIMB_TO_MISSION");
        }
        break;

      case Phase::CLIMB_TO_MISSION:
        // PX4 a assure le decollage (effet de sol, detection de sol) ; notre
        // controleur prend le relais pour rejoindre l'altitude de mission.
        if (telemetry_.altitude_relative_m > 0.9 * takeoff_altitude_) {
          waypoint_index_ = 0;
          if (waypoints_.empty()) {
            startLanding();
          } else {
            publishCurrentWaypoint();
            setPhase(Phase::WAYPOINTS, "WAYPOINTS");
          }
        }
        break;

      case Phase::WAYPOINTS:
        if (have_distance_ && distance_to_goal_ < waypoint_tolerance_) {
          RCLCPP_INFO(get_logger(), "waypoint %zu atteint", waypoint_index_ + 1);
          ++waypoint_index_;
          if (waypoint_index_ >= waypoints_.size()) {
            startLanding();
          } else {
            publishCurrentWaypoint();
            phase_start_ = now();  // relancer le compteur pour le waypoint suivant
          }
        }
        break;

      case Phase::LANDING:
        if (telemetry_.altitude_relative_m < 0.3f && !telemetry_.armed) {
          requestSafety(SafetyState::TERMINATED, "pose et desarme");
          setPhase(Phase::DONE, "DONE");
          publishEvent(
            dronoto_msgs::msg::SafetyEvent::INFO, "MISSION_COMPLETE",
            "mission terminee avec succes");
        }
        break;

      case Phase::DONE:
      case Phase::ABORTED:
        break;
    }
  }

  void startLanding()
  {
    requestSafety(SafetyState::LANDING, "waypoints termines");
    callTrigger(land_client_);
    setPhase(Phase::LANDING, "LANDING");
  }

  /// Publie un objectif a la position courante avant de basculer en offboard.
  /// Sans cela, le premier setpoint offboard pourrait etre tres eloigne et
  /// provoquer une embardee.
  void publishFirstGoalAtCurrentPosition()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "odom";
    goal.pose = telemetry_.pose;
    goal_pub_->publish(goal);
  }

  /// Objectif de montee : position horizontale courante, altitude de mission.
  void publishClimbGoal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "odom";
    goal.pose = telemetry_.pose;
    goal.pose.position.z = takeoff_altitude_;
    goal_pub_->publish(goal);
    RCLCPP_INFO(get_logger(), "montee vers %.1f m sous controle offboard", takeoff_altitude_);
  }

  void publishCurrentWaypoint()
  {
    auto wp = waypoints_[waypoint_index_];
    wp.header.stamp = now();
    goal_pub_->publish(wp);
    have_distance_ = false;  // ne pas reutiliser la distance de l'objectif precedent
    RCLCPP_INFO(
      get_logger(), "waypoint %zu/%zu : (%.1f, %.1f, %.1f)", waypoint_index_ + 1,
      waypoints_.size(), wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
  }

  void requestSafety(SafetyState target, const std::string & reason)
  {
    if (!safety_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "service safety/trigger_failsafe indisponible");
      return;
    }
    auto req = std::make_shared<dronoto_msgs::srv::TriggerFailsafe::Request>();
    req->target_state = static_cast<uint8_t>(target);
    req->reason = reason;
    safety_client_->async_send_request(req);
  }

  void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "service %s indisponible", client->get_service_name());
      return;
    }
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
  }

  Phase phase_{Phase::WAIT_TELEMETRY};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  dronoto_msgs::msg::SafetyState safety_{};
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t waypoint_index_{0};
  float distance_to_goal_{1e9f};
  bool have_telemetry_{false};
  bool have_distance_{false};

  double takeoff_altitude_{5.0};
  double waypoint_tolerance_{0.8};
  double phase_timeout_s_{60.0};
  bool autostart_{false};

  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr offboard_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
  rclcpp::Client<dronoto_msgs::srv::TriggerFailsafe>::SharedPtr safety_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExecutiveNode>());
  rclcpp::shutdown();
  return 0;
}
