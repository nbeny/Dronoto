#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "dronoto_core/frame_conversions.hpp"
#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
namespace frames = dronoto_core::frames;

/// Frontiere unique avec PX4.
///
/// Traduit les messages uORB en messages ROS 2 normalises, en convertissant
/// systematiquement NED/FRD (PX4) vers ENU/FLU (ROS). Publie la transformation
/// odom -> base_link et maintient le flux offboard.
///
/// AUCUN autre noeud du systeme ne parle a PX4 : la connaissance des conventions
/// PX4 est concentree ici, et tout le reste se teste sans PX4.
/// Voir docs/architecture/05-integration-px4.md.
class Px4InterfaceNode : public rclcpp::Node
{
public:
  Px4InterfaceNode()
  : Node("px4_interface")
  {
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("setpoint_stale_hold_s", 0.5);
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    stale_hold_s_ = get_parameter("setpoint_stale_hold_s").as_double();

    // PX4 publie en BEST_EFFORT. Un abonne RELIABLE ne recoit RIEN,
    // silencieusement : c'est le piege numero un de l'integration PX4/ROS 2.
    const auto px4_qos = rclcpp::SensorDataQoS();

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) { onOdometry(*msg); });

    // VERSIONNEMENT DES MESSAGES (PX4 >= 1.16).
    //
    // PX4 publie VehicleStatus sur le topic VERSIONNE /fmu/out/vehicle_status_v1.
    // Le topic /fmu/out/vehicle_status existe encore dans la liste mais ne
    // porte AUCUNE donnee. S'y abonner ne produit ni erreur ni avertissement :
    // le message reste simplement a sa valeur par defaut, donc armed=false et
    // preflight_ok=false pour toujours, et le drone n'arme jamais.
    //
    // On s'abonne aux deux pour rester tolerant a la version de PX4, et le
    // chien de garde ci-dessous rend l'absence de donnees BRUYANTE plutot que
    // silencieuse — c'est la vraie lecon de ce bug.
    auto on_status = [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      status_ = *msg;
      status_seen_ = true;
    };
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos, on_status);
    status_legacy_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos, on_status);

    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", px4_qos,
      [this](px4_msgs::msg::BatteryStatus::UniquePtr msg) { battery_ = *msg; });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) { local_pos_ = *msg; });

    setpoint_sub_ = create_subscription<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_final", rclcpp::QoS(1),
      [this](dronoto_msgs::msg::ControlSetpoint::UniquePtr msg) {
        last_setpoint_ = *msg;
        last_setpoint_time_ = now();
        have_setpoint_ = true;
      });

    telemetry_pub_ = create_publisher<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("state/odometry", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", rclcpp::QoS(10));
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", rclcpp::QoS(10));
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::QoS(10));
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());

    using px4_msgs::msg::VehicleCommand;
    makeTriggerService("px4/arm", "commande d'armement envoyee", [this] {
      sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    });
    makeTriggerService("px4/disarm", "commande de desarmement envoyee", [this] {
      sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    });
    makeTriggerService("px4/set_offboard", "flux offboard demarre, bascule differee", [this] {
      // Procedure d'entree en offboard documentee par PX4 : emettre le flux
      // AVANT de demander le mode. On arme le flux ici ; publishOffboardStream()
      // enverra la commande de bascule une fois assez de consignes emises.
      offboard_stream_enabled_ = true;
      offboard_switch_pending_ = true;
      offboard_stream_cycles_ = 0;
    });
    makeTriggerService("px4/takeoff", "decollage PX4 demande", [this] {
      disableOffboardStream();  // PX4 natif reprend le bus de consignes
      // Mode natif PX4 : mieux teste que le decollage en offboard et gere
      // correctement l'effet de sol (docs/architecture/05-integration-px4.md).
      sendCommand(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
    });
    makeTriggerService("px4/land", "atterrissage PX4 demande", [this] {
      disableOffboardStream();  // PX4 natif reprend le bus de consignes
      sendCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    });

    // 20 Hz : bien au-dessus du minimum de 2 Hz exige par PX4 pour l'offboard.
    offboard_timer_ = create_wall_timer(50ms, [this] { publishOffboardStream(); });

    // Chien de garde des abonnements PX4. Une desynchronisation de version de
    // topic ne produit aucune erreur : le message reste a sa valeur par defaut
    // et le systeme se bloque en silence. On la rend bruyante.
    subscription_watchdog_ = create_wall_timer(1s, [this] { checkPx4Subscriptions(); });

    RCLCPP_INFO(get_logger(), "px4_interface demarre, en attente des messages PX4");
  }

private:
  /// Fabrique un service std_srvs/Trigger qui execute une action et repond OK.
  /// Evite de repeter cinq fois le meme corps de lambda.
  template <typename Action>
  void makeTriggerService(const std::string & name, const std::string & reply, Action action)
  {
    services_.push_back(create_service<std_srvs::srv::Trigger>(
      name,
      [action, reply](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res) {
        action();
        res->success = true;
        res->message = reply;
      }));
  }

  /// Horodatage PX4 en microsecondes, requis par tous les messages /fmu/in/*.
  uint64_t px4TimestampUs() const { return static_cast<uint64_t>(now().nanoseconds() / 1000); }

  void onOdometry(const px4_msgs::msg::VehicleOdometry & msg)
  {
    if (!px4_seen_) {
      px4_seen_ = true;
      first_odometry_time_ = now();
      RCLCPP_INFO(get_logger(), "Premier message PX4 recu : liaison uXRCE-DDS active");
    }

    // --- conversion NED/FRD -> ENU/FLU ---
    const Eigen::Vector3d pos_ned(msg.position[0], msg.position[1], msg.position[2]);
    const Eigen::Vector3d vel_ned(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    // px4_msgs stocke le quaternion dans l'ordre (w, x, y, z).
    const Eigen::Quaterniond q_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);

    const Eigen::Vector3d pos_enu = frames::nedToEnu(pos_ned);
    const Eigen::Vector3d vel_enu = frames::nedToEnu(vel_ned);
    const Eigen::Quaterniond q_enu = frames::nedFrdToEnuFlu(q_ned);

    // Vitesse angulaire : FRD -> FLU, soit negation de y et z.
    const Eigen::Vector3d omega_flu(
      msg.angular_velocity[0], -msg.angular_velocity[1], -msg.angular_velocity[2]);

    const auto stamp = now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pos_enu.x();
    odom.pose.pose.position.y = pos_enu.y();
    odom.pose.pose.position.z = pos_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();
    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    // PX4 exprime la vitesse dans le repere monde ; nav_msgs/Odometry attend
    // le repere du corps. On y ramene.
    const Eigen::Vector3d vel_body = q_enu.conjugate() * vel_enu;
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    odom.twist.twist.angular.x = omega_flu.x();
    odom.twist.twist.angular.y = omega_flu.y();
    odom.twist.twist.angular.z = omega_flu.z();
    odom_pub_->publish(odom);

    // TF odom -> base_link : continu, ne saute JAMAIS.
    // Les discontinuites sont absorbees par map -> odom (REP-105).
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pos_enu.x();
    tf.transform.translation.y = pos_enu.y();
    tf.transform.translation.z = pos_enu.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    dronoto_msgs::msg::VehicleTelemetry tel;
    tel.header.stamp = stamp;
    tel.header.frame_id = odom_frame_;
    tel.pose = odom.pose.pose;
    tel.twist = odom.twist.twist;
    // local_pos_.z est en NED (positif vers le bas) par rapport a l'origine EKF.
    tel.altitude_relative_m = -local_pos_.z;

    tel.battery_remaining = battery_.remaining;
    tel.battery_voltage_v = battery_.voltage_v;
    tel.battery_current_a = battery_.current_a;

    tel.armed = (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    tel.nav_state = status_.nav_state;
    tel.offboard_active =
      (status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    tel.preflight_ok = status_.pre_flight_checks_pass;

    // Le GNSS detaille est cable en P2 avec le reste des capteurs. En P1 on
    // s'appuie sur les drapeaux de validite de l'EKF, qui sont ce qui compte
    // reellement pour decider si on peut voler.
    tel.gnss_ok = local_pos_.xy_valid && local_pos_.z_valid && local_pos_.xy_global;
    tel.gnss_satellites = 0;
    tel.gnss_eph_m = local_pos_.eph;
    tel.ekf_position_valid = local_pos_.xy_valid && local_pos_.z_valid;
    tel.ekf_velocity_valid = local_pos_.v_xy_valid && local_pos_.v_z_valid;

    telemetry_pub_->publish(tel);
  }

  /// Publie le flux offboard a 20 Hz, INCONDITIONNELLEMENT.
  ///
  /// Politique de consigne perimee (05-integration-px4.md, section 4) :
  ///   age < seuil  -> publier la consigne
  ///   age >= seuil -> maintien de la position COURANTE + evenement STALE_SETPOINT
  ///
  /// C'est ce qui rend un plantage du planificateur non fatal : le drone se met
  /// en vol stationnaire au lieu de sortir du mode offboard de facon incontrolee.
  void publishOffboardStream()
  {
    if (!px4_seen_) {
      return;  // pas encore de liaison PX4 : rien a piloter
    }

    // NE PAS emettre tant que l'offboard n'est pas demande.
    //
    // /fmu/in/trajectory_setpoint n'est pas une entree « offboard » dediee :
    // c'est le bus uORB interne par lequel TOUS les modes de vol pilotent le
    // controleur de position. Y publier des le demarrage met notre flux en
    // concurrence avec le navigateur pendant AUTO_TAKEOFF et AUTO_LAND, et le
    // vehicule devient incontrolable — constate : PX4 commande la descente et
    // le drone monte jusqu'a 14 km.
    //
    // On n'emet donc qu'a partir du moment ou l'offboard est demande, ce qui
    // est aussi la procedure d'entree documentee par PX4 : le flux precede la
    // commande de bascule.
    if (!offboard_stream_enabled_) {
      return;
    }

    const double age_s = have_setpoint_ ? (now() - last_setpoint_time_).seconds() : 1e9;
    const bool stale = (age_s >= stale_hold_s_);

    if (stale && !stale_reported_) {
      stale_reported_ = true;
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::ERROR, "STALE_SETPOINT",
        "aucune consigne fraiche : maintien de position");
    } else if (!stale && stale_reported_) {
      stale_reported_ = false;
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::INFO, "SETPOINT_RESTORED",
        "flux de consignes retabli");
    }

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = px4TimestampUs();
    mode.position = true;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = mode.timestamp;
    sp.velocity = {NAN, NAN, NAN};
    sp.acceleration = {NAN, NAN, NAN};

    if (stale) {
      // Maintien de la position COURANTE, pas de la derniere consigne : une
      // consigne perimee peut etre tres eloignee de la position reelle, et y
      // foncer serait exactement le contraire d'un comportement sur.
      sp.position = {local_pos_.x, local_pos_.y, local_pos_.z};
      sp.yaw = local_pos_.heading;
    } else {
      const Eigen::Vector3d pos_ned = frames::enuToNed(Eigen::Vector3d(
        last_setpoint_.position.x, last_setpoint_.position.y, last_setpoint_.position.z));
      sp.position = {
        static_cast<float>(pos_ned.x()), static_cast<float>(pos_ned.y()),
        static_cast<float>(pos_ned.z())};
      sp.yaw = static_cast<float>(frames::yawEnuToNed(last_setpoint_.yaw));

      if (last_setpoint_.valid_mask & dronoto_msgs::msg::ControlSetpoint::USE_VELOCITY) {
        const Eigen::Vector3d vel_ned = frames::enuToNed(Eigen::Vector3d(
          last_setpoint_.velocity.x, last_setpoint_.velocity.y, last_setpoint_.velocity.z));
        sp.velocity = {
          static_cast<float>(vel_ned.x()), static_cast<float>(vel_ned.y()),
          static_cast<float>(vel_ned.z())};
      }
      // Sinon les NaN posés plus haut indiquent a PX4 que le champ n'est pas contraint.
    }

    trajectory_pub_->publish(sp);

    // PX4 exige un flux etabli avant d'accepter la bascule en offboard.
    // 20 cycles a 20 Hz = 1 s, largement au-dessus du minimum de 2 Hz.
    if (offboard_switch_pending_ && ++offboard_stream_cycles_ >= 20) {
      offboard_switch_pending_ = false;
      // param1 = 1 (mode personnalise), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
      sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      RCLCPP_INFO(get_logger(), "flux offboard etabli, bascule demandee");
    }
  }

  /// Coupe le flux offboard : utilise quand on rend la main a un mode PX4
  /// natif (atterrissage, RTL), pour ne plus concurrencer le navigateur.
  void disableOffboardStream()
  {
    offboard_stream_enabled_ = false;
    offboard_switch_pending_ = false;
    offboard_stream_cycles_ = 0;
  }

  /// Detecte le cas « on recoit l'odometrie mais pas le statut ».
  ///
  /// C'est la signature d'une desynchronisation de nom ou de version de topic.
  /// Sans ce controle, le systeme se bloque en PREFLIGHT sans qu'aucun journal
  /// n'indique pourquoi : le message reste simplement a sa valeur par defaut.
  void checkPx4Subscriptions()
  {
    if (!px4_seen_ || status_seen_ || subscription_fault_reported_) {
      return;
    }
    if ((now() - first_odometry_time_).seconds() < 10.0) {
      return;  // laisser le temps a la decouverte DDS
    }

    subscription_fault_reported_ = true;
    publishEvent(
      dronoto_msgs::msg::SafetyEvent::CRITICAL, "PX4_STATUS_MISSING",
      "odometrie recue mais aucun VehicleStatus depuis 10 s : verifier le nom "
      "du topic (versionnement PX4, ex. vehicle_status_v1) et l'alignement de "
      "px4_msgs sur simulation/px4/PX4_VERSION");
  }

  void sendCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = px4TimestampUs();
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
    RCLCPP_WARN(get_logger(), "[%s] %s", code.c_str(), message.c_str());
  }

  // --- etat PX4 le plus recent ---
  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::BatteryStatus battery_{};
  px4_msgs::msg::VehicleLocalPosition local_pos_{};
  bool px4_seen_{false};
  bool status_seen_{false};
  bool subscription_fault_reported_{false};
  rclcpp::Time first_odometry_time_{0, 0, RCL_ROS_TIME};

  std::string odom_frame_;
  std::string base_frame_;
  double stale_hold_s_{0.5};

  dronoto_msgs::msg::ControlSetpoint last_setpoint_{};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  bool have_setpoint_{false};
  bool stale_reported_{false};
  bool offboard_stream_enabled_{false};
  bool offboard_switch_pending_{false};
  int offboard_stream_cycles_{0};

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_legacy_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_sub_;

  rclcpp::Publisher<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;
  rclcpp::TimerBase::SharedPtr subscription_watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4InterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
