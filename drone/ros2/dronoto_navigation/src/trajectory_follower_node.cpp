#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/// Suivi de waypoints par interpolation bornee en vitesse.
///
/// Entrees : navigation/goal (objectif), state/telemetry (position courante)
/// Sortie  : control/setpoint_raw a 20 Hz
///
/// P1 : interpolation lineaire. P3 remplace l'interieur par une trajectoire
/// polynomiale contrainte par l'ESDF, sans changer ces topics.
class TrajectoryFollowerNode : public rclcpp::Node
{
public:
  TrajectoryFollowerNode()
  : Node("trajectory_follower")
  {
    declare_parameter<double>("max_speed_ms", 4.0);
    declare_parameter<double>("max_climb_ms", 2.0);
    declare_parameter<double>("goal_tolerance_m", 0.5);
    declare_parameter<double>("slowdown_radius_m", 3.0);
    max_speed_ = get_parameter("max_speed_ms").as_double();
    max_climb_ = get_parameter("max_climb_ms").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance_m").as_double();
    slowdown_radius_ = get_parameter("slowdown_radius_m").as_double();

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "navigation/goal", rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) {
        goal_ = *msg;
        have_goal_ = true;
        // Le point de depart de l'interpolation est la position au moment ou
        // l'objectif est recu : le drone glisse vers la cible depuis la.
        carrot_ = currentPosition();
        have_carrot_ = have_telemetry_;
        RCLCPP_INFO(
          get_logger(), "nouvel objectif : (%.1f, %.1f, %.1f)", goal_.pose.position.x,
          goal_.pose.position.y, goal_.pose.position.z);
      });

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        have_telemetry_ = true;
      });

    setpoint_pub_ = create_publisher<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_raw", rclcpp::QoS(1));
    distance_pub_ = create_publisher<std_msgs::msg::Float32>(
      "navigation/distance_to_goal", rclcpp::QoS(10));

    timer_ = create_wall_timer(50ms, [this] { tick(); });  // 20 Hz
    RCLCPP_INFO(get_logger(), "trajectory_follower demarre");
  }

private:
  Eigen::Vector3d currentPosition() const
  {
    return Eigen::Vector3d(
      telemetry_.pose.position.x, telemetry_.pose.position.y, telemetry_.pose.position.z);
  }

  void tick()
  {
    if (!have_goal_ || !have_telemetry_) {
      return;  // rien a suivre : px4_interface maintiendra la position
    }

    const Eigen::Vector3d position = currentPosition();
    const Eigen::Vector3d goal(
      goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);

    if (!have_carrot_) {
      carrot_ = position;
      have_carrot_ = true;
    }

    const double distance = (goal - position).norm();
    std_msgs::msg::Float32 d;
    d.data = static_cast<float>(distance);
    distance_pub_->publish(d);

    advanceCarrotToward(goal, distance);
    publishSetpoint(position, goal, distance);
  }

  /// Deplace un point intermediaire (la « carotte ») vers l'objectif, a vitesse
  /// bornee. On ne publie pas l'objectif directement : PX4 accelererait sans
  /// limite vers une cible lointaine.
  void advanceCarrotToward(const Eigen::Vector3d & goal, double distance)
  {
    constexpr double kDt = 0.05;

    const Eigen::Vector3d to_goal = goal - carrot_;
    const double remaining = to_goal.norm();
    if (remaining <= 1e-3) {
      return;
    }

    // Ralentissement a l'approche : evite le depassement et l'oscillation.
    const double speed_scale = std::clamp(distance / slowdown_radius_, 0.15, 1.0);
    const Eigen::Vector3d direction = to_goal / remaining;

    Eigen::Vector3d step = direction * max_speed_ * speed_scale * kDt;
    // Limite verticale distincte : monter et descendre coutent cher en energie,
    // et PX4 borne deja MPC_Z_VEL_MAX_*.
    const double max_dz = max_climb_ * kDt;
    if (std::abs(step.z()) > max_dz) {
      step.z() = std::copysign(max_dz, step.z());
    }

    if (step.norm() >= remaining) {
      carrot_ = goal;
    } else {
      carrot_ += step;
    }
  }

  void publishSetpoint(
    const Eigen::Vector3d & position, const Eigen::Vector3d & goal, double distance)
  {
    dronoto_msgs::msg::ControlSetpoint sp;
    sp.header.stamp = now();
    sp.header.frame_id = "odom";
    sp.valid_mask = dronoto_msgs::msg::ControlSetpoint::USE_POSITION |
                    dronoto_msgs::msg::ControlSetpoint::USE_YAW;
    sp.position.x = carrot_.x();
    sp.position.y = carrot_.y();
    sp.position.z = carrot_.z();

    // Cap : orienter vers l'objectif tant qu'on est loin, sinon conserver le cap
    // demande. Tourner sur place a l'arrivee est inutile et couteux.
    if (distance > 2.0) {
      const Eigen::Vector3d horizontal = goal - position;
      sp.yaw = static_cast<float>(std::atan2(horizontal.y(), horizontal.x()));
    } else {
      sp.yaw = yawFromQuaternion(goal_.pose.orientation);
    }
    sp.source = "trajectory_follower";
    setpoint_pub_->publish(sp);
  }

  /// Extraction du lacet uniquement (rotation autour de Z en ENU).
  static float yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return static_cast<float>(std::atan2(siny, cosy));
  }

  geometry_msgs::msg::PoseStamped goal_{};
  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  Eigen::Vector3d carrot_{Eigen::Vector3d::Zero()};
  bool have_goal_{false};
  bool have_telemetry_{false};
  bool have_carrot_{false};

  double max_speed_{4.0};
  double max_climb_{2.0};
  double goal_tolerance_{0.5};
  double slowdown_radius_{3.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
