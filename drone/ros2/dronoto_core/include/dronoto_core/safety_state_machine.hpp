#ifndef DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_
#define DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>

namespace dronoto_core
{

/// Valeurs identiques a dronoto_msgs/SafetyState pour eviter toute traduction.
enum class SafetyState : uint8_t
{
  BOOT = 0,
  IDLE = 1,
  PREFLIGHT = 2,
  READY = 3,
  ARMED = 4,
  TAKEOFF = 5,
  NOMINAL = 6,
  DEGRADED = 7,
  HOLD = 8,
  RETURNING = 9,
  LANDING = 10,
  EMERGENCY_LAND = 11,
  TERMINATED = 12,
};

/// Instantane des conditions observees. Chaque champ est un fait mesure,
/// pas une decision : la decision est le role de la machine.
struct SystemConditions
{
  // --- conditions de securite, par ordre de priorite decroissante ---
  bool px4_failsafe_active = false;    ///< PX4 a declenche son propre failsafe
  bool battery_critical = false;       ///< sous le seuil critique
  bool state_estimate_lost = false;    ///< EKF diverge ou position invalide
  bool geofence_breached = false;      ///< hors de la zone autorisee
  bool battery_below_return = false;   ///< sous la reserve de retour calculee
  bool localization_poor = false;      ///< SLAM degenere (cable en P6)
  bool perception_lost = false;        ///< tous capteurs de perception perdus (P6)
  bool setpoint_stale = false;         ///< amont muet depuis trop longtemps
  bool link_lost_rth = false;          ///< liaison perdue + politique RTH (P6)
  bool localization_degraded = false;  ///< qualite reduite mais utilisable (P6)
  bool sensor_degraded = false;        ///< capteur non critique perdu (P6)

  // --- faits d'etat ---
  bool telemetry_fresh = false;  ///< on recoit des donnees du vehicule
  bool armed = false;
  bool preflight_ok = false;
  bool landed = true;
};

/// Resultat d'une evaluation.
struct SafetyDecision
{
  SafetyState state = SafetyState::BOOT;
  std::string reason;    ///< code stable, ex. "BATTERY_CRITICAL"
  bool allow_setpoints = false;
  bool changed = false;  ///< vrai si l'etat differe de l'evaluation precedente
};

/// Machine a etats de securite. Deterministe, sans dependance externe.
/// Voir docs/architecture/10-failsafe.md.
class SafetyStateMachine
{
public:
  explicit SafetyStateMachine(SafetyState initial = SafetyState::BOOT);

  /// Applique les gardes de securite sur l'etat courant.
  /// Les gardes ont priorite absolue sur toute progression nominale.
  SafetyDecision update(const SystemConditions & conditions);

  /// Demande une progression nominale (decollage, mission, atterrissage...).
  /// Refusee si la transition n'est pas legale ou si une garde l'interdit.
  bool requestTransition(SafetyState desired, const SystemConditions & conditions);

  SafetyState state() const { return state_; }
  const std::string & reason() const { return reason_; }

  /// Vrai si l'aeronef est en vol dans cet etat.
  static bool isInFlight(SafetyState s);

  /// Vrai si des consignes de trajectoire externes sont acceptees dans cet etat.
  static bool allowsSetpoints(SafetyState s);

  /// Vrai si la transition nominale from -> to est legale.
  static bool isNominalTransitionAllowed(SafetyState from, SafetyState to);

  static const char * toString(SafetyState s);

private:
  SafetyState state_;
  std::string reason_;
};

}  // namespace dronoto_core

#endif  // DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_
