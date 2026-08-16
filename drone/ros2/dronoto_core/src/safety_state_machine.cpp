#include "dronoto_core/safety_state_machine.hpp"

namespace dronoto_core
{

SafetyStateMachine::SafetyStateMachine(SafetyState initial)
: state_(initial), reason_("INIT")
{
}

bool SafetyStateMachine::isInFlight(SafetyState s)
{
  switch (s) {
    case SafetyState::TAKEOFF:
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
    case SafetyState::RETURNING:
    case SafetyState::LANDING:
    case SafetyState::EMERGENCY_LAND:
      return true;
    default:
      return false;
  }
}

bool SafetyStateMachine::allowsSetpoints(SafetyState s)
{
  switch (s) {
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
    case SafetyState::RETURNING:
    case SafetyState::LANDING:
      return true;
    default:
      // BOOT, IDLE, PREFLIGHT, READY, ARMED : au sol, aucune consigne externe.
      // TAKEOFF, EMERGENCY_LAND : PX4 pilote en mode natif, on ne s'en mele pas.
      return false;
  }
}

bool SafetyStateMachine::isNominalTransitionAllowed(SafetyState from, SafetyState to)
{
  if (from == to) {
    return true;
  }
  switch (from) {
    case SafetyState::BOOT:
      return to == SafetyState::IDLE;
    case SafetyState::IDLE:
      return to == SafetyState::PREFLIGHT;
    case SafetyState::PREFLIGHT:
      return to == SafetyState::READY || to == SafetyState::IDLE;
    case SafetyState::READY:
      return to == SafetyState::ARMED || to == SafetyState::IDLE;
    case SafetyState::ARMED:
      return to == SafetyState::TAKEOFF || to == SafetyState::IDLE;
    case SafetyState::TAKEOFF:
      return to == SafetyState::NOMINAL || to == SafetyState::HOLD;
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
      return to == SafetyState::NOMINAL || to == SafetyState::DEGRADED ||
             to == SafetyState::HOLD || to == SafetyState::RETURNING ||
             to == SafetyState::LANDING;
    case SafetyState::RETURNING:
      return to == SafetyState::LANDING || to == SafetyState::HOLD;
    case SafetyState::LANDING:
      return to == SafetyState::TERMINATED || to == SafetyState::HOLD;
    case SafetyState::EMERGENCY_LAND:
      return to == SafetyState::TERMINATED;
    case SafetyState::TERMINATED:
      return to == SafetyState::IDLE;  // reinitialisation apres atterrissage
  }
  return false;
}

SafetyDecision SafetyStateMachine::update(const SystemConditions & c)
{
  const SafetyState previous = state_;
  const bool in_flight = isInFlight(state_);

  // Table de priorite de docs/architecture/10-failsafe.md, section 2.
  // La PREMIERE condition qui s'applique gagne. L'ordre EST la specification.
  if (c.px4_failsafe_active) {
    // On cede : PX4 a l'autorite, on cesse simplement d'emettre.
    reason_ = "PX4_FAILSAFE";
    SafetyDecision d;
    d.state = state_;
    d.reason = reason_;
    d.allow_setpoints = false;
    d.changed = false;
    return d;
  }

  if (in_flight && c.battery_critical) {
    state_ = SafetyState::EMERGENCY_LAND;
    reason_ = "BATTERY_CRITICAL";
  } else if (in_flight && c.state_estimate_lost) {
    state_ = SafetyState::EMERGENCY_LAND;
    reason_ = "STATE_ESTIMATE_LOST";
  } else if (in_flight && c.geofence_breached) {
    state_ = SafetyState::RETURNING;
    reason_ = "GEOFENCE_BREACH";
  } else if (in_flight && c.battery_below_return) {
    state_ = SafetyState::RETURNING;
    reason_ = "BATTERY_BELOW_RETURN_RESERVE";
  } else if (in_flight && c.localization_poor) {
    state_ = SafetyState::HOLD;
    reason_ = "LOCALIZATION_POOR";
  } else if (in_flight && c.perception_lost) {
    state_ = SafetyState::HOLD;
    reason_ = "PERCEPTION_LOST";
  } else if (in_flight && c.setpoint_stale) {
    state_ = SafetyState::HOLD;
    reason_ = "STALE_SETPOINT";
  } else if (in_flight && c.link_lost_rth) {
    state_ = SafetyState::RETURNING;
    reason_ = "LINK_LOST";
  } else if (in_flight && (c.localization_degraded || c.sensor_degraded)) {
    // Ne pas retrograder un etat plus engage : RETURNING et LANDING l'emportent.
    if (state_ == SafetyState::NOMINAL || state_ == SafetyState::HOLD) {
      state_ = SafetyState::DEGRADED;
      reason_ = c.localization_degraded ? "LOCALIZATION_DEGRADED" : "SENSOR_DEGRADED";
    }
  } else if (state_ == SafetyState::HOLD || state_ == SafetyState::DEGRADED) {
    // Plus aucune condition : retour au vol nominal.
    state_ = SafetyState::NOMINAL;
    reason_ = "RECOVERED";
  }

  SafetyDecision d;
  d.state = state_;
  d.reason = reason_;
  d.allow_setpoints = allowsSetpoints(state_);
  d.changed = (state_ != previous);
  return d;
}

bool SafetyStateMachine::requestTransition(SafetyState desired, const SystemConditions & c)
{
  // Une progression nominale ne peut jamais annuler un etat d'urgence.
  // Verifie AVANT la table de transitions : c'est une garde, pas une regle
  // de sequencement.
  if (state_ == SafetyState::EMERGENCY_LAND && desired != SafetyState::TERMINATED) {
    return false;
  }

  if (!isNominalTransitionAllowed(state_, desired)) {
    return false;
  }

  // Gardes specifiques a certaines progressions.
  if (desired == SafetyState::READY && !c.preflight_ok) {
    return false;
  }
  if (desired == SafetyState::ARMED && !c.armed) {
    return false;
  }
  if (desired != SafetyState::IDLE && !c.telemetry_fresh) {
    return false;
  }

  state_ = desired;
  reason_ = "MISSION_REQUEST";
  return true;
}

const char * SafetyStateMachine::toString(SafetyState s)
{
  switch (s) {
    case SafetyState::BOOT: return "BOOT";
    case SafetyState::IDLE: return "IDLE";
    case SafetyState::PREFLIGHT: return "PREFLIGHT";
    case SafetyState::READY: return "READY";
    case SafetyState::ARMED: return "ARMED";
    case SafetyState::TAKEOFF: return "TAKEOFF";
    case SafetyState::NOMINAL: return "NOMINAL";
    case SafetyState::DEGRADED: return "DEGRADED";
    case SafetyState::HOLD: return "HOLD";
    case SafetyState::RETURNING: return "RETURNING";
    case SafetyState::LANDING: return "LANDING";
    case SafetyState::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case SafetyState::TERMINATED: return "TERMINATED";
  }
  return "UNKNOWN";
}

}  // namespace dronoto_core
