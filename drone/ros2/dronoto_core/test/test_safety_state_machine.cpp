#include <gtest/gtest.h>

#include <vector>

#include "dronoto_core/safety_state_machine.hpp"

using dronoto_core::SafetyState;
using dronoto_core::SafetyStateMachine;
using dronoto_core::SystemConditions;

namespace
{

/// Conditions nominales en vol : rien d'anormal.
SystemConditions healthyInFlight()
{
  SystemConditions c;
  c.telemetry_fresh = true;
  c.armed = true;
  c.preflight_ok = true;
  c.landed = false;
  return c;
}

const std::vector<SafetyState> kAllStates = {
  SafetyState::BOOT,      SafetyState::IDLE,    SafetyState::PREFLIGHT,
  SafetyState::READY,     SafetyState::ARMED,   SafetyState::TAKEOFF,
  SafetyState::NOMINAL,   SafetyState::DEGRADED, SafetyState::HOLD,
  SafetyState::RETURNING, SafetyState::LANDING, SafetyState::EMERGENCY_LAND,
  SafetyState::TERMINATED};

}  // namespace

// ------------------------------------------------- atteignabilite de l'urgence

TEST(SafetyStateMachine, EmergencyLandIsReachableFromEveryInFlightState)
{
  // Exigence de 10-failsafe.md : une machine a etats de securite dont l'etat le
  // plus sur n'est pas immediatement atteignable n'en est pas une.
  for (const auto s : kAllStates) {
    if (!SafetyStateMachine::isInFlight(s)) {
      continue;
    }
    SafetyStateMachine fsm(s);
    SystemConditions c = healthyInFlight();
    c.battery_critical = true;
    const auto d = fsm.update(c);
    EXPECT_EQ(d.state, SafetyState::EMERGENCY_LAND)
      << "depuis l'etat " << SafetyStateMachine::toString(s);
  }
}

// ----------------------------------------------------------- ordre de priorite

TEST(SafetyStateMachine, BatteryCriticalOutranksEverythingExceptPx4Failsafe)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.battery_critical = true;
  c.battery_below_return = true;
  c.setpoint_stale = true;
  c.geofence_breached = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::EMERGENCY_LAND);
}

TEST(SafetyStateMachine, Px4FailsafeSuppressesOurSetpoints)
{
  // On ne lutte pas contre PX4 : deux autorites concurrentes produisent
  // un comportement pire que chacune isolement.
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.px4_failsafe_active = true;
  const auto d = fsm.update(c);
  EXPECT_FALSE(d.allow_setpoints);
  EXPECT_EQ(d.reason, "PX4_FAILSAFE");
}

TEST(SafetyStateMachine, StateEstimateLostTriggersEmergencyLand)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.state_estimate_lost = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::EMERGENCY_LAND);
}

TEST(SafetyStateMachine, GeofenceBreachTriggersReturn)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.geofence_breached = true;
  const auto d = fsm.update(c);
  EXPECT_EQ(d.state, SafetyState::RETURNING);
  EXPECT_EQ(d.reason, "GEOFENCE_BREACH");
}

TEST(SafetyStateMachine, BatteryBelowReturnReserveTriggersReturn)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.battery_below_return = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::RETURNING);
}

TEST(SafetyStateMachine, StaleSetpointTriggersHold)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.setpoint_stale = true;
  const auto d = fsm.update(c);
  EXPECT_EQ(d.state, SafetyState::HOLD);
  EXPECT_EQ(d.reason, "STALE_SETPOINT");
}

TEST(SafetyStateMachine, DegradedSensorProducesDegradedNotHold)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.sensor_degraded = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::DEGRADED);
}

TEST(SafetyStateMachine, RecoveryFromHoldReturnsToNominal)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.setpoint_stale = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::HOLD);

  c.setpoint_stale = false;
  EXPECT_EQ(fsm.update(c).state, SafetyState::NOMINAL);
}

TEST(SafetyStateMachine, ReturningIsNotDowngradedByALesserCondition)
{
  // Une fois le retour engage, une condition moins grave ne doit pas l'annuler.
  SafetyStateMachine fsm(SafetyState::RETURNING);
  SystemConditions c = healthyInFlight();
  c.sensor_degraded = true;
  EXPECT_EQ(fsm.update(c).state, SafetyState::RETURNING);
}

// --------------------------------------------------- progression nominale

TEST(SafetyStateMachine, NominalSequenceIsAccepted)
{
  SafetyStateMachine fsm(SafetyState::BOOT);
  SystemConditions c;
  c.telemetry_fresh = true;

  EXPECT_TRUE(fsm.requestTransition(SafetyState::IDLE, c));
  EXPECT_TRUE(fsm.requestTransition(SafetyState::PREFLIGHT, c));

  c.preflight_ok = true;
  EXPECT_TRUE(fsm.requestTransition(SafetyState::READY, c));

  c.armed = true;
  EXPECT_TRUE(fsm.requestTransition(SafetyState::ARMED, c));
  EXPECT_TRUE(fsm.requestTransition(SafetyState::TAKEOFF, c));

  c.landed = false;
  EXPECT_TRUE(fsm.requestTransition(SafetyState::NOMINAL, c));
  EXPECT_EQ(fsm.state(), SafetyState::NOMINAL);
}

TEST(SafetyStateMachine, IllegalTransitionIsRejected)
{
  SafetyStateMachine fsm(SafetyState::IDLE);
  SystemConditions c;
  c.telemetry_fresh = true;
  // On ne saute pas de IDLE directement a NOMINAL.
  EXPECT_FALSE(fsm.requestTransition(SafetyState::NOMINAL, c));
  EXPECT_EQ(fsm.state(), SafetyState::IDLE);
}

TEST(SafetyStateMachine, ArmingIsRefusedWhenPreflightFailed)
{
  SafetyStateMachine fsm(SafetyState::PREFLIGHT);
  SystemConditions c;
  c.telemetry_fresh = true;
  c.preflight_ok = false;
  EXPECT_FALSE(fsm.requestTransition(SafetyState::READY, c));
}

TEST(SafetyStateMachine, SafetyGuardOverridesRequestedTransition)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.battery_critical = true;
  fsm.update(c);  // la garde force EMERGENCY_LAND
  // La mission demande de continuer : refuse.
  EXPECT_FALSE(fsm.requestTransition(SafetyState::NOMINAL, c));
  EXPECT_EQ(fsm.state(), SafetyState::EMERGENCY_LAND);
}

// ------------------------------------------------------------- proprietes

TEST(SafetyStateMachine, SetpointsAreForbiddenOnGroundAndDuringEmergency)
{
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::BOOT));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::IDLE));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::READY));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::ARMED));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::TAKEOFF));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::EMERGENCY_LAND));
  EXPECT_FALSE(SafetyStateMachine::allowsSetpoints(SafetyState::TERMINATED));

  EXPECT_TRUE(SafetyStateMachine::allowsSetpoints(SafetyState::NOMINAL));
  EXPECT_TRUE(SafetyStateMachine::allowsSetpoints(SafetyState::DEGRADED));
  EXPECT_TRUE(SafetyStateMachine::allowsSetpoints(SafetyState::HOLD));
  EXPECT_TRUE(SafetyStateMachine::allowsSetpoints(SafetyState::RETURNING));
}

TEST(SafetyStateMachine, EveryStateHasANonEmptyName)
{
  for (const auto s : kAllStates) {
    EXPECT_STRNE(SafetyStateMachine::toString(s), "");
    EXPECT_STRNE(SafetyStateMachine::toString(s), "UNKNOWN");
  }
}

TEST(SafetyStateMachine, ChangedFlagIsSetOnlyOnActualChange)
{
  SafetyStateMachine fsm(SafetyState::NOMINAL);
  SystemConditions c = healthyInFlight();
  c.setpoint_stale = true;
  EXPECT_TRUE(fsm.update(c).changed);
  EXPECT_FALSE(fsm.update(c).changed);
}
