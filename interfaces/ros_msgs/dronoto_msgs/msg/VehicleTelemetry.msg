# Etat normalise du vehicule, produit par px4_interface.
#
# Tout le reste du systeme lit ceci et jamais px4_msgs directement : c'est ce
# qui concentre la connaissance des conventions PX4 en un seul endroit
# (cf. docs/architecture/05-integration-px4.md).

std_msgs/Header header

geometry_msgs/Pose    pose            # repere ENU / FLU
geometry_msgs/Twist   twist           # repere ENU / FLU
float32 altitude_relative_m           # au-dessus du point de decollage

float32 battery_remaining             # [0, 1]
float32 battery_voltage_v
float32 battery_current_a

bool armed
uint8 nav_state                       # recopie de px4_msgs VehicleStatus.nav_state
bool offboard_active
bool preflight_ok

bool gnss_ok
uint8 gnss_satellites
float32 gnss_eph_m

bool ekf_position_valid
bool ekf_velocity_valid
