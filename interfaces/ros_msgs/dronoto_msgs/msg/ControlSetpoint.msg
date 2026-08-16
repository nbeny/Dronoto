# Consigne de trajectoire dans le repere ENU/FLU (ROS).
# Convertie en NED/FRD par px4_interface, seul noeud a connaitre les conventions PX4.

std_msgs/Header header

# Bits indiquant quels champs sont significatifs.
# Sans ce masque, on ne peut pas distinguer "vitesse nulle demandee"
# de "vitesse non specifiee".
uint8 USE_POSITION     = 1
uint8 USE_VELOCITY     = 2
uint8 USE_ACCELERATION = 4
uint8 USE_YAW          = 8
uint8 USE_YAW_RATE     = 16
uint8 valid_mask

geometry_msgs/Point   position       # m, repere ENU
geometry_msgs/Vector3 velocity       # m/s, repere ENU
geometry_msgs/Vector3 acceleration   # m/s^2, repere ENU
float32 yaw                          # rad, ENU (0 = Est, sens trigonometrique)
float32 yaw_rate                     # rad/s

# Origine de la consigne, pour le diagnostic et la tracabilite des vetos
string source
