# Etat du superviseur de securite. Publie en RELIABLE / TRANSIENT_LOCAL :
# tout noeud qui demarre doit connaitre l'etat courant sans attendre.
#
# Les valeurs sont identiques a dronoto_core::SafetyState pour eviter
# toute traduction. La table complete est declaree des maintenant, meme si
# P1 n'atteint qu'une partie des etats : ajouter des valeurs plus tard
# casserait la compatibilite des enregistrements.

std_msgs/Header header

uint8 BOOT            = 0
uint8 IDLE            = 1
uint8 PREFLIGHT       = 2
uint8 READY           = 3
uint8 ARMED           = 4
uint8 TAKEOFF         = 5
uint8 NOMINAL         = 6
uint8 DEGRADED        = 7
uint8 HOLD            = 8
uint8 RETURNING       = 9
uint8 LANDING         = 10
uint8 EMERGENCY_LAND  = 11
uint8 TERMINATED      = 12
uint8 state

# Condition qui a provoque l'etat courant (code stable, cf. SafetyEvent)
string reason

# Vrai si le superviseur remplace actuellement la consigne amont
bool setpoint_vetoed

# Consignes autorisees dans l'etat courant
bool allow_setpoints
