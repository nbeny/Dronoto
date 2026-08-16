# Evenement de securite horodate.
#
# Code machine ET message humain : le code permet le filtrage, l'alerte et
# l'analyse statistique ; le message est pour l'operateur. Se contenter d'un
# texte libre rendrait l'exploitation impossible.

std_msgs/Header header

uint8 DEBUG    = 0
uint8 INFO     = 1
uint8 WARNING  = 2
uint8 ERROR    = 3
uint8 CRITICAL = 4
uint8 severity

string code       # ex. "STALE_SETPOINT", "PREFLIGHT_FAILED", "TAKEOFF_COMPLETE"
string message    # texte lisible
string detail     # contexte structure, JSON compact
