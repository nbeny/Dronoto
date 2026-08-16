#!/usr/bin/env bash
# Lance l'agent uXRCE-DDS puis PX4 SITL avec Gazebo.
#
# HEADLESS=1 pour les tests, HEADLESS=0 pour voir la simulation.
set -euo pipefail

PX4="${DRONOTO_PX4_DIR:-$HOME/px4/PX4-Autopilot}"

[ -x "$PX4/build/px4_sitl_default/bin/px4" ] || {
  echo "ERREUR : PX4 SITL non construit dans $PX4"
  echo "         Lancer infrastructure/scripts/install_px4.sh"
  exit 1
}

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4501}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-x500_dronoto}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1}"
export HEADLESS="${HEADLESS:-0}"

AGENT_PORT="${PX4_UXRCE_DDS_PORT:-8888}"
AGENT_PID=""

# PX4 SITL persiste ses parametres dans rootfs/parameters.bson entre deux
# executions, et « param set-default » n'ecrase JAMAIS une valeur enregistree.
# Un run precedent peut donc contaminer le suivant — exactement le genre de
# chose qui fait passer un test neuf fois sur dix.
# PX4_KEEP_PARAMS=1 pour conserver l'etat (mise au point manuelle).
if [ "${PX4_KEEP_PARAMS:-0}" != "1" ]; then
  rm -f "$PX4/build/px4_sitl_default/rootfs/parameters"*.bson
fi

cleanup() {
  echo "Arret de la simulation..."
  [ -n "$AGENT_PID" ] && kill "$AGENT_PID" 2>/dev/null || true
  pkill -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -f 'gz sim' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Demarrage de l'agent Micro XRCE-DDS (UDP $AGENT_PORT)..."
MicroXRCEAgent udp4 -p "$AGENT_PORT" &
AGENT_PID=$!
sleep 2

echo "Demarrage de PX4 SITL (airframe $PX4_SYS_AUTOSTART, modele $PX4_SIM_MODEL,"
echo "                       monde $PX4_GZ_WORLD, headless=$HEADLESS)..."
cd "$PX4"

# PX4_INTERACTIVE=1 pour obtenir la console pxh> (developpement manuel).
# Par defaut on lance en mode demon (-d) : sans terminal attache, la console
# interactive part en boucle serree a reimprimer son prompt, ce qui noie les
# journaux sous des megaoctets de « pxh> » et brule du CPU pour rien.
if [ "${PX4_INTERACTIVE:-0}" = "1" ]; then
  ./build/px4_sitl_default/bin/px4
else
  ./build/px4_sitl_default/bin/px4 -d
fi
