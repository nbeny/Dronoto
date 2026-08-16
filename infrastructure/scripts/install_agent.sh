#!/usr/bin/env bash
# Construit et installe l'agent Micro XRCE-DDS, le pont entre PX4 et le
# graphe DDS de ROS 2 (docs/architecture/05-integration-px4.md, section 2).
#
# Version epinglee : l'agent et le client embarque dans PX4 doivent parler le
# meme protocole. v2.4.3 correspond a PX4 v1.16.
#
# Necessite les droits root (installation dans /usr/local).
set -euo pipefail

AGENT_VERSION="${AGENT_VERSION:-v2.4.3}"
SRC="${AGENT_SRC:-/opt/Micro-XRCE-DDS-Agent}"

if [ "$(id -u)" -ne 0 ]; then
  echo "ERREUR : ce script doit etre lance en root (sudo, ou wsl -u root)"
  exit 1
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "MicroXRCEAgent deja installe : $(command -v MicroXRCEAgent)"
  exit 0
fi

echo "=== dependances ==="
apt-get install -y -qq --no-install-recommends cmake build-essential git >/dev/null

if [ ! -d "$SRC/.git" ]; then
  echo "=== clonage ($AGENT_VERSION) ==="
  git clone -q -b "$AGENT_VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC"
fi

echo "=== compilation ==="
mkdir -p "$SRC/build"
cd "$SRC/build"
cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null
make -j"${AGENT_BUILD_JOBS:-4}" >/dev/null
make install >/dev/null
ldconfig /usr/local/lib/

echo "=== verification ==="
command -v MicroXRCEAgent
echo "Agent pret. Lancement en SITL : MicroXRCEAgent udp4 -p 8888"
