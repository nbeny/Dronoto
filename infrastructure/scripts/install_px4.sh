#!/usr/bin/env bash
# Clone et construit PX4-Autopilot a la version verrouillee dans
# simulation/px4/PX4_VERSION.
#
# PX4 est cloné HORS du dépôt (cf. docs/adr/0008-px4-hors-depot.md) :
#   - 30 000+ fichiers, dont aucun ne nous appartient
#   - sur WSL, un clone sur /mnt/c est ~50x plus lent en lecture qu'en fs natif
#
# Emplacement : $DRONOTO_PX4_DIR, par défaut ~/px4/PX4-Autopilot
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${DRONOTO_PX4_DIR:-$HOME/px4/PX4-Autopilot}"
VERSION_FILE="$REPO_ROOT/simulation/px4/PX4_VERSION"

[ -f "$VERSION_FILE" ] || { echo "ERREUR : $VERSION_FILE introuvable"; exit 1; }
PX4_VERSION="$(grep -v '^#' "$VERSION_FILE" | grep -v '^[[:space:]]*$' | head -1 | tr -d '[:space:]')"

echo "PX4 version cible : $PX4_VERSION"
echo "Emplacement       : $PX4_DIR"

if [ ! -d "$PX4_DIR/.git" ]; then
  echo "=== clonage (long : ~1 Go) ==="
  mkdir -p "$(dirname "$PX4_DIR")"
  git clone --recursive --branch "$PX4_VERSION" --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
  echo "=== depot deja present, verification de la version ==="
  current="$(git -C "$PX4_DIR" describe --tags --always 2>/dev/null || echo inconnu)"
  if [ "$current" != "$PX4_VERSION" ]; then
    echo "  version actuelle : $current, attendue : $PX4_VERSION"
    git -C "$PX4_DIR" fetch --depth 1 origin "refs/tags/$PX4_VERSION:refs/tags/$PX4_VERSION"
    git -C "$PX4_DIR" checkout "$PX4_VERSION"
    git -C "$PX4_DIR" submodule update --init --recursive --depth 1
  else
    echo "  version correcte"
  fi
fi

echo "=== installation des dependances PX4 ==="
if [ ! -f "$PX4_DIR/.dronoto_deps_done" ]; then
  bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools
  touch "$PX4_DIR/.dronoto_deps_done"
else
  echo "  deja fait"
fi

echo "=== installation des surcouches Dronoto ==="
DRONOTO_PX4_DIR="$PX4_DIR" bash "$REPO_ROOT/infrastructure/scripts/install_px4_overlays.sh"

echo "=== compilation de PX4 SITL ==="
# Parallelisme borne : sur WSL la RAM est limitee et un -j32 declenche un OOM
# qui se manifeste par une erreur de compilateur incomprehensible.
JOBS="${PX4_BUILD_JOBS:-$(( $(nproc) < 8 ? $(nproc) : 8 ))}"
echo "  make -j$JOBS"
make -C "$PX4_DIR" px4_sitl -j"$JOBS"

echo
echo "PX4 pret : $PX4_DIR"
echo "Ajouter a votre shell :  export DRONOTO_PX4_DIR=$PX4_DIR"
