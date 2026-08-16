#!/usr/bin/env bash
# Cree le workspace colcon et y lie les paquets du depot.
#
# Le workspace vit HORS du depot, sur le systeme de fichiers natif :
#   - build/ et install/ ne polluent pas git
#   - sous WSL, les milliers de fichiers intermediaires ne passent pas par 9p
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${DRONOTO_WS:-$HOME/dronoto_ws}"

mkdir -p "$WS/src"

link() {
  local target="$1" name="$2"
  rm -rf "${WS:?}/src/$name"
  ln -s "$target" "$WS/src/$name"
  echo "  lie $name"
}

echo "Depot     : $REPO_ROOT"
echo "Workspace : $WS"
echo

link "$REPO_ROOT/interfaces/ros_msgs/dronoto_msgs" dronoto_msgs
for pkg in dronoto_core dronoto_interface dronoto_safety \
           dronoto_navigation dronoto_mission dronoto_bringup; do
  if [ -d "$REPO_ROOT/drone/ros2/$pkg" ]; then
    link "$REPO_ROOT/drone/ros2/$pkg" "$pkg"
  fi
done

# px4_msgs DOIT correspondre a la version de PX4 (docs/architecture/05-integration-px4.md).
# La branche est derivee de simulation/px4/PX4_VERSION pour rester alignee.
PX4_VERSION="$(grep -v '^#' "$REPO_ROOT/simulation/px4/PX4_VERSION" \
  | grep -v '^[[:space:]]*$' | head -1 | tr -d '[:space:]')"
PX4_BRANCH="release/${PX4_VERSION#v}"
PX4_BRANCH="${PX4_BRANCH%.*}"   # v1.16.0 -> release/1.16

if [ ! -d "$WS/src/px4_msgs" ]; then
  echo "  clonage px4_msgs ($PX4_BRANCH)"
  git clone --depth 1 -b "$PX4_BRANCH" https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
else
  echo "  px4_msgs deja present"
fi

echo
echo "Termine. Construire avec :"
echo "  cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
