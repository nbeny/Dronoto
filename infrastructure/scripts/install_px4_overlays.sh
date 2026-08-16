#!/usr/bin/env bash
# Lie les modeles et airframes Dronoto dans l'arborescence PX4.
#
# Les fichiers vivent dans NOTRE depot et sont lies dans PX4 : la
# personnalisation reste versionnee chez nous et PX4 reste vierge.
# C'est ce qui rend le sous-module inutile (docs/adr/0008-px4-hors-depot.md).
#
# Idempotent : relancable sans effet de bord.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="${DRONOTO_PX4_DIR:-$HOME/px4/PX4-Autopilot}"
MODELS="$PX4/Tools/simulation/gz/models"
AIRFRAMES="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"

[ -d "$PX4" ] || {
  echo "ERREUR : PX4 introuvable dans $PX4"
  echo "         Lancer d'abord infrastructure/scripts/install_px4.sh"
  exit 1
}

rm -rf "$MODELS/x500_dronoto"
ln -s "$REPO_ROOT/simulation/px4/models/x500_dronoto" "$MODELS/x500_dronoto"
echo "  modele x500_dronoto lie"

rm -f "$AIRFRAMES/4501_gz_x500_dronoto"
ln -s "$REPO_ROOT/simulation/px4/airframes/4501_gz_x500_dronoto" \
      "$AIRFRAMES/4501_gz_x500_dronoto"
echo "  airframe 4501_gz_x500_dronoto lie"

CMAKE="$AIRFRAMES/CMakeLists.txt"
if grep -q "4501_gz_x500_dronoto" "$CMAKE"; then
  echo "  airframe deja enregistre dans CMakeLists.txt"
else
  # Insertion automatique : on ajoute la ligne juste avant la parenthese
  # fermante de px4_add_romfs_files(...). Une sauvegarde est conservee.
  cp "$CMAKE" "$CMAKE.dronoto.bak"
  python3 - "$CMAKE" <<'PY'
import re, sys
path = sys.argv[1]
src = open(path, encoding="utf-8").read()
m = re.search(r"px4_add_romfs_files\s*\((.*?)^\s*\)", src, re.S | re.M)
if not m:
    sys.exit("px4_add_romfs_files( ... ) introuvable : ajouter la ligne a la main")
block = m.group(1)
if "4501_gz_x500_dronoto" in block:
    sys.exit(0)
new_block = block.rstrip() + "\n\t4501_gz_x500_dronoto\n"
open(path, "w", encoding="utf-8").write(src[:m.start(1)] + new_block + src[m.end(1):])
print("  airframe enregistre dans CMakeLists.txt")
PY
fi

echo "Surcouches installees. Reconstruire PX4 : make -C $PX4 px4_sitl"
