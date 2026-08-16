# Plan d'implémentation — P0 Fondations + P1 Socle de vol

> **Pour les agents d'exécution :** SOUS-SKILL REQUISE — utiliser
> `superpowers:subagent-driven-development` (recommandé) ou
> `superpowers:executing-plans` pour exécuter ce plan tâche par tâche.
> Les étapes utilisent la syntaxe case à cocher (`- [ ]`).

**Objectif :** un quadricoptère qui décolle, rejoint des waypoints et atterrit de manière
autonome dans Gazebo, avec un harnais de test automatisé qui le prouve, sur un dépôt
reproductible avec CI verte.

**Architecture :** quatre nœuds ROS 2 en C++ forment la chaîne de vol —
`px4_interface` (frontière unique avec PX4), `trajectory_follower` (waypoints → consignes),
`safety_supervisor` (machine à états + veto), `mission_executive` (séquenceur de mission).
La logique pure (conversions de repères, machine à états) vit dans des bibliothèques C++
sans dépendance ROS, testées par gtest en millisecondes. Les scénarios de bout en bout sont
décrits en YAML et exécutés par un moteur Python au-dessus de pytest.

**Stack :** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16, uXRCE-DDS,
`px4_msgs`, C++17 / ament_cmake, Python 3.12 / pytest, Docker Compose, GitHub Actions.

**Dossier d'architecture :** [`docs/README.md`](../../README.md). Ce plan implémente les
phases P0 et P1 de [`17-roadmap.md`](../../architecture/17-roadmap.md).

---

## Écarts assumés par rapport au dossier d'architecture

Trois simplifications délibérées pour raccourcir le chemin jusqu'au premier vol. Chacune
est un report, pas un abandon. La tâche 2 met le dossier à jour en conséquence.

| Dossier | Ce plan | Raison |
|---|---|---|
| P1 : `mission_executive` en BehaviorTree.CPP | Séquenceur C++ à états (~120 lignes) | Introduire BT.CPP sur le chemin critique du premier vol ajoute une dépendance et un apprentissage pour zéro bénéfice : en P1 la mission est strictement séquentielle. BT.CPP arrive en **P4**, quand la boucle d'exploration a besoin de réévaluation réactive — c'est là qu'il paie. |
| P1 : monde `empty_field` maison | Monde `default` de PX4 | Le monde `default` de PX4 **est** un plan de sol nu. Faire trouver un monde maison par le lanceur PX4 demande de câbler `GZ_SIM_RESOURCE_PATH`, source de friction connue. Les mondes maison arrivent en **P3** avec `pillars`, où ils sont indispensables. |
| P1 : modèle `x500_lidar_dronoto` avec capteurs déclarés | Modèle `x500_dronoto` **sans** LiDAR ni caméra | La plomberie (airframe PX4, répertoire de modèle, `PX4_SIM_MODEL`) est ce qui a de la valeur maintenant ; ajouter un bloc `<sensor>` au SDF en **P2** est trivial. Rendre un LiDAR GPU inutilisé coûte des cycles à chaque exécution de test. |
| P0 : génération de code Protobuf | Absente | Protobuf ne sert qu'au protocole radio, qui n'existe qu'en **P5**. Mettre en place un générateur pour zéro consommateur est du travail sans objet. Les contrats de P1 sont des messages ROS 2 (`dronoto_msgs`), et eux sont bien en place dès la tâche 3. |
| P0 : `docker compose --profile sim up` lance Gazebo | Profils `drone` et `test` seulement | Faire tourner Gazebo en conteneur demande de câbler `/dev/dri`, l'affichage X11 et le multicast DDS — trois sources de friction pour un bénéfice nul tant que le développement est natif. La CI conteneurisée couvre ce qui compte (build et tests unitaires). Le profil `sim` arrive en **P6**, quand les campagnes L4 ont besoin d'exécutions isolées. |

---

## Prérequis

**Machine :** Ryzen 9 5950X, 32 Go, RX 6900 XT, ≥ 80 Go libres.

**Système :** Ubuntu 24.04 LTS **natif** (dual-boot). Voir
[ADR-0002](../../adr/0002-plateforme-de-developpement.md). WSL2 fonctionne en dégradé —
la tâche 1 signale les écarts.

**Connaissances supposées :** développeur compétent, mais aucune connaissance de ROS 2, PX4
ou Gazebo. Chaque commande est donnée en entier avec sa sortie attendue.

---

## Structure des fichiers

Ce que chaque fichier fait, décidé avant les tâches.

```
interfaces/ros_msgs/dronoto_msgs/       Contrats ROS 2 — source unique
  msg/ControlSetpoint.msg               consigne de la chaîne de contrôle
  msg/SafetyState.msg                   état du superviseur
  msg/SafetyEvent.msg                   événement horodaté
  msg/VehicleTelemetry.msg              état normalisé du véhicule
  srv/TriggerFailsafe.srv               forcer un état de sécurité

drone/ros2/dronoto_core/                Bibliothèques C++ PURES — zéro dépendance ROS
  include/dronoto_core/frame_conversions.hpp    NED↔ENU, FRD↔FLU, yaw
  include/dronoto_core/safety_state_machine.hpp machine à états, table de transitions
  src/frame_conversions.cpp
  src/safety_state_machine.cpp
  test/test_frame_conversions.cpp        gtest — exhaustif
  test/test_safety_state_machine.cpp     gtest — exhaustif

drone/ros2/dronoto_interface/           Frontière unique avec PX4
  src/px4_interface_node.cpp

drone/ros2/dronoto_safety/              Superviseur — autorité de veto
  src/safety_supervisor_node.cpp

drone/ros2/dronoto_navigation/          Suivi de waypoints
  src/trajectory_follower_node.cpp

drone/ros2/dronoto_mission/             Séquenceur de mission
  src/mission_executive_node.cpp

drone/ros2/dronoto_bringup/             Lancement et configuration
  launch/bringup_drone.launch.py         ◀── migrera tel quel sur le Jetson
  launch/bringup_sim.launch.py
  config/drone_params.yaml
  config/missions/square.yaml
  rviz/dronoto.rviz
  urdf/x500_dronoto.urdf.xacro

simulation/px4/                          PX4-Autopilot en sous-module épinglé
  airframes/4501_gz_x500_dronoto         airframe PX4 personnalisé
  models/x500_dronoto/model.sdf          modèle Gazebo personnalisé

tests/
  runner/scenario_runner.py              moteur du DSL YAML
  runner/sim_process.py                  démarrage/arrêt de la pile simulée
  scenarios/takeoff_land.yaml
  scenarios/waypoint_navigation.yaml
  conformance/test_bringup_no_sim_deps.py
  test_scenarios.py                      intégration pytest

infrastructure/docker/
  base/Dockerfile  simulation/Dockerfile
infrastructure/compose/docker-compose.yml
.github/workflows/ci.yml
```

**Pourquoi `dronoto_core` existe.** Les conversions de repères et la machine à états sont
la logique la plus critique et la plus testable du système. Les isoler dans une
bibliothèque sans ROS permet de les tester exhaustivement en millisecondes, sans démarrer
de nœud, sans DDS, sans simulateur. C'est ce qui rend possible la couverture > 90 % exigée
par [`14-tests.md`](../../architecture/14-tests.md).

---

# PHASE P0 — FONDATIONS

---

### Tâche 1 : Environnement de développement

Tâche manuelle, mais chaque étape a une vérification qui échoue visiblement si elle est
ratée. Compter 2 à 3 heures, dont beaucoup d'attente.

**Fichiers :**
- Créer : `docs/guides/installation.md`

- [ ] **Étape 1 : Installer Ubuntu 24.04 LTS en dual-boot**

Télécharger l'ISO depuis <https://releases.ubuntu.com/24.04/>, créer une clé USB
bootable, installer sur le disque `D:` ou `E:` (7 To libres constatés). Allouer au moins
80 Go.

Vérifier :

```bash
lsb_release -a
```

Attendu : `Description: Ubuntu 24.04.x LTS`

> **Repli WSL2** : `wsl --install -d Ubuntu-24.04` depuis PowerShell. Le reste du plan
> fonctionne, avec deux écarts : le rendu Gazebo est plus lent (traduction D3D12), et
> `ROCMExecutionProvider` sera indisponible en P7. Poursuivre sans autre changement.

- [ ] **Étape 2 : Installer ROS 2 Jazzy**

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

Vérifier :

```bash
ros2 --version && printenv ROS_DISTRO
```

Attendu : une version `ros2cli`, puis `jazzy`.

- [ ] **Étape 3 : Installer Gazebo Harmonic et le pont ROS**

```bash
sudo apt install -y ros-jazzy-ros-gz gz-harmonic
```

Vérifier :

```bash
gz sim --versions
```

Attendu : `8.x.x` (Harmonic est gz-sim 8).

- [ ] **Étape 4 : Installer les dépendances de build**

```bash
sudo apt install -y \
  build-essential cmake git python3-pip python3-venv \
  libeigen3-dev libgtest-dev \
  ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2
pip install --user --break-system-packages -U empy==3.3.4 pyros-genmsg setuptools pyyaml jinja2
```

Vérifier :

```bash
cmake --version && python3 -c "import em, yaml, jinja2; print('deps ok')"
```

Attendu : une version CMake ≥ 3.22, puis `deps ok`.

- [ ] **Étape 5 : Construire l'agent Micro XRCE-DDS**

```bash
cd ~
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig /usr/local/lib/
```

Vérifier :

```bash
which MicroXRCEAgent
```

Attendu : `/usr/local/bin/MicroXRCEAgent`

- [ ] **Étape 6 : Écrire le guide d'installation**

Créer `docs/guides/installation.md` reprenant les étapes 1 à 5 de cette tâche, avec la
section de repli WSL2. Ce fichier est la référence pour toute réinstallation.

- [ ] **Étape 7 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add docs/guides/installation.md
git commit -m "docs: guide d'installation de l'environnement de développement"
```

---

### Tâche 2 : Squelette du dépôt et workspace colcon

**Fichiers :**
- Créer : `.gitignore`, `.gitmodules` (via git), `.pre-commit-config.yaml`
- Créer : `infrastructure/scripts/setup_workspace.sh`
- Modifier : `docs/architecture/17-roadmap.md` (aligner P1 sur les écarts assumés)

- [ ] **Étape 1 : Créer le `.gitignore`**

```gitignore
# ROS 2 / colcon
build/
install/
log/

# Python
__pycache__/
*.py[cod]
.venv/
.pytest_cache/
.mypy_cache/

# C++
*.o
*.so
compile_commands.json

# Artefacts de test
tests/results/
*.bag
*.mcap

# Modèles IA (gérés hors git, cf. 15-structure-depot.md)
models/**/*.onnx

# Éditeurs / OS
.vscode/
.idea/
.DS_Store
```

- [ ] **Étape 2 : Ajouter PX4-Autopilot en sous-module épinglé**

Le verrouillage de version est une exigence de
[`05-integration-px4.md`](../../architecture/05-integration-px4.md) : une dérive de version
d'autopilote produit des messages silencieusement corrompus.

```bash
cd ~/Documents/GitHub/Dronoto
git submodule add https://github.com/PX4/PX4-Autopilot.git simulation/px4/PX4-Autopilot
cd simulation/px4/PX4-Autopilot
git checkout v1.16.0
git submodule update --init --recursive
cd ~/Documents/GitHub/Dronoto
```

Vérifier :

```bash
cd simulation/px4/PX4-Autopilot && git describe --tags && cd -
```

Attendu : `v1.16.0`

- [ ] **Étape 3 : Installer les dépendances PX4 et construire SITL**

Étape longue (15 à 30 minutes). Elle valide toute la chaîne de compilation.

```bash
bash simulation/px4/PX4-Autopilot/Tools/setup/ubuntu.sh
cd simulation/px4/PX4-Autopilot
make px4_sitl
cd ~/Documents/GitHub/Dronoto
```

Vérifier :

```bash
ls simulation/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

Attendu : le chemin s'affiche (le binaire existe).

- [ ] **Étape 4 : Créer le script de workspace**

`infrastructure/scripts/setup_workspace.sh` :

```bash
#!/usr/bin/env bash
# Crée le workspace colcon et y lie les paquets du dépôt.
# Le workspace vit HORS du dépôt : build/ et install/ ne polluent pas git.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${DRONOTO_WS:-$HOME/dronoto_ws}"

mkdir -p "$WS/src"

link() {
  local target="$1" name="$2"
  rm -rf "${WS:?}/src/$name"
  ln -s "$target" "$WS/src/$name"
  echo "  lié $name"
}

echo "Workspace : $WS"
link "$REPO_ROOT/interfaces/ros_msgs/dronoto_msgs" dronoto_msgs
for pkg in dronoto_core dronoto_interface dronoto_safety \
           dronoto_navigation dronoto_mission dronoto_bringup; do
  if [ -d "$REPO_ROOT/drone/ros2/$pkg" ]; then
    link "$REPO_ROOT/drone/ros2/$pkg" "$pkg"
  fi
done

# px4_msgs DOIT correspondre à la version de PX4 (cf. 05-integration-px4.md)
if [ ! -d "$WS/src/px4_msgs" ]; then
  git clone -b release/1.16 https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
  echo "  cloné px4_msgs (release/1.16)"
fi

echo
echo "Terminé. Construire avec :"
echo "  cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
```

```bash
chmod +x infrastructure/scripts/setup_workspace.sh
```

- [ ] **Étape 5 : Vérifier que le script fonctionne**

```bash
./infrastructure/scripts/setup_workspace.sh
ls -l ~/dronoto_ws/src/
```

Attendu : `dronoto_msgs` n'existe pas encore (créé en tâche 3), mais `px4_msgs` est cloné
et le script se termine sans erreur.

- [ ] **Étape 6 : Configurer pre-commit**

`.pre-commit-config.yaml` :

```yaml
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v5.0.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-yaml
        args: [--allow-multiple-documents]
      - id: check-added-large-files
        args: [--maxkb=2048]
      - id: mixed-line-ending
        args: [--fix=lf]

  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.8.4
    hooks:
      - id: ruff
        args: [--fix]
      - id: ruff-format

  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: v19.1.5
    hooks:
      - id: clang-format
        types_or: [c++, c]
```

`.clang-format` (style ROS 2) :

```yaml
BasedOnStyle: Google
ColumnLimit: 100
IndentWidth: 2
AccessModifierOffset: -2
AlwaysBreakTemplateDeclarations: Yes
DerivePointerAlignment: false
PointerAlignment: Middle
```

`pyproject.toml` à la racine :

```toml
[tool.ruff]
line-length = 100
target-version = "py312"
exclude = ["simulation/px4/PX4-Autopilot"]

[tool.ruff.lint]
select = ["E", "F", "W", "I", "UP", "B", "SIM"]

[tool.pytest.ini_options]
testpaths = ["tests"]
markers = [
    "scenario: test de scénario SITL (lent, niveau L3)",
]
```

```bash
pip install --user --break-system-packages pre-commit
pre-commit install
```

Vérifier :

```bash
pre-commit run --all-files
```

Attendu : tous les crochets passent (des fichiers peuvent être reformatés au premier
passage — relancer jusqu'à obtenir `Passed` partout).

- [ ] **Étape 7 : Aligner le dossier d'architecture sur les écarts assumés**

Dans `docs/architecture/17-roadmap.md`, section **P1**, remplacer la ligne :

```
- `mission_executive` minimal : arbre BT décollage → waypoint → atterrissage
```

par :

```
- `mission_executive` minimal : séquenceur à états décollage → waypoint → atterrissage
  (BehaviorTree.CPP introduit en P4, quand la réévaluation réactive devient nécessaire)
```

et la ligne :

```
- Modèle `x500_lidar_dronoto` en SDF (capteurs déclarés, même si non exploités)
- Monde `empty_field`
```

par :

```
- Modèle `x500_dronoto` en SDF (capteurs de perception ajoutés en P2)
- Monde `default` de PX4 (plan de sol nu) ; mondes maison introduits en P3
```

- [ ] **Étape 8 : Commit**

```bash
git add .gitignore .gitmodules .pre-commit-config.yaml .clang-format pyproject.toml \
        simulation/px4/PX4-Autopilot infrastructure/scripts/setup_workspace.sh \
        docs/architecture/17-roadmap.md
git commit -m "build: squelette du dépôt, workspace colcon, PX4 v1.16 épinglé, pre-commit"
```

---

### Tâche 3 : Paquet d'interfaces `dronoto_msgs`

Les contrats entre nœuds. Ils sont définis avant tout code parce que tout le reste en
dépend.

**Fichiers :**
- Créer : `interfaces/ros_msgs/dronoto_msgs/package.xml`
- Créer : `interfaces/ros_msgs/dronoto_msgs/CMakeLists.txt`
- Créer : `interfaces/ros_msgs/dronoto_msgs/msg/ControlSetpoint.msg`
- Créer : `interfaces/ros_msgs/dronoto_msgs/msg/SafetyState.msg`
- Créer : `interfaces/ros_msgs/dronoto_msgs/msg/SafetyEvent.msg`
- Créer : `interfaces/ros_msgs/dronoto_msgs/msg/VehicleTelemetry.msg`
- Créer : `interfaces/ros_msgs/dronoto_msgs/srv/TriggerFailsafe.srv`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>dronoto_msgs</name>
  <version>0.1.0</version>
  <description>Interfaces ROS 2 du projet Dronoto : messages, services et actions.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlSetpoint.msg"
  "msg/SafetyState.msg"
  "msg/SafetyEvent.msg"
  "msg/VehicleTelemetry.msg"
  "srv/TriggerFailsafe.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
```

- [ ] **Étape 3 : Créer `msg/ControlSetpoint.msg`**

Le message qui circule dans la chaîne de veto. Le champ `valid_mask` évite un piège :
sans lui, on ne peut pas distinguer « vitesse nulle demandée » de « vitesse non spécifiée ».

```
# Consigne de trajectoire dans le repère ENU/FLU (ROS).
# Convertie en NED/FRD par px4_interface, seul nœud à connaître les conventions PX4.

std_msgs/Header header

# Bits indiquant quels champs sont significatifs
uint8 USE_POSITION     = 1
uint8 USE_VELOCITY     = 2
uint8 USE_ACCELERATION = 4
uint8 USE_YAW          = 8
uint8 USE_YAW_RATE     = 16
uint8 valid_mask

geometry_msgs/Point   position       # m, repère ENU
geometry_msgs/Vector3 velocity       # m/s, repère ENU
geometry_msgs/Vector3 acceleration   # m/s², repère ENU
float32 yaw                          # rad, ENU (0 = Est, sens trigonométrique)
float32 yaw_rate                     # rad/s

# Origine de la consigne, pour le diagnostic et la traçabilité des vetos
string source
```

- [ ] **Étape 4 : Créer `msg/SafetyState.msg`**

Les états sont ceux de [`10-failsafe.md`](../../architecture/10-failsafe.md). La table
complète est déclarée dès maintenant même si P1 n'en atteint qu'une partie : ajouter des
valeurs d'énumération plus tard casserait la compatibilité des enregistrements.

```
# État du superviseur de sécurité. Publié en RELIABLE / TRANSIENT_LOCAL :
# tout nœud qui démarre doit connaître l'état courant sans attendre.

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

# Condition qui a provoqué l'état courant (code stable, cf. SafetyEvent)
string reason

# Vrai si le superviseur remplace actuellement la consigne amont
bool setpoint_vetoed

# Consignes autorisées dans l'état courant
bool allow_setpoints
```

- [ ] **Étape 5 : Créer `msg/SafetyEvent.msg`**

```
# Événement de sécurité horodaté. Code machine ET message humain :
# le code permet le filtrage et l'analyse, le message est pour l'opérateur.

std_msgs/Header header

uint8 DEBUG    = 0
uint8 INFO     = 1
uint8 WARNING  = 2
uint8 ERROR    = 3
uint8 CRITICAL = 4
uint8 severity

string code       # ex. "STALE_SETPOINT", "PREFLIGHT_FAILED", "TAKEOFF_COMPLETE"
string message    # texte lisible
string detail     # contexte structuré, JSON compact
```

- [ ] **Étape 6 : Créer `msg/VehicleTelemetry.msg`**

L'état normalisé du véhicule, produit par `px4_interface`. Tout le reste du système lit
ceci et jamais `px4_msgs` directement — c'est ce qui concentre la connaissance des
conventions PX4 en un seul endroit.

```
std_msgs/Header header

geometry_msgs/Pose    pose            # repère ENU / FLU
geometry_msgs/Twist   twist           # repère ENU / FLU
float32 altitude_relative_m           # au-dessus du point de décollage

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
```

- [ ] **Étape 7 : Créer `srv/TriggerFailsafe.srv`**

```
# Force le superviseur dans un état de sécurité. Utilisé par l'exécutif de mission,
# les tests, et plus tard les commandes venues du sol.

uint8 target_state      # valeur de SafetyState
string reason
---
bool accepted
string message
```

- [ ] **Étape 8 : Construire le paquet**

```bash
./infrastructure/scripts/setup_workspace.sh
cd ~/dronoto_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dronoto_msgs
```

Attendu : `Summary: 1 package finished [.. s]`, aucune erreur.

- [ ] **Étape 9 : Vérifier que les interfaces sont visibles**

```bash
cd ~/dronoto_ws && source install/setup.bash
ros2 interface show dronoto_msgs/msg/SafetyState | head -20
ros2 interface list | grep dronoto
```

Attendu : la définition de `SafetyState` s'affiche, et les cinq interfaces `dronoto_msgs`
sont listées.

- [ ] **Étape 10 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add interfaces/
git commit -m "feat(msgs): interfaces ROS 2 dronoto_msgs (contrats inter-nœuds)"
```

---

### Tâche 4 : Intégration continue

La CI est mise en place avant d'écrire du code applicatif : une CI ajoutée après coup ne
rattrape jamais les régressions déjà introduites.

**Fichiers :**
- Créer : `.github/workflows/ci.yml`

- [ ] **Étape 1 : Créer le workflow**

```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:

concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: true

jobs:
  lint:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - run: pip install pre-commit
      - run: pre-commit run --all-files --show-diff-on-failure

  build-and-test:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy-ros-base
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: false     # PX4 n'est pas nécessaire pour construire les paquets ROS

      - name: Installer les dépendances
        run: |
          apt-get update
          apt-get install -y --no-install-recommends \
            python3-colcon-common-extensions libeigen3-dev \
            ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-ament-cmake-gtest \
            git

      - name: Préparer le workspace
        run: |
          mkdir -p /ws/src
          ln -s "$GITHUB_WORKSPACE/interfaces/ros_msgs/dronoto_msgs" /ws/src/dronoto_msgs
          for pkg in dronoto_core dronoto_interface dronoto_safety \
                     dronoto_navigation dronoto_mission dronoto_bringup; do
            [ -d "$GITHUB_WORKSPACE/drone/ros2/$pkg" ] && \
              ln -s "$GITHUB_WORKSPACE/drone/ros2/$pkg" "/ws/src/$pkg"
          done
          git clone --depth 1 -b release/1.16 https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

      - name: Construire
        shell: bash
        run: |
          source /opt/ros/jazzy/setup.bash
          cd /ws && colcon build --symlink-install --event-handlers console_direct+

      - name: Tester
        shell: bash
        run: |
          source /opt/ros/jazzy/setup.bash
          cd /ws && colcon test --event-handlers console_direct+ \
            --packages-skip px4_msgs
          colcon test-result --verbose
```

**Note sur `--packages-skip px4_msgs`** : `px4_msgs` est un paquet tiers, ses tests ne sont
pas notre responsabilité et leur exécution allonge la CI de plusieurs minutes.

- [ ] **Étape 2 : Pousser et vérifier que la CI passe**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: lint, build et test sur chaque pull request"
git push
```

Vérifier sur GitHub : les deux jobs `lint` et `build-and-test` sont verts.

> Si `build-and-test` échoue sur le clonage de `px4_msgs`, vérifier que la branche
> `release/1.16` existe toujours en amont : `git ls-remote --heads
> https://github.com/PX4/px4_msgs.git | grep 1.16`. Si elle a été renommée, ajuster ici
> **et** dans `setup_workspace.sh` — les deux doivent rester alignés.

---

*Fin de la phase P0. À ce stade : environnement reproductible, dépôt structuré, contrats
définis, CI verte. Rien ne vole encore.*

---

# PHASE P1 — SOCLE DE VOL

---

### Tâche 5 : Bibliothèque de conversions de repères

**La source d'erreur numéro un de toute intégration PX4/ROS.** PX4 raisonne en NED
(Nord-Est-Bas) pour le monde et FRD (Avant-Droite-Bas) pour le corps ; ROS raisonne en ENU
(Est-Nord-Haut) et FLU (Avant-Gauche-Haut). Une erreur ici produit un drone qui part dans
la mauvaise direction avec une parfaite assurance.

On l'isole donc dans une bibliothèque pure, testée exhaustivement, sans ROS.

**Fichiers :**
- Créer : `drone/ros2/dronoto_core/package.xml`
- Créer : `drone/ros2/dronoto_core/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_core/include/dronoto_core/frame_conversions.hpp`
- Créer : `drone/ros2/dronoto_core/src/frame_conversions.cpp`
- Test : `drone/ros2/dronoto_core/test/test_frame_conversions.cpp`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_core</name>
  <version>0.1.0</version>
  <description>Logique pure Dronoto : conversions de repères, machine à états de sécurité.
    Aucune dépendance ROS 2 dans le code — testable en millisecondes.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>eigen</depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/frame_conversions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp)
  target_link_libraries(test_frame_conversions ${PROJECT_NAME})
endif()

ament_package()
```

- [ ] **Étape 3 : Écrire l'en-tête**

`include/dronoto_core/frame_conversions.hpp` :

```cpp
#ifndef DRONOTO_CORE__FRAME_CONVERSIONS_HPP_
#define DRONOTO_CORE__FRAME_CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dronoto_core
{

/// Conversions entre les conventions PX4 (NED monde / FRD corps) et
/// ROS (ENU monde / FLU corps), conformement a REP-103.
///
/// NED->ENU et ENU->NED sont la MEME operation (permutation x/y, negation z) :
/// c'est une involution. Les deux noms existent pour la lisibilite du site d'appel.
namespace frames
{

/// Position ou vecteur : (x, y, z)_NED -> (y, x, -z)_ENU
Eigen::Vector3d nedToEnu(const Eigen::Vector3d & ned);

/// Position ou vecteur : (x, y, z)_ENU -> (y, x, -z)_NED
Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu);

/// Orientation complete : q_(NED->FRD) -> q_(ENU->FLU)
Eigen::Quaterniond nedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd);

/// Orientation complete : q_(ENU->FLU) -> q_(NED->FRD)
Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu);

/// Cap NED (0 = Nord, sens horaire) -> cap ENU (0 = Est, sens trigonometrique).
/// Resultat normalise dans (-pi, pi].
double yawNedToEnu(double yaw_ned);

/// Cap ENU -> cap NED. Resultat normalise dans (-pi, pi].
double yawEnuToNed(double yaw_enu);

/// Ramene un angle dans (-pi, pi].
double wrapPi(double angle);

}  // namespace frames
}  // namespace dronoto_core

#endif  // DRONOTO_CORE__FRAME_CONVERSIONS_HPP_
```

- [ ] **Étape 4 : Écrire le test — il doit échouer**

`test/test_frame_conversions.cpp` :

```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "dronoto_core/frame_conversions.hpp"

using dronoto_core::frames::enuFluToNedFrd;
using dronoto_core::frames::enuToNed;
using dronoto_core::frames::nedFrdToEnuFlu;
using dronoto_core::frames::nedToEnu;
using dronoto_core::frames::wrapPi;
using dronoto_core::frames::yawEnuToNed;
using dronoto_core::frames::yawNedToEnu;

constexpr double kEps = 1e-9;

// ---------------------------------------------------------------- positions

TEST(FrameConversions, NedToEnuSwapsXyAndNegatesZ)
{
  // 1 m Nord, 2 m Est, 3 m vers le BAS
  const Eigen::Vector3d ned(1.0, 2.0, 3.0);
  const Eigen::Vector3d enu = nedToEnu(ned);
  EXPECT_NEAR(enu.x(), 2.0, kEps);   // Est
  EXPECT_NEAR(enu.y(), 1.0, kEps);   // Nord
  EXPECT_NEAR(enu.z(), -3.0, kEps);  // Haut : 3 m vers le bas = -3 m vers le haut
}

TEST(FrameConversions, PositionConversionIsAnInvolution)
{
  const Eigen::Vector3d ned(-4.5, 17.25, -0.75);
  const Eigen::Vector3d back = enuToNed(nedToEnu(ned));
  EXPECT_NEAR(back.x(), ned.x(), kEps);
  EXPECT_NEAR(back.y(), ned.y(), kEps);
  EXPECT_NEAR(back.z(), ned.z(), kEps);
}

TEST(FrameConversions, AltitudeSignIsCorrect)
{
  // 50 m d'ALTITUDE = -50 en NED
  const Eigen::Vector3d ned(0.0, 0.0, -50.0);
  EXPECT_NEAR(nedToEnu(ned).z(), 50.0, kEps);
}

// ------------------------------------------------------------- orientations

TEST(FrameConversions, IdentityNedFrdPointsNorthWhichIsPlusYInEnu)
{
  // Identite en NED/FRD = nez au Nord, a plat.
  // En ENU, le Nord est +Y, donc l'axe avant du corps doit pointer vers +Y.
  const Eigen::Quaterniond q_ned = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q_enu = nedFrdToEnuFlu(q_ned);

  const Eigen::Vector3d forward_enu = q_enu * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(forward_enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(forward_enu.y(), 1.0, 1e-6);
  EXPECT_NEAR(forward_enu.z(), 0.0, 1e-6);
}

TEST(FrameConversions, UpAxisIsPreservedThroughOrientationConversion)
{
  // Corps a plat : l'axe Z du corps pointe vers le BAS en FRD,
  // donc vers le HAUT en FLU apres conversion.
  const Eigen::Quaterniond q_enu = nedFrdToEnuFlu(Eigen::Quaterniond::Identity());
  const Eigen::Vector3d up_enu = q_enu * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(up_enu.z(), 1.0, 1e-6);
}

TEST(FrameConversions, OrientationConversionRoundTrips)
{
  // Attitude quelconque : roulis 0,3 / tangage -0,2 / lacet 1,1 rad
  const Eigen::Quaterniond q_ned =
    Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());

  const Eigen::Quaterniond back = enuFluToNedFrd(nedFrdToEnuFlu(q_ned));
  // Comparer par produit scalaire : q et -q representent la meme rotation
  EXPECT_NEAR(std::abs(back.dot(q_ned)), 1.0, 1e-9);
}

TEST(FrameConversions, OrientationConversionPreservesNormalization)
{
  const Eigen::Quaterniond q_ned =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  EXPECT_NEAR(nedFrdToEnuFlu(q_ned).norm(), 1.0, 1e-9);
}

// ---------------------------------------------------------------------- cap

TEST(FrameConversions, YawNorthNedIsNinetyDegreesEnu)
{
  EXPECT_NEAR(yawNedToEnu(0.0), M_PI / 2.0, kEps);    // Nord  -> +90 deg ENU
  EXPECT_NEAR(yawNedToEnu(M_PI / 2.0), 0.0, kEps);    // Est   ->   0 deg ENU
  EXPECT_NEAR(yawNedToEnu(-M_PI / 2.0), M_PI, kEps);  // Ouest -> 180 deg ENU
}

TEST(FrameConversions, YawConversionIsAnInvolution)
{
  for (double y = -3.0; y < 3.0; y += 0.37) {
    EXPECT_NEAR(yawEnuToNed(yawNedToEnu(y)), wrapPi(y), 1e-9) << "yaw=" << y;
  }
}

TEST(FrameConversions, WrapPiNormalizesToHalfOpenInterval)
{
  EXPECT_NEAR(wrapPi(0.0), 0.0, kEps);
  EXPECT_NEAR(wrapPi(M_PI), M_PI, kEps);         // borne incluse
  EXPECT_NEAR(wrapPi(-M_PI), M_PI, kEps);        // -pi se ramene a +pi
  EXPECT_NEAR(wrapPi(3.0 * M_PI), M_PI, kEps);
  EXPECT_NEAR(wrapPi(2.0 * M_PI + 0.5), 0.5, kEps);
  EXPECT_NEAR(wrapPi(-2.0 * M_PI - 0.5), -0.5, kEps);
}
```

Lancer — le paquet ne peut pas encore s'éditer de liens, c'est attendu :

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_core
```

Attendu : ÉCHEC. `src/frame_conversions.cpp` n'existe pas, CMake s'arrête sur
`Cannot find source file: src/frame_conversions.cpp`.

- [ ] **Étape 5 : Écrire l'implémentation**

`src/frame_conversions.cpp` :

```cpp
#include "dronoto_core/frame_conversions.hpp"

#include <cmath>

namespace dronoto_core
{
namespace frames
{
namespace
{

/// Rotation de pi autour de l'axe (1, 1, 0)/sqrt(2) : envoie NED sur ENU.
/// Constructeur Eigen::Quaterniond(w, x, y, z).
const Eigen::Quaterniond kNedEnu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);

/// Rotation de pi autour de l'axe X : envoie FRD (aeronautique) sur FLU (base_link).
const Eigen::Quaterniond kFrdFlu(0.0, 1.0, 0.0, 0.0);

}  // namespace

Eigen::Vector3d nedToEnu(const Eigen::Vector3d & ned)
{
  return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu)
{
  // Involution : exactement la meme permutation.
  return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

Eigen::Quaterniond nedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd)
{
  return (kNedEnu * q_ned_frd * kFrdFlu).normalized();
}

Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu)
{
  // kNedEnu et kFrdFlu sont leurs propres inverses (rotations de pi),
  // donc la transformation inverse a la meme forme.
  return (kNedEnu * q_enu_flu * kFrdFlu).normalized();
}

double wrapPi(double angle)
{
  // std::remainder ramene dans [-pi, pi]. On envoie -pi sur +pi pour obtenir
  // l'intervalle semi-ouvert (-pi, pi] documente dans l'en-tete.
  const double wrapped = std::remainder(angle, 2.0 * M_PI);
  return (wrapped <= -M_PI) ? M_PI : wrapped;
}

double yawNedToEnu(double yaw_ned)
{
  return wrapPi(M_PI / 2.0 - yaw_ned);
}

double yawEnuToNed(double yaw_enu)
{
  return wrapPi(M_PI / 2.0 - yaw_enu);
}

}  // namespace frames
}  // namespace dronoto_core
```

- [ ] **Étape 6 : Lancer les tests — ils doivent passer**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_core
colcon test --packages-select dronoto_core
colcon test-result --verbose --test-result-base build/dronoto_core
```

Attendu : `[  PASSED  ] 10 tests.`

- [ ] **Étape 7 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_core
git commit -m "feat(core): conversions de repères NED/ENU et FRD/FLU, testées exhaustivement"
```

---

### Tâche 6 : Modèle Gazebo et airframe PX4

**Fichiers :**
- Créer : `simulation/px4/models/x500_dronoto/model.sdf`
- Créer : `simulation/px4/models/x500_dronoto/model.config`
- Créer : `simulation/px4/airframes/4501_gz_x500_dronoto`
- Créer : `infrastructure/scripts/install_px4_overlays.sh`
- Créer : `infrastructure/scripts/run_sim.sh`

- [ ] **Étape 1 : Copier le modèle x500 de PX4 comme point de départ**

```bash
cd ~/Documents/GitHub/Dronoto
mkdir -p simulation/px4/models
cp -r simulation/px4/PX4-Autopilot/Tools/simulation/gz/models/x500 \
      simulation/px4/models/x500_dronoto
ls simulation/px4/models/x500_dronoto/
```

Attendu : `model.config` et `model.sdf` sont listés.

- [ ] **Étape 2 : Adapter `model.config`**

Remplacer tout le contenu de `simulation/px4/models/x500_dronoto/model.config` par :

```xml
<?xml version="1.0"?>
<model>
  <name>x500_dronoto</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Quadricoptère Dronoto dérivé du x500 de PX4. Masse et inerties calées sur la
    configuration matérielle cible (docs/architecture/16-materiel.md).
    Les capteurs de perception (LiDAR Mid-360, caméra) sont ajoutés en phase P2.
  </description>
</model>
```

- [ ] **Étape 3 : Adapter le nom, la masse et l'inertie dans `model.sdf`**

Dans `simulation/px4/models/x500_dronoto/model.sdf` :

1. Remplacer la ligne `<model name='x500'>` par `<model name='x500_dronoto'>`.
2. Remplacer le bloc `<inertial>` du lien `base_link` par :

```xml
      <inertial>
        <!-- Masse et inerties : docs/architecture/03-simulation.md.
             À recaler sur la masse réelle mesurée (~3,4 kg) en phase P9. -->
        <mass>2.5</mass>
        <inertia>
          <ixx>0.029125</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.029125</iyy>
          <iyz>0</iyz>
          <izz>0.055225</izz>
        </inertia>
      </inertial>
```

Vérifier :

```bash
grep -n "x500_dronoto\|<mass>" simulation/px4/models/x500_dronoto/model.sdf | head
```

Attendu : le nom du modèle est `x500_dronoto` et la masse du `base_link` vaut `2.5`.

- [ ] **Étape 4 : Créer l'airframe PX4**

`simulation/px4/airframes/4501_gz_x500_dronoto` :

```sh
#!/bin/sh
#
# @name Dronoto x500 (Gazebo)
# @type Quadrotor x
#

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_dronoto}

. ${R}etc/init.d/rc.mc_defaults

param set-default SIM_GZ_EN 1

param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 1
param set-default SENS_EN_MAGSIM 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.15
param set-default CA_ROTOR0_PY 0.15
param set-default CA_ROTOR0_KM 0.05
param set-default CA_ROTOR1_PX -0.15
param set-default CA_ROTOR1_PY -0.15
param set-default CA_ROTOR1_KM 0.05
param set-default CA_ROTOR2_PX 0.15
param set-default CA_ROTOR2_PY -0.15
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -0.15
param set-default CA_ROTOR3_PY 0.15
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150
param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

# --- Parametres Dronoto ---
# Limites de mission : docs/architecture/03-simulation.md
param set-default MPC_XY_VEL_MAX 12.0
param set-default MPC_Z_VEL_MAX_UP 3.0
param set-default MPC_Z_VEL_MAX_DN 1.5
param set-default MPC_TKO_SPEED 1.5
param set-default MPC_LAND_SPEED 0.7

# Fenetre de tolerance offboard : PX4 quitte offboard si le flux s'interrompt.
# Coherent avec le seuil STALE_SETPOINT de px4_interface (tache 8).
param set-default COM_OF_LOSS_T 0.5

# Autoriser l'armement sans radiocommande : indispensable en SITL headless.
param set-default COM_RC_IN_MODE 4
param set-default NAV_RCL_ACT 0
```

- [ ] **Étape 5 : Créer le script d'installation des surcouches PX4**

Les fichiers vivent dans notre dépôt et sont liés dans le sous-module PX4 : la
personnalisation reste versionnée chez nous et le sous-module reste propre.

`infrastructure/scripts/install_px4_overlays.sh` :

```bash
#!/usr/bin/env bash
# Lie les modeles et airframes Dronoto dans le sous-module PX4-Autopilot.
# Idempotent : relancable sans effet de bord.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="$REPO_ROOT/simulation/px4/PX4-Autopilot"
MODELS="$PX4/Tools/simulation/gz/models"
AIRFRAMES="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"

[ -d "$PX4" ] || { echo "ERREUR : sous-module PX4 absent (git submodule update --init)"; exit 1; }

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
  echo "  ATTENTION : ajouter manuellement la ligne suivante dans"
  echo "              $CMAKE"
  echo "              a l'interieur de px4_add_romfs_files( ... ) :"
  echo
  echo "      4501_gz_x500_dronoto"
  echo
  exit 1
fi

echo "Surcouches installees. Reconstruire PX4 : make -C $PX4 px4_sitl"
```

```bash
chmod +x infrastructure/scripts/install_px4_overlays.sh
```

- [ ] **Étape 6 : Enregistrer l'airframe et lancer le script**

Éditer
`simulation/px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
et ajouter la ligne `4501_gz_x500_dronoto` à l'intérieur de l'appel
`px4_add_romfs_files(`, en respectant l'ordre numérique (juste après les autres airframes
`45xx` s'il y en a).

L'édition est manuelle et non scriptée délibérément : ce fichier appartient au sous-module
amont et sa structure peut évoluer. Une substitution automatique y serait fragile et
échouerait silencieusement.

```bash
./infrastructure/scripts/install_px4_overlays.sh
```

Attendu : `airframe deja enregistre dans CMakeLists.txt` puis le message de reconstruction.
Si le script sort en erreur, la ligne n'a pas été ajoutée au bon endroit.

- [ ] **Étape 7 : Reconstruire PX4 avec la surcouche**

```bash
make -C simulation/px4/PX4-Autopilot px4_sitl
```

Attendu : compilation réussie.

- [ ] **Étape 8 : Créer le script de lancement de la simulation**

`infrastructure/scripts/run_sim.sh` :

```bash
#!/usr/bin/env bash
# Lance l'agent uXRCE-DDS puis PX4 SITL avec Gazebo.
# HEADLESS=1 pour les tests, HEADLESS=0 pour voir la simulation.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="$REPO_ROOT/simulation/px4/PX4-Autopilot"

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4501}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-x500_dronoto}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1}"
export HEADLESS="${HEADLESS:-0}"

cleanup() {
  echo "Arret de la simulation..."
  kill "${AGENT_PID:-}" 2>/dev/null || true
  pkill -f 'px4 ' 2>/dev/null || true
  pkill -f 'gz sim' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

AGENT_PORT="${PX4_UXRCE_DDS_PORT:-8888}"
echo "Demarrage de l'agent Micro XRCE-DDS (UDP $AGENT_PORT)..."
MicroXRCEAgent udp4 -p "$AGENT_PORT" &
AGENT_PID=$!
sleep 2

echo "Demarrage de PX4 SITL (airframe $PX4_SYS_AUTOSTART, modele $PX4_SIM_MODEL)..."
cd "$PX4"
./build/px4_sitl_default/bin/px4
```

```bash
chmod +x infrastructure/scripts/run_sim.sh
```

- [ ] **Étape 9 : Vérifier que le drone apparaît et que les topics PX4 sortent**

Terminal 1 :

```bash
./infrastructure/scripts/run_sim.sh
```

Attendu : la fenêtre Gazebo s'ouvre avec un quadricoptère posé sur un plan de sol, et la
console PX4 affiche `INFO  [commander] Ready for takeoff!`.

Terminal 2 :

```bash
source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash
ros2 topic list | grep fmu | head
ros2 topic echo /fmu/out/vehicle_status --once --qos-reliability best_effort
```

Attendu : une liste de topics `/fmu/in/...` et `/fmu/out/...`, puis un message
`VehicleStatus` complet.

> **Piège majeur, à connaître une fois pour toutes.** Si `ros2 topic echo /fmu/out/...`
> reste muet **sans afficher d'erreur**, c'est le QoS : PX4 publie en `BEST_EFFORT` et un
> abonné `RELIABLE` n'établit jamais la correspondance, silencieusement. En ligne de
> commande, toujours ajouter `--qos-reliability best_effort`. Dans le code, tous les
> abonnements à `/fmu/out/*` utilisent `rclcpp::SensorDataQoS()`.

- [ ] **Étape 10 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add simulation/px4/models simulation/px4/airframes infrastructure/scripts
git commit -m "feat(sim): modèle Gazebo x500_dronoto, airframe PX4 4501, scripts de lancement"
```

---

### Tâche 7 : `px4_interface` — lecture de l'état PX4

La frontière unique avec l'autopilote. **Aucun autre nœud du système ne parlera jamais à
PX4** ([`05-integration-px4.md`](../../architecture/05-integration-px4.md)) : la
connaissance des conventions PX4 est concentrée ici, et tout le reste se teste sans PX4.

**Fichiers :**
- Créer : `drone/ros2/dronoto_interface/package.xml`
- Créer : `drone/ros2/dronoto_interface/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_interface/src/px4_interface_node.cpp`

- [ ] **Étape 1 : Vérifier les noms de champs réels de `px4_msgs`**

À faire **avant** d'écrire le code. Les définitions PX4 évoluent d'une version à l'autre ;
les vérifier prend deux minutes et évite une heure d'erreurs de compilation obscures.

```bash
source ~/dronoto_ws/install/setup.bash
ros2 interface show px4_msgs/msg/VehicleOdometry
ros2 interface show px4_msgs/msg/VehicleStatus | head -60
ros2 interface show px4_msgs/msg/BatteryStatus | grep -E "remaining|voltage|current"
ros2 interface show px4_msgs/msg/VehicleLocalPosition | grep -E "^float32 [xyz]|_valid"
ros2 topic info -v /fmu/out/vehicle_gps_position 2>/dev/null | grep "Type:"
```

Noter les noms exacts. Le code ci-dessous suppose :
`VehicleOdometry.position/q/velocity`, `VehicleStatus.nav_state/arming_state`,
`BatteryStatus.remaining/voltage_v/current_a`,
`VehicleLocalPosition.z/xy_valid/v_xy_valid`.

**Si un nom diffère, adapter le code — pas la vérification.** Les écarts sont attendus et
c'est le rôle de cette étape de les révéler tôt.

- [ ] **Étape 2 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_interface</name>
  <version>0.1.0</version>
  <description>Frontière unique avec PX4 : traduction uORB vers messages ROS 2 normalisés.
    Aucun autre nœud ne communique avec PX4.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>px4_msgs</depend>
  <depend>dronoto_msgs</depend>
  <depend>dronoto_core</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_eigen</depend>
  <depend>eigen</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 3 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4_interface_node src/px4_interface_node.cpp)
ament_target_dependencies(px4_interface_node
  rclcpp px4_msgs dronoto_msgs dronoto_core nav_msgs geometry_msgs tf2_ros tf2_eigen
)
target_link_libraries(px4_interface_node Eigen3::Eigen)

install(TARGETS px4_interface_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

- [ ] **Étape 4 : Écrire la partie lecture du nœud**

`src/px4_interface_node.cpp` — version complète de cette tâche. La tâche 8 y **ajoute** la
partie écriture ; ne pas supprimer ce qui suit.

```cpp
#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "dronoto_core/frame_conversions.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
namespace frames = dronoto_core::frames;

/// Frontiere unique avec PX4.
///
/// Traduit les messages uORB en messages ROS 2 normalises, en convertissant
/// systematiquement NED/FRD (PX4) vers ENU/FLU (ROS). Publie la transformation
/// odom -> base_link.
class Px4InterfaceNode : public rclcpp::Node
{
public:
  Px4InterfaceNode()
  : Node("px4_interface")
  {
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    // PX4 publie en BEST_EFFORT. Un abonne RELIABLE ne recoit RIEN, silencieusement.
    const auto px4_qos = rclcpp::SensorDataQoS();

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) { onOdometry(*msg); });

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { status_ = *msg; });

    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", px4_qos,
      [this](px4_msgs::msg::BatteryStatus::UniquePtr msg) { battery_ = *msg; });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) { local_pos_ = *msg; });

    telemetry_pub_ = create_publisher<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("state/odometry", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "px4_interface demarre, en attente des messages PX4");
  }

protected:
  // --- etat PX4 le plus recent ---
  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::BatteryStatus battery_{};
  px4_msgs::msg::VehicleLocalPosition local_pos_{};
  bool px4_seen_{false};

  std::string odom_frame_;
  std::string base_frame_;

  /// Horodatage PX4 en microsecondes, requis par tous les messages /fmu/in/*.
  uint64_t px4TimestampUs() const
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000);
  }

private:
  void onOdometry(const px4_msgs::msg::VehicleOdometry & msg)
  {
    if (!px4_seen_) {
      px4_seen_ = true;
      RCLCPP_INFO(get_logger(), "Premier message PX4 recu : liaison uXRCE-DDS active");
    }

    // --- conversion NED/FRD -> ENU/FLU ---
    const Eigen::Vector3d pos_ned(msg.position[0], msg.position[1], msg.position[2]);
    const Eigen::Vector3d vel_ned(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    // px4_msgs stocke le quaternion dans l'ordre (w, x, y, z)
    const Eigen::Quaterniond q_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);

    const Eigen::Vector3d pos_enu = frames::nedToEnu(pos_ned);
    const Eigen::Vector3d vel_enu = frames::nedToEnu(vel_ned);
    const Eigen::Quaterniond q_enu = frames::nedFrdToEnuFlu(q_ned);

    // Vitesse angulaire : FRD -> FLU, soit negation de y et z
    const Eigen::Vector3d omega_flu(
      msg.angular_velocity[0], -msg.angular_velocity[1], -msg.angular_velocity[2]);

    const auto stamp = now();

    // --- nav_msgs/Odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pos_enu.x();
    odom.pose.pose.position.y = pos_enu.y();
    odom.pose.pose.position.z = pos_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();
    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    // La vitesse lineaire de PX4 est exprimee dans le repere monde ;
    // nav_msgs/Odometry attend le repere du corps. On y ramene.
    const Eigen::Vector3d vel_body = q_enu.conjugate() * vel_enu;
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    odom.twist.twist.angular.x = omega_flu.x();
    odom.twist.twist.angular.y = omega_flu.y();
    odom.twist.twist.angular.z = omega_flu.z();
    odom_pub_->publish(odom);

    // --- TF odom -> base_link (continu, ne saute jamais : cf. 04-architecture-ros2.md) ---
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pos_enu.x();
    tf.transform.translation.y = pos_enu.y();
    tf.transform.translation.z = pos_enu.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    // --- telemetrie normalisee ---
    dronoto_msgs::msg::VehicleTelemetry tel;
    tel.header.stamp = stamp;
    tel.header.frame_id = odom_frame_;
    tel.pose = odom.pose.pose;
    tel.twist = odom.twist.twist;
    // local_pos_.z est en NED (positif vers le bas) par rapport a l'origine EKF
    tel.altitude_relative_m = -local_pos_.z;

    tel.battery_remaining = battery_.remaining;
    tel.battery_voltage_v = battery_.voltage_v;
    tel.battery_current_a = battery_.current_a;

    tel.armed = (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    tel.nav_state = status_.nav_state;
    tel.offboard_active =
      (status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    tel.preflight_ok = status_.pre_flight_checks_pass;

    // Le GNSS detaille est cable en P2 avec le reste des capteurs.
    // En P1 on se contente des drapeaux de validite de l'EKF.
    tel.gnss_ok = local_pos_.xy_valid && local_pos_.z_valid;
    tel.gnss_satellites = 0;
    tel.gnss_eph_m = local_pos_.eph;
    tel.ekf_position_valid = local_pos_.xy_valid && local_pos_.z_valid;
    tel.ekf_velocity_valid = local_pos_.v_xy_valid && local_pos_.v_z_valid;

    telemetry_pub_->publish(tel);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  rclcpp::Publisher<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4InterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Étape 5 : Construire**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_interface
```

Attendu : compilation réussie. En cas d'erreur sur un nom de champ, revenir à l'étape 1 :
la définition réelle de `px4_msgs` fait foi.

- [ ] **Étape 6 : Vérifier en simulation**

Terminal 1 : `./infrastructure/scripts/run_sim.sh`

Terminal 2 :

```bash
source ~/dronoto_ws/install/setup.bash
ros2 run dronoto_interface px4_interface_node
```

Attendu : `Premier message PX4 recu : liaison uXRCE-DDS active`.

Terminal 3 :

```bash
source ~/dronoto_ws/install/setup.bash
ros2 topic echo /state/telemetry --once
ros2 run tf2_ros tf2_echo odom base_link
```

Attendu : un message `VehicleTelemetry` avec `altitude_relative_m` proche de 0 (drone au
sol), `armed: false`, et une transformation `odom → base_link` publiée en continu.

**Vérification de signe, à ne pas sauter.** Dans la fenêtre Gazebo, déplacer le drone
manuellement vers le haut (menu de manipulation) et confirmer que
`translation.z` **augmente**. Un signe inversé ici se propagerait dans tout le système et
serait très coûteux à diagnostiquer plus tard.

- [ ] **Étape 7 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_interface
git commit -m "feat(interface): lecture de l'état PX4, télémétrie normalisée et TF odom->base_link"
```

---

### Tâche 8 : `px4_interface` — flux offboard et commandes

Le cœur de la sécurité du nœud : PX4 exige un flux de consignes à plus de 2 Hz sous peine
de quitter le mode offboard. Ce nœud publie **inconditionnellement à 20 Hz**, quelle que
soit l'activité de l'amont, en appliquant la politique de consigne périmée de
[`05-integration-px4.md §4`](../../architecture/05-integration-px4.md).

**Fichiers :**
- Modifier : `drone/ros2/dronoto_interface/src/px4_interface_node.cpp`
- Modifier : `drone/ros2/dronoto_interface/package.xml` (ajouter `std_srvs`)
- Modifier : `drone/ros2/dronoto_interface/CMakeLists.txt` (ajouter `std_srvs`)

- [ ] **Étape 1 : Ajouter la dépendance `std_srvs`**

Dans `package.xml`, après `<depend>tf2_eigen</depend>` :

```xml
  <depend>std_srvs</depend>
```

Dans `CMakeLists.txt`, après `find_package(Eigen3 REQUIRED)` :

```cmake
find_package(std_srvs REQUIRED)
```

et ajouter `std_srvs` à la liste de `ament_target_dependencies(px4_interface_node ...)`.

- [ ] **Étape 2 : Ajouter les inclusions**

Dans `src/px4_interface_node.cpp`, après `#include "px4_msgs/msg/vehicle_status.hpp"` :

```cpp
#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "std_srvs/srv/trigger.hpp"
```

- [ ] **Étape 3 : Ajouter les membres et la logique dans le constructeur**

Dans le constructeur `Px4InterfaceNode()`, juste avant le `RCLCPP_INFO` final :

```cpp
    declare_parameter<double>("setpoint_stale_warn_s", 0.1);
    declare_parameter<double>("setpoint_stale_hold_s", 0.5);
    stale_warn_s_ = get_parameter("setpoint_stale_warn_s").as_double();
    stale_hold_s_ = get_parameter("setpoint_stale_hold_s").as_double();

    setpoint_sub_ = create_subscription<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_final", rclcpp::QoS(1),
      [this](dronoto_msgs::msg::ControlSetpoint::UniquePtr msg) {
        last_setpoint_ = *msg;
        last_setpoint_time_ = now();
        have_setpoint_ = true;
      });

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", rclcpp::QoS(10));
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", rclcpp::QoS(10));
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::QoS(10));
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
      "px4/arm",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        res->success = true;
        res->message = "commande d'armement envoyee";
      });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
      "px4/disarm",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        res->success = true;
        res->message = "commande de desarmement envoyee";
      });

    set_offboard_srv_ = create_service<std_srvs::srv::Trigger>(
      "px4/set_offboard",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        // param1 = 1 (mode personnalise), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        res->success = true;
        res->message = "bascule en offboard demandee";
      });

    takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
      "px4/takeoff",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        // Mode natif PX4 : mieux teste que le decollage en offboard,
        // et gere correctement l'effet de sol (cf. 05-integration-px4.md).
        sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
        res->success = true;
        res->message = "decollage PX4 demande";
      });

    land_srv_ = create_service<std_srvs::srv::Trigger>(
      "px4/land",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        res->success = true;
        res->message = "atterrissage PX4 demande";
      });

    // 20 Hz : bien au-dessus du minimum de 2 Hz exige par PX4.
    offboard_timer_ = create_wall_timer(50ms, [this] { publishOffboardStream(); });
```

- [ ] **Étape 4 : Ajouter les méthodes privées**

Dans la section `private:` de la classe, avant les déclarations de membres :

```cpp
  /// Publie le flux offboard a 20 Hz, INCONDITIONNELLEMENT.
  ///
  /// Politique de consigne perimee (05-integration-px4.md) :
  ///   age < 100 ms  -> publier telle quelle
  ///   100..500 ms   -> republier la derniere (extrapolation figee)
  ///   >= 500 ms     -> maintien de position + evenement STALE_SETPOINT
  void publishOffboardStream()
  {
    if (!px4_seen_) {
      return;  // pas encore de liaison PX4 : rien a piloter
    }

    const double age_s = have_setpoint_ ? (now() - last_setpoint_time_).seconds() : 1e9;
    const bool stale = (age_s >= stale_hold_s_);

    if (stale && !stale_reported_) {
      stale_reported_ = true;
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::ERROR, "STALE_SETPOINT",
        "aucune consigne fraiche : maintien de position");
    } else if (!stale && stale_reported_) {
      stale_reported_ = false;
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::INFO, "SETPOINT_RESTORED",
        "flux de consignes retabli");
    }

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = px4TimestampUs();
    mode.position = true;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = mode.timestamp;

    if (stale) {
      // Maintien de la position COURANTE, pas de la derniere consigne :
      // une consigne perimee peut etre tres eloignee de la position reelle.
      sp.position = {local_pos_.x, local_pos_.y, local_pos_.z};
      sp.yaw = local_pos_.heading;
      sp.velocity = {NAN, NAN, NAN};
    } else {
      const Eigen::Vector3d pos_enu(
        last_setpoint_.position.x, last_setpoint_.position.y, last_setpoint_.position.z);
      const Eigen::Vector3d pos_ned = frames::enuToNed(pos_enu);
      sp.position = {
        static_cast<float>(pos_ned.x()), static_cast<float>(pos_ned.y()),
        static_cast<float>(pos_ned.z())};
      sp.yaw = static_cast<float>(frames::yawEnuToNed(last_setpoint_.yaw));

      if (last_setpoint_.valid_mask & dronoto_msgs::msg::ControlSetpoint::USE_VELOCITY) {
        const Eigen::Vector3d vel_ned = frames::enuToNed(Eigen::Vector3d(
          last_setpoint_.velocity.x, last_setpoint_.velocity.y, last_setpoint_.velocity.z));
        sp.velocity = {
          static_cast<float>(vel_ned.x()), static_cast<float>(vel_ned.y()),
          static_cast<float>(vel_ned.z())};
      } else {
        // NaN indique a PX4 que le champ n'est pas contraint.
        sp.velocity = {NAN, NAN, NAN};
      }
    }

    trajectory_pub_->publish(sp);
  }

  void sendCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = px4TimestampUs();
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
    RCLCPP_WARN(get_logger(), "[%s] %s", code.c_str(), message.c_str());
  }
```

- [ ] **Étape 5 : Ajouter les déclarations de membres**

À la fin de la section `private:`, après `tf_broadcaster_` :

```cpp
  rclcpp::Subscription<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_offboard_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;

  dronoto_msgs::msg::ControlSetpoint last_setpoint_{};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  bool have_setpoint_{false};
  bool stale_reported_{false};
  double stale_warn_s_{0.1};
  double stale_hold_s_{0.5};
```

Ajouter aussi `#include <cmath>` en tête de fichier pour `NAN`.

- [ ] **Étape 6 : Construire**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_interface
```

Attendu : compilation réussie.

- [ ] **Étape 7 : Vérifier le vol manuel piloté par services**

Terminal 1 : `./infrastructure/scripts/run_sim.sh`
Terminal 2 : `ros2 run dronoto_interface px4_interface_node`
Terminal 3 :

```bash
source ~/dronoto_ws/install/setup.bash
ros2 service call /px4/arm std_srvs/srv/Trigger
sleep 2
ros2 service call /px4/takeoff std_srvs/srv/Trigger
```

Attendu : **le drone décolle dans Gazebo.** C'est le premier vol du projet.

```bash
ros2 topic echo /state/telemetry --once | grep altitude_relative_m
```

Attendu : une altitude de quelques mètres.

```bash
ros2 service call /px4/land std_srvs/srv/Trigger
```

Attendu : le drone redescend et se pose.

- [ ] **Étape 8 : Vérifier la politique de consigne périmée**

Le terminal 2 doit avoir affiché, dès le démarrage,
`[STALE_SETPOINT] aucune consigne fraiche : maintien de position`.

C'est le comportement correct : aucun nœud ne publie encore sur `control/setpoint_final`,
donc `px4_interface` maintient la position au lieu de laisser le mode offboard s'effondrer.
Vérifier que le message n'apparaît **qu'une seule fois** (pas à chaque cycle de 50 ms) —
c'est le rôle du drapeau `stale_reported_`.

- [ ] **Étape 9 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_interface
git commit -m "feat(interface): flux offboard 20 Hz, politique de consigne périmée, services de commande"
```

---

### Tâche 9 : Machine à états de sécurité (bibliothèque pure)

[`10-failsafe.md`](../../architecture/10-failsafe.md) exige que cette machine soit
**testée exhaustivement** — c'est possible précisément parce qu'elle est petite et
déterministe, et c'est la raison pour laquelle on a refusé d'y mettre de l'IA. On la place
donc dans `dronoto_core`, sans ROS.

P1 implémente un sous-ensemble des conditions (celles observables sans capteurs de
perception). La table de priorité complète est en place ; P6 remplira les conditions
manquantes sans changer la structure.

**Fichiers :**
- Créer : `drone/ros2/dronoto_core/include/dronoto_core/safety_state_machine.hpp`
- Créer : `drone/ros2/dronoto_core/src/safety_state_machine.cpp`
- Test : `drone/ros2/dronoto_core/test/test_safety_state_machine.cpp`
- Modifier : `drone/ros2/dronoto_core/CMakeLists.txt`

- [ ] **Étape 1 : Écrire l'en-tête**

`include/dronoto_core/safety_state_machine.hpp` :

```cpp
#ifndef DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_
#define DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>

namespace dronoto_core
{

/// Valeurs identiques a dronoto_msgs/SafetyState pour eviter toute traduction.
enum class SafetyState : uint8_t
{
  BOOT = 0,
  IDLE = 1,
  PREFLIGHT = 2,
  READY = 3,
  ARMED = 4,
  TAKEOFF = 5,
  NOMINAL = 6,
  DEGRADED = 7,
  HOLD = 8,
  RETURNING = 9,
  LANDING = 10,
  EMERGENCY_LAND = 11,
  TERMINATED = 12,
};

/// Instantane des conditions observees. Chaque champ est un fait mesure,
/// pas une decision : la decision est le role de la machine.
struct SystemConditions
{
  // --- conditions de securite, par ordre de priorite decroissante ---
  bool px4_failsafe_active = false;   ///< PX4 a declenche son propre failsafe
  bool battery_critical = false;      ///< sous le seuil critique
  bool state_estimate_lost = false;   ///< EKF diverge ou position invalide
  bool geofence_breached = false;     ///< hors de la zone autorisee
  bool battery_below_return = false;  ///< sous la reserve de retour calculee
  bool localization_poor = false;     ///< SLAM degenere (cable en P6)
  bool perception_lost = false;       ///< tous capteurs de perception perdus (P6)
  bool setpoint_stale = false;        ///< amont muet depuis trop longtemps
  bool link_lost_rth = false;         ///< liaison perdue + politique RTH (P6)
  bool localization_degraded = false; ///< qualite reduite mais utilisable (P6)
  bool sensor_degraded = false;       ///< capteur non critique perdu (P6)

  // --- faits d'etat ---
  bool telemetry_fresh = false;       ///< on recoit des donnees du vehicule
  bool armed = false;
  bool preflight_ok = false;
  bool landed = true;
};

/// Resultat d'une evaluation.
struct SafetyDecision
{
  SafetyState state = SafetyState::BOOT;
  std::string reason;         ///< code stable, ex. "BATTERY_CRITICAL"
  bool allow_setpoints = false;
  bool changed = false;       ///< vrai si l'etat differe de l'evaluation precedente
};

/// Machine a etats de securite. Deterministe, sans allocation dans le chemin chaud,
/// sans dependance externe. Voir docs/architecture/10-failsafe.md.
class SafetyStateMachine
{
public:
  explicit SafetyStateMachine(SafetyState initial = SafetyState::BOOT);

  /// Applique les gardes de securite sur l'etat courant.
  /// Les gardes ont priorite absolue sur toute progression nominale.
  SafetyDecision update(const SystemConditions & conditions);

  /// Demande une progression nominale (decollage, mission, atterrissage...).
  /// Refusee si la transition n'est pas legale ou si une garde l'interdit.
  bool requestTransition(SafetyState desired, const SystemConditions & conditions);

  SafetyState state() const { return state_; }
  const std::string & reason() const { return reason_; }

  /// Vrai si l'aeronef est en vol dans cet etat.
  static bool isInFlight(SafetyState s);

  /// Vrai si des consignes de trajectoire externes sont acceptees dans cet etat.
  static bool allowsSetpoints(SafetyState s);

  /// Vrai si la transition nominale from -> to est legale.
  static bool isNominalTransitionAllowed(SafetyState from, SafetyState to);

  static const char * toString(SafetyState s);

private:
  SafetyState state_;
  std::string reason_;
};

}  // namespace dronoto_core

#endif  // DRONOTO_CORE__SAFETY_STATE_MACHINE_HPP_
```

- [ ] **Étape 2 : Écrire le test — il doit échouer**

`test/test_safety_state_machine.cpp` :

```cpp
#include <gtest/gtest.h>

#include <vector>

#include "dronoto_core/safety_state_machine.hpp"

using dronoto_core::SafetyDecision;
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
  SafetyState::BOOT,      SafetyState::IDLE,       SafetyState::PREFLIGHT,
  SafetyState::READY,     SafetyState::ARMED,      SafetyState::TAKEOFF,
  SafetyState::NOMINAL,   SafetyState::DEGRADED,   SafetyState::HOLD,
  SafetyState::RETURNING, SafetyState::LANDING,    SafetyState::EMERGENCY_LAND,
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
```

Ajouter le test au `CMakeLists.txt` de `dronoto_core`, dans le bloc `if(BUILD_TESTING)` :

```cmake
  ament_add_gtest(test_safety_state_machine test/test_safety_state_machine.cpp)
  target_link_libraries(test_safety_state_machine ${PROJECT_NAME})
```

et ajouter `src/safety_state_machine.cpp` à `add_library(${PROJECT_NAME} SHARED ...)`.

Lancer :

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_core
```

Attendu : ÉCHEC — `Cannot find source file: src/safety_state_machine.cpp`.

- [ ] **Étape 3 : Écrire l'implémentation**

`src/safety_state_machine.cpp` :

```cpp
#include "dronoto_core/safety_state_machine.hpp"

namespace dronoto_core
{

SafetyStateMachine::SafetyStateMachine(SafetyState initial)
: state_(initial), reason_("INIT")
{
}

bool SafetyStateMachine::isInFlight(SafetyState s)
{
  switch (s) {
    case SafetyState::TAKEOFF:
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
    case SafetyState::RETURNING:
    case SafetyState::LANDING:
    case SafetyState::EMERGENCY_LAND:
      return true;
    default:
      return false;
  }
}

bool SafetyStateMachine::allowsSetpoints(SafetyState s)
{
  switch (s) {
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
    case SafetyState::RETURNING:
    case SafetyState::LANDING:
      return true;
    default:
      // BOOT, IDLE, PREFLIGHT, READY, ARMED : au sol, aucune consigne externe.
      // TAKEOFF, EMERGENCY_LAND : PX4 pilote en mode natif, on ne s'en mele pas.
      return false;
  }
}

bool SafetyStateMachine::isNominalTransitionAllowed(SafetyState from, SafetyState to)
{
  if (from == to) {
    return true;
  }
  switch (from) {
    case SafetyState::BOOT:
      return to == SafetyState::IDLE;
    case SafetyState::IDLE:
      return to == SafetyState::PREFLIGHT;
    case SafetyState::PREFLIGHT:
      return to == SafetyState::READY || to == SafetyState::IDLE;
    case SafetyState::READY:
      return to == SafetyState::ARMED || to == SafetyState::IDLE;
    case SafetyState::ARMED:
      return to == SafetyState::TAKEOFF || to == SafetyState::IDLE;
    case SafetyState::TAKEOFF:
      return to == SafetyState::NOMINAL || to == SafetyState::HOLD;
    case SafetyState::NOMINAL:
    case SafetyState::DEGRADED:
    case SafetyState::HOLD:
      return to == SafetyState::NOMINAL || to == SafetyState::DEGRADED ||
             to == SafetyState::HOLD || to == SafetyState::RETURNING ||
             to == SafetyState::LANDING;
    case SafetyState::RETURNING:
      return to == SafetyState::LANDING || to == SafetyState::HOLD;
    case SafetyState::LANDING:
      return to == SafetyState::TERMINATED || to == SafetyState::HOLD;
    case SafetyState::EMERGENCY_LAND:
      return to == SafetyState::TERMINATED;
    case SafetyState::TERMINATED:
      return to == SafetyState::IDLE;  // reinitialisation apres atterrissage
  }
  return false;
}

SafetyDecision SafetyStateMachine::update(const SystemConditions & c)
{
  const SafetyState previous = state_;
  const bool in_flight = isInFlight(state_);

  // Table de priorite de docs/architecture/10-failsafe.md, section 2.
  // La PREMIERE condition qui s'applique gagne. L'ordre EST la specification.
  if (c.px4_failsafe_active) {
    // On cede : PX4 a l'autorite, on cesse simplement d'emettre.
    reason_ = "PX4_FAILSAFE";
    SafetyDecision d;
    d.state = state_;
    d.reason = reason_;
    d.allow_setpoints = false;
    d.changed = false;
    return d;
  }

  if (in_flight && c.battery_critical) {
    state_ = SafetyState::EMERGENCY_LAND;
    reason_ = "BATTERY_CRITICAL";
  } else if (in_flight && c.state_estimate_lost) {
    state_ = SafetyState::EMERGENCY_LAND;
    reason_ = "STATE_ESTIMATE_LOST";
  } else if (in_flight && c.geofence_breached) {
    state_ = SafetyState::RETURNING;
    reason_ = "GEOFENCE_BREACH";
  } else if (in_flight && c.battery_below_return) {
    state_ = SafetyState::RETURNING;
    reason_ = "BATTERY_BELOW_RETURN_RESERVE";
  } else if (in_flight && c.localization_poor) {
    state_ = SafetyState::HOLD;
    reason_ = "LOCALIZATION_POOR";
  } else if (in_flight && c.perception_lost) {
    state_ = SafetyState::HOLD;
    reason_ = "PERCEPTION_LOST";
  } else if (in_flight && c.setpoint_stale) {
    state_ = SafetyState::HOLD;
    reason_ = "STALE_SETPOINT";
  } else if (in_flight && c.link_lost_rth) {
    state_ = SafetyState::RETURNING;
    reason_ = "LINK_LOST";
  } else if (in_flight && (c.localization_degraded || c.sensor_degraded)) {
    // Ne pas retrograder un etat plus engage : RETURNING et LANDING l'emportent.
    if (state_ == SafetyState::NOMINAL || state_ == SafetyState::HOLD) {
      state_ = SafetyState::DEGRADED;
      reason_ = c.localization_degraded ? "LOCALIZATION_DEGRADED" : "SENSOR_DEGRADED";
    }
  } else if (state_ == SafetyState::HOLD || state_ == SafetyState::DEGRADED) {
    // Plus aucune condition : retour au vol nominal.
    state_ = SafetyState::NOMINAL;
    reason_ = "RECOVERED";
  }

  SafetyDecision d;
  d.state = state_;
  d.reason = reason_;
  d.allow_setpoints = allowsSetpoints(state_);
  d.changed = (state_ != previous);
  return d;
}

bool SafetyStateMachine::requestTransition(SafetyState desired, const SystemConditions & c)
{
  if (!isNominalTransitionAllowed(state_, desired)) {
    return false;
  }

  // Gardes specifiques a certaines progressions.
  if (desired == SafetyState::READY && !c.preflight_ok) {
    return false;
  }
  if (desired == SafetyState::ARMED && !c.armed) {
    return false;
  }
  if (desired != SafetyState::IDLE && !c.telemetry_fresh) {
    return false;
  }
  // Une progression nominale ne peut jamais annuler un etat d'urgence.
  if (state_ == SafetyState::EMERGENCY_LAND && desired != SafetyState::TERMINATED) {
    return false;
  }

  state_ = desired;
  reason_ = "MISSION_REQUEST";
  return true;
}

const char * SafetyStateMachine::toString(SafetyState s)
{
  switch (s) {
    case SafetyState::BOOT: return "BOOT";
    case SafetyState::IDLE: return "IDLE";
    case SafetyState::PREFLIGHT: return "PREFLIGHT";
    case SafetyState::READY: return "READY";
    case SafetyState::ARMED: return "ARMED";
    case SafetyState::TAKEOFF: return "TAKEOFF";
    case SafetyState::NOMINAL: return "NOMINAL";
    case SafetyState::DEGRADED: return "DEGRADED";
    case SafetyState::HOLD: return "HOLD";
    case SafetyState::RETURNING: return "RETURNING";
    case SafetyState::LANDING: return "LANDING";
    case SafetyState::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case SafetyState::TERMINATED: return "TERMINATED";
  }
  return "UNKNOWN";
}

}  // namespace dronoto_core
```

- [ ] **Étape 4 : Lancer les tests**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_core
colcon test --packages-select dronoto_core
colcon test-result --verbose --test-result-base build/dronoto_core
```

Attendu : les 10 tests de conversions **et** les 17 tests de la machine à états passent.

- [ ] **Étape 5 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_core
git commit -m "feat(core): machine à états de sécurité, priorités et transitions testées exhaustivement"
```

---

### Tâche 10 : `safety_supervisor` — le veto

Le nœud qui a l'**autorité absolue** sur les consignes. Il s'intercale entre le suivi de
trajectoire et `px4_interface` : rien n'atteint PX4 sans passer par lui.

**Fichiers :**
- Créer : `drone/ros2/dronoto_safety/package.xml`
- Créer : `drone/ros2/dronoto_safety/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_safety/src/safety_supervisor_node.cpp`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_safety</name>
  <version>0.1.0</version>
  <description>Superviseur de sécurité : machine à états et veto sur les consignes.
    Autorité absolue sur la chaîne de contrôle.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>dronoto_msgs</depend>
  <depend>dronoto_core</depend>
  <depend>px4_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
ament_target_dependencies(safety_supervisor_node
  rclcpp dronoto_msgs dronoto_core px4_msgs
)

install(TARGETS safety_supervisor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

- [ ] **Étape 3 : Écrire le nœud**

`src/safety_supervisor_node.cpp` :

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "dronoto_core/safety_state_machine.hpp"
#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "dronoto_msgs/msg/safety_state.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "dronoto_msgs/srv/trigger_failsafe.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using dronoto_core::SafetyState;
using dronoto_core::SafetyStateMachine;
using dronoto_core::SystemConditions;

/// Superviseur de securite. Autorite de veto absolue sur la chaine de controle.
///
/// Entree  : control/setpoint_safe (sortie de l'evitement reactif, ou du suivi
///           de trajectoire directement en P1 via remappage)
/// Sortie  : control/setpoint_final (seule entree de px4_interface)
///
/// Le superviseur peut supprimer ou remplacer toute consigne sans cooperation
/// de l'amont : un composant de securite qui a besoin que le composant fautif
/// coopere n'est pas un composant de securite.
class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode()
  : Node("safety_supervisor"), fsm_(SafetyState::BOOT)
  {
    declare_parameter<double>("telemetry_timeout_s", 1.0);
    declare_parameter<double>("setpoint_timeout_s", 0.5);
    declare_parameter<double>("battery_critical_ratio", 0.10);
    declare_parameter<double>("battery_return_ratio", 0.25);
    telemetry_timeout_s_ = get_parameter("telemetry_timeout_s").as_double();
    setpoint_timeout_s_ = get_parameter("setpoint_timeout_s").as_double();
    battery_critical_ratio_ = get_parameter("battery_critical_ratio").as_double();
    battery_return_ratio_ = get_parameter("battery_return_ratio").as_double();

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        last_telemetry_time_ = now();
        have_telemetry_ = true;
      });

    failsafe_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
      "/fmu/out/failsafe_flags", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::FailsafeFlags::UniquePtr msg) { failsafe_flags_ = *msg; });

    setpoint_sub_ = create_subscription<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_safe", rclcpp::QoS(1),
      [this](dronoto_msgs::msg::ControlSetpoint::UniquePtr msg) {
        incoming_setpoint_ = *msg;
        last_setpoint_time_ = now();
        have_setpoint_ = true;
      });

    // TRANSIENT_LOCAL : tout noeud qui demarre recoit immediatement l'etat courant
    // sans attendre la publication suivante (cf. 04-architecture-ros2.md).
    state_pub_ = create_publisher<dronoto_msgs::msg::SafetyState>(
      "safety/state", rclcpp::QoS(1).reliable().transient_local());
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());
    setpoint_pub_ = create_publisher<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_final", rclcpp::QoS(1));

    transition_srv_ = create_service<dronoto_msgs::srv::TriggerFailsafe>(
      "safety/trigger_failsafe",
      [this](const dronoto_msgs::srv::TriggerFailsafe::Request::SharedPtr req,
             dronoto_msgs::srv::TriggerFailsafe::Response::SharedPtr res) {
        const auto target = static_cast<SafetyState>(req->target_state);
        const bool ok = fsm_.requestTransition(target, buildConditions());
        res->accepted = ok;
        res->message = ok ? std::string("transition acceptee vers ") +
                              SafetyStateMachine::toString(target)
                          : std::string("transition refusee depuis ") +
                              SafetyStateMachine::toString(fsm_.state());
        if (ok) {
          publishEvent(
            dronoto_msgs::msg::SafetyEvent::INFO, "STATE_TRANSITION",
            std::string(SafetyStateMachine::toString(target)) + " (" + req->reason + ")");
          publishState(true);
        }
      });

    timer_ = create_wall_timer(100ms, [this] { tick(); });  // 10 Hz
    RCLCPP_INFO(get_logger(), "safety_supervisor demarre en etat BOOT");
  }

private:
  SystemConditions buildConditions() const
  {
    SystemConditions c;
    const auto t = now();

    c.telemetry_fresh =
      have_telemetry_ && (t - last_telemetry_time_).seconds() < telemetry_timeout_s_;

    if (c.telemetry_fresh) {
      c.armed = telemetry_.armed;
      c.preflight_ok = telemetry_.preflight_ok;
      c.landed = telemetry_.altitude_relative_m < 0.5f;
      c.battery_critical = telemetry_.battery_remaining < battery_critical_ratio_;
      c.battery_below_return = telemetry_.battery_remaining < battery_return_ratio_;
      c.state_estimate_lost = !telemetry_.ekf_position_valid;
    } else {
      // Aucune telemetrie : on suppose le pire cas, jamais le meilleur.
      c.state_estimate_lost = true;
    }

    c.px4_failsafe_active = failsafe_flags_.mode_req_angular_velocity ? false : false;
    // Le champ generique de failsafe actif varie selon les versions de PX4.
    // On s'appuie sur les conditions individuelles pertinentes.
    c.px4_failsafe_active =
      failsafe_flags_.battery_warning >= 2 || failsafe_flags_.local_position_invalid;

    const bool setpoint_fresh =
      have_setpoint_ && (t - last_setpoint_time_).seconds() < setpoint_timeout_s_;
    c.setpoint_stale = SafetyStateMachine::isInFlight(fsm_.state()) && !setpoint_fresh;

    // Conditions cablees en P6 : liaison, SLAM, perception, geofence.
    return c;
  }

  void tick()
  {
    const auto conditions = buildConditions();
    const auto decision = fsm_.update(conditions);

    if (decision.changed) {
      publishEvent(
        dronoto_msgs::msg::SafetyEvent::WARNING, decision.reason,
        std::string("etat -> ") + SafetyStateMachine::toString(decision.state));
    }

    publishState(decision.changed);
    forwardOrVetoSetpoint(decision);
  }

  /// Le point de veto. Trois cas, dans cet ordre.
  void forwardOrVetoSetpoint(const dronoto_core::SafetyDecision & decision)
  {
    if (!decision.allow_setpoints) {
      // 1. L'etat interdit toute consigne externe : on n'emet RIEN.
      //    px4_interface appliquera sa politique de consigne perimee
      //    (maintien de position), ce qui est le comportement sur.
      return;
    }

    const bool setpoint_fresh =
      have_setpoint_ && (now() - last_setpoint_time_).seconds() < setpoint_timeout_s_;

    if (!setpoint_fresh) {
      // 2. Amont muet : on n'emet rien non plus, meme raisonnement.
      return;
    }

    // 3. Consigne fraiche et etat permissif : transmission telle quelle.
    auto out = incoming_setpoint_;
    out.header.stamp = now();
    out.source = incoming_setpoint_.source + "|supervisor";
    setpoint_pub_->publish(out);
  }

  void publishState(bool /*changed*/)
  {
    dronoto_msgs::msg::SafetyState msg;
    msg.header.stamp = now();
    msg.state = static_cast<uint8_t>(fsm_.state());
    msg.reason = fsm_.reason();
    msg.allow_setpoints = SafetyStateMachine::allowsSetpoints(fsm_.state());
    msg.setpoint_vetoed = !msg.allow_setpoints;
    state_pub_->publish(msg);
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
    RCLCPP_INFO(get_logger(), "[%s] %s", code.c_str(), message.c_str());
  }

  SafetyStateMachine fsm_;

  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  dronoto_msgs::msg::ControlSetpoint incoming_setpoint_{};
  px4_msgs::msg::FailsafeFlags failsafe_flags_{};

  rclcpp::Time last_telemetry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  bool have_telemetry_{false};
  bool have_setpoint_{false};

  double telemetry_timeout_s_{1.0};
  double setpoint_timeout_s_{0.5};
  double battery_critical_ratio_{0.10};
  double battery_return_ratio_{0.25};

  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyState>::SharedPtr state_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Service<dronoto_msgs::srv::TriggerFailsafe>::SharedPtr transition_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Étape 4 : Vérifier les champs réels de `FailsafeFlags`**

```bash
source ~/dronoto_ws/install/setup.bash
ros2 interface show px4_msgs/msg/FailsafeFlags | grep -E "battery_warning|local_position_invalid"
```

Si l'un de ces champs n'existe pas dans votre version, remplacer l'expression de
`c.px4_failsafe_active` par les champs disponibles. Supprimer aussi la ligne morte
`c.px4_failsafe_active = failsafe_flags_.mode_req_angular_velocity ? false : false;` qui
n'est là que pour signaler explicitement l'endroit à adapter.

- [ ] **Étape 5 : Construire et vérifier**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_safety
```

Terminal 1 : `./infrastructure/scripts/run_sim.sh`
Terminal 2 : `ros2 run dronoto_interface px4_interface_node`
Terminal 3 : `ros2 run dronoto_safety safety_supervisor_node`
Terminal 4 :

```bash
source ~/dronoto_ws/install/setup.bash
ros2 topic echo /safety/state --once
```

Attendu : `state: 0` (BOOT), `allow_setpoints: false`, `setpoint_vetoed: true`.

```bash
ros2 service call /safety/trigger_failsafe dronoto_msgs/srv/TriggerFailsafe \
  "{target_state: 1, reason: 'test manuel'}"
```

Attendu : `accepted=True` (BOOT → IDLE est légal).

```bash
ros2 service call /safety/trigger_failsafe dronoto_msgs/srv/TriggerFailsafe \
  "{target_state: 6, reason: 'saut illegal'}"
```

Attendu : `accepted=False` — IDLE → NOMINAL est refusé. **C'est le comportement recherché** :
la machine à états refuse les sauts, y compris demandés par un opérateur.

- [ ] **Étape 6 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_safety
git commit -m "feat(safety): superviseur de sécurité avec veto sur la chaîne de consignes"
```

---

### Tâche 11 : `trajectory_follower` — suivi de waypoints

Transforme un objectif de pose en flux de consignes à 20 Hz, borné en vitesse. En P1 c'est
une interpolation linéaire ; P3 le remplace par la génération de trajectoire polynomiale
contrainte par l'ESDF, **sans changer les topics d'entrée et de sortie**.

**Fichiers :**
- Créer : `drone/ros2/dronoto_navigation/package.xml`
- Créer : `drone/ros2/dronoto_navigation/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_navigation/src/trajectory_follower_node.cpp`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_navigation</name>
  <version>0.1.0</version>
  <description>Navigation Dronoto : suivi de trajectoire et, à partir de P3,
    planification globale et évitement réactif.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>dronoto_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>eigen</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_follower_node src/trajectory_follower_node.cpp)
ament_target_dependencies(trajectory_follower_node
  rclcpp dronoto_msgs geometry_msgs std_msgs
)
target_link_libraries(trajectory_follower_node Eigen3::Eigen)

install(TARGETS trajectory_follower_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

- [ ] **Étape 3 : Écrire le nœud**

`src/trajectory_follower_node.cpp` :

```cpp
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "dronoto_msgs/msg/control_setpoint.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/// Suivi de waypoints par interpolation bornee en vitesse.
///
/// Entrees : navigation/goal (objectif), state/telemetry (position courante)
/// Sortie  : control/setpoint_raw a 20 Hz
///
/// P1 : interpolation lineaire. P3 remplace l'interieur par une trajectoire
/// polynomiale contrainte par l'ESDF, sans changer ces topics.
class TrajectoryFollowerNode : public rclcpp::Node
{
public:
  TrajectoryFollowerNode()
  : Node("trajectory_follower")
  {
    declare_parameter<double>("max_speed_ms", 4.0);
    declare_parameter<double>("max_climb_ms", 2.0);
    declare_parameter<double>("goal_tolerance_m", 0.5);
    declare_parameter<double>("slowdown_radius_m", 3.0);
    max_speed_ = get_parameter("max_speed_ms").as_double();
    max_climb_ = get_parameter("max_climb_ms").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance_m").as_double();
    slowdown_radius_ = get_parameter("slowdown_radius_m").as_double();

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "navigation/goal", rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) {
        goal_ = *msg;
        have_goal_ = true;
        // Le point de depart de l'interpolation est la position au moment
        // ou l'objectif est recu : le drone glisse vers la cible depuis la.
        carrot_ = currentPosition();
        have_carrot_ = have_telemetry_;
        RCLCPP_INFO(
          get_logger(), "nouvel objectif : (%.1f, %.1f, %.1f)", goal_.pose.position.x,
          goal_.pose.position.y, goal_.pose.position.z);
      });

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        have_telemetry_ = true;
      });

    setpoint_pub_ = create_publisher<dronoto_msgs::msg::ControlSetpoint>(
      "control/setpoint_raw", rclcpp::QoS(1));
    distance_pub_ = create_publisher<std_msgs::msg::Float32>(
      "navigation/distance_to_goal", rclcpp::QoS(10));

    timer_ = create_wall_timer(50ms, [this] { tick(); });  // 20 Hz
    RCLCPP_INFO(get_logger(), "trajectory_follower demarre");
  }

private:
  Eigen::Vector3d currentPosition() const
  {
    return Eigen::Vector3d(
      telemetry_.pose.position.x, telemetry_.pose.position.y, telemetry_.pose.position.z);
  }

  void tick()
  {
    if (!have_goal_ || !have_telemetry_) {
      return;  // rien a suivre : px4_interface maintiendra la position
    }

    const Eigen::Vector3d position = currentPosition();
    const Eigen::Vector3d goal(
      goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);

    if (!have_carrot_) {
      carrot_ = position;
      have_carrot_ = true;
    }

    const double distance = (goal - position).norm();
    std_msgs::msg::Float32 d;
    d.data = static_cast<float>(distance);
    distance_pub_->publish(d);

    // --- avancee de la "carotte" vers l'objectif, bornee en vitesse ---
    // On deplace un point intermediaire plutot que d'envoyer directement
    // l'objectif : PX4 accelererait sans limite vers une cible lointaine.
    const double dt = 0.05;
    Eigen::Vector3d to_goal = goal - carrot_;
    const double remaining = to_goal.norm();

    if (remaining > 1e-3) {
      // Ralentissement a l'approche : evite le depassement et l'oscillation.
      const double speed_scale =
        std::min(1.0, std::max(0.15, distance / slowdown_radius_));
      Eigen::Vector3d direction = to_goal / remaining;

      Eigen::Vector3d step = direction * max_speed_ * speed_scale * dt;
      // Limite verticale distincte : monter et descendre coutent cher en energie
      // et PX4 borne deja MPC_Z_VEL_MAX_*.
      const double max_dz = max_climb_ * dt;
      if (std::abs(step.z()) > max_dz) {
        step.z() = std::copysign(max_dz, step.z());
      }

      if (step.norm() >= remaining) {
        carrot_ = goal;
      } else {
        carrot_ += step;
      }
    }

    dronoto_msgs::msg::ControlSetpoint sp;
    sp.header.stamp = now();
    sp.header.frame_id = "odom";
    sp.valid_mask = dronoto_msgs::msg::ControlSetpoint::USE_POSITION |
                    dronoto_msgs::msg::ControlSetpoint::USE_YAW;
    sp.position.x = carrot_.x();
    sp.position.y = carrot_.y();
    sp.position.z = carrot_.z();

    // Cap : orienter vers l'objectif tant qu'on est loin, sinon conserver
    // le cap demande. Tourner sur place a l'arrivee est inutile et couteux.
    if (distance > 2.0) {
      const Eigen::Vector3d horizontal = goal - position;
      sp.yaw = static_cast<float>(std::atan2(horizontal.y(), horizontal.x()));
    } else {
      sp.yaw = yawFromQuaternion(goal_.pose.orientation);
    }
    sp.source = "trajectory_follower";
    setpoint_pub_->publish(sp);
  }

  static float yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    // Extraction du lacet uniquement (rotation autour de Z en ENU).
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return static_cast<float>(std::atan2(siny, cosy));
  }

  geometry_msgs::msg::PoseStamped goal_{};
  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  Eigen::Vector3d carrot_{Eigen::Vector3d::Zero()};
  bool have_goal_{false};
  bool have_telemetry_{false};
  bool have_carrot_{false};

  double max_speed_{4.0};
  double max_climb_{2.0};
  double goal_tolerance_{0.5};
  double slowdown_radius_{3.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<dronoto_msgs::msg::ControlSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Étape 4 : Construire**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_navigation
```

Attendu : compilation réussie.

- [ ] **Étape 5 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_navigation
git commit -m "feat(navigation): suivi de waypoints par interpolation bornée en vitesse"
```

---

### Tâche 12 : `mission_executive` — séquenceur de mission

Orchestre la mission complète. Séquenceur à états explicite en P1 ; BehaviorTree.CPP
arrive en P4 quand la boucle d'exploration a besoin de réévaluation réactive.

**Fichiers :**
- Créer : `drone/ros2/dronoto_mission/package.xml`
- Créer : `drone/ros2/dronoto_mission/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_mission/src/mission_executive_node.cpp`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_mission</name>
  <version>0.1.0</version>
  <description>Exécutif de mission Dronoto : séquencement décollage, waypoints,
    atterrissage.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>dronoto_msgs</depend>
  <depend>dronoto_core</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mission_executive_node src/mission_executive_node.cpp)
ament_target_dependencies(mission_executive_node
  rclcpp dronoto_msgs dronoto_core geometry_msgs std_msgs std_srvs
)

install(TARGETS mission_executive_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

- [ ] **Étape 3 : Écrire le nœud**

`src/mission_executive_node.cpp` :

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "dronoto_core/safety_state_machine.hpp"
#include "dronoto_msgs/msg/safety_event.hpp"
#include "dronoto_msgs/msg/safety_state.hpp"
#include "dronoto_msgs/msg/vehicle_telemetry.hpp"
#include "dronoto_msgs/srv/trigger_failsafe.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using dronoto_core::SafetyState;

/// Phases de la mission P1. Strictement sequentielles.
enum class Phase
{
  WAIT_TELEMETRY,
  PREFLIGHT,
  ARMING,
  TAKEOFF,
  ENTER_OFFBOARD,
  WAYPOINTS,
  LANDING,
  DONE,
  ABORTED,
};

/// Executif de mission : decollage, waypoints, atterrissage.
///
/// Ne pilote JAMAIS directement : il demande des transitions au superviseur et
/// publie des objectifs pour le suivi de trajectoire. Le superviseur peut
/// refuser ou outrepasser a tout moment.
class MissionExecutiveNode : public rclcpp::Node
{
public:
  MissionExecutiveNode()
  : Node("mission_executive")
  {
    declare_parameter<double>("takeoff_altitude_m", 5.0);
    declare_parameter<double>("waypoint_tolerance_m", 0.8);
    declare_parameter<double>("phase_timeout_s", 60.0);
    declare_parameter<bool>("autostart", true);
    // Waypoints aplatis : [x1, y1, z1, x2, y2, z2, ...] en repere ENU local.
    declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});

    takeoff_altitude_ = get_parameter("takeoff_altitude_m").as_double();
    waypoint_tolerance_ = get_parameter("waypoint_tolerance_m").as_double();
    phase_timeout_s_ = get_parameter("phase_timeout_s").as_double();
    autostart_ = get_parameter("autostart").as_bool();
    loadWaypoints(get_parameter("waypoints").as_double_array());

    telemetry_sub_ = create_subscription<dronoto_msgs::msg::VehicleTelemetry>(
      "state/telemetry", rclcpp::QoS(10),
      [this](dronoto_msgs::msg::VehicleTelemetry::UniquePtr msg) {
        telemetry_ = *msg;
        have_telemetry_ = true;
      });

    safety_sub_ = create_subscription<dronoto_msgs::msg::SafetyState>(
      "safety/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](dronoto_msgs::msg::SafetyState::UniquePtr msg) { safety_ = *msg; });

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      "navigation/distance_to_goal", rclcpp::QoS(10),
      [this](std_msgs::msg::Float32::UniquePtr msg) {
        distance_to_goal_ = msg->data;
        have_distance_ = true;
      });

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "navigation/goal", rclcpp::QoS(1).reliable().transient_local());
    phase_pub_ = create_publisher<std_msgs::msg::String>(
      "mission/phase", rclcpp::QoS(1).reliable().transient_local());
    event_pub_ = create_publisher<dronoto_msgs::msg::SafetyEvent>(
      "safety/events", rclcpp::QoS(50).reliable().transient_local());

    arm_client_ = create_client<std_srvs::srv::Trigger>("px4/arm");
    takeoff_client_ = create_client<std_srvs::srv::Trigger>("px4/takeoff");
    offboard_client_ = create_client<std_srvs::srv::Trigger>("px4/set_offboard");
    land_client_ = create_client<std_srvs::srv::Trigger>("px4/land");
    safety_client_ = create_client<dronoto_msgs::srv::TriggerFailsafe>(
      "safety/trigger_failsafe");

    timer_ = create_wall_timer(200ms, [this] { tick(); });  // 5 Hz
    phase_start_ = now();
    RCLCPP_INFO(
      get_logger(), "mission_executive demarre : %zu waypoints, decollage a %.1f m",
      waypoints_.size(), takeoff_altitude_);
  }

private:
  void loadWaypoints(const std::vector<double> & flat)
  {
    if (flat.size() % 3 != 0) {
      RCLCPP_ERROR(
        get_logger(), "parametre 'waypoints' invalide : %zu valeurs, multiple de 3 attendu",
        flat.size());
      return;
    }
    for (size_t i = 0; i + 2 < flat.size(); i += 3) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "odom";
      wp.pose.position.x = flat[i];
      wp.pose.position.y = flat[i + 1];
      wp.pose.position.z = flat[i + 2];
      wp.pose.orientation.w = 1.0;
      waypoints_.push_back(wp);
    }
  }

  void setPhase(Phase p, const std::string & label)
  {
    phase_ = p;
    phase_start_ = now();
    std_msgs::msg::String msg;
    msg.data = label;
    phase_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "phase -> %s", label.c_str());
    publishEvent(dronoto_msgs::msg::SafetyEvent::INFO, "MISSION_PHASE", label);
  }

  bool phaseTimedOut() const { return (now() - phase_start_).seconds() > phase_timeout_s_; }

  void abort(const std::string & why)
  {
    publishEvent(dronoto_msgs::msg::SafetyEvent::ERROR, "MISSION_ABORTED", why);
    RCLCPP_ERROR(get_logger(), "mission interrompue : %s", why.c_str());
    callTrigger(land_client_);
    setPhase(Phase::ABORTED, "ABORTED");
  }

  void tick()
  {
    if (!autostart_ || phase_ == Phase::DONE || phase_ == Phase::ABORTED) {
      return;
    }

    // Le superviseur a l'autorite : s'il n'est plus dans un etat compatible
    // avec la poursuite de la mission, on abandonne.
    const auto safety_state = static_cast<SafetyState>(safety_.state);
    if (safety_state == SafetyState::EMERGENCY_LAND) {
      abort("le superviseur a declenche un atterrissage d'urgence");
      return;
    }

    if (phaseTimedOut()) {
      abort("delai de phase depasse");
      return;
    }

    switch (phase_) {
      case Phase::WAIT_TELEMETRY:
        if (have_telemetry_) {
          requestSafety(SafetyState::IDLE, "demarrage de mission");
          requestSafety(SafetyState::PREFLIGHT, "verifications prealables");
          setPhase(Phase::PREFLIGHT, "PREFLIGHT");
        }
        break;

      case Phase::PREFLIGHT:
        // Verifications prealables de 05-integration-px4.md, sous-ensemble P1.
        if (telemetry_.preflight_ok && telemetry_.ekf_position_valid &&
            telemetry_.battery_remaining > 0.30f) {
          requestSafety(SafetyState::READY, "verifications passees");
          callTrigger(arm_client_);
          setPhase(Phase::ARMING, "ARMING");
        }
        break;

      case Phase::ARMING:
        if (telemetry_.armed) {
          requestSafety(SafetyState::ARMED, "arme");
          callTrigger(takeoff_client_);
          requestSafety(SafetyState::TAKEOFF, "decollage");
          setPhase(Phase::TAKEOFF, "TAKEOFF");
        }
        break;

      case Phase::TAKEOFF:
        // Attendre 90 % de l'altitude cible : exiger 100 % ferait patiner
        // sur le regime permanent du controleur d'altitude.
        if (telemetry_.altitude_relative_m > 0.9 * takeoff_altitude_) {
          publishFirstGoalAtCurrentPosition();
          callTrigger(offboard_client_);
          setPhase(Phase::ENTER_OFFBOARD, "ENTER_OFFBOARD");
        }
        break;

      case Phase::ENTER_OFFBOARD:
        if (telemetry_.offboard_active) {
          requestSafety(SafetyState::NOMINAL, "offboard actif");
          waypoint_index_ = 0;
          publishCurrentWaypoint();
          setPhase(Phase::WAYPOINTS, "WAYPOINTS");
        }
        break;

      case Phase::WAYPOINTS:
        if (waypoints_.empty()) {
          startLanding();
        } else if (have_distance_ && distance_to_goal_ < waypoint_tolerance_) {
          RCLCPP_INFO(get_logger(), "waypoint %zu atteint", waypoint_index_);
          ++waypoint_index_;
          if (waypoint_index_ >= waypoints_.size()) {
            startLanding();
          } else {
            publishCurrentWaypoint();
            phase_start_ = now();  // relancer le compteur pour le waypoint suivant
          }
        }
        break;

      case Phase::LANDING:
        if (telemetry_.altitude_relative_m < 0.3f && !telemetry_.armed) {
          requestSafety(SafetyState::TERMINATED, "pose et desarme");
          setPhase(Phase::DONE, "DONE");
          publishEvent(
            dronoto_msgs::msg::SafetyEvent::INFO, "MISSION_COMPLETE",
            "mission terminee avec succes");
        }
        break;

      case Phase::DONE:
      case Phase::ABORTED:
        break;
    }
  }

  void startLanding()
  {
    requestSafety(SafetyState::LANDING, "waypoints termines");
    callTrigger(land_client_);
    setPhase(Phase::LANDING, "LANDING");
  }

  /// Publie un objectif a la position courante avant de basculer en offboard.
  /// Sans cela, le premier setpoint offboard pourrait etre tres eloigne et
  /// provoquer une embardee.
  void publishFirstGoalAtCurrentPosition()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "odom";
    goal.pose = telemetry_.pose;
    goal_pub_->publish(goal);
  }

  void publishCurrentWaypoint()
  {
    auto wp = waypoints_[waypoint_index_];
    wp.header.stamp = now();
    goal_pub_->publish(wp);
    have_distance_ = false;  // ne pas reutiliser la distance de l'objectif precedent
    RCLCPP_INFO(
      get_logger(), "waypoint %zu/%zu : (%.1f, %.1f, %.1f)", waypoint_index_ + 1,
      waypoints_.size(), wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
  }

  void requestSafety(SafetyState target, const std::string & reason)
  {
    if (!safety_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "service safety/trigger_failsafe indisponible");
      return;
    }
    auto req = std::make_shared<dronoto_msgs::srv::TriggerFailsafe::Request>();
    req->target_state = static_cast<uint8_t>(target);
    req->reason = reason;
    safety_client_->async_send_request(req);
  }

  void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "service %s indisponible", client->get_service_name());
      return;
    }
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void publishEvent(uint8_t severity, const std::string & code, const std::string & message)
  {
    dronoto_msgs::msg::SafetyEvent ev;
    ev.header.stamp = now();
    ev.severity = severity;
    ev.code = code;
    ev.message = message;
    event_pub_->publish(ev);
  }

  Phase phase_{Phase::WAIT_TELEMETRY};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

  dronoto_msgs::msg::VehicleTelemetry telemetry_{};
  dronoto_msgs::msg::SafetyState safety_{};
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t waypoint_index_{0};
  float distance_to_goal_{1e9f};
  bool have_telemetry_{false};
  bool have_distance_{false};

  double takeoff_altitude_{5.0};
  double waypoint_tolerance_{0.8};
  double phase_timeout_s_{60.0};
  bool autostart_{true};

  rclcpp::Subscription<dronoto_msgs::msg::VehicleTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<dronoto_msgs::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Publisher<dronoto_msgs::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr offboard_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
  rclcpp::Client<dronoto_msgs::srv::TriggerFailsafe>::SharedPtr safety_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExecutiveNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Étape 4 : Construire**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dronoto_mission
```

Attendu : compilation réussie.

- [ ] **Étape 5 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_mission
git commit -m "feat(mission): séquenceur décollage, waypoints, atterrissage"
```

---

### Tâche 13 : `dronoto_bringup` — URDF, TF et fichiers de lancement

`bringup_drone.launch.py` est **le fichier qui migrera tel quel sur le Jetson**. Le fait
qu'il ne référence ni Gazebo, ni PX4 SITL, ni aucun outil de simulation n'est pas une
intention : c'est vérifié par un test (tâche 16).

**Fichiers :**
- Créer : `drone/ros2/dronoto_bringup/package.xml`
- Créer : `drone/ros2/dronoto_bringup/CMakeLists.txt`
- Créer : `drone/ros2/dronoto_bringup/urdf/x500_dronoto.urdf.xacro`
- Créer : `drone/ros2/dronoto_bringup/config/drone_params.yaml`
- Créer : `drone/ros2/dronoto_bringup/launch/bringup_drone.launch.py`
- Créer : `drone/ros2/dronoto_bringup/launch/bringup_sim.launch.py`
- Créer : `drone/ros2/dronoto_bringup/rviz/dronoto.rviz`

- [ ] **Étape 1 : Créer `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dronoto_bringup</name>
  <version>0.1.0</version>
  <description>Lancement et configuration du système Dronoto.</description>
  <maintainer email="ynebocin@gmail.com">Nicolas BENY</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>dronoto_interface</exec_depend>
  <exec_depend>dronoto_safety</exec_depend>
  <exec_depend>dronoto_navigation</exec_depend>
  <exec_depend>dronoto_mission</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Étape 2 : Créer `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(dronoto_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Étape 3 : Créer l'URDF**

`urdf/x500_dronoto.urdf.xacro` — la géométrie des capteurs. Les positions correspondent à
la configuration matérielle cible de
[`16-materiel.md`](../../architecture/16-materiel.md) ; les capteurs eux-mêmes sont
branchés en P2 mais leurs repères existent dès maintenant, ce qui évite de rejouer les
calibrations plus tard.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x500_dronoto">

  <link name="base_link">
    <visual>
      <geometry><box size="0.35 0.35 0.10"/></geometry>
      <material name="carbon"><color rgba="0.15 0.15 0.18 1.0"/></material>
    </visual>
  </link>

  <!-- IMU : dans le contrôleur de vol, au centre -->
  <link name="imu_link"/>
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR Livox Mid-360, monté au-dessus du corps (branché en P2) -->
  <link name="lidar_link"/>
  <joint name="base_to_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.10 0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- Caméra avant, inclinée de 15 degrés vers le bas (branchée en P2) -->
  <link name="cam_front_link"/>
  <joint name="base_to_cam_front" type="fixed">
    <parent link="base_link"/>
    <child link="cam_front_link"/>
    <origin xyz="0.12 0 0.02" rpy="0 0.2618 0"/>
  </joint>

  <!-- Antenne GNSS, sur mât -->
  <link name="gnss_link"/>
  <joint name="base_to_gnss" type="fixed">
    <parent link="base_link"/>
    <child link="gnss_link"/>
    <origin xyz="-0.08 0 0.15" rpy="0 0 0"/>
  </joint>

</robot>
```

- [ ] **Étape 4 : Créer la configuration**

`config/drone_params.yaml` :

```yaml
px4_interface:
  ros__parameters:
    odom_frame: "odom"
    base_frame: "base_link"
    setpoint_stale_warn_s: 0.1
    setpoint_stale_hold_s: 0.5

safety_supervisor:
  ros__parameters:
    telemetry_timeout_s: 1.0
    setpoint_timeout_s: 0.5
    battery_critical_ratio: 0.10
    battery_return_ratio: 0.25

trajectory_follower:
  ros__parameters:
    max_speed_ms: 4.0
    max_climb_ms: 2.0
    goal_tolerance_m: 0.5
    slowdown_radius_m: 3.0

mission_executive:
  ros__parameters:
    takeoff_altitude_m: 5.0
    waypoint_tolerance_m: 0.8
    phase_timeout_s: 60.0
    autostart: false
    waypoints: []
```

`autostart: false` par défaut : lancer la pile ne doit **jamais** faire décoller un
aéronef par surprise. Les scénarios de test l'activent explicitement.

- [ ] **Étape 5 : Créer `bringup_drone.launch.py`**

```python
"""Lancement des nœuds embarqués Dronoto.

Ce fichier ne référence AUCUN outil de simulation : il migrera tel quel sur le
calculateur embarqué. La propriété est vérifiée par
tests/conformance/test_bringup_no_sim_deps.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("dronoto_bringup")
    default_params = os.path.join(share, "config", "drone_params.yaml")
    urdf_xacro = os.path.join(share, "urdf", "x500_dronoto.urdf.xacro")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Fichier de paramètres des nœuds embarqués.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="false",
            description="Démarrer la mission automatiquement. false par défaut : "
                        "lancer la pile ne doit jamais faire décoller un aéronef.",
        ),

        # --- description du robot et repères statiques ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_xacro]),
            }],
        ),
        # map -> odom : identité en P1. Le graphe de poses la publiera en P4.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),

        # --- chaîne de vol ---
        Node(
            package="dronoto_interface",
            executable="px4_interface_node",
            name="px4_interface",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="dronoto_navigation",
            executable="trajectory_follower_node",
            name="trajectory_follower",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="dronoto_safety",
            executable="safety_supervisor_node",
            name="safety_supervisor",
            parameters=[params_file],
            # En P1, aucun évitement réactif ne s'intercale : le superviseur lit
            # directement la sortie du suivi de trajectoire. En P3, ce remappage
            # disparaît et reactive_avoidance produit control/setpoint_safe.
            remappings=[("control/setpoint_safe", "control/setpoint_raw")],
            output="screen",
        ),
        Node(
            package="dronoto_mission",
            executable="mission_executive_node",
            name="mission_executive",
            parameters=[params_file, {"autostart": autostart}],
            output="screen",
        ),
    ])
```

- [ ] **Étape 6 : Créer `bringup_sim.launch.py`**

```python
"""Lancement complet en simulation : PX4 SITL + Gazebo + agent + nœuds embarqués.

Sépare délibérément la simulation (ce fichier) des nœuds embarqués
(bringup_drone.launch.py), qui restent réutilisables tels quels sur matériel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("dronoto_bringup")
    repo_root = os.environ.get(
        "DRONOTO_REPO", os.path.expanduser("~/Documents/GitHub/Dronoto")
    )
    run_sim = os.path.join(repo_root, "infrastructure", "scripts", "run_sim.sh")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "drone_params.yaml"),
        ),

        ExecuteProcess(
            cmd=["bash", run_sim],
            additional_env={"HEADLESS": headless},
            output="screen",
            name="px4_gazebo",
        ),

        # Laisser PX4 et Gazebo démarrer avant les nœuds embarqués : sans ce délai,
        # px4_interface attend simplement, mais les journaux sont illisibles.
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(share, "launch", "bringup_drone.launch.py")
                    ),
                    launch_arguments={
                        "autostart": autostart,
                        "params_file": params_file,
                    }.items(),
                ),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "dronoto.rviz")],
            condition=IfCondition(use_rviz),
        ),
    ])
```

- [ ] **Étape 7 : Créer la configuration RViz**

`rviz/dronoto.rviz` :

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Global Options:
    Fixed Frame: odom
    Frame Rate: 30
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 5
      Plane Cell Count: 40
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Enabled: true
      Topic:
        Value: /state/odometry
        Reliability Policy: Best Effort
      Keep: 500
      Position Tolerance: 0.5
    - Class: rviz_default_plugins/Pose
      Name: Goal
      Enabled: true
      Topic:
        Value: /navigation/goal
        Durability Policy: Transient Local
      Color: 0; 255; 0
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 40
      Pitch: 0.6
```

- [ ] **Étape 8 : Construire et lancer la pile complète**

```bash
cd ~/dronoto_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

export DRONOTO_REPO=~/Documents/GitHub/Dronoto
ros2 launch dronoto_bringup bringup_sim.launch.py autostart:=true \
  params_file:=$DRONOTO_REPO/drone/ros2/dronoto_bringup/config/drone_params.yaml
```

Attendu, dans l'ordre : Gazebo s'ouvre, RViz s'ouvre, puis les journaux affichent la
progression des phases `PREFLIGHT` → `ARMING` → `TAKEOFF` → `ENTER_OFFBOARD` →
`WAYPOINTS` → `LANDING` → `DONE`.

**Le drone décolle, monte à 5 m, redescend et se pose** (aucun waypoint n'est configuré par
défaut, donc la phase `WAYPOINTS` enchaîne immédiatement sur l'atterrissage).

Dans RViz, la trajectoire s'affiche et l'arbre TF `map → odom → base_link → capteurs` est
complet.

- [ ] **Étape 9 : Vérifier le vol avec waypoints**

```bash
ros2 launch dronoto_bringup bringup_sim.launch.py autostart:=true \
  params_file:=$DRONOTO_REPO/drone/ros2/dronoto_bringup/config/drone_params.yaml \
  --ros-args -p mission_executive.waypoints:="[10.0, 0.0, 5.0, 10.0, 10.0, 5.0, 0.0, 10.0, 5.0, 0.0, 0.0, 5.0]"
```

> Si la surcharge de paramètre en ligne de commande ne s'applique pas au nœud inclus,
> éditer directement `waypoints` dans `config/drone_params.yaml`. La tâche 15 rend cela
> propre via les fichiers de scénario.

Attendu : le drone parcourt un carré de 10 m à 5 m d'altitude, puis atterrit.

- [ ] **Étape 10 : Commit**

```bash
cd ~/Documents/GitHub/Dronoto
git add drone/ros2/dronoto_bringup
git commit -m "feat(bringup): URDF, TF, configuration et fichiers de lancement drone et simulation"
```

---

### Tâche 14 : Moteur de scénarios de test

Le harnais qui transforme « le drone a l'air de voler » en « le drone vole, prouvé
automatiquement, dix fois de suite ». C'est le livrable qui rend P1 vérifiable, et le socle
sur lequel P6 branchera l'injection de pannes.

**Fichiers :**
- Créer : `tests/runner/__init__.py`
- Créer : `tests/runner/sim_process.py`
- Créer : `tests/runner/scenario_runner.py`
- Créer : `tests/test_scenarios.py`
- Créer : `tests/conftest.py`

- [ ] **Étape 1 : Créer `tests/runner/__init__.py`**

```python
"""Harnais d'exécution des scénarios de test SITL (niveau L3)."""
```

- [ ] **Étape 2 : Créer `tests/runner/sim_process.py`**

```python
"""Gestion du cycle de vie de la pile simulée pour un scénario.

Chaque scénario tourne dans son propre ROS_DOMAIN_ID pour permettre l'exécution
parallèle de plusieurs simulations sans collision de découverte DDS.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path


def repo_root() -> Path:
    """Racine du dépôt, déduite de l'emplacement de ce fichier."""
    return Path(__file__).resolve().parents[2]


@dataclass
class SimStack:
    """Pile simulée : PX4 + Gazebo + agent uXRCE-DDS + nœuds embarqués."""

    domain_id: int = 0
    headless: bool = True
    params_file: Path | None = None
    autostart: bool = True
    startup_timeout_s: float = 90.0
    _procs: list[subprocess.Popen] = field(default_factory=list)

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        env["HEADLESS"] = "1" if self.headless else "0"
        env["DRONOTO_REPO"] = str(repo_root())
        # Ports PX4 dérivés du domaine : deux simulations ne doivent jamais
        # partager un port, sinon la seconde échoue de façon très obscure.
        env["PX4_UXRCE_DDS_PORT"] = str(8888 + self.domain_id)
        env["GZ_PARTITION"] = f"dronoto{self.domain_id}"
        # LIMITE CONNUE : isoler completement deux simulations concurrentes demande
        # aussi de configurer le port cote CLIENT PX4 (parametre UXRCE_DDS_PRT), ce
        # qui n'est pas fait ici. En P1 les scenarios s'executent en serie, donc
        # domain_id vaut toujours 0. Le parallelisme reel arrive en P6 avec les
        # campagnes Monte-Carlo, ou cette limite devra etre levee.
        return env

    def start(self) -> None:
        env = self._env()

        sim = subprocess.Popen(
            ["bash", str(repo_root() / "infrastructure" / "scripts" / "run_sim.sh")],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._procs.append(sim)
        time.sleep(10.0)  # laisser PX4 et Gazebo s'initialiser

        cmd = [
            "ros2", "launch", "dronoto_bringup", "bringup_drone.launch.py",
            f"autostart:={'true' if self.autostart else 'false'}",
        ]
        if self.params_file is not None:
            cmd.append(f"params_file:={self.params_file}")

        drone = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._procs.append(drone)

    def stop(self) -> None:
        # Tuer le groupe de processus : PX4 et Gazebo lancent des enfants qui
        # survivraient à un kill du seul parent et bloqueraient l'exécution suivante.
        for proc in reversed(self._procs):
            if proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                continue
        time.sleep(2.0)
        for proc in reversed(self._procs):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()
        # Filet de sécurité : des processus orphelins rendent le test suivant
        # non reproductible, ce qui est pire qu'un échec franc.
        subprocess.run(["pkill", "-9", "-f", "px4_sitl_default/bin/px4"], check=False)
        subprocess.run(["pkill", "-9", "-f", "gz sim"], check=False)
        subprocess.run(["pkill", "-9", "-f", "MicroXRCEAgent"], check=False)

    def __enter__(self) -> SimStack:
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()
```

- [ ] **Étape 3 : Créer `tests/runner/scenario_runner.py`**

```python
"""Exécution d'un scénario YAML et évaluation de ses assertions.

Le DSL de P1 est volontairement restreint : phases, assertions instantanées,
assertions continues et assertions finales. P6 y ajoute l'injection de pannes,
sans changer la structure du fichier.
"""

from __future__ import annotations

import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from dronoto_msgs.msg import SafetyEvent, SafetyState, VehicleTelemetry
from std_msgs.msg import String

from .sim_process import SimStack, repo_root

_STATE_NAMES = {
    0: "BOOT", 1: "IDLE", 2: "PREFLIGHT", 3: "READY", 4: "ARMED", 5: "TAKEOFF",
    6: "NOMINAL", 7: "DEGRADED", 8: "HOLD", 9: "RETURNING", 10: "LANDING",
    11: "EMERGENCY_LAND", 12: "TERMINATED",
}


@dataclass
class Observation:
    """Instantané de l'état du système, échantillonné en continu."""

    t: float
    altitude_m: float
    battery: float
    armed: bool
    safety_state: str
    position: tuple[float, float, float]


class ScenarioObserver(Node):
    """Nœud ROS 2 qui observe la mission sans jamais y intervenir."""

    def __init__(self) -> None:
        super().__init__("scenario_observer")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        events_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.observations: list[Observation] = []
        self.events: list[str] = []
        self.phase: str = ""
        self._t0: float | None = None
        self._telemetry: VehicleTelemetry | None = None
        self._safety_state: int = 0

        self.create_subscription(
            VehicleTelemetry, "state/telemetry", self._on_telemetry, 10
        )
        self.create_subscription(SafetyState, "safety/state", self._on_safety, latched)
        self.create_subscription(SafetyEvent, "safety/events", self._on_event, events_qos)
        self.create_subscription(String, "mission/phase", self._on_phase, latched)
        self.create_timer(0.1, self._sample)

    def _on_telemetry(self, msg: VehicleTelemetry) -> None:
        self._telemetry = msg
        if self._t0 is None:
            self._t0 = time.monotonic()

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _on_event(self, msg: SafetyEvent) -> None:
        self.events.append(msg.code)

    def _on_phase(self, msg: String) -> None:
        self.phase = msg.data

    def _sample(self) -> None:
        if self._telemetry is None or self._t0 is None:
            return
        t = self._telemetry
        self.observations.append(
            Observation(
                t=time.monotonic() - self._t0,
                altitude_m=float(t.altitude_relative_m),
                battery=float(t.battery_remaining),
                armed=bool(t.armed),
                safety_state=_STATE_NAMES.get(self._safety_state, "?"),
                position=(t.pose.position.x, t.pose.position.y, t.pose.position.z),
            )
        )

    @property
    def elapsed(self) -> float:
        return 0.0 if self._t0 is None else time.monotonic() - self._t0

    def latest(self) -> Observation | None:
        return self.observations[-1] if self.observations else None


@dataclass
class ScenarioResult:
    """Résultat d'une exécution : verdict, échecs et métriques."""

    name: str
    passed: bool
    failures: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)


def _check(label: str, value: Any, constraint: Any) -> str | None:
    """Retourne un message d'échec, ou None si la contrainte est satisfaite."""
    if isinstance(constraint, dict):
        if "min" in constraint and value < constraint["min"]:
            return f"{label} = {value!r} < min {constraint['min']!r}"
        if "max" in constraint and value > constraint["max"]:
            return f"{label} = {value!r} > max {constraint['max']!r}"
        if "eq" in constraint and value != constraint["eq"]:
            return f"{label} = {value!r} != {constraint['eq']!r}"
        return None
    if value != constraint:
        return f"{label} = {value!r} != {constraint!r}"
    return None


def _write_params(scenario: dict) -> Path | None:
    """Matérialise la section 'params' du scénario en fichier de paramètres ROS 2."""
    params = scenario.get("params")
    if not params:
        return None
    tmp = Path(tempfile.mkdtemp(prefix="dronoto_scn_")) / "params.yaml"
    tmp.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
    return tmp


def run_scenario(path: Path, domain_id: int = 0) -> ScenarioResult:
    """Exécute un scénario et retourne son résultat."""
    scenario = yaml.safe_load(path.read_text(encoding="utf-8"))
    name = scenario.get("name", path.stem)
    timeout_s = float(scenario.get("timeout_s", 300))
    failures: list[str] = []
    # Initialise avant le try : le return final y accede meme si la pile
    # n'a jamais demarre, ce qui doit produire un echec lisible et non un NameError.
    metrics: dict[str, Any] = {}

    params_file = _write_params(scenario)
    rclpy.init(args=None)
    observer = ScenarioObserver()

    stack = SimStack(
        domain_id=domain_id,
        headless=bool(scenario.get("headless", True)),
        params_file=params_file,
        autostart=True,
    )

    try:
        stack.start()

        pending = sorted(
            scenario.get("timeline", []), key=lambda step: float(step["at"])
        )
        continuous = scenario.get("assert_continuous", {})
        deadline = time.monotonic() + timeout_s + 30.0  # marge de démarrage

        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            now = observer.elapsed
            latest = observer.latest()
            if latest is None:
                continue

            # Assertions continues : évaluées à chaque échantillon.
            for key, constraint in continuous.items():
                msg = _check(key, getattr(latest, key), constraint)
                if msg:
                    failures.append(f"[continu t={now:.1f}s] {msg}")

            # Assertions ponctuelles de la chronologie.
            while pending and now >= float(pending[0]["at"]):
                step = pending.pop(0)
                for key, constraint in step.get("assert", {}).items():
                    value = observer.phase if key == "phase" else getattr(latest, key)
                    msg = _check(key, value, constraint)
                    if msg:
                        failures.append(f"[t={step['at']}s] {msg}")

            if observer.phase in ("DONE", "ABORTED"):
                break

        # --- assertions finales ---
        latest = observer.latest()
        altitudes = [o.altitude_m for o in observer.observations]
        metrics = {
            "duration_s": round(observer.elapsed, 1),
            "max_altitude_m": round(max(altitudes), 2) if altitudes else 0.0,
            "final_phase": observer.phase,
            "samples": len(observer.observations),
            "events": observer.events,
        }
        if latest is not None:
            metrics["final_altitude_m"] = round(latest.altitude_m, 2)
            metrics["final_position"] = [round(v, 2) for v in latest.position]
            metrics["distance_to_home_m"] = round(
                (latest.position[0] ** 2 + latest.position[1] ** 2) ** 0.5, 2
            )
            metrics["final_battery"] = round(latest.battery, 3)
            metrics["final_armed"] = latest.armed

        for key, constraint in scenario.get("final_assertions", {}).items():
            if key == "phase":
                value = observer.phase
            elif key == "events_contain":
                missing = [c for c in constraint if c not in observer.events]
                if missing:
                    failures.append(f"[final] événements absents : {missing}")
                continue
            elif key in metrics:
                value = metrics[key]
            elif latest is not None and hasattr(latest, key):
                value = getattr(latest, key)
            else:
                failures.append(f"[final] métrique inconnue : {key}")
                continue
            msg = _check(key, value, constraint)
            if msg:
                failures.append(f"[final] {msg}")

        if not observer.observations:
            failures.append("aucune télémétrie reçue : la pile n'a pas démarré")

    finally:
        stack.stop()
        observer.destroy_node()
        rclpy.shutdown()

    return ScenarioResult(
        name=name, passed=not failures, failures=failures, metrics=metrics
    )


def scenarios_dir() -> Path:
    return repo_root() / "tests" / "scenarios"
```

- [ ] **Étape 4 : Créer `tests/conftest.py`**

```python
"""Configuration pytest partagée."""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def pytest_configure(config):
    if "AMENT_PREFIX_PATH" not in os.environ:
        raise RuntimeError(
            "Environnement ROS 2 non sourcé. Lancer :\n"
            "  source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash"
        )
```

- [ ] **Étape 5 : Créer `tests/test_scenarios.py`**

```python
"""Exécution des scénarios SITL (niveau L3).

Lents et gourmands : marqués `scenario` pour être exclus des exécutions rapides.
    pytest -m "not scenario"     # rapide
    pytest -m scenario           # scénarios complets
"""

import json
from pathlib import Path

import pytest

from tests.runner.scenario_runner import run_scenario, scenarios_dir

SCENARIOS = sorted(scenarios_dir().glob("*.yaml"))


@pytest.mark.scenario
@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda p: p.stem)
def test_scenario(scenario_path: Path, tmp_path: Path) -> None:
    result = run_scenario(scenario_path)

    report = tmp_path / f"{scenario_path.stem}.json"
    report.write_text(
        json.dumps(
            {"name": result.name, "passed": result.passed,
             "failures": result.failures, "metrics": result.metrics},
            indent=2,
        ),
        encoding="utf-8",
    )

    assert result.passed, (
        f"{result.name} a échoué :\n  "
        + "\n  ".join(result.failures)
        + f"\n\nMétriques : {json.dumps(result.metrics, indent=2)}"
    )
```

- [ ] **Étape 6 : Vérifier que le harnais se charge**

```bash
cd ~/Documents/GitHub/Dronoto
source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash
python3 -c "from tests.runner.scenario_runner import run_scenario; print('harnais ok')"
pytest --collect-only -m scenario
```

Attendu : `harnais ok`, puis `no tests ran` (aucun scénario n'existe encore — c'est la
tâche 15).

- [ ] **Étape 7 : Commit**

```bash
git add tests/
git commit -m "test: moteur d'exécution des scénarios SITL avec DSL YAML"
```

---

### Tâche 15 : Scénarios `takeoff_land` et `waypoint_navigation`

Les deux scénarios qui constituent le critère de sortie de P1.

**Fichiers :**
- Créer : `tests/scenarios/takeoff_land.yaml`
- Créer : `tests/scenarios/waypoint_navigation.yaml`

- [ ] **Étape 1 : Créer `takeoff_land.yaml`**

```yaml
name: "Décollage, stationnaire, atterrissage"
description: >
  Vérifie la séquence de vol de base : vérifications préalables, armement,
  décollage à 5 m, bascule en offboard, atterrissage et désarmement.
  C'est le scénario le plus fondamental du projet : s'il échoue, rien d'autre
  n'a de sens.

headless: true
timeout_s: 180

params:
  mission_executive:
    ros__parameters:
      autostart: true
      takeoff_altitude_m: 5.0
      waypoint_tolerance_m: 0.8
      phase_timeout_s: 60.0
      waypoints: []
  trajectory_follower:
    ros__parameters:
      max_speed_ms: 4.0
      max_climb_ms: 2.0
      goal_tolerance_m: 0.5
      slowdown_radius_m: 3.0
  safety_supervisor:
    ros__parameters:
      telemetry_timeout_s: 1.0
      setpoint_timeout_s: 0.5
      battery_critical_ratio: 0.10
      battery_return_ratio: 0.25
  px4_interface:
    ros__parameters:
      odom_frame: "odom"
      base_frame: "base_link"
      setpoint_stale_warn_s: 0.1
      setpoint_stale_hold_s: 0.5

timeline:
  - at: 5
    assert:
      # Au sol, avant le décollage : la batterie doit être quasi pleine.
      battery: {min: 0.90}

  - at: 25
    assert:
      altitude_m: {min: 4.0}
      armed: true

assert_continuous:
  # Aucun dépassement d'altitude : le contrôleur ne doit pas osciller.
  altitude_m: {max: 8.0}
  # La batterie ne doit jamais approcher le seuil critique sur un vol si court ;
  # si c'est le cas, le modèle énergétique ou la mission ont un problème.
  battery: {min: 0.60}

final_assertions:
  phase: "DONE"
  final_altitude_m: {max: 0.5}
  final_armed: false
  max_altitude_m: {min: 4.5, max: 8.0}
  distance_to_home_m: {max: 3.0}
  events_contain:
    - "MISSION_COMPLETE"
```

- [ ] **Étape 2 : Lancer le scénario**

```bash
cd ~/Documents/GitHub/Dronoto
source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash
pytest -m scenario -k takeoff_land -v
```

Attendu : `PASSED` en 60 à 120 secondes.

En cas d'échec, le message affiche les assertions violées **et** les métriques complètes.
Diagnostics les plus fréquents :

| Symptôme | Cause probable |
|---|---|
| « aucune télémétrie reçue » | La pile n'a pas démarré : relancer manuellement `bringup_sim.launch.py` et lire les journaux |
| Phase bloquée en `PREFLIGHT` | `preflight_ok` reste faux : vérifier `ros2 topic echo /fmu/out/vehicle_status` |
| Phase bloquée en `ENTER_OFFBOARD` | PX4 refuse la bascule : le flux `offboard_control_mode` n'atteint pas 2 Hz, ou aucune consigne valide ne l'accompagne |
| `final_armed: true` | Le désarmement automatique après atterrissage n'a pas eu lieu : augmenter `timeout_s` |
| Processus orphelins entre deux exécutions | `pkill -9 -f px4_sitl` puis relancer ; vérifier `SimStack.stop()` |

- [ ] **Étape 3 : Créer `waypoint_navigation.yaml`**

```yaml
name: "Navigation par waypoints"
description: >
  Parcours d'un carré de 20 m de côté à 6 m d'altitude, puis retour au point de
  départ et atterrissage. Vérifie la précision d'arrivée et l'absence de
  dépassement.

headless: true
timeout_s: 360

params:
  mission_executive:
    ros__parameters:
      autostart: true
      takeoff_altitude_m: 6.0
      waypoint_tolerance_m: 0.8
      phase_timeout_s: 90.0
      # Carré de 20 m parcouru dans le sens trigonométrique, retour à l'origine.
      waypoints: [20.0,  0.0, 6.0,
                  20.0, 20.0, 6.0,
                   0.0, 20.0, 6.0,
                   0.0,  0.0, 6.0]
  trajectory_follower:
    ros__parameters:
      max_speed_ms: 4.0
      max_climb_ms: 2.0
      goal_tolerance_m: 0.5
      slowdown_radius_m: 3.0
  safety_supervisor:
    ros__parameters:
      telemetry_timeout_s: 1.0
      setpoint_timeout_s: 0.5
      battery_critical_ratio: 0.10
      battery_return_ratio: 0.25
  px4_interface:
    ros__parameters:
      odom_frame: "odom"
      base_frame: "base_link"
      setpoint_stale_warn_s: 0.1
      setpoint_stale_hold_s: 0.5

timeline:
  - at: 30
    assert:
      altitude_m: {min: 5.0}
      safety_state: "NOMINAL"

assert_continuous:
  altitude_m: {max: 10.0}
  battery: {min: 0.40}

final_assertions:
  phase: "DONE"
  final_altitude_m: {max: 0.5}
  final_armed: false
  # Critère de sortie P1 : précision d'arrivée au waypoint meilleure que 0,5 m.
  # Le dernier waypoint est l'origine, donc la distance finale la mesure.
  distance_to_home_m: {max: 0.5}
  max_altitude_m: {min: 5.5, max: 10.0}
  events_contain:
    - "MISSION_COMPLETE"
```

- [ ] **Étape 4 : Lancer le scénario**

```bash
pytest -m scenario -k waypoint_navigation -v
```

Attendu : `PASSED` en 3 à 6 minutes.

> **Si `distance_to_home_m` dépasse 0,5 m**, c'est le réglage du suivi de trajectoire, pas
> un bug : réduire `slowdown_radius_m` (approche plus franche) ou `waypoint_tolerance_m`
> (le séquenceur passe au waypoint suivant trop tôt). Le critère de sortie P1 de
> [`17-roadmap.md`](../../architecture/17-roadmap.md) est explicitement de 0,5 m — c'est
> le moment de le tenir.

- [ ] **Étape 5 : Ajouter l'action `kill_node` au moteur de scénarios**

Le troisième critère de sortie de P1 exige de vérifier qu'en tuant `trajectory_follower`
en vol, le drone passe en `HOLD` en moins de 500 ms. C'est le test qui prouve que la
politique de consigne périmée fonctionne — autrement dit, qu'un plantage du planificateur
n'est pas fatal.

Dans `tests/runner/scenario_runner.py`, ajouter l'import :

```python
import subprocess
```

puis, juste avant `def run_scenario(`, la fonction :

```python
def _kill_node(executable: str) -> None:
    """Tue brutalement un nœud pour simuler son plantage.

    P1 n'a pas encore d'injecteur de pannes (P6) ; un SIGKILL sur le processus
    est la simulation la plus fidèle d'un plantage, et la plus simple.
    """
    subprocess.run(["pkill", "-9", "-f", executable], check=False)
```

et, dans la boucle de la chronologie, juste après `step = pending.pop(0)` :

```python
                if "kill_node" in step:
                    _kill_node(step["kill_node"])
```

- [ ] **Étape 6 : Créer `stale_setpoint_hold.yaml`**

`tests/scenarios/stale_setpoint_hold.yaml` :

```yaml
name: "Plantage du suivi de trajectoire : passage en HOLD"
description: >
  Tue trajectory_follower en plein vol et vérifie que le drone se stabilise au
  lieu de partir en vrille. Prouve la politique de consigne périmée de
  px4_interface et la condition STALE_SETPOINT du superviseur.
  Troisième critère de sortie de P1.

headless: true
timeout_s: 240

params:
  mission_executive:
    ros__parameters:
      autostart: true
      takeoff_altitude_m: 8.0
      waypoint_tolerance_m: 0.8
      phase_timeout_s: 120.0
      # Un waypoint lointain : le drone sera en route au moment du plantage.
      waypoints: [60.0, 0.0, 8.0]
  trajectory_follower:
    ros__parameters:
      max_speed_ms: 4.0
      max_climb_ms: 2.0
      goal_tolerance_m: 0.5
      slowdown_radius_m: 3.0
  safety_supervisor:
    ros__parameters:
      telemetry_timeout_s: 1.0
      setpoint_timeout_s: 0.5
      battery_critical_ratio: 0.10
      battery_return_ratio: 0.25
  px4_interface:
    ros__parameters:
      odom_frame: "odom"
      base_frame: "base_link"
      setpoint_stale_warn_s: 0.1
      setpoint_stale_hold_s: 0.5

timeline:
  - at: 40
    assert:
      altitude_m: {min: 7.0}
      safety_state: "NOMINAL"

  - at: 45
    kill_node: "trajectory_follower_node"

  # 1,5 s apres le plantage : le superviseur DOIT avoir bascule.
  # Marge confortable sur le budget de 500 ms, pour absorber la periode
  # d'echantillonnage de l'observateur (100 ms) et celle du superviseur (100 ms).
  - at: 46.5
    assert:
      safety_state: "HOLD"

  # Le drone doit rester en l'air et stable, pas tomber ni deriver.
  - at: 60
    assert:
      safety_state: "HOLD"
      altitude_m: {min: 6.0, max: 10.0}

assert_continuous:
  altitude_m: {max: 12.0}

final_assertions:
  # La mission est interrompue par le delai de phase : c'est le resultat ATTENDU.
  # Ce qui compte est que le drone soit reste maitrise tout du long.
  events_contain:
    - "STALE_SETPOINT"
```

- [ ] **Étape 7 : Lancer le scénario**

```bash
pytest -m scenario -k stale_setpoint_hold -v
```

Attendu : `PASSED`. Le drone monte à 8 m, part vers le waypoint lointain, puis se fige en
vol stationnaire quand le suivi de trajectoire meurt.

Vérifier dans les métriques du rapport que l'événement `STALE_SETPOINT` est présent et que
l'altitude ne s'est jamais effondrée. **Si le drone descend ou dérive après le plantage**,
c'est que `px4_interface` republie la dernière consigne au lieu de maintenir la position
courante : relire l'étape 4 de la tâche 8.

- [ ] **Étape 8 : Commit**

```bash
git add tests/scenarios/ tests/runner/scenario_runner.py
git commit -m "test: scénarios SITL décollage/atterrissage, waypoints, et passage en HOLD sur plantage"
```

---

### Tâche 16 : Tests de conformité architecturale

Ces tests gardent les propriétés que le dossier d'architecture décrit. Une architecture qui
n'existe que dans un document dérive en quelques mois ; une architecture testée tient.

**Fichiers :**
- Créer : `tests/conformance/__init__.py`
- Créer : `tests/conformance/test_bringup_no_sim_deps.py`
- Créer : `tests/conformance/test_single_px4_boundary.py`

- [ ] **Étape 1 : Créer `tests/conformance/__init__.py`**

```python
"""Tests de conformité : vérifient que l'architecture n'a pas été violée."""
```

- [ ] **Étape 2 : Écrire le test de découplage simulation**

`tests/conformance/test_bringup_no_sim_deps.py` :

```python
"""bringup_drone.launch.py doit migrer tel quel sur le calculateur embarqué.

Propriété de docs/architecture/04-architecture-ros2.md, section 8 : le fichier
de lancement embarqué ne référence ni Gazebo, ni PX4 SITL, ni l'injecteur de
pannes. Ce test le vérifie, plutôt que de l'espérer.
"""

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
BRINGUP_DRONE = REPO / "drone/ros2/dronoto_bringup/launch/bringup_drone.launch.py"

FORBIDDEN = [
    "gz ",
    "gazebo",
    "px4_sitl",
    "run_sim.sh",
    "fault_injector",
    "sensor_faults",
    "PX4-Autopilot",
]


def test_bringup_drone_exists():
    assert BRINGUP_DRONE.is_file(), f"fichier introuvable : {BRINGUP_DRONE}"


@pytest.mark.parametrize("token", FORBIDDEN)
def test_bringup_drone_has_no_simulation_reference(token: str):
    content = BRINGUP_DRONE.read_text(encoding="utf-8").lower()
    # On ignore les lignes de commentaire : expliquer pourquoi la simulation est
    # absente est légitime, la référencer dans le code ne l'est pas.
    code = "\n".join(
        line for line in content.splitlines()
        if not line.strip().startswith("#")
    )
    assert token.lower() not in code, (
        f"bringup_drone.launch.py référence '{token}'. Ce fichier doit pouvoir "
        f"être lancé tel quel sur le Jetson : déplacer cette référence vers "
        f"bringup_sim.launch.py."
    )


def test_bringup_sim_includes_bringup_drone():
    """La simulation réutilise le lancement embarqué, elle ne le duplique pas."""
    sim = REPO / "drone/ros2/dronoto_bringup/launch/bringup_sim.launch.py"
    content = sim.read_text(encoding="utf-8")
    assert "bringup_drone.launch.py" in content, (
        "bringup_sim.launch.py doit inclure bringup_drone.launch.py. "
        "Dupliquer la liste des nœuds ferait diverger simulation et matériel."
    )
```

- [ ] **Étape 3 : Écrire le test de frontière PX4 unique**

`tests/conformance/test_single_px4_boundary.py` :

```python
"""Un seul nœud parle à PX4.

Propriété de docs/architecture/05-integration-px4.md : la connaissance des
conventions PX4 (repères NED, unités, sémantique des modes) est concentrée dans
px4_interface. Tout autre nœud qui s'abonnerait à /fmu/* la dupliquerait, et les
deux copies divergeraient.

Exception explicite : safety_supervisor lit /fmu/out/failsafe_flags. C'est
délibéré et documenté — le superviseur doit observer ce que PX4 a détecté sans
passer par un intermédiaire qui pourrait défaillir.
"""

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROS_PACKAGES = REPO / "drone/ros2"

ALLOWED = {
    "dronoto_interface": None,  # tous les topics /fmu/*
    "dronoto_safety": {"/fmu/out/failsafe_flags"},
}

FMU_TOPIC = re.compile(r'"(/fmu/(?:in|out)/[a-z_0-9]+)"')


def test_only_px4_interface_talks_to_px4():
    violations = []

    for source in ROS_PACKAGES.rglob("*.cpp"):
        package = source.relative_to(ROS_PACKAGES).parts[0]
        topics = set(FMU_TOPIC.findall(source.read_text(encoding="utf-8")))
        if not topics:
            continue

        if package not in ALLOWED:
            violations.append(f"{package} ({source.name}) utilise {sorted(topics)}")
            continue

        allowed = ALLOWED[package]
        if allowed is None:
            continue
        extra = topics - allowed
        if extra:
            violations.append(f"{package} ({source.name}) utilise en trop {sorted(extra)}")

    assert not violations, (
        "Seul px4_interface doit communiquer avec PX4 :\n  "
        + "\n  ".join(violations)
        + "\n\nSi une nouvelle exception est justifiée, l'ajouter à ALLOWED "
          "ET la documenter dans docs/architecture/05-integration-px4.md."
    )
```

- [ ] **Étape 4 : Lancer les tests de conformité**

```bash
cd ~/Documents/GitHub/Dronoto
source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash
pytest tests/conformance/ -v
```

Attendu : tous les tests passent. Ils s'exécutent en moins d'une seconde et n'ont besoin ni
de simulation ni de matériel — ils tourneront donc sur **chaque** pull request.

- [ ] **Étape 5 : Brancher les tests de conformité sur la CI**

Dans `.github/workflows/ci.yml`, ajouter un job après `lint` :

```yaml
  conformance:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - run: pip install pytest pyyaml
      - name: Tests de conformité architecturale
        env:
          AMENT_PREFIX_PATH: /tmp/fake  # conftest exige un environnement ROS
        run: pytest tests/conformance/ -v
```

- [ ] **Étape 6 : Commit**

```bash
git add tests/conformance/ .github/workflows/ci.yml
git commit -m "test: conformité architecturale (découplage simulation, frontière PX4 unique)"
```

---

### Tâche 17 : Conteneurisation de la simulation

La CI conteneurisée est ce qui garantit la reproductibilité. Le développement local reste
natif pour la vitesse d'itération —
[`13-docker-infra.md §1`](../../architecture/13-docker-infra.md) explique l'asymétrie.

**Fichiers :**
- Créer : `infrastructure/docker/base/Dockerfile`
- Créer : `infrastructure/docker/drone/Dockerfile`
- Créer : `infrastructure/compose/docker-compose.yml`
- Créer : `.dockerignore`

- [ ] **Étape 1 : Créer `.dockerignore`**

```
simulation/px4/PX4-Autopilot/build/
**/build/
**/install/
**/log/
.git/
docs/
models/**/*.onnx
```

- [ ] **Étape 2 : Créer l'image de base**

`infrastructure/docker/base/Dockerfile` :

```dockerfile
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      libeigen3-dev \
      git \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-eigen \
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages --no-cache-dir pyyaml pytest

WORKDIR /ws
```

- [ ] **Étape 3 : Créer l'image des nœuds embarqués**

`infrastructure/docker/drone/Dockerfile` :

```dockerfile
# --- étape de compilation ---
FROM dronoto-base:latest AS builder

RUN mkdir -p /ws/src && \
    git clone --depth 1 -b release/1.16 https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

COPY interfaces/ros_msgs/dronoto_msgs /ws/src/dronoto_msgs
COPY drone/ros2/dronoto_core        /ws/src/dronoto_core
COPY drone/ros2/dronoto_interface   /ws/src/dronoto_interface
COPY drone/ros2/dronoto_safety      /ws/src/dronoto_safety
COPY drone/ros2/dronoto_navigation  /ws/src/dronoto_navigation
COPY drone/ros2/dronoto_mission     /ws/src/dronoto_mission
COPY drone/ros2/dronoto_bringup     /ws/src/dronoto_bringup

RUN source /opt/ros/jazzy/setup.bash && \
    cd /ws && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# --- étape d'exécution ---
FROM ros:jazzy-ros-core AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-tf2-ros \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /opt/dronoto

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/dronoto/setup.bash'  >> /root/.bashrc

ENV DRONOTO_INSTALL=/opt/dronoto
CMD ["bash", "-lc", "ros2 launch dronoto_bringup bringup_drone.launch.py"]
```

- [ ] **Étape 4 : Créer le fichier Compose**

`infrastructure/compose/docker-compose.yml` :

```yaml
# Profils :
#   drone : nœuds embarqués seuls (nécessite PX4 SITL sur l'hôte)
#   test  : construction et exécution des tests unitaires en conteneur
#
# network_mode: host est INDISPENSABLE : la découverte DDS utilise le multicast,
# que le réseau bridge de Docker isole entre conteneurs — sans erreur visible.
# Voir docs/architecture/13-docker-infra.md, section 4.

services:

  drone:
    profiles: ["drone", "full"]
    build:
      context: ../..
      dockerfile: infrastructure/docker/drone/Dockerfile
    image: dronoto-drone:latest
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    command: >
      bash -lc "ros2 launch dronoto_bringup bringup_drone.launch.py autostart:=false"

  test:
    profiles: ["test"]
    build:
      context: ../..
      dockerfile: infrastructure/docker/drone/Dockerfile
      target: builder
    image: dronoto-drone-builder:latest
    network_mode: host
    command: >
      bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws &&
                colcon test --packages-select dronoto_core &&
                colcon test-result --verbose"
```

- [ ] **Étape 5 : Construire et vérifier**

```bash
cd ~/Documents/GitHub/Dronoto
docker build -t dronoto-base:latest -f infrastructure/docker/base/Dockerfile .
docker compose -f infrastructure/compose/docker-compose.yml --profile test build
docker compose -f infrastructure/compose/docker-compose.yml --profile test up \
  --abort-on-container-exit
```

Attendu : les tests unitaires de `dronoto_core` passent dans le conteneur.

```bash
docker images dronoto-drone --format '{{.Repository}} {{.Size}}'
```

Attendu : moins de 2,5 Go, conformément à la cible de
[`13-docker-infra.md §2`](../../architecture/13-docker-infra.md).

- [ ] **Étape 6 : Commit**

```bash
git add infrastructure/docker infrastructure/compose .dockerignore
git commit -m "build: images Docker et profils Compose pour les nœuds embarqués et les tests"
```

---

### Tâche 18 : Critère de sortie de P1

Une phase n'est pas terminée parce que le code est écrit, mais parce que la métrique est
atteinte. C'est l'exigence de
[`14-tests.md §10`](../../architecture/14-tests.md).

**Fichiers :**
- Créer : `infrastructure/scripts/run_p1_exit_criteria.sh`
- Modifier : `docs/architecture/17-roadmap.md`

- [ ] **Étape 1 : Écrire le script de validation**

`infrastructure/scripts/run_p1_exit_criteria.sh` :

```bash
#!/usr/bin/env bash
# Critere de sortie de P1 : les deux scenarios passent 10 fois de suite.
# Duree : 45 a 90 minutes.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.."
cd "$REPO_ROOT"

RUNS="${RUNS:-10}"
RESULTS="tests/results/p1_exit_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS"

pass=0
fail=0

for i in $(seq 1 "$RUNS"); do
  echo "===== execution $i/$RUNS ====="
  if pytest -m scenario -v --junitxml="$RESULTS/run_$i.xml" > "$RESULTS/run_$i.log" 2>&1; then
    pass=$((pass + 1))
    echo "  REUSSITE"
  else
    fail=$((fail + 1))
    echo "  ECHEC (voir $RESULTS/run_$i.log)"
  fi
  # Laisser les processus se terminer avant l'execution suivante :
  # un orphelin rendrait le resultat suivant non reproductible.
  sleep 5
done

echo
echo "===== resultat ====="
echo "  reussites : $pass / $RUNS"
echo "  echecs    : $fail / $RUNS"
echo "  rapports  : $RESULTS"

if [ "$fail" -eq 0 ]; then
  echo "  CRITERE DE SORTIE P1 : ATTEINT"
  exit 0
fi
echo "  CRITERE DE SORTIE P1 : NON ATTEINT"
exit 1
```

```bash
chmod +x infrastructure/scripts/run_p1_exit_criteria.sh
```

- [ ] **Étape 2 : Exécuter la validation**

```bash
source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash
./infrastructure/scripts/run_p1_exit_criteria.sh
```

Attendu : `reussites : 10 / 10` et `CRITERE DE SORTIE P1 : ATTEINT`.

**Si une seule exécution échoue, P1 n'est pas terminée.** Un test qui passe neuf fois sur
dix révèle une condition de course, un délai d'attente trop serré ou un processus orphelin
— exactement le genre de défaut qui deviendra ingérable en P4 quand la pile sera trois fois
plus grosse. Diagnostiquer maintenant coûte une journée ; le laisser passer coûte des
semaines de tests intermittents.

Pistes selon le mode d'échec observé :

| Mode d'échec | Piste |
|---|---|
| Échec toujours sur la même phase | Bug déterministe : reproduire en `headless: false` et observer |
| Échec aléatoire au démarrage | Délai d'initialisation trop court dans `SimStack.start()` |
| Échec après plusieurs exécutions | Processus orphelins : vérifier `ps aux \| grep -E "px4\|gz sim"` entre deux runs |
| `distance_to_home_m` juste au-dessus du seuil | Réglage du suivi de trajectoire, pas un défaut logiciel |

- [ ] **Étape 3 : Enregistrer les métriques de référence**

Créer `docs/guides/metriques-p1.md` avec les valeurs mesurées lors de la validation :

```markdown
# Métriques de référence — fin de P1

Mesurées le <date>, sur Ryzen 9 5950X / Ubuntu 24.04 / RX 6900 XT.

| Métrique | Valeur mesurée | Cible |
|---|---|---|
| Taux de réussite (10 exécutions) | <valeur> | 10/10 |
| Durée de `takeoff_land` | <valeur> s | < 180 s |
| Durée de `waypoint_navigation` | <valeur> s | < 360 s |
| Précision d'arrivée au waypoint | <valeur> m | < 0,5 m |
| Dépassement d'altitude au décollage | <valeur> m | < 1,0 m |
| Facteur temps réel de la simulation | <valeur> | > 1,0 |

Ces valeurs servent de référence de non-régression. Une dégradation notable
lors d'une phase ultérieure est un signal à investiguer, pas à accepter.
```

Remplir les valeurs à partir des rapports JSON produits par les scénarios.

- [ ] **Étape 4 : Marquer P1 comme terminée**

Dans `docs/architecture/17-roadmap.md`, section **P1**, ajouter juste après le titre :

```markdown
> **Statut : TERMINÉE** le <date>. Critères de sortie validés — voir
> [`docs/guides/metriques-p1.md`](../guides/metriques-p1.md).
```

- [ ] **Étape 5 : Commit final**

```bash
git add infrastructure/scripts/run_p1_exit_criteria.sh docs/guides/metriques-p1.md \
        docs/architecture/17-roadmap.md
git commit -m "test: critère de sortie P1 validé, métriques de référence enregistrées"
git push
```

---

## Ce qui est en place à la fin de ce plan

```
Un drone décolle, parcourt un carré de 20 m à 6 m d'altitude,
revient à moins de 0,5 m de son point de départ et se pose.
Sans intervention. Dix fois de suite. Prouvé par un test automatisé.
```

| Composant | État |
|---|---|
| Environnement reproductible | Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.16 épinglé |
| Contrats | `dronoto_msgs` : 4 messages, 1 service |
| Logique pure testée | Conversions de repères (10 tests), machine à états (17 tests) |
| Chaîne de vol | `px4_interface` → `trajectory_follower` → `safety_supervisor` → PX4 |
| Sécurité | Machine à états, veto sur consignes, politique de consigne périmée |
| Harnais de test | DSL YAML, moteur d'exécution, 3 scénarios, tests de conformité |
| CI | Lint, conformité, build, tests unitaires sur chaque PR |
| Docker | Images de base et embarquée, profils Compose |

## Ce qui n'est délibérément pas fait

Reporté, pas oublié. Chaque report est justifié dans la section « écarts assumés » ou dans
la phase concernée.

| Élément | Phase |
|---|---|
| LiDAR, caméra, capteurs de perception dans le SDF | P2 |
| SLAM, carte 3D, ESDF | P2 |
| Mondes maison (`pillars`, `urban_block`) | P3 |
| Planification globale, évitement réactif | P3 |
| BehaviorTree.CPP pour l'exécutif | P4 |
| Exploration autonome | P4 |
| Communication radio, serveur, dashboard | P5 |
| Injection de pannes, matrice de dégradation complète | P6 |
| Perception IA | P7 |

## Prochaine étape

Rédiger le plan d'implémentation de **P2 — Perception et SLAM**, qui est la phase la plus
incertaine du projet et où se joue la valeur technique réelle : intégration de FAST-LIO2,
injection de l'odométrie externe dans EKF2, calibration de `EKF2_EV_DELAY`, construction de
la carte OctoMap, et l'atteinte du budget de dérive (ATE < 0,5 m sur 500 m).
