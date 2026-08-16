# Guide d'installation

Environnement de développement Dronoto. Compter 1 à 3 heures, dont beaucoup
d'attente (compilation de PX4).

> **Ce guide décrit ce qui a réellement été exécuté**, y compris les écarts par
> rapport au plan initial. Les commandes sont celles qui ont fonctionné.

---

## 1. Système

**Cible : Ubuntu 24.04 LTS.** Deux options, par ordre de préférence.

### Option A — Linux natif (recommandé)

Dual-boot ou machine dédiée. C'est le choix de référence
([ADR-0002](../adr/0002-plateforme-de-developpement.md)) : rendu Gazebo direct,
découverte DDS multicast native, ROCm utilisable, pas de couche VM.

### Option B — WSL2 (fonctionne, avec des écarts)

```powershell
wsl --install -d Ubuntu-24.04
```

Écarts connus et mesurés sur cet environnement :

| Aspect | Impact |
|---|---|
| Rendu Gazebo | Traduction D3D12 via WSLg, plus lent qu'en natif |
| E/S sur `/mnt/c` | **~50× plus lent en lecture** que le système de fichiers WSL |
| ROCm (PyTorch GPU) | Non supporté sur RDNA2 — sans effet avant P7 |
| RAM | WSL n'obtient par défaut que ~50 % de la RAM hôte |

**Conséquences pratiques**, appliquées par les scripts de ce dépôt :

- Le workspace colcon vit dans `$HOME/dronoto_ws` (système de fichiers natif),
  pas sur `/mnt/c`. Seules les sources y sont lues, ce qui est négligeable.
- PX4 est cloné dans `$HOME/px4` et non dans le dépôt
  ([ADR-0008](../adr/0008-px4-hors-depot.md)).

Si la compilation de PX4 échoue avec une erreur de compilateur incompréhensible,
c'est probablement un OOM. Deux remèdes : réduire le parallélisme
(`PX4_BUILD_JOBS=2`) ou augmenter la RAM allouée à WSL en créant
`%USERPROFILE%\.wslconfig` :

```ini
[wsl2]
memory=24GB
processors=16
```

puis `wsl --shutdown` (ceci arrête **toutes** les distributions WSL en cours).

Vérifier :

```bash
lsb_release -a          # attendu : Ubuntu 24.04
nproc && free -g        # cœurs et RAM disponibles
```

---

## 2. ROS 2 Jazzy

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

Vérifier : `ros2 --version && echo $ROS_DISTRO` → `jazzy`

---

## 3. Gazebo Harmonic et dépendances de build

```bash
sudo apt install -y \
  ros-jazzy-ros-gz gz-harmonic \
  python3-colcon-common-extensions python3-vcstool \
  build-essential cmake git libeigen3-dev \
  ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-ament-cmake-gtest \
  ros-jazzy-vision-msgs
```

Vérifier :

```bash
gz sim --versions       # attendu : 8.x (Harmonic)
which colcon cmake
```

---

## 4. Agent Micro XRCE-DDS

Le pont entre PX4 et le graphe DDS de ROS 2. **Sans lui, PX4 démarre
normalement mais aucun topic `/fmu/*` n'apparaît** — panne silencieuse
classique.

```bash
sudo bash infrastructure/scripts/install_agent.sh
```

Vérifier : `which MicroXRCEAgent` → `/usr/local/bin/MicroXRCEAgent`

---

## 5. PX4-Autopilot

Cloné **hors du dépôt** ([ADR-0008](../adr/0008-px4-hors-depot.md)), à la
version verrouillée dans `simulation/px4/PX4_VERSION`.

```bash
# Sous WSL sans sudo NOPASSWD, lancer en root puis rendre la propriété :
#   wsl -d Ubuntu -u root -- bash infrastructure/scripts/install_px4.sh
#   sudo chown -R $USER:$USER ~/px4

export PX4_BUILD_JOBS=4      # borner le parallélisme si la RAM est limitée
bash infrastructure/scripts/install_px4.sh
```

Le script enchaîne : clonage (~2 Go), dépendances PX4, pose des surcouches
Dronoto (modèle et airframe liés depuis le dépôt), compilation de SITL.

Ajouter à `~/.bashrc` :

```bash
export DRONOTO_PX4_DIR=$HOME/px4/PX4-Autopilot
```

Vérifier : `ls $DRONOTO_PX4_DIR/build/px4_sitl_default/bin/px4`

---

## 6. Workspace ROS 2

```bash
bash infrastructure/scripts/setup_workspace.sh
cd ~/dronoto_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

La première construction est longue : `px4_msgs` génère plusieurs centaines de
messages (10 à 20 minutes). Les suivantes sont rapides.

Vérifier :

```bash
ros2 interface list | grep dronoto      # 5 interfaces
colcon test --packages-select dronoto_core && colcon test-result --verbose
```

Attendu : **27 tests, 0 échec** (10 conversions de repères, 17 machine à états).

---

## 7. Premier lancement

```bash
export DRONOTO_REPO=$(pwd)          # depuis la racine du dépôt
ros2 launch dronoto_bringup bringup_sim.launch.py
```

Gazebo et RViz s'ouvrent, le drone apparaît, et la console PX4 affiche
`Ready for takeoff!`.

**Le drone ne décolle pas** : `autostart` vaut `false` par défaut. Lancer la
pile ne doit jamais faire décoller un aéronef par surprise.

Pour un vol :

```bash
ros2 launch dronoto_bringup bringup_sim.launch.py autostart:=true
```

---

## 8. Tests

```bash
# Rapides, sans simulation (conformité architecturale)
pytest tests/conformance/ -v

# Scénarios SITL complets (lents)
pytest -m scenario -v

# Critère de sortie P1 : 10 exécutions consécutives
bash infrastructure/scripts/run_p1_exit_criteria.sh
```

---

## Dépannage

| Symptôme | Cause |
|---|---|
| `ros2 topic echo /fmu/out/...` reste muet, **sans erreur** | QoS. PX4 publie en `BEST_EFFORT` ; ajouter `--qos-reliability best_effort`. Dans le code, utiliser `rclcpp::SensorDataQoS()`. |
| Aucun topic `/fmu/*` | Agent Micro XRCE-DDS absent ou non démarré (§4) |
| `bad interpreter: /usr/bin/env bash^M` | Fins de ligne CRLF. `.gitattributes` force LF ; refaire `git checkout`. |
| Erreur de compilateur incompréhensible pendant PX4 | OOM. Réduire `PX4_BUILD_JOBS`. |
| Nœud tué au démarrage sur un paramètre | Une liste vide en YAML n'a pas de type inférable. Retirer la ligne. |
| `Unable to parse the value of parameter robot_description as yaml` | `Command(xacro)` doit être enveloppé dans `ParameterValue(..., value_type=str)`. |
| Un scénario échoue après plusieurs exécutions | Processus orphelins : `pkill -9 -f 'px4\|gz sim'` |
