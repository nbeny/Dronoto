# 13 — Docker et infrastructure

## 1. Ce qu'on conteneurise, et ce qu'on ne conteneurise pas

Docker n'est pas gratuit en robotique : il complique l'accès au GPU, aux périphériques, au
réseau DDS et à l'affichage graphique. On l'applique donc là où il paie et pas ailleurs.

| Composant | Conteneurisé | Raison |
|---|---|---|
| Serveur FastAPI | **Oui** | Déploiement classique, aucune contrainte matérielle |
| PostgreSQL / TimescaleDB | **Oui** | Image officielle, isolation propre |
| Dashboard (build + service) | **Oui** | — |
| Passerelle station sol | **Oui** (avec accès `/dev/tty*` au réel) | — |
| Simulation (Gazebo + PX4) | **Oui**, avec réserves | Reproductibilité précieuse, mais GPU et affichage à configurer |
| Nœuds ROS 2 du drone | **Oui** en CI, **optionnel** en développement | En développement local, un workspace natif est plus rapide à itérer |
| Développement quotidien | **Non** | Un colcon natif dans un workspace Ubuntu est plus rapide qu'un rebuild d'image |

**Le point qui compte** : les tests de CI tournent **exclusivement** en conteneur. C'est ce
qui garantit la reproductibilité. Le développement local peut être natif pour la vitesse
d'itération, mais rien n'est considéré comme validé tant que ça n'a pas passé la CI
conteneurisée. Cette asymétrie est délibérée : elle donne la vitesse en développement et la
rigueur en validation.

## 2. Images

```
infrastructure/docker/
├── base/Dockerfile              ros:jazzy-ros-base + outils communs
├── simulation/Dockerfile        base + Gazebo Harmonic + PX4 SITL + mondes
├── drone/Dockerfile             base + paquets ROS 2 Dronoto + ONNX Runtime
├── server/Dockerfile            python:3.12-slim + FastAPI + Protobuf généré
├── groundstation/Dockerfile     python:3.12-slim + pyserial + Protobuf
├── dashboard/Dockerfile         node:22 (build) → nginx:alpine (service)
└── dev/Dockerfile               drone + outils de dev (gdb, RViz, Groot, IDE)
```

Toutes les images sont **multi-étapes** : compilation dans une étape lourde, exécution dans
une étape minimale. Les images `drone` et `server` sont celles qui iront un jour sur le
Jetson — leur taille compte réellement.

Cible : `drone` < 2,5 Go, `server` < 400 Mo, `dashboard` < 60 Mo.

## 3. Profils Compose

Compose avec profils, pour ne lancer que ce dont on a besoin.

```yaml
# infrastructure/compose/docker-compose.yml
services:

  simulation:
    profiles: ["sim", "full", "test"]
    build: { context: ../.., dockerfile: infrastructure/docker/simulation/Dockerfile }
    network_mode: host                 # indispensable pour la découverte DDS
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - PX4_SIM_MODEL=x500_lidar_dronoto
      - GZ_SIM_RESOURCE_PATH=/workspace/simulation/worlds:/workspace/simulation/models
      - DISPLAY=${DISPLAY}
      - HEADLESS=${HEADLESS:-1}
    devices:
      - /dev/dri:/dev/dri              # accès GPU AMD (Mesa RADV)
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ../../simulation:/workspace/simulation

  drone:
    profiles: ["drone", "full", "test"]
    build: { context: ../.., dockerfile: infrastructure/docker/drone/Dockerfile }
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - DRONE_ID=${DRONE_ID:-1}
      - ONNX_EXECUTION_PROVIDER=${ONNX_EP:-CPUExecutionProvider}
      - DRONOTO_CONFIG_PROFILE=${PROFILE:-sim_default}
    devices:
      - /dev/dri:/dev/dri
    volumes:
      - ../../models:/workspace/models:ro
      - dronoto-spool:/var/lib/dronoto/spool     # le spool DOIT survivre au redémarrage
    depends_on: [simulation]

  groundstation:
    profiles: ["ground", "full", "test"]
    build: { context: ../.., dockerfile: infrastructure/docker/groundstation/Dockerfile }
    network_mode: host
    environment:
      - LINK_TYPE=${LINK_TYPE:-virtual}
      - LINK_CONFIG=/config/radio_rfd868x.yaml
      - SERVER_IPC=tcp://127.0.0.1:5555
    volumes:
      - ../../simulation/config:/config:ro
    # au réel : devices: ["/dev/ttyUSB0:/dev/ttyUSB0"]

  server:
    profiles: ["server", "full", "test"]
    build: { context: ../.., dockerfile: infrastructure/docker/server/Dockerfile }
    ports: ["8000:8000"]
    environment:
      - DATABASE_URL=postgresql+asyncpg://dronoto:dronoto@postgres:5432/dronoto
      - GROUNDSTATION_IPC=tcp://host.docker.internal:5555
    depends_on: { postgres: { condition: service_healthy } }

  postgres:
    profiles: ["server", "full", "test"]
    image: timescale/timescaledb-ha:pg16
    environment:
      - POSTGRES_USER=dronoto
      - POSTGRES_PASSWORD=dronoto
      - POSTGRES_DB=dronoto
    volumes: [ "dronoto-pgdata:/var/lib/postgresql/data" ]
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U dronoto"]
      interval: 5s
      retries: 10

  dashboard:
    profiles: ["dash", "full"]
    build: { context: ../.., dockerfile: infrastructure/docker/dashboard/Dockerfile }
    ports: ["3000:80"]
    depends_on: [server]

volumes:
  dronoto-pgdata:
  dronoto-spool:
```

Usage :

```bash
docker compose --profile sim   up     # simulation seule
docker compose --profile full  up     # tout
docker compose --profile test  up --abort-on-container-exit
```

## 4. Le problème du réseau DDS

C'est le piège numéro un de ROS 2 en conteneur, et il vaut mieux le traiter frontalement.

**Le problème** : la découverte DDS utilise le multicast. Le réseau bridge par défaut de
Docker isole le multicast entre conteneurs, donc les nœuds ne se voient pas et le système
semble simplement... ne rien faire. Le symptôme (aucun message, aucune erreur) est
particulièrement pénible à diagnostiquer.

**Solution retenue : `network_mode: host`** pour tous les conteneurs qui participent au
graphe ROS 2. Simple, fiable, sans surprise. Coût : on perd l'isolation réseau entre ces
conteneurs — acceptable pour un système où ils sont conçus pour communiquer.

Pour les tests parallèles, l'isolation se fait par `ROS_DOMAIN_ID` (0 à 101, un par
simulation concurrente) plutôt que par réseau. Chaque simulation reçoit aussi ses propres
ports PX4 et uXRCE-DDS, dérivés de l'identifiant.

**Alternative rejetée** : réseau Docker dédié + configuration DDS en liste de pairs
statique. Plus propre conceptuellement, mais impose une configuration XML par
implémentation DDS et casse dès qu'un conteneur change d'adresse. Complexité sans bénéfice
à cette échelle.

## 5. GPU et affichage

### GPU AMD sous Linux natif

```yaml
devices:
  - /dev/dri:/dev/dri
group_add:
  - video
  - render
```

Aucun runtime spécial nécessaire — c'est nettement plus simple que le `nvidia-container-toolkit`
requis côté NVIDIA. Mesa RADV est dans l'image de base ; Gazebo utilise le GPU sans
configuration additionnelle.

Pour ONNX Runtime avec ROCm (optionnel, Linux natif seulement) :

```yaml
devices: [ "/dev/kfd:/dev/kfd", "/dev/dri:/dev/dri" ]
environment:
  - HSA_OVERRIDE_GFX_VERSION=10.3.0     # force le support gfx1030 (RDNA2, RX 6900 XT)
```

`HSA_OVERRIDE_GFX_VERSION` est un contournement non officiel mais largement éprouvé pour
faire accepter les cartes RDNA2 par ROCm. Il fonctionne en pratique ; il n'est pas
supporté par AMD. On l'utilise comme accélération opportuniste, jamais comme dépendance —
le fournisseur CPU reste le défaut, cohérent avec [08](08-ia.md).

### Affichage

- **Linux natif** : montage de `/tmp/.X11-unix` + `DISPLAY`, plus `xhost +local:docker`.
- **WSL2** : WSLg expose déjà `/tmp/.X11-unix` et `/mnt/wslg`. Fonctionne, avec la perte de
  performance décrite en [01 §1](01-choix-technologiques.md).
- **CI** : `HEADLESS=1`, aucun affichage. Gazebo tourne en `-s` (serveur seul) avec rendu
  logiciel pour les capteurs.

## 6. Développement

Le mode de travail quotidien recommandé n'est pas Docker, mais un workspace natif :

```
~/dronoto_ws/
├── src/  → lien symbolique vers le dépôt
├── build/  install/  log/
```

`colcon build --symlink-install` permet de modifier du Python sans reconstruire. Pour C++,
`--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` donne les symboles de débogage sans
sacrifier les performances — indispensable pour profiler FAST-LIO2 ou l'insertion OctoMap.

Un **devcontainer** VS Code est fourni pour ceux qui préfèrent l'isolation, avec les
extensions ROS 2, C++ et Python préconfigurées.

## 7. CI

```yaml
# .github/workflows/ci.yml — structure
jobs:
  lint:          # ruff, mypy, clang-format, clang-tidy, eslint     ~2 min
  proto:         # génère le Protobuf, vérifie qu'il est à jour      ~1 min
  unit:          # pytest + gtest (L0)                               ~5 min
  build:         # colcon build complet + images Docker              ~15 min
  component:     # launch_testing (L1)                               ~10 min
  integration:   # multi-nœuds sans Gazebo, rejeu de rosbag (L2)     ~15 min
  scenario:      # SITL headless (L3) — nightly, pas à chaque PR     ~45 min
  soak:          # Monte-Carlo (L4) — hebdomadaire                   ~4 h
```

Les jobs `lint`, `proto`, `unit`, `build`, `component` et `integration` s'exécutent sur
chaque pull request : environ 50 minutes, acceptable. Les niveaux L3 et L4 sont trop longs
et trop coûteux pour bloquer une PR — ils tournent la nuit et le week-end, avec échec
notifié.

Deux vérifications spécifiques dans le job `proto` : le code Protobuf généré et versionné
doit correspondre aux `.proto` (sinon quelqu'un a modifié un schéma sans régénérer), et le
test « aucun message du plan de contrôle » ([09 §1](09-communication.md)) s'exécute là.

**Cache** : les artefacts `colcon build` et les couches Docker sont mis en cache
agressivement. Sans cela, 15 minutes de build à chaque PR décourage les petites
contributions et pousse aux gros commits — un effet pervers dont on se passe.

## 8. Observabilité

| Signal | Outil | Portée |
|---|---|---|
| Logs ROS 2 | `rcutils` → fichiers + `/rosout` | Bord |
| Logs applicatifs | `structlog` (JSON) | Serveur, station sol |
| Métriques | Prometheus (`/metrics` sur le serveur) | Sol |
| Métriques bord | `diagnostic_msgs` → télémétrie → Prometheus | Bord → sol |
| Traces | `rosbag2` (bord), journal d'audit (serveur) | Les deux |
| Tableaux de bord | Grafana (optionnel, profil `monitoring`) | Sol |

**`rosbag2` est l'outil de diagnostic principal du système embarqué.** Chaque vol
enregistre un bag avec tous les topics sauf les nuages de points bruts (trop volumineux —
seuls les nuages décimés sont conservés). Un bag permet de rejouer un vol complet hors
ligne et de reproduire un bug sans simulateur. C'est ce qui rend le débogage possible : on
ne peut pas mettre un point d'arrêt dans un drone en vol, mais on peut rejouer ce qu'il a
vu.
