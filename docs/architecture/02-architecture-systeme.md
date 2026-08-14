# 02 — Architecture système

## 1. Vue d'ensemble

```
╔════════════════════════════════════════════════════════════════════════════════╗
║  SOL — non critique pour la sécurité du vol                                    ║
║                                                                                ║
║   ┌──────────────────┐        ┌────────────────────────────────────────────┐  ║
║   │    DASHBOARD     │◀──────▶│              SERVEUR (FastAPI)             │  ║
║   │  React + deck.gl │  REST  │                                            │  ║
║   │                  │   WS   │  ┌────────┐ ┌────────┐ ┌────────┐ ┌─────┐  │  ║
║   └──────────────────┘        │  │mission │ │ drone  │ │telemetry│ │ map │  │  ║
║                               │  └────────┘ └────────┘ └────────┘ └─────┘  │  ║
║                               │  ┌────────┐        ┌──────────────────┐    │  ║
║                               │  │  ai    │        │  bus interne     │    │  ║
║                               │  └────────┘        └──────────────────┘    │  ║
║                               └───────────────┬────────────────────────────┘  ║
║                                               │ IPC (ZeroMQ / socket UNIX)     ║
║                               ┌───────────────▼────────────────────────────┐  ║
║   ┌──────────────────┐        │        PASSERELLE STATION SOL              │  ║
║   │   PostgreSQL     │◀──────▶│   possède la liaison radio, parle DLP      │  ║
║   │ Timescale+PostGIS│        │   CommunicationLink (côté sol)             │  ║
║   └──────────────────┘        └───────────────┬────────────────────────────┘  ║
╚═══════════════════════════════════════════════│════════════════════════════════╝
                                                │
                        ╔═══════════════════════▼═══════════════════════╗
                        ║   CANAL RADIO — non fiable par conception      ║
                        ║   DLP / Protobuf   ~50 kbps   ~100 ms   5 % PER║
                        ║   perte totale possible et attendue           ║
                        ╚═══════════════════════│═══════════════════════╝
                                                │
╔═══════════════════════════════════════════════│════════════════════════════════╗
║  BORD — critique, autonome, doit fonctionner  │ seul                           ║
║                               ┌───────────────▼────────────────────────────┐   ║
║                               │       comm_manager (CommunicationLink)     │   ║
║                               │   spool disque · priorités · adaptation    │   ║
║                               └───────────────┬────────────────────────────┘   ║
║                                               │                                ║
║   ┌────────────────────────────────┐  ┌───────▼─────────┐                     ║
║   │  PERCEPTION                    │  │ mission_        │                     ║
║   │  lidar_odometry (FAST-LIO2)    │─▶│ executive       │                     ║
║   │  map_server_3d (OctoMap)       │  │ (BehaviorTree)  │                     ║
║   │  perception_detector (ONNX)    │  └───────┬─────────┘                     ║
║   │  risk_map                      │          │                               ║
║   └──────────────┬─────────────────┘          ▼                               ║
║                  │                    ┌──────────────────┐                    ║
║                  ├───────────────────▶│  EXPLORATION     │                    ║
║                  │                    │  frontières→NBV  │                    ║
║                  │                    └────────┬─────────┘                    ║
║                  │                             ▼                              ║
║                  │                    ┌──────────────────┐                    ║
║                  ├───────────────────▶│  NAVIGATION      │                    ║
║                  │                    │  planner→suivi   │                    ║
║                  │                    └────────┬─────────┘                    ║
║                  │                             ▼                              ║
║                  │                    ┌──────────────────┐                    ║
║                  └───────────────────▶│ reactive_avoid.  │ veto              ║
║                                       └────────┬─────────┘                    ║
║   ┌───────────────────────────┐                ▼                              ║
║   │   safety_supervisor       │───────▶┌──────────────────┐                   ║
║   │   FSM — autorité absolue  │ veto   │  px4_interface   │                   ║
║   │   watchdogs               │        └────────┬─────────┘                   ║
║   └───────────────────────────┘                 │ uXRCE-DDS                   ║
╚═════════════════════════════════════════════════│══════════════════════════════╝
                                                  │
                          ┌───────────────────────▼───────────────────────┐
                          │                   PX4                          │
                          │   EKF2 · contrôle position/attitude/taux       │
                          │   mixeur · failsafes indépendants              │
                          └───────────────────────┬───────────────────────┘
                                                  ▼
                                              MOTEURS
```

## 2. Les trois plans

Une propriété structurante du système : les données ne circulent pas toutes au même
rythme, ni avec les mêmes exigences. Les confondre est la source d'erreur classique de ce
type d'architecture. On sépare donc explicitement trois plans.

| Plan | Fréquence | Latence tolérée | Perte tolérée | Traverse la radio ? | Exemples |
|---|---|---|---|---|---|
| **Contrôle** | 50-1000 Hz | < 20 ms | Aucune | **Jamais** | Setpoints de trajectoire, odométrie externe, sorties moteur |
| **Données** | 1-30 Hz | < 500 ms | Tolérée | Résumée / échantillonnée | Nuages de points, carte, détections, image |
| **Gestion** | 0,1-1 Hz | Secondes à minutes | Tolérée avec reprise | **Oui, c'est son rôle** | Missions, waypoints, télémétrie, événements, alertes |

Le plan de contrôle est **entièrement embarqué**. Il ne quitte jamais le drone. C'est
l'expression technique de la contrainte « pas de contrôle moteur depuis le serveur », et
c'est vérifiable : le schéma Protobuf du protocole radio ne contient aucun message du plan
de contrôle, et un test automatisé le vérifie à chaque build
([14 — Tests](14-tests.md), test `test_protocol_has_no_control_plane_messages`).

## 3. Flux de données principaux

### 3.1 Boucle de perception → carte

```
gz_sim (gpu_lidar)
   │  sensor_msgs/PointCloud2 @ 10 Hz, ~20 000 points
   ▼
/drone_1/sensors/lidar/points
   │
   ├──▶ lidar_odometry (FAST-LIO2) ──┬──▶ /state/lidar_odometry  @ 10 Hz  (nav_msgs/Odometry)
   │      + /sensors/imu @ 250 Hz    │       │
   │                                 │       ├──▶ px4_interface ──▶ PX4 EKF2  (odométrie externe @ 30-50 Hz)
   │                                 │       └──▶ pose_graph ──▶ TF map→odom (correction lente)
   │                                 │
   │                                 └──▶ /map/registered_cloud  @ 10 Hz  (nuage désskewé, en repère map)
   │                                          │
   │                                          ▼
   │                                    map_server_3d (OctoMap)
   │                                          │
   │                                          ├──▶ /map/octomap_binary       @ 1 Hz   (visualisation, radio)
   │                                          ├──▶ /map/octomap_full         @ 0,2 Hz (persistance)
   │                                          └──▶ /map/esdf_local           @ 5 Hz   (planification)
   │
   └──▶ reactive_avoidance  (chemin court, ne passe PAS par la carte — voir §5)
```

### 3.2 Boucle de décision → vol

```
mission_executive (BehaviorTree)
   │  action ExploreArea
   ▼
frontier_detector ──▶ /exploration/frontiers  (clusters)
   ▼
viewpoint_sampler ──▶ candidats (position + cap)
   ▼
nbv_selector      ──▶ /exploration/goal  (geometry_msgs/PoseStamped)
   │   scoring : gain d'information · coût de trajet · risque · batterie · qualité de localisation
   ▼
global_planner (action PlanPath)  ──▶ /navigation/path  (A*/JPS sur voxels)
   ▼
trajectory_generator ──▶ /navigation/trajectory  (polynôme lissé, contraint par l'ESDF)
   ▼
trajectory_follower  ──▶ /control/setpoint_raw   @ 20 Hz
   ▼
reactive_avoidance   ──▶ /control/setpoint_safe  @ 30 Hz   ◀── VETO possible
   ▼
safety_supervisor    ──▶ /control/setpoint_final @ 20 Hz   ◀── VETO absolu
   ▼
px4_interface        ──▶ px4_msgs/TrajectorySetpoint via uXRCE-DDS
   ▼
PX4
```

Deux points de veto en série. Chacun peut remplacer la consigne par une consigne sûre
(freinage, maintien de position) sans coopération de l'amont. C'est délibéré : un
composant de sécurité qui a besoin que le composant fautif coopère n'est pas un composant
de sécurité.

### 3.3 Boucle de supervision (traverse la radio)

```
BORD                                                          SOL
────                                                          ───
telemetry_aggregator
   │  agrège l'état à 1 Hz (adaptatif 0,2-2 Hz)
   ▼
comm_manager
   │  encode Protobuf, priorise, met en spool si déconnecté
   ▼
CommunicationLink.send() ═══ canal ═══▶ CommunicationLink.receive()
                                                   │
                                          passerelle station sol
                                                   │ décode, valide
                                                   ▼
                                          bus interne du serveur
                                             ├──▶ telemetry (écrit dans TimescaleDB)
                                             ├──▶ drone     (met à jour l'état courant)
                                             ├──▶ map       (assemble les deltas)
                                             └──▶ WebSocket ──▶ dashboard
```

Le retour (sol → drone) suit le même chemin en sens inverse : une commande du dashboard
devient un `CommandEnvelope` DLP, acquitté par le drone. **Le serveur n'attend jamais
d'acquittement de manière bloquante** ; une commande non acquittée est réémise selon une
politique, et son statut est visible dans le dashboard.

## 4. Protocoles, par frontière

| Frontière | Protocole | Transport | Justification |
|---|---|---|---|
| Gazebo ↔ ROS 2 | Messages `gz-transport` pontés | `ros_gz_bridge` | Officiel, typé, remplacé par des pilotes réels au passage matériel |
| Nœuds ROS 2 entre eux | DDS-RTPS (Fast DDS) | UDP local / partagé | Standard ROS 2, typé, QoS configurable |
| Calculateur ↔ PX4 | uXRCE-DDS (uORB exposé en DDS) | UDP (SITL) / série ou Ethernet (réel) | Officiel PX4 ≥ 1.14, typé, faible latence |
| Drone ↔ station sol | **DLP** (Protobuf, cadré, priorisé) | `CommunicationLink` : UDP simulé, série 868 MHz réel | Conçu pour 50 kbps, voir [09](09-communication.md) |
| Station sol ↔ serveur | Messages internes | ZeroMQ ou socket UNIX | Découple la possession de la radio du serveur applicatif |
| Serveur ↔ dashboard | REST (OpenAPI) + WebSocket | HTTP/1.1, WS | REST pour les commandes et l'historique, WS pour le flux temps réel |
| Serveur ↔ base | SQL | TCP | — |
| PX4 ↔ QGroundControl | MAVLink | UDP tunnelé sur DLP (optionnel) | Reprise en main par un opérateur humain avec un outil standard |

## 5. Chemin court / chemin long

Distinction essentielle pour l'évitement d'obstacles, et une des rares décisions où se
tromper tue un drone.

```
CHEMIN LONG  (informé, lent, faillible)
  LiDAR → SLAM → OctoMap → ESDF → planificateur → trajectoire
  Latence ≈ 200-500 ms.  Dépend de la justesse de la localisation.
  Si le SLAM diverge, ce chemin produit des trajectoires fausses avec assurance.

CHEMIN COURT  (réflexe, rapide, robuste)
  LiDAR → filtre de sécurité → veto sur la consigne
  Latence < 50 ms.  N'utilise QUE le nuage brut dans le repère capteur.
  Ne dépend NI de la carte NI de la localisation.
```

Le chemin court n'utilise pas la carte, et c'est tout l'intérêt : il reste valide quand le
SLAM diverge, quand la transformation `map → odom` saute, quand la carte est périmée. Il ne
sait pas où est le drone dans le monde — il sait juste qu'il y a des points à trois mètres
devant, dans le repère du capteur, et que la consigne actuelle y mène. Il freine.

Un système qui ne possède que le chemin long a une **corrélation de mode commun** : la même
erreur de localisation corrompt simultanément la carte et le plan. Le chemin court brise
cette corrélation.

## 6. Modèle de déploiement

### En simulation (une machine)

```
┌─ Processus / conteneurs ────────────────────────────────────────────┐
│                                                                     │
│  gz sim (serveur)  ◀─── ros_gz_bridge ───▶  graphe ROS 2            │
│         ▲                                        ▲                  │
│         │ plugin PX4                             │ DDS              │
│  px4 SITL (build sitl)  ◀── uXRCE-DDS Agent ────┘                  │
│                                                                     │
│  nœuds drone (conteneur drone-ros, network_mode: host)              │
│  fault_injector  ·  virtual_radio (deux extrémités)                 │
│                                                                     │
│  passerelle sol  ·  serveur FastAPI  ·  postgres  ·  dashboard      │
└─────────────────────────────────────────────────────────────────────┘
```

Le canal radio virtuel est un **processus distinct** qui relaie entre deux sockets UDP en
appliquant le modèle de dégradation. Ni le drone ni le sol ne savent qu'ils sont dégradés :
ils voient une liaison qui perd des paquets, exactement comme la vraie. C'est ce qui rend
le test de résilience honnête.

### Sur matériel réel (cible)

```
DRONE                                       SOL
  Jetson Orin NX                              PC ou Raspberry Pi
    nœuds ROS 2 (identiques)                    passerelle sol
    uXRCE-DDS Agent                             serveur (ou distant)
        │ série/Ethernet                        │
    Pixhawk 6X                              modem RFD 868x + antenne directive
        │
    modem RFD 868x
```

Les nœuds ROS 2 embarqués sont **littéralement les mêmes binaires**. Ce qui change :
`gz sim` disparaît, remplacé par les pilotes de capteurs réels (Livox SDK2,
`realsense-ros`) ; `VirtualRadioLink` est remplacé par `SerialRadioLink` ; le fournisseur
ONNX Runtime passe de CPU à TensorRT. Trois substitutions, aucune réécriture. C'est la
mesure de la qualité de l'architecture.

## 7. Extension multi-drones

Prévue dès la V1, à coût nul, parce que la rétro-adapter coûte cher.

- **ROS 2** : chaque drone vit dans son propre namespace (`/drone_1/...`) et son propre
  domaine DDS. En simulation multi-drones, un `ROS_DOMAIN_ID` par drone évite les
  collisions de découverte.
- **TF** : préfixe de repère par drone (`drone_1/base_link`). Un repère `world` commun sert
  d'ancrage cartographique partagé.
- **DLP** : chaque trame porte un `drone_id`. La station sol démultiplexe.
- **Base** : `drone_id` est une colonne de première classe, présente dans les clés
  primaires composites des hypertables de télémétrie.
- **Serveur** : les ressources sont déjà `/drones/{id}/...`.

Ce qui n'est **pas** prévu et ne doit pas être improvisé : la fusion de cartes entre drones,
l'allocation coopérative de zones, l'évitement inter-drones. Ce sont des problèmes de
recherche à part entière, hors périmètre ([00 §5](00-vision-et-principes.md)).

## 8. Budget de latence de bout en bout

Les chiffres qui contraignent la conception. Un budget non écrit est un budget non tenu.

| Chaîne | Budget | Conséquence du dépassement |
|---|---|---|
| LiDAR → veto d'évitement → consigne PX4 | **< 50 ms** | Collision. Contrainte dure. |
| IMU → EKF2 → contrôle attitude | < 5 ms (dans PX4) | Instabilité. Géré par PX4. |
| FAST-LIO2 : scan → odométrie | < 100 ms | EKF2 rejette l'odométrie externe (fenêtre de fusion dépassée) |
| Nuage → insertion OctoMap | < 200 ms | Carte en retard, planification sur données périmées |
| Détection d'objectif → nouvelle trajectoire | < 2 s | Drone en vol stationnaire inutile, batterie gaspillée |
| Perte de liaison → détection embarquée | < 5 s | Retard de bascule en autonomie |
| Télémétrie bord → affichage dashboard | < 3 s | Confort opérateur, non critique |
| Commande dashboard → acquittement drone | < 10 s | Confort opérateur, non critique |

Ces budgets sont **mesurés en continu** par le nœud `health_monitor` et exportés comme
métriques ; leur dépassement est une assertion de test au niveau L3
([14 — Tests](14-tests.md)).
