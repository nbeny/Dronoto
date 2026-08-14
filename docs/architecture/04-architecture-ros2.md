# 04 — Architecture ROS 2

## 1. Conventions

**Namespace.** Tous les nœuds et topics d'un drone vivent sous `/drone_<id>/`. Les
exemples ci-dessous omettent le préfixe pour la lisibilité ; il est toujours présent.

**Repères TF.** Préfixés par drone (`drone_1/base_link`) sauf les repères globaux partagés
(`earth`, `map`).

**Langage.** C++ pour tout ce qui est dans une boucle temps réel ou dans le chemin de
sécurité ; Python pour la logique de haut niveau, l'orchestration et l'outillage. Le
critère n'est pas la préférence mais la latence et le déterminisme : un nœud dont le retard
peut faire tomber le drone n'est pas écrit dans un langage à ramasse-miettes.

| Nœud | Langage | Justification |
|---|---|---|
| `px4_interface`, `reactive_avoidance`, `safety_supervisor`, `trajectory_follower` | **C++** | Chemin de contrôle, latence bornée, pas de GC |
| `lidar_odometry`, `map_server_3d`, `esdf_builder`, `global_planner` | **C++** | Calcul dense |
| `mission_executive` | **C++** | BehaviorTree.CPP |
| `frontier_detector`, `viewpoint_sampler`, `nbv_selector` | **C++** | Boucle serrée sur les voxels |
| `perception_detector`, `semantic_mapper`, `risk_map` | **Python** | ONNX Runtime, itération rapide, hors chemin critique |
| `comm_manager`, `telemetry_aggregator`, `health_monitor` | **Python** | Logique, I/O, sérialisation |
| `fault_injector`, `sensor_faults` | **Python** | Outillage de simulation |

**Nommage.** `snake_case` pour les nœuds et topics. Les topics sont groupés par domaine :
`sensors/`, `state/`, `map/`, `navigation/`, `exploration/`, `control/`, `safety/`, `link/`,
`perception/`, `sim/`.

## 2. Arbre TF

Conforme à REP-105, étendu pour le cas GNSS + SLAM.

```
earth                      ECEF. Ancrage global, publié une fois au démarrage.
  │  transformation statique établie au premier fix GNSS
  ▼
map                        ENU local, origine = point de départ de la mission.
  │                        Repère de la carte. Discontinu : peut sauter lors
  │                        d'une fermeture de boucle ou d'une correction GNSS.
  │  publié par pose_graph, ~1 Hz, CORRECTION LENTE
  ▼
odom                       Repère continu et lisse. Ne saute JAMAIS.
  │                        Dérive lentement par rapport à map.
  │  publié par lidar_odometry, 10 Hz
  ▼
base_link                  Centre de gravité du drone, convention FRD→FLU convertie.
  │  transformations statiques (URDF)
  ├──▶ imu_link
  ├──▶ lidar_link          (x=0,10  y=0  z=0,08  — au-dessus du corps)
  ├──▶ cam_front_link      (x=0,12  y=0  z=0,02  pitch=-15°)
  ├──▶ gnss_link
  └──▶ battery_link
```

**La règle qui compte** : `odom → base_link` est **continu**, `map → odom` absorbe
**toutes** les discontinuités. Les contrôleurs et l'évitement travaillent dans `odom` (ils
ne supportent pas les sauts) ; la carte, la planification et l'exploration travaillent dans
`map` (elles ont besoin de la cohérence globale). Violer cette séparation — par exemple en
téléportant `base_link` lors d'une correction GNSS — produit un à-coup de commande qui, sur
un vrai drone, se traduit par une embardée.

## 3. Nœuds

### 3.1 Groupe interface

#### `px4_interface` (C++)

Frontière unique avec l'autopilote. **Aucun autre nœud ne parle à PX4.** C'est une règle
architecturale : elle concentre en un seul endroit la connaissance des conventions PX4
(repères NED, unités, timestamps, sémantique des modes) et permet de tester tout le reste
sans PX4.

| Direction | Contenu |
|---|---|
| PX4 → système | `VehicleOdometry`, `VehicleStatus`, `BatteryStatus`, `FailsafeFlags`, `VehicleGpsPosition`, `SensorCombined` → normalisés en messages ROS 2 standard et `dronoto_msgs` |
| Système → PX4 | `TrajectorySetpoint`, `OffboardControlMode`, `VehicleCommand` (armement, mode, RTL), `VehicleOdometry` (odométrie externe pour EKF2) |

Responsabilités particulières :

- **Conversion de repères** NED (PX4) ↔ ENU/FLU (ROS 2). Source d'erreur numéro un dans ce
  type d'intégration ; isolée ici, couverte par des tests unitaires exhaustifs.
- **Maintien du flux offboard** : PX4 exige des `OffboardControlMode` à ≥ 2 Hz sous peine de
  quitter le mode. Le nœud publie à 20 Hz un flux continu, en répétant la dernière consigne
  valide si l'amont se tait. Il compte les répétitions et lève `STALE_SETPOINT` au-delà de
  0,5 s — l'amont est alors considéré défaillant.
- **Injection de l'odométrie externe** dans EKF2 à 30-50 Hz, avec covariance et
  compensation de latence.

#### `sensor_faults` (Python, simulation uniquement)

Intercalé entre le pont Gazebo et le graphe embarqué. Applique bruit, biais, latence et
pannes. Absent au réel, remplacé par les pilotes matériels — le graphe en aval est
identique.

### 3.2 Groupe estimation et cartographie

#### `lidar_odometry` (C++, FAST-LIO2)

Entrées : `sensors/lidar/points`, `sensors/imu`.
Sorties : `state/lidar_odometry` (`nav_msgs/Odometry`, 10 Hz), `map/registered_cloud`
(nuage désskewé dans `map`), TF `odom → base_link`.

Publie aussi `state/odometry_health` (`dronoto_msgs/OdometryHealth`) : nombre de points
appariés, résidu moyen, condition du système, indicateur de dégénérescence. **Cette sortie
est aussi importante que la pose elle-même** : c'est elle qui alimente le critère « qualité
de localisation » du scoring d'exploration et le déclenchement des failsafes. Un SLAM qui
ne dit pas quand il doute est un SLAM dangereux.

#### `pose_graph` (C++, GTSAM)

Entrées : poses clés de `lidar_odometry`, `sensors/gnss`, détections de fermeture de boucle.
Sortie : TF `map → odom`, `map/trajectory_optimized`.

Optimise à ~1 Hz. Applique les corrections par **interpolation lissée** sur 2 s, jamais par
saut instantané. Livré en P4 ; en P2-P3, `map → odom` est l'identité.

#### `map_server_3d` (C++, OctoMap)

Entrée : `map/registered_cloud`.
Sorties : `map/octomap_binary` (1 Hz), `map/octomap_full` (0,2 Hz),
`map/occupancy_delta` (`dronoto_msgs/OccupancyDelta`, 2 Hz — cellules modifiées uniquement,
c'est ce qui part par radio).

Résolution 0,25 m en V1. Fenêtre glissante configurable pour borner la mémoire ; au-delà,
les régions anciennes sont écrites sur disque et rechargeables.

#### `esdf_builder` (C++)

Entrée : `map/octomap_binary`.
Sortie : `map/esdf_local` (grille de distance signée, 40 × 40 × 20 m à 0,2 m, 5 Hz).
Recalcul incrémental sur les cellules modifiées. Consommé par le planificateur et
l'optimiseur de trajectoire.

### 3.3 Groupe perception IA

#### `perception_detector` (Python, ONNX Runtime)

Entrée : `sensors/cam_front/image_raw` (sous-échantillonné à 2-5 Hz).
Sortie : `perception/detections` (`vision_msgs/Detection3DArray` après projection depth).

Sous-échantillonne délibérément : la perception sémantique alimente des décisions
d'exploration, pas une boucle de contrôle. Tourner à 15 Hz consommerait du CPU et de
l'énergie pour rien.

#### `semantic_mapper` (Python)

Fusionne les détections dans une couche sémantique alignée sur l'OctoMap.
Sortie : `map/semantic_layer`.

#### `risk_map` (Python)

Combine occupation, segmentation du terrain, pente et incertitude en un champ de risque
scalaire. Sortie : `map/risk`. Consommé par le scoring d'exploration et par la sélection de
site d'atterrissage.

### 3.4 Groupe navigation

#### `global_planner` (C++)

Serveur d'action `PlanPath`. A*/JPS sur les voxels OctoMap, avec un coût combinant
longueur, dégagement (via ESDF) et risque. Chemin post-traité par raccourcissement
(line-of-sight shortcutting).

#### `trajectory_generator` (C++)

Transforme un chemin géométrique en trajectoire temporelle : polynômes minimum-snap par
segment, contraints en vitesse et accélération, repoussés des obstacles par les gradients
de l'ESDF. Sortie : `navigation/trajectory` (`dronoto_msgs/Trajectory`).

#### `trajectory_follower` (C++, 20 Hz)

Échantillonne la trajectoire au temps courant, produit `control/setpoint_raw`
(position + vitesse + accélération d'anticipation + cap). Ne referme pas de boucle
d'attitude — c'est le travail de PX4.

#### `reactive_avoidance` (C++, 30 Hz) — **premier point de veto**

Entrées : `sensors/lidar/points` (**brut, repère capteur**), `control/setpoint_raw`.
Sortie : `control/setpoint_safe`.

N'utilise ni la carte, ni la localisation, ni TF `map`. Voir
[02 §5](02-architecture-systeme.md). Trois comportements par ordre de priorité :

1. **Freinage d'urgence** si un obstacle est dans le cône de vol à moins de la distance
   d'arrêt calculée à la vitesse courante ;
2. **Répulsion** : ajoute une composante de vitesse s'éloignant des points proches ;
3. **Passe-plat** si tout est dégagé.

Publie `safety/avoidance_status` (distance au plus proche obstacle, veto actif ou non).

### 3.5 Groupe exploration

#### `frontier_detector` (C++, 1 Hz)

Entrée : `map/octomap_binary`. Sortie : `exploration/frontiers`
(`dronoto_msgs/FrontierArray`). Détecte les voxels libres adjacents à de l'inconnu, les
regroupe (union-find sur voisinage 26), filtre les clusters trop petits.

#### `viewpoint_sampler` (C++)

Génère des poses candidates autour de chaque cluster, en respectant le FOV du LiDAR, une
distance minimale aux obstacles et les limites d'altitude de la mission.
Sortie : `exploration/candidates`.

#### `nbv_selector` (C++, 0,5-1 Hz)

Évalue chaque candidat (gain d'information par lancer de rayons dans l'OctoMap, coût du
trajet, risque, faisabilité batterie, qualité de localisation attendue) et publie
`exploration/goal` plus `exploration/scored_candidates` (pour le débogage et le dashboard —
voir le score, c'est comprendre la décision). Détail : [07](07-exploration.md).

### 3.6 Groupe mission et sécurité

#### `mission_executive` (C++, BehaviorTree.CPP)

Serveur d'action `ExecuteMission`. Orchestre : décollage, exploration, retour, atterrissage,
maintien en vol stationnaire, reprise. L'arbre est chargé depuis un XML — modifiable sans
recompiler, inspectable avec Groot.

#### `safety_supervisor` (C++, 10 Hz) — **veto absolu**

Machine à états explicite. Souscrit à tout ce qui compte (état PX4, batterie, santé de
l'odométrie, santé de la liaison, âge des topics, statut d'évitement) et **peut réécrire ou
supprimer toute consigne**. Publie `safety/state` (`dronoto_msgs/SafetyState`, QoS
`transient_local` pour que tout nouveau souscripteur reçoive immédiatement l'état courant).

Détail : [10 — Failsafe](10-failsafe.md).

#### `health_monitor` (Python, 2 Hz)

Chiens de garde sur tous les topics critiques (âge du dernier message, fréquence effective,
latence de bout en bout). Publie `diagnostics` (`diagnostic_msgs/DiagnosticArray`). C'est
lui qui mesure les budgets de latence de [02 §8](02-architecture-systeme.md).

### 3.7 Groupe communication

#### `comm_manager` (Python)

Possède l'instance `CommunicationLink`. Encode/décode le DLP, gère les files de priorité,
le spool disque, l'adaptation de débit et la détection de perte de liaison.
Sorties : `link/status` (`dronoto_msgs/LinkStatus`, 1 Hz), `link/inbound_command`.
Détail : [09](09-communication.md).

#### `telemetry_aggregator` (Python, 1 Hz adaptatif)

Assemble un instantané cohérent de l'état du drone et le remet au `comm_manager`. Ajuste sa
cadence selon la bande passante disponible signalée par `link/status`.

## 4. Topics

Fréquences nominales. QoS : `R` = reliable, `BE` = best effort, `TL` = transient local,
`V` = volatile. Profondeur entre crochets.

### Capteurs (produits par la simulation ou les pilotes)

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `sensors/imu` | `sensor_msgs/Imu` | 250 | BE·V[10] |
| `sensors/lidar/points` | `sensor_msgs/PointCloud2` | 10 | BE·V[2] |
| `sensors/gnss` | `sensor_msgs/NavSatFix` | 5 | BE·V[5] |
| `sensors/mag` | `sensor_msgs/MagneticField` | 50 | BE·V[5] |
| `sensors/baro` | `sensor_msgs/FluidPressure` | 50 | BE·V[5] |
| `sensors/cam_front/image_raw` | `sensor_msgs/Image` | 15 | BE·V[2] |
| `sensors/cam_front/depth` | `sensor_msgs/Image` | 15 | BE·V[2] |
| `sensors/cam_front/camera_info` | `sensor_msgs/CameraInfo` | 15 | R·TL[1] |

### État

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `state/odometry` | `nav_msgs/Odometry` | 50 | BE·V[10] |
| `state/lidar_odometry` | `nav_msgs/Odometry` | 10 | BE·V[10] |
| `state/odometry_health` | `dronoto_msgs/OdometryHealth` | 10 | R·V[5] |
| `state/battery` | `dronoto_msgs/BatteryState` | 2 | R·V[5] |
| `state/vehicle_status` | `dronoto_msgs/VehicleStatus` | 5 | R·TL[1] |
| `state/gnss_status` | `dronoto_msgs/GnssStatus` | 5 | R·V[5] |

### Carte

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `map/registered_cloud` | `sensor_msgs/PointCloud2` | 10 | BE·V[2] |
| `map/octomap_binary` | `octomap_msgs/Octomap` | 1 | R·TL[1] |
| `map/octomap_full` | `octomap_msgs/Octomap` | 0,2 | R·TL[1] |
| `map/occupancy_delta` | `dronoto_msgs/OccupancyDelta` | 2 | R·V[20] |
| `map/esdf_local` | `dronoto_msgs/EsdfGrid` | 5 | BE·V[2] |
| `map/semantic_layer` | `dronoto_msgs/SemanticLayer` | 0,5 | R·TL[1] |
| `map/risk` | `dronoto_msgs/RiskGrid` | 1 | R·V[2] |

### Exploration et navigation

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `exploration/frontiers` | `dronoto_msgs/FrontierArray` | 1 | R·V[2] |
| `exploration/candidates` | `dronoto_msgs/ViewpointArray` | 1 | R·V[2] |
| `exploration/scored_candidates` | `dronoto_msgs/ScoredViewpointArray` | 1 | BE·V[2] |
| `exploration/goal` | `geometry_msgs/PoseStamped` | 0,5 | R·TL[1] |
| `exploration/coverage` | `dronoto_msgs/CoverageStats` | 0,5 | R·TL[1] |
| `navigation/path` | `nav_msgs/Path` | à la demande | R·TL[1] |
| `navigation/trajectory` | `dronoto_msgs/Trajectory` | à la demande | R·TL[1] |

### Contrôle (chaîne de veto)

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `control/setpoint_raw` | `dronoto_msgs/ControlSetpoint` | 20 | R·V[1] |
| `control/setpoint_safe` | `dronoto_msgs/ControlSetpoint` | 20 | R·V[1] |
| `control/setpoint_final` | `dronoto_msgs/ControlSetpoint` | 20 | R·V[1] |

Profondeur 1 : une consigne périmée est pire qu'aucune consigne. On ne veut jamais qu'une
file accumule des consignes anciennes.

### Sécurité, liaison, perception

| Topic | Type | Hz | QoS |
|---|---|---|---|
| `safety/state` | `dronoto_msgs/SafetyState` | 10 | R·TL[1] |
| `safety/avoidance_status` | `dronoto_msgs/AvoidanceStatus` | 30 | BE·V[1] |
| `safety/events` | `dronoto_msgs/SafetyEvent` | événementiel | R·TL[50] |
| `link/status` | `dronoto_msgs/LinkStatus` | 1 | R·TL[1] |
| `link/inbound_command` | `dronoto_msgs/GroundCommand` | événementiel | R·V[20] |
| `perception/detections` | `vision_msgs/Detection3DArray` | 2-5 | BE·V[5] |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | 2 | R·V[10] |
| `sim/ground_truth` | `nav_msgs/Odometry` | 100 | BE·V[10] |
| `sim/active_faults` | `dronoto_msgs/FaultStatus` | 1 | R·TL[1] |

**Choix de QoS notables et pourquoi.** `safety/state` et `safety/events` sont
`transient_local` : un nœud qui démarre ou redémarre doit connaître l'état de sécurité
courant sans attendre la publication suivante. À l'inverse, tous les flux capteur sont
`best effort` : retransmettre un nuage de points vieux de 200 ms nuit plus qu'il n'aide, et
la fiabilité DDS sur des messages volumineux à haute fréquence provoque des à-coups de
latence.

## 5. Services

| Service | Type | Serveur | Usage |
|---|---|---|---|
| `safety/trigger_failsafe` | `dronoto_msgs/srv/TriggerFailsafe` | `safety_supervisor` | Forcer un état de sécurité (RTH, atterrissage, stationnaire) |
| `safety/set_policy` | `dronoto_msgs/srv/SetSafetyPolicy` | `safety_supervisor` | Modifier la politique de perte de liaison, les seuils |
| `mission/load` | `dronoto_msgs/srv/LoadMission` | `mission_executive` | Charger une mission sans la démarrer |
| `mission/query_state` | `dronoto_msgs/srv/QueryMissionState` | `mission_executive` | État détaillé de l'arbre de comportement |
| `map/save` | `dronoto_msgs/srv/SaveMap` | `map_server_3d` | Persister la carte (fin de mission, point de reprise) |
| `map/load` | `dronoto_msgs/srv/LoadMap` | `map_server_3d` | Reprise après redémarrage |
| `map/reset` | `std_srvs/srv/Trigger` | `map_server_3d` | Nouvelle mission |
| `exploration/set_area` | `dronoto_msgs/srv/SetExplorationArea` | `nbv_selector` | Définir ou redéfinir le polygone à explorer |
| `exploration/set_weights` | `dronoto_msgs/srv/SetScoringWeights` | `nbv_selector` | Ajuster les poids du scoring à chaud (réglage, dashboard) |
| `px4/arm`, `px4/disarm` | `std_srvs/srv/Trigger` | `px4_interface` | Armement, avec vérifications préalables |
| `px4/set_mode` | `dronoto_msgs/srv/SetFlightMode` | `px4_interface` | — |
| `sim/inject_fault` | `dronoto_msgs/srv/InjectFault` | `fault_injector` | Simulation uniquement |
| `sim/clear_faults` | `std_srvs/srv/Trigger` | `fault_injector` | Simulation uniquement |

**Règle** : aucun service n'est appelé depuis le chemin de contrôle. Les services sont
bloquants par nature ; les mettre dans une boucle temps réel est une faute classique. Le
chemin de contrôle n'utilise que des topics.

## 6. Actions

Les actions couvrent les opérations longues, annulables, avec retour d'avancement — c'est
exactement le profil des opérations de mission.

| Action | Type | Serveur | Objectif / Retour / Résultat |
|---|---|---|---|
| `mission/execute` | `dronoto_msgs/action/ExecuteMission` | `mission_executive` | **Objectif** : mission (zone, contraintes, politiques). **Retour** : phase, couverture %, batterie, temps restant estimé. **Résultat** : statut final, statistiques, référence de la carte. |
| `navigation/plan_path` | `dronoto_msgs/action/PlanPath` | `global_planner` | **Objectif** : départ, arrivée, contraintes. **Retour** : nœuds explorés. **Résultat** : chemin, coût, faisabilité. |
| `navigation/goto_pose` | `dronoto_msgs/action/GoToPose` | `trajectory_follower` | **Objectif** : pose cible, tolérance, vitesse max. **Retour** : distance restante, temps estimé. **Résultat** : atteint / interrompu / échec. |
| `flight/takeoff` | `dronoto_msgs/action/Takeoff` | `mission_executive` | **Objectif** : altitude. **Retour** : altitude courante. |
| `flight/land` | `dronoto_msgs/action/Land` | `mission_executive` | **Objectif** : pose (optionnelle), recherche de site sûr. **Retour** : phase de descente. |
| `flight/return_home` | `dronoto_msgs/action/ReturnHome` | `mission_executive` | **Objectif** : altitude de croisière, comportement à l'arrivée. **Retour** : distance restante, marge de batterie. |
| `exploration/explore_area` | `dronoto_msgs/action/ExploreArea` | `mission_executive` | **Objectif** : polygone, bande d'altitude, critère d'arrêt. **Retour** : couverture, frontières restantes, objectif courant. |

**Toutes les actions sont annulables et leur annulation laisse le drone dans un état sûr**
(vol stationnaire stabilisé), jamais dans un état indéterminé. Cette propriété est testée
explicitement pour chaque action (L1).

## 7. Paramètres

Fichiers YAML par nœud dans `drone/ros2/config/`, chargés au lancement, versionnés.
Les paramètres réglables à chaud (poids du scoring, seuils de failsafe, cadence de
télémétrie) déclarent des descripteurs avec bornes et sont validés par des callbacks — un
paramètre hors borne est refusé, pas appliqué silencieusement.

Profils de configuration : `sim_default`, `sim_degraded`, `hardware_jetson`. Un profil est
un jeu complet et cohérent ; on ne mélange pas.

## 8. Groupes de lancement

```
bringup_sim.launch.py          simulation complète : gz + PX4 + pont + drone + RViz
bringup_drone.launch.py        nœuds embarqués seuls (réutilisé au réel tel quel)
  ├── interface.launch.py      px4_interface, agent uXRCE-DDS
  ├── perception.launch.py     lidar_odometry, map_server_3d, esdf_builder
  ├── ai.launch.py             perception_detector, semantic_mapper, risk_map
  ├── navigation.launch.py     global_planner, trajectory_generator/follower, reactive_avoidance
  ├── exploration.launch.py    frontier_detector, viewpoint_sampler, nbv_selector
  ├── mission.launch.py        mission_executive
  ├── safety.launch.py         safety_supervisor, health_monitor
  └── comm.launch.py           comm_manager, telemetry_aggregator
bringup_ground.launch.py       passerelle station sol
test_scenario.launch.py        simulation headless + injecteur de pannes + assertions
```

`bringup_drone.launch.py` est **le fichier qui migrera tel quel sur le Jetson**. Le fait
qu'il ne référence ni Gazebo, ni le pont, ni l'injecteur de pannes est une propriété
vérifiée, pas une intention.

## 9. Composition et exécuteurs

Les nœuds de la chaîne critique (`px4_interface`, `reactive_avoidance`,
`trajectory_follower`, `safety_supervisor`) sont des **composants** chargés dans un
conteneur unique. Bénéfice : communication intra-processus sans copie (`intra_process_comms`),
ce qui supprime la sérialisation et retire plusieurs millisecondes du budget des 50 ms.

Les nœuds lourds mais non critiques (`lidar_odometry`, `map_server_3d`,
`perception_detector`) tournent en processus séparés : leur défaillance ou leur famine CPU
ne doit pas emporter la chaîne de sécurité. C'est une décision d'isolation de fautes, pas de
performance.

Exécuteurs multithread avec groupes de callbacks explicites pour les nœuds qui mélangent
timers rapides et callbacks lents — sinon un callback lent bloque le timer de contrôle. Ce
piège est la cause la plus fréquente de retards inexpliqués en ROS 2 ; on le traite par
conception.
