# 15 — Structure du dépôt

## 1. Monorepo

Un seul dépôt. Justification : les contrats (messages ROS 2, schémas Protobuf, modèles de
données) sont partagés entre le drone, le serveur et le dashboard. En dépôts séparés,
chaque modification de contrat devient une coordination de versions à trois — pour un
développeur, c'est du pur surcoût. Le monorepo garantit qu'un commit est cohérent partout.

Le passage en multi-dépôts se justifierait si des équipes distinctes travaillaient à des
rythmes différents. Ce n'est pas le cas, et ce ne le sera pas avant longtemps.

## 2. Arborescence

Adaptée de la proposition du brief. Les ajouts sont signalés et justifiés.

```
Dronoto/
│
├── interfaces/                       ◀── AJOUT : source unique des contrats
│   ├── proto/dronoto/v1/             schémas Protobuf (liaison radio)
│   │   ├── link.proto
│   │   ├── telemetry.proto
│   │   ├── mission.proto
│   │   ├── event.proto
│   │   └── map.proto
│   ├── ros_msgs/                     paquet ROS 2 dronoto_msgs
│   │   ├── msg/    OdometryHealth.msg, SafetyState.msg, LinkStatus.msg, ...
│   │   ├── srv/    InjectFault.srv, TriggerFailsafe.srv, ...
│   │   └── action/ ExecuteMission.action, ExploreArea.action, ...
│   └── schemas/                      JSON Schema : scénarios, config, missions
│
├── simulation/
│   ├── gazebo/
│   │   ├── models/x500_lidar_dronoto/    SDF, meshes, capteurs
│   │   ├── plugins/                      plugins Gazebo spécifiques
│   │   └── config/
│   ├── px4/
│   │   ├── PX4-Autopilot/                sous-module, commit ÉPINGLÉ
│   │   ├── airframes/                    définition de l'airframe Dronoto
│   │   └── params/                       jeux de paramètres PX4 versionnés
│   ├── worlds/
│   │   ├── empty_field.sdf   pillars.sdf   urban_block.sdf
│   │   └── forest.sdf        warehouse_interior.sdf   mixed_mission.sdf
│   ├── config/
│   │   ├── radio_rfd868x.yaml    radio_microhard.yaml
│   │   ├── sensor_noise.yaml     wind_profiles.yaml
│   └── launch/
│
├── drone/                            ◀── tout ce qui vole
│   ├── ros2/                         workspace colcon
│   │   ├── dronoto_bringup/          fichiers de lancement, configuration
│   │   ├── dronoto_interface/        px4_interface (C++)
│   │   ├── dronoto_perception/       detector, semantic_mapper, risk_map (Python)
│   │   ├── dronoto_slam/             lidar_odometry, pose_graph (C++)
│   │   ├── dronoto_mapping/          map_server_3d, esdf_builder (C++)
│   │   ├── dronoto_navigation/       planner, trajectory, follower, avoidance (C++)
│   │   ├── dronoto_exploration/      frontiers, viewpoints, nbv (C++)
│   │   ├── dronoto_mission/          mission_executive, arbres BT (C++)
│   │   ├── dronoto_safety/           safety_supervisor, health_monitor (C++)
│   │   ├── dronoto_comm/             comm_manager, telemetry_aggregator (Python)
│   │   └── dronoto_sim/              sensor_faults, fault_injector (Python) — SIM SEULEMENT
│   ├── communication/                ◀── bibliothèque Python PURE, sans ROS
│   │   ├── link/                     base.py, virtual.py, loopback.py, serial.py, ip.py
│   │   ├── protocol/                 framing.py, codec.py, priority.py
│   │   ├── spool/                    store-and-forward
│   │   └── channel_model/            modèle de propagation radio
│   └── perception/                   ◀── bibliothèque Python PURE, sans ROS
│       ├── inference/                runtime ONNX, gestion des fournisseurs
│       ├── detectors/  segmenters/
│       └── postprocess/
│
├── server/
│   ├── api/       modules/    core/
│   ├── groundstation/
│   ├── migrations/                   Alembic
│   └── tests/
│
├── dashboard/
│   ├── src/  { components/  views/  hooks/  api/  store/  layers/ }
│   ├── public/  tiles/               ◀── tuiles de carte hors ligne
│   └── tests/
│
├── models/                           ◀── artefacts IA (pointeurs DVC, pas les binaires)
│   ├── detector/   segmentation/
│   └── README.md
│
├── infrastructure/
│   ├── docker/    compose/    ci/
│   └── scripts/                      bootstrap, setup workspace, génération proto
│
├── tests/                            ◀── AJOUT : tests transverses
│   ├── scenarios/                    scénarios L3 en YAML
│   ├── soak/                         campagnes L4
│   ├── integration/                  L2
│   ├── conformance/                  tests d'architecture
│   ├── bags/                         rosbags de référence (DVC)
│   └── runner/                       moteur d'exécution du DSL
│
├── tools/                            ◀── AJOUT : outillage
│   ├── calibrate_ev_delay.py
│   ├── analyze_bag.py                métriques hors ligne depuis un rosbag
│   ├── map_postprocess.py            Open3D : maillage, nettoyage, export
│   ├── replay_mission.py
│   └── generate_proto.sh
│
├── docs/
│   ├── architecture/                 ce dossier
│   ├── adr/                          décisions d'architecture
│   ├── guides/                       installation, développement, exploitation
│   └── superpowers/specs/            spécifications détaillées par sous-projet
│
├── .github/workflows/
├── CLAUDE.md                         conventions pour l'assistance IA
├── README.md
└── LICENSE                           Apache 2.0
```

## 3. Écarts par rapport à la proposition du brief

Quatre modifications, chacune motivée.

### `interfaces/` — source unique des contrats

**Le changement le plus important.** Le brief plaçait implicitement les définitions de
messages dans chaque composant. Les regrouper garantit qu'il n'existe **qu'une seule
définition** de chaque contrat, à partir de laquelle on génère le code Python (drone et
serveur), TypeScript (dashboard) et C++ (nœuds critiques).

Sans cela, on maintient trois définitions à la main de la même structure de télémétrie, et
elles divergent — c'est une certitude, pas un risque. Le bug qui en résulte (un champ
interprété différemment aux deux bouts) est silencieux et se manifeste en vol.

### Séparation des bibliothèques pures et des nœuds ROS 2

`drone/communication/` et `drone/perception/` sont des **bibliothèques Python sans aucune
dépendance ROS 2**. Les nœuds ROS (`dronoto_comm`, `dronoto_perception`) sont de fines
enveloppes autour d'elles.

Trois bénéfices concrets :

- **Tests L0 rapides** : tester le protocole DLP ou le modèle de canal ne demande ni ROS ni
  environnement — pytest pur, quelques secondes.
- **Réutilisation au sol** : la passerelle station sol utilise `drone/communication/`
  telle quelle. Une seule implémentation du protocole aux deux extrémités, donc pas de
  divergence possible entre l'émetteur et le récepteur.
- **Portabilité** : cette logique n'est pas prisonnière de ROS 2.

### `tests/` au niveau racine

Les scénarios L3, les campagnes L4 et les tests de conformité sont transverses par nature —
ils ne peuvent pas vivre dans un paquet particulier. Les tests unitaires, eux, restent avec
leur code.

### `models/` séparé

Les artefacts ONNX sont volumineux et binaires. Ils ne vont pas dans git mais dans DVC ou
des releases GitHub, référencés par hachage. Le répertoire contient les pointeurs et les
métadonnées versionnées.

## 4. Conventions

### Branches et commits

- `main` protégée, toujours verte.
- Branches `feat/`, `fix/`, `docs/`, `refactor/`, `test/`.
- Commits conventionnels avec portée : `feat(exploration): ajoute l'hystérésis du NBV`.
- Une PR = une préoccupation. La CI doit passer.

### Versionnement

Version sémantique globale. Les composants ne sont pas versionnés séparément — c'est la
conséquence assumée du monorepo, et cela simplifie la vie.

**Compatibilité des schémas Protobuf** : les numéros de champ ne sont **jamais** réutilisés,
les champs supprimés sont marqués `reserved`. Cette discipline garantit qu'un drone
embarquant une ancienne version du protocole reste compréhensible par un serveur récent —
propriété indispensable dès qu'un drone est déployé sur le terrain et pas trivialement
reflashable.

### Style

| Langage | Formatage | Lint | Types |
|---|---|---|---|
| Python | ruff format | ruff | mypy strict sur les bibliothèques pures |
| C++ | clang-format (ROS 2) | clang-tidy | C++17 |
| TypeScript | prettier | eslint | strict |

Tout est appliqué par pre-commit et vérifié en CI.

### Documentation

- Chaque paquet ROS 2 a un `README.md` : rôle, entrées, sorties, paramètres.
- Les décisions structurantes deviennent des ADR dans `docs/adr/`.
- Les documents d'architecture sont mis à jour **avec** le code, dans la même PR. Un
  document d'architecture obsolète est pire qu'absent : il induit en erreur.

### Licence

**Apache 2.0.** Compatible avec l'écosystème (ROS 2 et PX4 sont sous Apache 2.0 et
BSD-3), inclut une clause de brevet, et permet un usage commercial ultérieur sans
contamination — contrairement à la GPL, qui poserait problème si le projet devait un jour
intégrer des composants propriétaires de partenaires.
