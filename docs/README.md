# Dronoto — Dossier d'architecture

Drone autonome piloté par IA : cartographie 3D, exploration autonome, liaison radio
longue portée. Conçu pour fonctionner **100 % en simulation** d'abord, avec une
architecture transposable sur matériel réel sans réécriture.

> **Statut** : dossier d'architecture — version 1.0 (2026-08-14).
> Aucun code n'est encore écrit. Ce dossier est le contrat qui gouverne l'implémentation.

---

## Comment lire ce dossier

Lis dans l'ordre si tu découvres le projet. Sinon, chaque document est autonome.

| # | Document | Contenu |
|---|----------|---------|
| 00 | [Vision, principes et périmètre](architecture/00-vision-et-principes.md) | Objectifs, principes d'ingénierie non négociables, découpage en sous-projets |
| 01 | [Choix technologiques](architecture/01-choix-technologiques.md) | Stack définitive, alternatives rejetées et **pourquoi** |
| 02 | [Architecture système](architecture/02-architecture-systeme.md) | Diagrammes, plans (control/data/management), flux de données, protocoles |
| 03 | [Simulation](architecture/03-simulation.md) | Gazebo, PX4 SITL, modèle du drone, capteurs, modèle de vent et de bruit |
| 04 | [Architecture ROS 2](architecture/04-architecture-ros2.md) | Nodes, topics, services, actions, QoS, arbre TF, conventions |
| 05 | [Intégration PX4 / MAVLink](architecture/05-integration-px4.md) | uXRCE-DDS, offboard, EKF2, modes de vol, frontière de responsabilité |
| 06 | [Localisation, SLAM et cartographie 3D](architecture/06-slam-et-cartographie.md) | FAST-LIO2, fusion IMU/LiDAR/GNSS, perte GPS, dérive, carte 3D, zones inconnues |
| 07 | [Exploration autonome](architecture/07-exploration.md) | Frontières, échantillonnage de points de vue, scoring, NBV, planification |
| 08 | [Système IA](architecture/08-ia.md) | Où l'IA apporte de la valeur, où elle est nuisible, runtime d'inférence sans CUDA |
| 09 | [Communication radio](architecture/09-communication.md) | `CommunicationLink`, protocole DLP, modèle de dégradation radio, store-and-forward |
| 10 | [Failsafe et résilience](architecture/10-failsafe.md) | Machine à états de sécurité, matrice de dégradation, défense en profondeur |
| 11 | [Serveur](architecture/11-serveur.md) | API, modèles de données, base de données, bus d'événements |
| 12 | [Dashboard](architecture/12-dashboard.md) | Stack front, vues, rendu 3D, temps réel |
| 13 | [Docker et infrastructure](architecture/13-docker-infra.md) | Images, profils Compose, GPU, réseau DDS |
| 14 | [Stratégie de tests](architecture/14-tests.md) | Pyramide L0→L4, injection de pannes, DSL de scénarios, métriques, CI |
| 15 | [Structure du dépôt](architecture/15-structure-depot.md) | Arborescence, monorepo, contrats partagés, conventions |
| 16 | [Matériel futur](architecture/16-materiel.md) | BOM détaillée, coûts, contraintes réglementaires |
| 17 | [Roadmap](architecture/17-roadmap.md) | Phases P0→P9, livrables, critères de sortie mesurables |

## Décisions d'architecture (ADR)

Les décisions structurantes et contestables sont isolées pour pouvoir être révisées
sans toucher au reste du dossier.

| ADR | Décision |
|-----|----------|
| [0001](adr/0001-simulateur.md) | Gazebo Sim Harmonic plutôt qu'Isaac Sim |
| [0002](adr/0002-plateforme-de-developpement.md) | Ubuntu 24.04 natif ; pas d'achat de GPU NVIDIA |
| [0003](adr/0003-distribution-ros2.md) | ROS 2 Jazzy plutôt que Lyrical Luth |
| [0004](adr/0004-slam.md) | FAST-LIO2 comme odométrie LiDAR-inertielle |
| [0005](adr/0005-pas-de-nav2.md) | Ne pas utiliser Nav2 pour la navigation 3D |
| [0006](adr/0006-serveur-monolithe-modulaire.md) | Monolithe modulaire plutôt que microservices |
| [0007](adr/0007-protocole-radio.md) | Protobuf sur un protocole propre (DLP) |

---

## Résumé exécutif

**Le principe qui gouverne tout** : la sécurité et le contrôle temps réel sont
**locaux et déterministes** ; l'intelligence et la supervision sont **distantes et
optionnelles**. Le drone doit accomplir sa mission avec le serveur éteint, la radio
coupée et le GPS perdu. Le serveur est un organe de commande et d'observation, jamais
un organe de vol.

**Stack retenue** :

```
Simulation   Gazebo Sim Harmonic + PX4 SITL v1.16
Middleware   ROS 2 Jazzy (Ubuntu 24.04) — uXRCE-DDS vers PX4
Estimation   PX4 EKF2 (IMU/GNSS/baro/mag/EV) + FAST-LIO2 (odométrie LiDAR-inertielle)
Carte        OctoMap (occupation + inconnu) + ESDF local roulant
Exploration  Frontières + échantillonnage de points de vue + scoring multicritère (NBV)
Planification A*/JPS sur voxels + lissage polynomial + filtre réactif de sécurité
IA           ONNX Runtime multi-backend (CPU / DirectML / ROCm / TensorRT), PyTorch pour l'entraînement
Radio        Interface CommunicationLink + protocole DLP (Protobuf) + modèle de canal déterministe
Serveur      FastAPI (monolithe modulaire) + PostgreSQL/TimescaleDB/PostGIS
Dashboard    React + TypeScript + MapLibre GL + deck.gl
Infra        Docker Compose à profils, GitHub Actions
```

**Rejetés et pourquoi** (résumé — détail dans [01](architecture/01-choix-technologiques.md)) :
Isaac Sim (CUDA obligatoire, intégration PX4 faible), AirSim (abandonné), Gazebo Classic
(fin de vie), Nav2 (conçu pour le 2D au sol), slam_toolbox (2D uniquement), nvblox (CUDA),
microservices (coût opérationnel injustifié en V1), MAVLink comme protocole applicatif
radio (trop rigide pour des missions haut niveau).

**Ce que le projet n'est pas** : ce n'est pas un autopilote. PX4 fait le vol. Ce projet
construit la couche au-dessus : perception, cartographie, décision, communication,
supervision.
