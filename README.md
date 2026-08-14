# Dronoto

Drone autonome piloté par IA : cartographie 3D, exploration autonome de zones inconnues,
liaison radio longue portée. Conçu pour fonctionner **100 % en simulation** d'abord, avec
une architecture transposable sur matériel réel sans réécriture.

> **État du projet** : dossier d'architecture (v1.0, 2026-08-14). Aucun code n'est encore
> écrit. La phase P0 démarre l'implémentation.

## Le principe qui gouverne tout

**La sécurité et le contrôle temps réel sont locaux et déterministes ; l'intelligence et la
supervision sont distantes et optionnelles.**

Le drone accomplit sa mission avec le serveur éteint, la radio coupée et le GPS perdu. Le
serveur est un organe de commande et d'observation, jamais un organe de vol.

```
Serveur → mission haut niveau → radio → calculateur embarqué → ROS 2 → PX4 → moteurs
                                          └── autonome, déterministe, sûr ──┘
```

## Stack

| Domaine | Choix |
|---|---|
| Simulation | Gazebo Sim Harmonic + PX4 SITL |
| Middleware | ROS 2 Jazzy (Ubuntu 24.04) |
| Lien autopilote | uXRCE-DDS + `px4_ros2_cpp` |
| Localisation | FAST-LIO2 + PX4 EKF2 + graphe de poses GTSAM |
| Carte 3D | OctoMap (libre / occupé / **inconnu**) + ESDF local |
| Exploration | Frontières → points de vue → scoring multicritère → NBV |
| IA | ONNX Runtime multi-backend (CPU / ROCm / DirectML / TensorRT) |
| Radio | `CommunicationLink` + protocole DLP (Protobuf) |
| Serveur | FastAPI monolithe modulaire + PostgreSQL/TimescaleDB/PostGIS |
| Dashboard | React + TypeScript + MapLibre GL + deck.gl |

## Documentation

**→ [Dossier d'architecture complet](docs/README.md)**

18 documents couvrant l'architecture système, les choix technologiques et leurs
alternatives rejetées, l'architecture ROS 2, le SLAM, l'exploration autonome, l'IA, la
communication radio, les failsafes, le serveur, le dashboard, l'infrastructure, la
stratégie de tests, le matériel futur et la roadmap. Plus 7 ADR pour les décisions
structurantes et révisables.

## Roadmap

```
P0  Fondations                     2 sem
P1  Ça vole                        3 sem   ◀── premier vol autonome testé
P2  Ça se localise et cartographie 5 sem
P3  Ça évite et navigue            4 sem
P4  Ça explore seul                5 sem   ◀── cœur robotique terminé
P5  Ça communique et se supervise  5 sem
P6  Ça survit                      4 sem   ◀── scénario de résilience complet
P7  Ça comprend ce qu'il voit      4 sem
P8  Durcissement et flotte         4 sem
P9  Matériel réel                  8 sem+
```

Détail et critères de sortie mesurables : [17 — Roadmap](docs/architecture/17-roadmap.md).

## Licence

Apache 2.0
