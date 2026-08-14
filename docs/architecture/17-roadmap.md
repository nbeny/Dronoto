# 17 — Roadmap

## Principe

**Chaque phase produit quelque chose qui fonctionne et qui se teste.** Pas de phase
« infrastructure » de six semaines avant le premier vol. Le premier décollage autonome est
en semaine 3.

Chaque phase a :
- un **objectif** en une phrase ;
- des **livrables** concrets ;
- des **critères de sortie mesurables** — la phase n'est pas finie tant qu'ils ne sont pas
  atteints, et « le code est écrit » n'est pas un critère ;
- une **estimation** en semaines pour une personne à temps partiel sérieux (~15 h/semaine).

Les estimations sont des ordres de grandeur. La variance sur ce type de projet est élevée,
surtout en P2 (le SLAM est là où on perd du temps).

---

## Vue d'ensemble

```
P0  Fondations                    2 sem   ▓▓
P1  Ça vole                       3 sem   ▓▓▓
P2  Ça se localise et cartographie 5 sem  ▓▓▓▓▓
P3  Ça évite et navigue           4 sem   ▓▓▓▓
P4  Ça explore seul               5 sem   ▓▓▓▓▓
P5  Ça communique et se supervise 5 sem   ▓▓▓▓▓
P6  Ça survit                     4 sem   ▓▓▓▓
P7  Ça comprend ce qu'il voit     4 sem   ▓▓▓▓
P8  Durcissement et flotte        4 sem   ▓▓▓▓
P9  Matériel réel                 8 sem+  ▓▓▓▓▓▓▓▓
                                  ─────
                          logiciel : ~36 semaines
```

Environ **8 à 9 mois de logiciel** avant de toucher au matériel. C'est cohérent avec la
contrainte « ne passer au hardware réel qu'après validation complète du logiciel ».

---

## P0 — Fondations *(2 semaines)*

**Objectif** : un environnement de travail reproductible et une CI verte.

**Livrables**
- Ubuntu 24.04 installé (dual-boot), ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL épinglé
- Structure du dépôt ([15](15-structure-depot.md)), paquet `dronoto_msgs`, schémas Protobuf
- Génération de code Protobuf (Python, TypeScript, C++) automatisée
- Images Docker et profils Compose
- CI GitHub Actions : lint, proto, unit, build
- Ce dossier d'architecture committé

**Critères de sortie**
- `docker compose --profile sim up` lance Gazebo avec un drone qui apparaît
- `colcon build` réussit de zéro en moins de 15 min
- La CI passe sur une PR vide
- `tools/generate_proto.sh` produit du code identique à celui versionné

---

## P1 — Ça vole *(3 semaines)*

**Objectif** : décollage, waypoint, atterrissage, en autonomie, testés automatiquement.

**Livrables**
- Modèle `x500_lidar_dronoto` en SDF (capteurs déclarés, même si non exploités)
- Monde `empty_field`
- `px4_interface` : conversions de repères, flux offboard, armement, modes
- `mission_executive` minimal : arbre BT décollage → waypoint → atterrissage
- `safety_supervisor` V1 : états, chiens de garde de base, veto sur consigne périmée
- `trajectory_follower` simple (interpolation de waypoints)
- Arbre TF complet, visualisation RViz
- Scénarios L3 `takeoff_land.yaml`, `waypoint_navigation.yaml` et le moteur de DSL

**Critères de sortie**
- Les deux scénarios passent **10 fois de suite** sans intervention
- Précision d'arrivée au waypoint < 0,5 m
- Couper `trajectory_follower` en vol → le drone passe en `HOLD` en moins de 500 ms
- Tests L0 des conversions de repères à 100 % de couverture

> **C'est le jalon le plus important du projet.** À la fin de P1, il y a un drone autonome
> qui vole et un harnais de test qui le prouve. Tout le reste s'accroche là-dessus.

---

## P2 — Ça se localise et cartographie *(5 semaines)*

**Objectif** : le drone construit une carte 3D exploitable et sait où il est sans GPS.

**Livrables**
- LiDAR simulé calé sur le Mid-360, IMU, GNSS, magnétomètre, baromètre
- `sensor_faults` : bruit, biais, latences (pannes en P6)
- `lidar_odometry` (FAST-LIO2 porté ROS 2) + `odometry_health`
- Injection d'odométrie externe dans EKF2, calibration de `EKF2_EV_DELAY`
- `map_server_3d` (OctoMap) + affichage RViz
- `esdf_builder`
- Outillage de métriques : ATE, complétude et précision de carte
- Scénarios `slam_accuracy.yaml`
- Tests L2 sur rosbags de référence

**Critères de sortie**
- **ATE < 0,5 m RMS sur 500 m** avec GNSS ([06 §4](06-slam-et-cartographie.md))
- **ATE < 2,0 m RMS sur 200 m** sans GNSS
- Carte visible dans RViz, cohérente avec le monde, complétude > 85 %
- `EKF2_EV_DELAY` mesuré et documenté
- Insertion OctoMap < 200 ms au 95e centile
- `estimator_status_flags` confirme la fusion de l'odométrie externe

**Risque principal** : le portage ROS 2 de FAST-LIO2 et son réglage. Prévoir une semaine de
marge. Repli si le budget de dérive n'est pas tenu : bascule vers GLIM
([ADR-0004](../adr/0004-slam.md)).

---

## P3 — Ça évite et navigue *(4 semaines)*

**Objectif** : le drone atteint un point en contournant les obstacles, sans jamais toucher.

**Livrables**
- Monde `pillars`
- `global_planner` (A*/JPS sur voxels), action `PlanPath`
- `trajectory_generator` (lissage polynomial contraint par l'ESDF)
- `reactive_avoidance` — **le premier point de veto**
- Replanification sur changement de carte
- Scénarios `obstacle_avoidance.yaml`, `reactive_veto.yaml`

**Critères de sortie**
- **Zéro collision sur 20 exécutions** de `obstacle_avoidance.yaml`
- Distance minimale aux obstacles > 1,5 m sur toutes les exécutions
- Latence LiDAR → veto < 50 ms au 99e centile
- Le veto fonctionne **avec la carte désactivée** (preuve d'indépendance du chemin court)
- Planification < 500 ms sur un volume 100 × 100 × 40 m

---

## P4 — Ça explore seul *(5 semaines)*

**Objectif** : mission « cartographie cette zone », le drone décide seul de tout le reste.

**Livrables**
- Mondes `urban_block` et `forest`
- `frontier_detector` (avec filtrage d'atteignabilité et mise à jour incrémentale)
- `viewpoint_sampler`, `nbv_selector` avec le scoring complet
- Hystérésis et engagement minimal
- `pose_graph` (GTSAM) : fermeture de boucle, facteurs GNSS, correction `map → odom`
- Arbre BT d'exploration complet
- Métriques d'exploration
- Scénarios `exploration_coverage.yaml`, `exploration_efficiency.yaml`, `slam_loop_closure.yaml`

**Critères de sortie**
- **Couverture ≥ 90 % de `urban_block` en moins de 15 minutes**
- Efficacité de trajet < 2,0
- Oscillations < 4 changements d'objectif par minute
- Latence de décision NBV < 500 ms au 95e centile
- Fermeture de boucle : réduction de dérive mesurable sur `slam_loop_closure.yaml`
- Retour à la base déclenché par la **contrainte de faisabilité batterie**, pas par un seuil fixe

> À la fin de P4, le cœur robotique du projet est fait. Ce qui suit est de l'intégration
> système — important, mais moins incertain.

---

## P5 — Ça communique et se supervise *(5 semaines)*

**Objectif** : créer une mission depuis un dashboard, la voir s'exécuter, récupérer la carte.

Parallélisable avec P2-P4 : ne dépend que de P1.

**Livrables**
- Bibliothèque `drone/communication/` : `CommunicationLink`, `VirtualRadioLink`,
  `LoopbackLink`, DLP, modèle de canal, spool
- `comm_manager`, `telemetry_aggregator`
- Passerelle station sol
- Serveur FastAPI : modules missions, drones, telemetry, maps, events ; bus interne
- PostgreSQL + TimescaleDB + PostGIS, migrations Alembic
- API REST complète + WebSocket
- Dashboard : bandeau, carte 2D, télémétrie, événements, contrôle de mission
- Scénario `mission_modification.yaml`

**Critères de sortie**
- Mission créée au dashboard → exécutée par le drone → carte visible au sol
- Télémétrie affichée avec moins de 3 s de latence
- Modification de mission en vol appliquée et acquittée (versionnement fonctionnel)
- Budget radio respecté : plan de gestion < 5 % de 50 kbps
- `test_no_control_plane_on_radio` passe
- Tests L0 du protocole et du canal à > 90 % de couverture

---

## P6 — Ça survit *(4 semaines)*

**Objectif** : toutes les pannes du brief sont injectables, testées et survécues.

**Livrables**
- `fault_injector` complet ([03 §7](03-simulation.md))
- Occlusion GNSS géométrique dans `urban_block`
- `safety_supervisor` complet : matrice de dégradation, priorité des conditions
- Politiques de perte de liaison, spool et resynchronisation validés
- Retour à la base par trajectoire inverse en GNSS perdu
- Sélection de site d'atterrissage (géométrique seule ; sémantique en P7)
- Mondes `warehouse_interior`, `mixed_mission`
- **Tous** les scénarios de résilience, dont `full_resilience.yaml`
- Première campagne L4

**Critères de sortie**
- **`full_resilience.yaml` (les 13 étapes du brief) passe**
- Chaque ligne de la matrice de dégradation a un test qui passe
- L4 `rth_reliability` : **taux de réussite ≥ 99 % sur 100 exécutions**
- Zéro collision sur toute la campagne L4
- Tests L0 de la machine à états : exhaustifs
- `server_down.yaml` : aucun impact mesurable sur le drone

---

## P7 — Ça comprend ce qu'il voit *(4 semaines)*

**Objectif** : la perception IA enrichit l'exploration et l'atterrissage, sans les conditionner.

**Livrables**
- Runtime `drone/perception/inference/` : ONNX Runtime multi-fournisseur
- `perception_detector` (YOLOv8n), `semantic_mapper`, `risk_map`
- Couche sémantique avec accumulation bayésienne
- Sélection de site d'atterrissage : géométrie (veto) + sémantique (pondération)
- Pondération sémantique dans le scoring d'exploration
- Pipeline d'auto-annotation depuis Gazebo (jeu de validation)
- Observabilité IA : latences, distributions de confiance, version de modèle

**Critères de sortie**
- mAP@0,5 > 0,45 sur le jeu de validation
- Latence d'inférence < 100 ms au 95e centile sur CPU
- **`mission_without_ai.yaml` passe** — mission complète, nœuds IA arrêtés
- Site d'atterrissage : zéro sélection sur eau ou pente dans les tests
- Bascule de fournisseur ONNX par configuration, sans modification de code

---

## P8 — Durcissement et flotte *(4 semaines)*

**Objectif** : plusieurs drones, sécurité de la liaison, stabilité prouvée dans la durée.

**Livrables**
- Multi-drones : namespaces, domaines DDS, démultiplexage sol, dashboard multi-drones
- Sécurité : HMAC obligatoire sur la liaison, anti-rejeu, chiffrement optionnel
- Serveur : authentification, RBAC, journal d'audit
- Dashboard : rejeu de mission, vue flotte
- Profilage et optimisation ciblés (les métriques de latence désignent les cibles)
- Campagnes L4 hebdomadaires automatisées
- Évaluation de la migration ROS 2 Lyrical Luth

**Critères de sortie**
- 3 drones simultanés en simulation, sans dégradation des budgets de latence
- L4 hebdomadaire vert **4 semaines consécutives**
- Trame non authentifiée rejetée et journalisée
- Documentation d'exploitation complète

---

## P9 — Matériel réel *(8 semaines et plus)*

**Objectif** : voler pour de vrai, en suivant le chemin de transition de
[16 §6](16-materiel.md).

**Livrables**
- Étapes H1 à H9
- Pilotes réels : Livox SDK2, OAK-D, GNSS RTK
- `SerialRadioLink`
- Fournisseur ONNX TensorRT sur Jetson
- Recalage du modèle Gazebo sur la masse et la consommation réelles
- Dossier réglementaire (enregistrement, Remote ID, assurance ; SORA si BVLOS)
- Procédures d'essais et checklists

**Critères de sortie**
- H2 : ATE du SLAM sur données réelles dans le même ordre de grandeur qu'en simulation
- H4 : tous les failsafes vérifiés au sol, hélices retirées
- H7 : vol offboard autonome, pilote en veille, sans reprise nécessaire
- H9 : mission d'exploration complète en conditions réelles, en catégorie A3

**Les étapes H1 et H2 peuvent démarrer dès P4** — il suffit du LiDAR, d'une IMU et du
Jetson. C'est le moyen le moins cher de dérisquer tôt la partie la plus incertaine.

---

## Ce qui vient après

Pistes classées par valeur décroissante, à réévaluer une fois P9 atteint :

| Piste | Intérêt | Note |
|---|---|---|
| Optimisation de l'autonomie (planification énergétique) | Élevé | Gain direct sur la capacité opérationnelle |
| `wavemap` en remplacement d'OctoMap | Moyen-élevé | Si la mémoire ou le CPU deviennent limitants |
| Fusion de cartes multi-drones | Élevé | Débloque l'exploration coopérative — projet à part entière |
| Prédiction apprise du gain d'information | Moyen | Uniquement si le lancer de rayons dépasse son budget |
| Isaac Sim pour l'entraînement de la perception | Moyen | Nécessite un GPU NVIDIA ; utile pour le sim-to-real visuel |
| RL pour la politique d'exploration | **Faible** | Barre fixée haut ([07 §7](07-exploration.md)) : > 20 % de gain hors distribution |
| Migration ROS 2 Lyrical Luth | Faible-moyen | Quand l'écosystème aura rattrapé |

---

## Risques principaux

| Risque | Probabilité | Impact | Atténuation |
|---|---|---|---|
| Réglage de FAST-LIO2 plus long que prévu | Élevée | P2 décalé | Marge d'une semaine ; repli GLIM ; commencer par des mondes simples |
| Performance de la simulation insuffisante en `urban_block` | Moyenne | L4 impraticable | Réduire la densité LiDAR ; mondes headless ; le 5950X a de la marge |
| Instabilité du mode offboard PX4 | Moyenne | P1 décalé | Utiliser `px4_ros2_cpp` plutôt que du code maison ; version PX4 épinglée |
| Découverte DDS en conteneur | Moyenne | Perte de temps diffuse | `network_mode: host` décidé d'emblée ([13 §4](13-docker-infra.md)) |
| Dérive de version PX4 / `px4_msgs` | Moyenne | Bugs silencieux | Sous-module épinglé, mise à jour délibérée et testée |
| Sous-estimation de P5 (serveur + dashboard) | Élevée | Calendrier | Périmètre V1 réduit assumé ; le dashboard peut rester minimal |
| Perte de motivation sur un projet de 9 mois | **Élevée** | Abandon | **C'est le vrai risque.** Chaque phase produit une démonstration visible. P1 en 3 semaines, pas en 3 mois. |

Le dernier risque est celui qui tue le plus de projets de cette ampleur, et c'est pourquoi
la roadmap est construite autour de démonstrations fréquentes plutôt que d'un grand
assemblage final.
