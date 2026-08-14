# 14 — Stratégie de tests

## 1. Principe

Un système dont on ne peut pas tester la défaillance n'est pas testé. La stratégie n'est
donc pas organisée autour de la couverture de code mais autour des **modes de défaillance**
recensés dans [10 — Failsafe](10-failsafe.md) : chaque ligne de la matrice de dégradation
correspond à au moins un test automatisé.

Deuxième principe : **tout test doit être reproductible**. Toutes les sources d'aléa
(bruit capteur, canal radio, vent, échantillonnage) tirent de générateurs à graine fixée
par le scénario. Un échec est rejouable à l'identique. Un test qui échoue une fois sur dix
sans qu'on puisse le reproduire est pire qu'inutile : il apprend à ignorer les échecs.

## 2. Les cinq niveaux

```
     ┌──────────────────────────────────────────────┐
L4   │  SOAK / MONTE-CARLO   N=100 exécutions       │  hebdo    4 h
     │  Assertions statistiques                     │
     ├──────────────────────────────────────────────┤
L3   │  SCÉNARIOS SITL       Gazebo + PX4 + stack   │  nightly  45 min
     │  Injection de pannes, assertions de bout     │
     ├──────────────────────────────────────────────┤
L2   │  INTÉGRATION          Multi-nœuds, rosbag    │  par PR   15 min
     │  Sans Gazebo                                 │
     ├──────────────────────────────────────────────┤
L1   │  COMPOSANT            launch_testing         │  par PR   10 min
     │  Un nœud + producteurs factices              │
     ├──────────────────────────────────────────────┤
L0   │  UNITAIRE             pytest / gtest         │  par PR    5 min
     │  Logique pure, sans ROS                      │
     └──────────────────────────────────────────────┘
```

Beaucoup de projets robotiques n'ont que L3 : « on lance la simulation et on regarde ». Ça
ne marche pas — c'est lent, non déterministe, et quand ça casse on ne sait pas où. La
pyramide inverse le rapport : l'essentiel de la logique est testé en L0 en quelques
secondes, et L3 vérifie l'intégration, pas la logique.

## 3. L0 — Tests unitaires

Logique pure, aucune dépendance à ROS 2, exécution en secondes.

| Domaine | Ce qui est testé |
|---|---|
| Protocole DLP | Encodage/décodage, CRC, fragmentation, réassemblage, trames corrompues, ordre des priorités |
| Modèle de canal radio | Perte de parcours, PER en fonction du SNR, déterminisme avec graine, occlusion |
| Scoring d'exploration | Chaque terme isolé, normalisation, contrainte batterie, hystérésis |
| Détection de frontières | Sur des octrees synthétiques aux propriétés connues |
| Machine à états de sécurité | **Toutes** les transitions, priorité des conditions, atteignabilité de `EMERGENCY_LAND` depuis chaque état en vol |
| Conversions de repères | NED↔ENU, FRD↔FLU, quaternions, aller-retour, cas dégénérés |
| Calcul d'énergie | Réserve de retour, faisabilité, marges |
| Planificateur | A* sur grilles connues, optimalité, absence de chemin, raccourcissement |
| Gestion du spool | Rotation, quotas, ordre de vidage, fusion des deltas |
| Modèles de données | Validation Pydantic, sérialisation, cas limites |

**La machine à états de sécurité mérite un traitement particulier** : sa table de
transitions est testée **exhaustivement** (produit cartésien états × conditions). C'est
possible parce qu'elle est petite et déterministe — et c'est précisément pourquoi on a
refusé d'y mettre de l'IA.

Cible de couverture : **> 90 % sur les paquets `failsafe`, `communication`, `exploration`**.
Ailleurs, la couverture est un indicateur, pas un objectif.

## 4. L1 — Tests de composant

Un nœud réel, ses producteurs et consommateurs simulés, via `launch_testing`.

```python
# Exemple : le veto d'évitement fonctionne-t-il ?
def test_reactive_avoidance_vetoes_toward_obstacle(node_under_test):
    publish_pointcloud(obstacle_at=(3.0, 0.0, 0.0))       # 3 m devant
    publish_setpoint(velocity=(5.0, 0.0, 0.0))            # fonce dedans
    out = wait_for("control/setpoint_safe", timeout=0.2)
    assert out.velocity.x < 1.0                            # a freiné
    assert get("safety/avoidance_status").veto_active
```

Chaque nœud a sa suite. Vérifications systématiques pour tous :

- publie-t-il à la fréquence annoncée ?
- se comporte-t-il correctement quand une entrée manque ?
- se comporte-t-il correctement quand une entrée est périmée ?
- ses actions sont-elles annulables, et l'annulation laisse-t-elle un état sûr ?
- respecte-t-il son budget de latence ?

## 5. L2 — Tests d'intégration

Plusieurs nœuds réels, sans Gazebo. Les données viennent de **rosbags enregistrés**.

C'est le niveau le plus rentable et le plus souvent négligé. Il permet de tester la chaîne
perception → carte → exploration sur des données réalistes, en quelques secondes, de
manière parfaitement déterministe.

| Test | Chaîne | Assertion |
|---|---|---|
| Chaîne SLAM | rosbag LiDAR+IMU → `lidar_odometry` → `map_server_3d` | ATE < seuil, carte cohérente |
| Chaîne d'exploration | octree fixe → frontières → candidats → NBV | Objectif dans la zone, score cohérent |
| Chaîne de commande | commande sol → `comm_manager` → `mission_executive` | Mission acceptée et acquittée |
| Coupure de liaison | `LoopbackLink` coupé puis rétabli | Spool complet, resynchronisation sans perte |
| Cascade de failsafe | Injection d'états dégradés | Transitions correctes, consignes vetoées |

Les rosbags de référence sont versionnés (DVC ou release GitHub) et régénérés lorsque les
interfaces changent. Ils constituent la mémoire des cas difficiles rencontrés.

## 6. L3 — Scénarios SITL

Système complet, Gazebo headless, pannes injectées selon une chronologie, assertions de
bout en bout.

### DSL de scénario

```yaml
# tests/scenarios/gps_loss_continues_mission.yaml
name: "Perte GPS pendant l'exploration, la mission continue"
description: >
  Vérifie qu'à la perte du GNSS, EKF2 bascule sur l'odométrie externe, que la
  qualité de navigation est correctement signalée et que l'exploration se poursuit
  avec des paramètres dégradés.

world: urban_block
drone: x500_lidar_dronoto
seed: 20260814                  # OBLIGATOIRE — reproductibilité
timeout_s: 600
realtime_factor: 3.0

mission:
  type: survey_area
  area:
    polygon: [[0,0], [150,0], [150,150], [0,150]]
    min_alt_m: 15
    max_alt_m: 50
  constraints:
    coverage_target_pct: 80
    link_loss_policy: CONTINUE

timeline:
  - at: 0s
    action: start_mission
  - at: 30s
    assert: { safety_state: NOMINAL, altitude_m: { min: 14 } }
  - at: 120s
    inject: { fault: gnss_loss, mode: abrupt }
  - at: 125s
    assert:
      nav_quality: SLAM_ONLY
      ekf_fusing_external_vision: true
      mission_status: ACTIVE          # LE point du test
  - at: 125s..400s
    assert_continuous:
      min_obstacle_distance_m: { min: 1.5 }
      slam_ate_m:              { max: 2.0 }
      coverage_pct:            { increasing: true }
  - at: 400s
    assert: { coverage_pct: { min: 60 } }

final_assertions:
  - no_collision: true
  - landed_safely: true
  - distance_to_home_m: { max: 5.0 }
  - no_control_plane_message_on_radio: true
  - all_latency_budgets_respected: true
```

Le DSL rend un scénario lisible et modifiable sans écrire de code de test. C'est ce qui
permet d'en avoir cinquante plutôt que cinq.

### Catalogue des scénarios

| Fichier | Ce qu'il vérifie | Phase |
|---|---|---|
| `takeoff_land.yaml` | Décollage, stationnaire, atterrissage | P1 |
| `waypoint_navigation.yaml` | Suivi de waypoints, précision | P1 |
| `obstacle_avoidance.yaml` | Traversée d'un champ de piliers sans collision | P3 |
| `reactive_veto.yaml` | Obstacle surgissant : le veto agit | P3 |
| `slam_accuracy.yaml` | ATE sur une trajectoire de 500 m | P2 |
| `slam_loop_closure.yaml` | Fermeture de boucle, réduction de dérive | P4 |
| `slam_degenerate.yaml` | Couloir sans structure : dégénérescence détectée | P4 |
| `exploration_coverage.yaml` | ≥ 90 % de couverture dans le temps imparti | P4 |
| `exploration_efficiency.yaml` | Efficacité de trajet < 2,0, oscillations < 4/min | P4 |
| `gps_loss_continues_mission.yaml` | Perte GNSS, mission continue | P6 |
| `gps_spoofing.yaml` | GNSS trompeur rejeté | P6 |
| `gps_recovery.yaml` | Retour GNSS, cohérence exigée avant réintégration | P6 |
| `lidar_failure.yaml` | Perte LiDAR, mode dégradé puis retour | P6 |
| `radio_blackout.yaml` | Coupure radio, spool, resynchronisation | P6 |
| `radio_degradation.yaml` | Dégradation progressive, adaptation de débit | P6 |
| `battery_rth.yaml` | Réserve dynamique, retour, atterrissage | P6 |
| `battery_critical.yaml` | Atterrissage d'urgence sur site valide | P6 |
| `server_down.yaml` | Serveur arrêté : aucun impact sur le drone | P6 |
| `geofence_breach.yaml` | Retour immédiat sur franchissement | P6 |
| `cpu_overload.yaml` | Dégradation ordonnée sous charge | P6 |
| `mission_modification.yaml` | Modification en vol, versionnement | P5 |
| `full_resilience.yaml` | **Le scénario 13 étapes du brief** | P6 |
| `mission_without_ai.yaml` | Mission complète, nœuds IA arrêtés | P7 |

### Métriques collectées

Chaque exécution L3 produit un rapport JSON. Toutes ces métriques sont suivies dans le
temps — une régression est visible avant d'être douloureuse.

| Métrique | Calcul |
|---|---|
| ATE du SLAM | RMS de l'erreur de position vs `sim/ground_truth` |
| Erreur relative de pose | Dérive par 100 m |
| Couverture | Volume observé / volume explorable |
| Complétude de la carte | Voxels corrects / voxels réels (vérité terrain Gazebo) |
| Précision de la carte | Faux positifs d'occupation |
| Distance minimale aux obstacles | Minimum sur le vol |
| Énergie consommée | Wh, et Wh par m³ cartographié |
| Efficacité de trajet | Distance parcourue / optimum théorique |
| Latences | p50/p95/p99 par chaîne critique |
| Trafic radio | Octets par classe de priorité, taux de perte |
| Temps de réaction aux pannes | Injection → changement d'état |

## 7. L4 — Soak et Monte-Carlo

Le même scénario, N fois, avec des graines différentes. Les assertions deviennent
statistiques.

```yaml
# tests/soak/rth_reliability.yaml
base_scenario: battery_rth.yaml
runs: 100
vary:
  seed:            random
  wind.mean.speed: uniform(0, 10)
  start_position:  random_in_area
  world:           [urban_block, forest, empty_field]

assertions:
  - metric: rth_success_rate       , min: 0.99
  - metric: landing_distance_m_p95 , max: 8.0
  - metric: min_obstacle_distance_m, min: 1.0     # sur TOUTES les exécutions
  - metric: collisions             , max: 0       # tolérance zéro
```

C'est ici qu'on trouve les bugs rares : la condition de course qui se produit une fois sur
trente, la configuration de vent qui met le contrôleur en saturation, la géométrie qui
déclenche une dégénérescence du SLAM. Ces bugs sont invisibles en L3 et fatals en vol réel.

Avec 16 cœurs et 4 simulations parallèles, 100 exécutions prennent environ 4 heures — une
nuit de week-end. C'est praticable, et c'est ce qui justifie le choix d'un CPU à 16 cœurs
comme critère matériel dominant ([01 §1](01-choix-technologiques.md)).

## 8. Tests de conformité architecturale

Tests qui vérifient que l'architecture n'a pas été violée. Ils gardent les propriétés que
ce dossier décrit.

| Test | Vérifie |
|---|---|
| `test_no_control_plane_on_radio` | Le descripteur Protobuf ne contient aucun message de contrôle |
| `test_no_simulation_branches` | Aucun `if simulation` dans les paquets embarqués (analyse d'AST) |
| `test_no_ground_truth_subscription` | Aucun nœud embarqué ne souscrit à `sim/*` |
| `test_module_boundaries` | `import-linter` : aucun import inter-modules illégal côté serveur |
| `test_only_px4_interface_talks_to_px4` | Un seul nœud souscrit ou publie sur `/fmu/*` |
| `test_proto_generated_is_current` | Le code généré correspond aux `.proto` |
| `test_all_topics_documented` | Chaque topic publié figure dans [04](04-architecture-ros2.md) |
| `test_bringup_drone_has_no_sim_deps` | Le lancement embarqué ne référence ni Gazebo ni l'injecteur |

Ces tests sont peu coûteux et rendent l'architecture **exécutable** plutôt que déclarative.
Une architecture qui n'existe que dans un document dérive en quelques mois ; une
architecture testée tient.

## 9. Tests du serveur et du dashboard

| Niveau | Portée | Outils |
|---|---|---|
| Unitaire | Logique métier des modules | pytest |
| API | Chaque endpoint, codes de statut, validation | pytest + httpx |
| Base | Migrations, requêtes, hypertables | pytest + Postgres de test |
| WebSocket | Diffusion, limitation de débit, reconnexion | pytest-asyncio |
| Intégration | Serveur + passerelle + drone simulé | Compose profil `test` |
| Frontend unitaire | Composants, hooks | Vitest + Testing Library |
| Frontend E2E | Parcours opérateur complets | Playwright |

Parcours E2E prioritaires : créer et démarrer une mission ; suivre un vol en direct ; réagir
à une perte de liaison (l'interface affiche-t-elle correctement l'incertitude ?) ; envoyer
un waypoint et suivre son statut ; rejouer une mission terminée.

## 10. Critères de qualité par phase

Chaque phase de la [roadmap](17-roadmap.md) a des critères de sortie mesurables. Une phase
n'est pas terminée tant qu'ils ne sont pas atteints — pas « le code est écrit », mais « la
métrique est atteinte et le test passe ».

| Phase | Critère de sortie principal |
|---|---|
| P1 | `takeoff_land.yaml` et `waypoint_navigation.yaml` passent 10 fois de suite |
| P2 | ATE < 0,5 m sur 500 m avec GNSS ; carte visible et cohérente dans RViz |
| P3 | Zéro collision sur 20 exécutions de `obstacle_avoidance.yaml` |
| P4 | Couverture ≥ 90 % sur `urban_block` en moins de 15 min |
| P5 | Mission créée depuis le dashboard, exécutée, carte remontée |
| P6 | `full_resilience.yaml` passe ; L4 `rth_reliability` ≥ 99 % |
| P7 | mAP > 0,45 sur le jeu de validation ; `mission_without_ai.yaml` passe |
| P8 | 3 drones simultanés ; L4 hebdomadaire vert 4 semaines de suite |
