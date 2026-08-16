# 05 — Intégration PX4 / MAVLink

## 1. Frontière de responsabilité

La question qui décide de la robustesse du système : **qu'est-ce que PX4 fait, et
qu'est-ce que nous faisons ?** La réponse doit être nette, parce qu'un chevauchement
produit des conflits d'autorité impossibles à déboguer en vol.

| PX4 (contrôleur de vol) | Calculateur embarqué (nous) |
|---|---|
| Contrôle des taux angulaires (~1 kHz) | Rien |
| Contrôle d'attitude (~250 Hz) | Rien |
| Contrôle de vitesse et de position (~50 Hz) | Rien |
| Mixage et sorties moteur | Rien |
| EKF2 : estimation d'état fusionnée | Fournir l'odométrie externe |
| Détection et gestion des failsafes matériels | Superviseur de sécurité applicatif |
| Armement, modes de vol, géorepérage matériel | Demander l'armement et les changements de mode |
| RTL et atterrissage de secours autonomes | Décider quand les demander |
| — | Perception, SLAM, cartographie |
| — | Planification et génération de trajectoire |
| — | Décision d'exploration |
| — | Communication avec le sol |
| — | Consignes de trajectoire (position/vitesse/accélération) |

**Nous ne descendons jamais sous le niveau « consigne de trajectoire ».** Aucune consigne
d'attitude, de taux ou de poussée n'est publiée. Conséquence directe : si notre code
s'arrête, PX4 conserve un aéronef stable et déclenche ses propres failsafes. C'est la
propriété P2 rendue concrète.

## 2. Transport : uXRCE-DDS

```
┌─────────────────────────────┐          ┌──────────────────────────────────┐
│           PX4               │          │      Calculateur embarqué        │
│                             │          │                                  │
│  uORB  ──▶ uxrce_dds_client │◀════════▶│ MicroXRCEAgent ──▶ topics DDS   │
│                             │  UDP     │                     │            │
│                             │  (SITL)  │                     ▼            │
│                             │  série   │              px4_interface       │
│                             │  (réel)  │                     │            │
└─────────────────────────────┘          │                     ▼            │
                                         │            reste du graphe ROS 2 │
                                         └──────────────────────────────────┘
```

**Configuration.** En SITL, le client PX4 se connecte à l'agent en UDP sur le port 8888 —
lancement automatique par le fichier de lancement. Sur matériel réel : UART à 921 600 bauds
(TELEM2) ou, mieux, Ethernet si le calculateur et le Pixhawk le supportent — le Pixhawk 6X
a un port Ethernet, ce qui supprime le goulot série et sa fragilité.

**Versionnement des messages.** PX4 ≥ 1.16 introduit le versionnement des messages.

> **Constaté à l'implémentation, et coûteux.** PX4 v1.16 publie `VehicleStatus` sur le
> topic **versionné** `/fmu/out/vehicle_status_v1`. Le topic `/fmu/out/vehicle_status`
> apparaît toujours dans `ros2 topic list`, avec le bon type — mais **ne porte aucune
> donnée**. S'y abonner ne produit ni erreur, ni avertissement, ni message : la structure
> reste simplement à sa valeur par défaut.
>
> Conséquence observée : `armed` et `preflight_ok` restaient faux en permanence, l'exécutif
> de mission attendait indéfiniment une condition qui ne pouvait pas arriver, et **rien
> dans les journaux ne l'indiquait**. PX4 était prêt à armer depuis le début.
>
> Deux protections en découlent, toutes deux implémentées :
> 1. `px4_interface` s'abonne au topic versionné **et** au topic historique, pour tolérer
>    les deux conventions.
> 2. Un **chien de garde** émet un événement `CRITICAL` si l'odométrie arrive mais pas le
>    statut au bout de 10 s. Une panne silencieuse devient bruyante — c'est la vraie leçon,
>    plus que le correctif lui-même.
>
> Vérification systématique avant d'écrire un abonnement PX4 :
> `ros2 topic list | grep '_v[0-9]*$'` puis `ros2 topic hz <topic> --qos-reliability
> best_effort` pour confirmer qu'il publie réellement.

Deux conséquences pratiques à respecter dès le début :

1. Le paquet `px4_msgs` est épinglé sur la branche correspondant à la version de PX4
   utilisée (`release/1.16`), pas sur `main`. Un `px4_msgs` désaligné produit des messages
   silencieusement corrompus — le pire mode de défaillance possible.
2. Si le besoin de découpler les versions apparaît (par exemple, mettre à jour PX4 sans
   recompiler tout le graphe), PX4 fournit un **nœud de traduction de messages**. On ne
   l'introduit qu'au moment où c'est nécessaire, pas par anticipation.

La version de PX4 est **verrouillée dans le dépôt** (sous-module git avec un commit fixe) et
sa mise à jour est un acte délibéré, testé, avec sa propre entrée de journal de
modifications. La dérive de version d'autopilote est une cause classique de régressions
inexplicables.

## 3. Topics PX4 consommés et publiés

### Entrants (PX4 → nous)

| Topic uORB | Contenu | Usage |
|---|---|---|
| `/fmu/out/vehicle_odometry` | Pose et vitesse estimées par EKF2 | Source de `state/odometry` |
| `/fmu/out/vehicle_status` | Mode de vol, armement, type de navigation | Superviseur de sécurité |
| `/fmu/out/failsafe_flags` | Drapeaux de failsafe PX4 | Superviseur — observe ce que PX4 a détecté |
| `/fmu/out/battery_status` | Tension, courant, SoC, temps restant | Superviseur, exploration, télémétrie |
| `/fmu/out/vehicle_gps_position` | Fix GNSS brut, nombre de satellites, DOP | Détection de dégradation GNSS |
| `/fmu/out/sensor_combined` | IMU synchronisée | FAST-LIO2 (en préférence à l'IMU Gazebo : c'est l'IMU du vrai chemin) |
| `/fmu/out/vehicle_local_position` | Position locale NED | Recoupement |
| `/fmu/out/vehicle_command_ack` | Acquittements de commandes | Confirmation d'armement et de changement de mode |
| `/fmu/out/estimator_status_flags` | Sources de fusion actives d'EKF2 | **Observabilité critique** : quelle source EKF2 utilise réellement |

`estimator_status_flags` mérite d'être souligné. Il indique si EKF2 fusionne le GNSS,
l'odométrie externe, le flux optique, etc. Sans ce topic, on croit savoir sur quoi
l'estimation repose ; avec lui, on le sait. Il alimente directement l'affichage du
dashboard et les assertions des tests de perte GPS.

### Sortants (nous → PX4)

| Topic uORB | Fréquence | Contenu |
|---|---|---|
| `/fmu/in/offboard_control_mode` | 20 Hz | Déclare quels champs de consigne sont valides |
| `/fmu/in/trajectory_setpoint` | 20 Hz | Position, vitesse, accélération, cap |
| `/fmu/in/vehicle_visual_odometry` | 30-50 Hz | Odométrie externe issue de FAST-LIO2 |
| `/fmu/in/vehicle_command` | événementiel | Armement, mode, RTL, atterrissage |

## 4. Mode offboard

### Contraintes imposées par PX4

1. Un flux `OffboardControlMode` doit précéder l'entrée en mode offboard et être maintenu
   à **plus de 2 Hz**. En dessous, PX4 quitte offboard et bascule en failsafe.
2. Une consigne cohérente doit accompagner chaque message de mode.
3. Le drone doit être armé et les vérifications préalables passées.

### Ce que fait `px4_interface`

Le nœud publie **inconditionnellement à 20 Hz**, indépendamment de l'activité de l'amont.
S'il ne reçoit pas de nouvelle consigne, il applique cette politique :

```
âge de la dernière consigne < 100 ms  ──▶  publier telle quelle
100 ms ≤ âge < 500 ms                 ──▶  republier la dernière (extrapolation figée)
âge ≥ 500 ms                          ──▶  publier une consigne de maintien de position
                                           + lever SafetyEvent(STALE_SETPOINT)
                                           + le superviseur passe en HOLD
```

Ce comportement est ce qui rend un plantage du planificateur non fatal : le drone se met en
vol stationnaire au lieu de sortir du mode offboard de façon incontrôlée.

### Utilisation de `px4_ros2_cpp`

Plutôt que d'implémenter à la main la logique de mode, on utilise la bibliothèque
d'interface `px4_ros2_cpp` :

- **modes personnalisés enregistrés** auprès de PX4, avec leurs conditions de santé — PX4
  connaît alors l'existence de nos modes et peut les refuser proprement ;
- **arbitrage propre** entre modes personnalisés et modes PX4 ;
- **rapport de santé de mode** : si notre mode se déclare non sain, PX4 replie
  automatiquement sur un mode sûr. C'est une couche de sécurité offerte, qu'il serait
  absurde de ne pas prendre.

Modes personnalisés définis :
`DronotoExplore`, `DronotoGoTo`, `DronotoReturnHome`, `DronotoSafeLand`.

## 5. Fusion EKF2 et odométrie externe

C'est le cœur technique de l'intégration, et c'est là que se joue la survie à la perte GPS.

### Principe

EKF2 est un filtre de Kalman étendu qui fusionne les sources disponibles. Il n'a **pas
besoin** qu'on lui dise laquelle utiliser : il possède déjà une logique de sélection et de
repli. Notre travail n'est pas de le remplacer mais de **lui fournir une source
supplémentaire de qualité, avec une covariance honnête.**

```
        IMU  ──────────────────────┐
        baromètre ────────────────┐│
        magnétomètre ────────────┐││
        GNSS  ───────────────────┼┼┼──▶  EKF2  ──▶  état utilisé par le contrôleur
        odométrie externe ───────┘││
        (FAST-LIO2, 30-50 Hz)     ││
                                  ││
        └── notre contribution ────┘│
                                    └── PX4 gère la pondération et le repli
```

### Paramètres EKF2

| Paramètre | Valeur | Raison |
|---|---|---|
| `EKF2_EV_CTRL` | position + vitesse + cap activés | Fusionner tout ce que FAST-LIO2 fournit de fiable |
| `EKF2_GPS_CTRL` | position + vitesse activées | GNSS comme source complémentaire, pas exclusive |
| `EKF2_HGT_REF` | GPS si disponible, sinon baromètre, EV en intérieur | Référence d'altitude selon le contexte |
| `EKF2_EV_DELAY` | mesuré en P2 (~80-120 ms attendu) | **Le paramètre le plus important.** Une erreur ici dégrade la fusion silencieusement. |
| `EKF2_EV_POS_X/Y/Z` | position du LiDAR par rapport à l'IMU | Bras de levier ; erreur = biais systématique |
| `EKF2_EV_NOISE_MD` | utiliser la covariance du message | On envoie une covariance calculée, pas fixe — voir ci-dessous |

**Mesure de `EKF2_EV_DELAY`.** Ce n'est pas une valeur à deviner. La procédure, exécutée en
P2 et rejouée à chaque changement de configuration : on effectue un vol d'excitation
(oscillations verticales et horizontales), on enregistre `sim/ground_truth` et
`state/lidar_odometry`, on calcule la corrélation croisée entre les deux signaux de vitesse,
et le décalage qui la maximise est le retard. Automatisé dans `tools/calibrate_ev_delay.py`.

### Covariance honnête

Le point qui distingue une intégration correcte d'une intégration naïve : **la covariance
publiée avec l'odométrie externe doit refléter la confiance réelle de l'instant**, pas une
constante.

`lidar_odometry` calcule sa covariance à partir de la qualité de l'appariement (nombre de
correspondances, résidu, conditionnement de la matrice d'information). Quand le SLAM entre
dans un couloir sans structure — un environnement dégénéré où la contrainte le long de
l'axe est faible —, la covariance dans cette direction **augmente**, et EKF2 pondère
automatiquement moins cette source.

Publier une covariance optimiste constante est la manière la plus efficace de faire tomber
un drone en environnement dégénéré : le filtre fait confiance à une mesure fausse. On
préfère une source qui admet son incertitude.

## 6. Gestion de la perte de GNSS

Séquence complète, avec les responsabilités réparties :

```
t=0    GNSS nominal
       EKF2 fusionne GNSS + IMU + baro + mag + EV
       Notre superviseur : NAV_QUALITY = GOOD

t=1    Dégradation (satellites ↓, DOP ↑)
       Détecté par vehicle_gps_position + notre gnss_status
       Superviseur : NAV_QUALITY = DEGRADED
       Exploration : le poids « qualité de localisation » monte dans le scoring
                     → préférence pour des zones favorables au SLAM
       Télémétrie : événement émis vers le sol

t=2    Perte de fix
       EKF2 arrête de fusionner le GNSS (mécanisme interne, aucune action de notre part)
       L'estimation repose sur EV + IMU + baro
       estimator_status_flags le confirme → on VÉRIFIE que EV est bien fusionnée
       Superviseur : NAV_QUALITY = SLAM_ONLY
       Conséquences appliquées :
         · vitesse max réduite (12 → 6 m/s)
         · rayon d'exploration borné autour de la dernière position sûre
         · le retour à la base bascule sur le suivi de trajectoire enregistrée
           (breadcrumb) plutôt que sur une ligne droite GNSS
       Continuation de la mission : OUI

t=3    Le SLAM se dégrade à son tour (environnement dégénéré)
       odometry_health signale la dégénérescence
       Superviseur : NAV_QUALITY = POOR
         · vol stationnaire ou remontée pour retrouver de la structure
         · si la dégradation persiste > 10 s → atterrissage d'urgence contrôlé

t=4    Retour du GNSS
       ⚠ Ne PAS faire confiance immédiatement.
       Le multi-trajets en sortie de canyon produit des positions fausses et confiantes.
       Politique : test de cohérence sur 5 s (comparaison de l'innovation GNSS
       avec la solution EV) avant de réautoriser la fusion pleine.
       EKF2 fait déjà un test d'innovation ; on ajoute un verrou applicatif car
       le cas « GPS confiant et faux » est le plus dangereux des deux.
```

Le point le plus important de cette séquence est **t=4**. La perte de GPS est facile : on
la détecte et on bascule. Le retour de GPS est difficile, parce qu'un GPS qui ment est
indistinguable d'un GPS qui dit vrai, sauf par recoupement. C'est le scénario que le monde
`urban_block` est conçu pour produire ([03 §6](03-simulation.md)).

## 7. MAVLink : quel rôle exactement

MAVLink n'est **pas** le protocole de la liaison de mission. Il est conservé pour deux
usages précis :

1. **QGroundControl.** Sur le drone réel, un opérateur doit pouvoir se connecter avec un
   outil standard pour la configuration, la calibration, l'analyse de logs et surtout la
   reprise en main manuelle. C'est une exigence de sécurité opérationnelle, pas un confort.
   Le flux MAVLink est tunnelé dans un canal DLP de basse priorité (`BULK`) et ne peut donc
   jamais affamer la télémétrie de sécurité.

2. **Radiocommande de secours.** Sur le drone réel, une liaison RC directe indépendante
   (ELRS ou équivalent) donne au pilote l'autorité ultime, sans passer par le calculateur
   embarqué ni par le modem de mission. Cette voie est **physiquement séparée** et n'existe
   pas en simulation, mais l'architecture ne doit rien faire qui l'empêche — notamment, ne
   jamais désactiver l'entrée RC ni le failsafe de perte RC de PX4.

## 8. Séquences d'armement et de décollage

Vérifications préalables appliquées par `mission_executive` **avant** de demander
l'armement. Chacune correspond à un mode de défaillance observé sur des projets réels :

```
1.  PX4 accessible et à jour de version              → sinon ABORT
2.  EKF2 convergé (variances sous seuil)             → sinon ATTENDRE (max 60 s)
3.  Référence de position locale valide              → sinon ABORT
4.  Batterie > seuil de départ (75 %) et cohérente   → sinon ABORT
5.  Tous les topics capteurs frais (< 1 s)           → sinon ABORT
6.  lidar_odometry initialisée et saine              → sinon ATTENDRE
7.  Carte réinitialisée, position de base enregistrée → toujours
8.  Superviseur en état READY                        → sinon ABORT
9.  Aucun failsafe PX4 actif                         → sinon ABORT
10. Zone de mission valide et cohérente avec la base → sinon ABORT
```

Ces vérifications sont exécutées **et journalisées** : chaque décollage produit une trace
horodatée du résultat de chaque vérification. Quand une mission tourne mal, cette trace est
la première chose qu'on regarde.

Séquence de décollage : armement → montée verticale jusqu'à `takeoff_altitude` (5 m par
défaut) en mode PX4 natif `TAKEOFF` → transition vers offboard une fois stabilisé. On
n'utilise **pas** l'offboard pour décoller : le mode natif de PX4 est mieux testé et gère
correctement l'effet de sol et la détection de sol.
