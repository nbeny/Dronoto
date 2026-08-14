# 03 — Simulation

## 1. Principe directeur

**Le code embarqué ne sait pas qu'il est en simulation.** La simulation se substitue à
exactement trois choses, et à rien d'autre :

1. les **pilotes de capteurs** (le LiDAR simulé publie sur le même topic, avec le même
   type, que le pilote Livox) ;
2. le **canal radio** (`VirtualRadioLink` au lieu de `SerialRadioLink`) ;
3. la **dynamique du véhicule** (Gazebo au lieu de la physique réelle).

L'autopilote n'est **pas** simulé : PX4 SITL exécute le vrai code source de PX4, avec le
vrai EKF2, les vrais contrôleurs, le vrai mixeur et les vrais failsafes. C'est ce qui rend
la validation transposable.

Aucun `if simulation:` n'est autorisé dans les paquets `perception`, `slam`, `navigation`,
`exploration`, `failsafe`. La sélection se fait par paramètre de lancement et par
injection de dépendance. Un test de conformité (`test_no_simulation_branches`) analyse
l'AST de ces paquets et échoue si un tel branchement apparaît.

## 2. Composition de la simulation

```
┌──────────────────────────────────────────────────────────────────────┐
│  gz sim  (Gazebo Sim Harmonic, serveur ; GUI optionnelle)             │
│                                                                      │
│   Monde SDF                                                          │
│    ├── modèle terrain / bâtiments / végétation                       │
│    ├── plugin de vent  (gz::sim::systems::WindEffects + rafales)      │
│    └── modèle x500_lidar_dronoto                                     │
│         ├── corps + 4 rotors (MulticopterMotorModel)                 │
│         ├── plugin PX4 (mavlink_interface / gz bridge)               │
│         ├── capteurs : imu, navsat, magnetometer, air_pressure,      │
│         │              gpu_lidar, camera, depth_camera               │
│         └── plugin batterie (LinearBatteryPlugin)                    │
└──────────────────────┬───────────────────────────────────────────────┘
                       │ gz-transport
        ┌──────────────▼──────────────┐
        │        ros_gz_bridge        │  gz msgs ↔ ROS 2 msgs
        └──────────────┬──────────────┘
                       │
        ┌──────────────▼──────────────┐        ┌────────────────────────┐
        │      sensor_faults          │◀───────│    fault_injector      │
        │  (nœud de dégradation)      │        │  scénario YAML, service│
        └──────────────┬──────────────┘        └────────────────────────┘
                       │  topics « propres » consommés par le drone
                       ▼
              graphe ROS 2 embarqué
```

Point d'architecture important : **la dégradation des capteurs n'est pas faite dans
Gazebo.** Gazebo produit des données nominales (avec son bruit gaussien de base) ; un nœud
`sensor_faults` intercalé applique les pannes, latences, dérives et coupures pilotées par
l'injecteur. Raison : les pannes deviennent contrôlables à l'exécution par un service ROS 2,
scriptables, et reproductibles — impossible si elles sont figées dans le SDF. C'est le
principe P4 appliqué.

## 3. Modèle du drone

Base : le modèle `x500` de PX4 (quadricoptère 500 mm, correspondant au kit Holybro X500 V2
retenu pour le matériel réel — voir [16](16-materiel.md)). On en dérive
`x500_lidar_dronoto` avec les capteurs du projet.

| Paramètre | Valeur | Origine |
|---|---|---|
| Empattement | 500 mm | Holybro X500 V2 |
| Masse à vide | 1,55 kg | Kit X500 V2 |
| Charge utile | 0,95 kg | Jetson Orin NX + Mid-360 + caméra + modem |
| Masse totale | **2,5 kg** | Cohérent avec le matériel cible |
| Rotors | 4, config X | — |
| Hélices | 10 × 4,5" | X500 V2 |
| Poussée max par rotor | ~14 N | ≈ 2,3:1 rapport poussée/poids à 2,5 kg |
| Inertie `Ixx, Iyy, Izz` | 0,029, 0,029, 0,055 kg·m² | Estimation modèle X500 |
| Coefficient de traînée | 0,10 | Réglé pour une vitesse terminale plausible |
| Vitesse horizontale max | 12 m/s | Limite de mission, pas limite physique |
| Vitesse verticale max | 3 m/s montée / 1,5 m/s descente | Sécurité |
| Accélération max | 5 m/s² | Limite de génération de trajectoire |

**Sur le réalisme physique** — la question posée dans le brief mérite une réponse franche.
Gazebo + DART simule correctement la dynamique de corps rigide, la poussée, les couples et
la traînée. Il ne simule **pas** l'effet de sol, l'interaction inter-rotors, la déformation
des hélices, le vortex ring state ni les turbulences fines. Faut-il s'en soucier ?

Non, pour ce projet, et voici pourquoi : ces phénomènes affectent la **boucle de contrôle
d'attitude**, qui est le domaine de PX4 — et PX4 est validé en vol réel par des milliers
d'utilisateurs. Ce projet valide les couches **au-dessus** : est-ce que le SLAM converge,
est-ce que l'exploration couvre la zone, est-ce que le failsafe se déclenche. Ces
propriétés dépendent de la géométrie, des capteurs, de la latence et de la logique — que
Gazebo reproduit fidèlement. Investir dans un modèle aérodynamique haute fidélité serait
optimiser la mauvaise variable.

Ce qui compte réellement pour la transposabilité, et qui est modélisé : la masse et
l'inertie réalistes, la poussée finie (donc la saturation), la consommation électrique
dépendante de la charge, le vent, les latences capteur et la géométrie exacte des capteurs
(positions, FOV, portée).

## 4. Capteurs simulés

Tous les capteurs sont déclarés dans le SDF et publiés à leur fréquence nominale. La
colonne « équivalent réel » garantit que les topics et les types ne changeront pas au
passage matériel.

| Capteur | Type Gazebo | Topic ROS 2 | Fréquence | Équivalent réel |
|---|---|---|---|---|
| IMU | `imu` | `/drone_1/sensors/imu` | 250 Hz | ICM-42688-P (dans Pixhawk 6X) |
| GNSS | `navsat` | `/drone_1/sensors/gnss` | 5 Hz | u-blox ZED-F9P (RTK) |
| Magnétomètre | `magnetometer` | `/drone_1/sensors/mag` | 50 Hz | BMM150 |
| Baromètre | `air_pressure` | `/drone_1/sensors/baro` | 50 Hz | ICP-20100 |
| LiDAR 3D | `gpu_lidar` | `/drone_1/sensors/lidar/points` | 10 Hz | Livox Mid-360 |
| Caméra RGB | `camera` | `/drone_1/sensors/cam_front/image_raw` | 15 Hz | OAK-D Pro W / RealSense D456 |
| Caméra depth | `depth_camera` | `/drone_1/sensors/cam_front/depth` | 15 Hz | idem, canal depth |
| Batterie | `LinearBatteryPlugin` | via PX4 `BatteryStatus` | 10 Hz | Module PM06 + estimation PX4 |
| Vérité terrain | `odometry_publisher` | `/drone_1/sim/ground_truth` | 100 Hz | **N'existe pas au réel** |

**La vérité terrain est un citoyen à part.** Elle est publiée sur un topic préfixé `sim/`
et **aucun paquet embarqué n'a le droit d'y souscrire**. Elle sert exclusivement aux tests
(erreur de trajectoire absolue du SLAM, complétude de la carte, distance minimale aux
obstacles). Un test de conformité vérifie qu'aucun nœud des paquets embarqués ne souscrit
à `sim/*`. Sans cette règle, on finit par tricher sans le vouloir.

### Configuration du LiDAR

Calquée sur le Livox Mid-360, le capteur retenu pour le matériel réel.

```
FOV horizontal    360°
FOV vertical      -7° à +52°   (asymétrique : voit vers le haut, peu vers le bas)
Portée            0,1 m – 40 m  (à 10 % de réflectivité) ; 70 m à 80 %
Points par seconde 200 000
Fréquence de scan  10 Hz
Bruit de portée    σ = 2 cm (gaussien) + composante proportionnelle à la distance
```

Le FOV vertical asymétrique n'est pas un détail : il signifie que **le drone voit mal
directement sous lui**. C'est une propriété réelle du Mid-360 qui a des conséquences
directes sur la détection de site d'atterrissage et sur la descente. La reproduire en
simulation évite une mauvaise surprise au passage matériel — c'est exactement le genre de
chose qui justifie de calquer la simulation sur le matériel cible dès le début.

Gazebo ne reproduit pas le motif de scan non répétitif du Mid-360 (il échantillonne selon
une grille régulière). C'est une divergence connue, acceptée, documentée : elle rend la
simulation légèrement **optimiste** sur la densité de couverture instantanée. Compensation :
on réduit le nombre de rayons simulés par rapport aux 200 000 pts/s nominaux pour ne pas
sur-estimer l'information disponible.

## 5. Modélisation des perturbations

### 5.1 Vent

Trois composantes, additives, toutes paramétrables :

```yaml
wind:
  mean:      { speed: 4.0, direction_deg: 270, vertical: 0.0 }   # vent moyen
  gusts:                                                          # rafales
    enabled: true
    mean_interval_s: 25.0        # loi de Poisson
    duration_s:      { min: 1.5, max: 5.0 }
    magnitude_ms:    { min: 2.0, max: 8.0 }
  turbulence:                                                     # bruit continu
    model: dryden                # Dryden, standard aéronautique
    intensity: moderate
  altitude_gradient:             # profil logarithmique de couche limite
    enabled: true
    roughness_length_m: 0.1
```

Le modèle de Dryden est retenu pour la turbulence parce qu'il est le standard aéronautique
(filtres de bruit blanc de densité spectrale spécifiée) plutôt qu'un bruit gaussien naïf.
Le gradient d'altitude compte pour un drone qui monte de 5 à 80 m : le vent y est
sensiblement plus fort en haut.

### 5.2 Bruit, biais et latence capteurs

Appliqués par le nœud `sensor_faults`, pas par Gazebo.

| Capteur | Bruit | Biais | Latence | Notes |
|---|---|---|---|---|
| IMU accéléromètre | σ = 0,02 m/s²/√Hz | marche aléatoire, 0,001 m/s³/√Hz | 2 ms | Réglé sur ICM-42688-P |
| IMU gyromètre | σ = 0,003 rad/s/√Hz | marche aléatoire, 1e-5 rad/s²/√Hz | 2 ms | idem |
| GNSS position | σ = 0,5 m H / 1,0 m V (RTK fix : 0,02 m) | dérive lente corrélée, τ = 300 s | 150 ms | La latence GNSS est réelle et compte |
| GNSS vitesse | σ = 0,05 m/s | — | 150 ms | — |
| Magnétomètre | σ = 0,5 µT | biais dur + biais mou (paramétrable) | 5 ms | Le biais dur simule les perturbations de la plateforme |
| Baromètre | σ = 0,1 m | dérive de pression, τ = 600 s | 10 ms | La dérive barométrique force la fusion |
| LiDAR | σ = 2 cm + 0,1 % de la distance | — | 50 ms (scan + transfert) | Points manquants sur surfaces réfléchissantes |
| Caméra | bruit de photons + flou de mouvement | — | 60 ms | — |

Chaque source de bruit tire d'un `numpy.random.Generator` propre, initialisé à partir d'une
**graine de scénario** et de l'identifiant du capteur. Conséquence : deux exécutions avec la
même graine sont identiques, et changer un capteur ne décale pas les tirages des autres.
C'est indispensable pour qu'un échec de test soit reproductible.

### 5.3 Modèle de batterie

La batterie est modélisée par un état de charge alimenté par un modèle de puissance, pas
par un simple décompte linéaire — sinon les tests de retour à la base sont sans valeur.

```
P_total = P_hover(masse, densité_air)
        + P_manoeuvre(‖a‖, ‖v‖)
        + P_avionique  (≈ 8 W : Pixhawk, capteurs, modem)
        + P_calcul     (≈ 15 W au repos, 25 W avec inférence IA active)

SoC(t+dt) = SoC(t) − P_total · dt / E_utile
tension = f(SoC)  courbe de décharge Li-ion, avec chute sous charge
```

Deux conséquences volontaires et importantes :

- **Voler vite coûte plus cher.** Le planificateur d'exploration doit donc arbitrer
  vitesse contre autonomie, ce qui rend le critère « batterie » du scoring réel plutôt que
  décoratif.
- **Faire tourner l'IA coûte de l'énergie.** C'est un vrai arbitrage embarqué, souvent
  ignoré. Il devient visible et testable.

La réserve de retour à la base est calculée dynamiquement : `énergie_retour = distance_base
/ vitesse_croisière × P_estimée × facteur_sécurité(1,4)` — pas un seuil fixe en pourcentage.
Un seuil fixe est faux : 25 % de batterie suffit à 200 m de la base et est insuffisant à
3 km.

## 6. Mondes de simulation

Six mondes, chacun conçu pour éprouver une propriété précise. Ils sont versionnés dans
`simulation/worlds/`.

| Monde | Dimensions | Objectif | Phase |
|---|---|---|---|
| `empty_field` | 200 × 200 m, plat | Vol de base, décollage, atterrissage, waypoints | P1 |
| `pillars` | 100 × 100 m, 40 piliers verticaux | Évitement, planification, densité de carte contrôlée | P3 |
| `urban_block` | 300 × 300 m, bâtiments 5-25 m, canyons | Exploration, occlusion GNSS réaliste, multi-trajets | P4 |
| `forest` | 200 × 200 m, végétation dense | Robustesse du SLAM en milieu non structuré, LiDAR bruité | P4 |
| `warehouse_interior` | 60 × 40 × 12 m, intérieur | Perte totale de GNSS, SLAM seul, exploration confinée | P6 |
| `mixed_mission` | 500 × 500 m, extérieur + intérieur | Scénario complet 13 étapes, transitions GNSS | P6 |

Le monde `urban_block` porte une propriété subtile et précieuse : **l'occlusion GNSS est
géométrique**, pas scriptée. Le nœud `sensor_faults` calcule la visibilité satellite à
partir de la géométrie des bâtiments (ligne de vue vers des satellites simulés) et dégrade
le GNSS en conséquence — perte de fix en canyon, retour en zone dégagée. Une perte de GPS
scriptée par minuterie ne teste que la bascule ; une perte géométrique teste aussi le
**mauvais cas** : le GPS qui revient avec une position fausse à cause des multi-trajets.
C'est le cas dangereux, et c'est celui qu'on veut voir.

## 7. Injection de pannes

Le nœud `fault_injector` est le pivot de toute la stratégie de résilience. Il expose :

- un **service ROS 2** `dronoto_msgs/srv/InjectFault` pour déclencher une panne à la volée ;
- un **scénario YAML** exécuté sur une chronologie (voir [14 — Tests](14-tests.md)) ;
- un topic d'état `/sim/active_faults` pour l'observabilité et l'affichage RViz.

Catalogue des pannes injectables :

| Catégorie | Panne | Paramètres |
|---|---|---|
| GNSS | Perte de fix | durée, progressive ou brutale |
| GNSS | Dégradation | HDOP, nombre de satellites, biais de position |
| GNSS | Multi-trajets | amplitude et durée du biais |
| GNSS | Usurpation (spoofing) | dérive de position injectée |
| LiDAR | Panne totale | durée |
| LiDAR | Dégradation | fraction de points perdus, portée réduite |
| LiDAR | Occlusion | secteur angulaire masqué (simule salissure/givre) |
| Caméra | Panne, surexposition, flou | durée, intensité |
| IMU | Saut de biais | amplitude, axe |
| Radio | Coupure totale | durée |
| Radio | Dégradation | perte de paquets, latence, débit |
| Batterie | Chute rapide | nouveau taux de décharge |
| Batterie | Cellule défaillante | chute de tension |
| Calcul | Surcharge CPU | charge injectée → observe la dégradation des latences |
| Moteur | Perte partielle de poussée | rotor, fraction |
| Serveur | Indisponibilité | durée |

Le point non négociable : **les pannes sont pilotées par le temps de simulation**, pas par
l'horloge murale. Elles se produisent donc exactement au même instant à chaque exécution,
même si la machine est chargée. Sans cela, les tests de résilience sont intermittents.

## 8. Performance et exécution

- **Mode headless** par défaut pour les tests (`gz sim -s -r --headless-rendering`). La GUI
  Gazebo et RViz sont pour le développement, pas pour la CI.
- **Lockstep** entre PX4 SITL et Gazebo activé : le pas de simulation est synchronisé avec
  l'autopilote. Ça garantit le déterminisme et permet d'accélérer le temps sans casser le
  contrôle.
- **Facteur d'accélération** : viser 2-4× le temps réel sur le 5950X pour les mondes
  simples, ~1× pour `urban_block` avec LiDAR + caméras. Mesuré et suivi en CI ; une chute
  du facteur est un signal de régression de performance.
- **Parallélisation** : les tests L3 lancent plusieurs simulations simultanées, isolées par
  `ROS_DOMAIN_ID` et par des ports distincts. Avec 16 cœurs, 4 simulations parallèles sont
  réalistes ; c'est ce qui rend les campagnes Monte-Carlo (L4) praticables.
- **Rendu** : sous Linux natif, Gazebo utilise le pilote Mesa RADV sur la RX 6900 XT sans
  configuration particulière. Sous WSL2, forcer `LIBGL_ALWAYS_SOFTWARE=0` et accepter la
  perte de performance du chemin D3D12.
