# 01 — Choix technologiques

Chaque section suit la même structure : **décision**, **pourquoi**, **rejeté et
pourquoi**, **condition de révision**. La dernière ligne compte autant que les autres :
un choix sans condition de révision est un dogme.

---

## 1. Plateforme de développement

**Décision : Ubuntu 24.04 LTS en natif (dual-boot). WSL2 supporté en mode dégradé.**

Le matériel disponible (Ryzen 9 5950X 16C/32T, 32 Go, RX 6900 XT 16 Go, 7 To libres)
est excellent pour ce projet — mais pour une raison contre-intuitive : **cette charge de
travail est limitée par le CPU, pas par le GPU**. Gazebo Sim exécute la physique, les
capteurs et le pas de simulation sur CPU ; PX4 SITL est mono-thread intensif ; FAST-LIO2
est du calcul CPU dense ; OctoMap est du CPU et de la mémoire. Le GPU ne sert qu'au rendu
des caméras et du LiDAR, ce que la RX 6900 XT fait sans effort.

Pourquoi le natif plutôt que WSL2 :

| Aspect | Linux natif | WSL2 |
|---|---|---|
| Rendu Gazebo (OpenGL/ogre2) | Pilote Mesa direct | Traduction D3D12 via WSLg — 30-50 % de perte, artefacts connus sur les capteurs GPU |
| Découverte DDS multicast | Native | Réseau NAT ; nécessite un fichier de configuration DDS spécifique et casse le multi-machine |
| ROCm / PyTorch GPU | Fonctionne sur gfx1030 avec `HSA_OVERRIDE_GFX_VERSION=10.3.0` | Non supporté sur RDNA2 |
| Latence et jitter d'ordonnancement | Maîtrisés | Couche VM supplémentaire, horloge dérivante |
| Docker | Natif | Via Docker Desktop, surcouche |
| Coût | 0 € | 0 € |

Le mode WSL2 reste documenté et testé en CI parce qu'il abaisse la barrière d'entrée pour
un contributeur sous Windows, mais les mesures de performance de référence sont prises
sous Linux natif.

**Sur l'achat d'un GPU NVIDIA : ne pas acheter maintenant.** L'analyse détaillée est dans
[ADR-0002](../adr/0002-plateforme-de-developpement.md). En résumé : rien dans les phases
P0-P6 n'en profiterait, la seule chose qu'un GPU NVIDIA débloquerait (Isaac Sim, RL massif)
relève des phases P7-P8, et si le besoin se confirme alors, une RTX 5060 Ti 16 Go
(~450 €) sera le bon point d'entrée — la VRAM prime sur la puissance brute. **Le seul
achat matériel qui améliorerait le projet dès aujourd'hui, c'est 32 Go de RAM
supplémentaires (~110 €)** : la stack complète (Gazebo + PX4 + ROS 2 + SLAM + Postgres +
dashboard + IDE) frôle les 32 Go.

*Révision si* : passage au reinforcement learning (P8) ou besoin de rendu photoréaliste
pour le sim-to-real de la perception visuelle.

---

## 2. Simulateur

**Décision : Gazebo Sim Harmonic (gz-sim 8) + PX4 SITL.**

Pourquoi :

- **C'est le simulateur de référence de PX4.** Les modèles de drone, les plugins capteurs
  et les scripts de lancement sont maintenus dans `PX4-Autopilot` lui-même. Cela supprime
  une couche d'intégration entière, qui est exactement là où les projets de ce type
  s'enlisent.
- **Le pont `ros_gz_bridge` est officiel et maintenu**, versionné avec ROS 2 Jazzy.
- **Le moteur de capteurs est adéquat** : `gpu_lidar` produit des nuages de points 3D
  paramétrables (canaux, portée, FOV, bruit), les caméras RGB et depth existent, le bruit
  gaussien et les biais sont configurables par SDF.
- **Le modèle physique est suffisant** pour la question posée. Gazebo avec DART simule
  correctement la dynamique du corps rigide, et PX4 SITL exécute **le vrai code de
  l'autopilote** — c'est ça qui compte pour la transposabilité, bien plus que la finesse
  aérodynamique. Le mixeur, l'EKF2, les modes de vol, les failsafes testés en simulation
  sont ceux qui voleront.

**Rejeté :**

| Alternative | Pourquoi non |
|---|---|
| **NVIDIA Isaac Sim** | Exige CUDA et des RT cores : **impossible sur RX 6900 XT**. Même avec un GPU NVIDIA, l'intégration PX4 est communautaire et fragile là où celle de Gazebo est officielle. Sa valeur réelle est le rendu photoréaliste (sim-to-real de la vision) et le RL massivement parallèle — deux besoins de phase tardive. Adopter Isaac Sim en P1 coûterait des semaines d'intégration pour un bénéfice nul avant P7. |
| **Gazebo Classic (gazebo11)** | Fin de vie en janvier 2025. Construire dessus en 2026 est une dette immédiate. |
| **AirSim** | Archivé par Microsoft en 2022 ; le successeur (Project AirSim) est propriétaire et payant. Élimine la contrainte « open source ». |
| **Webots** | Bon simulateur, mais l'intégration PX4 est nettement moins mature et le modèle de capteur LiDAR moins riche. Aucun avantage compensatoire. |
| **jMAVSim** | Pas de capteurs de perception (ni LiDAR ni caméra). Utile pour tester le contrôle de vol seul, inutile ici. |
| **Flightmare / Flightgoggles** | Orientés recherche RL, communautés petites, non maintenus au rythme de PX4. |

*Révision si* : besoin de rendu photoréaliste pour entraîner un modèle de perception
destiné au réel (P7+), ou passage au RL nécessitant des milliers d'environnements
parallèles (P8+). Dans ce cas, Isaac Sim viendrait **en complément** pour l'entraînement
hors ligne, pas en remplacement pour la validation système.

Détail : [ADR-0001](../adr/0001-simulateur.md), [03 — Simulation](03-simulation.md).

---

## 3. Middleware robotique

**Décision : ROS 2 Jazzy Jalisco sur Ubuntu 24.04, RMW par défaut Fast DDS.**

Pourquoi ROS 2 tout court : c'est le seul écosystème qui apporte simultanément le modèle
de publication/souscription typé, TF2 (l'arbre de transformations, indispensable dès qu'on
mélange LiDAR, IMU et GNSS), RViz, `rosbag2` (rejeu déterministe, socle de la stratégie de
tests), et les portages des algorithmes de SLAM et de perception qu'on va consommer.
Écrire ce middleware soi-même est un projet à part entière.

Pourquoi **Jazzy** et pas **Lyrical Luth** (sorti le 22 mai 2026, LTS jusqu'en 2031) :

- Jazzy est LTS jusqu'en **mai 2029** — trois ans, plus que la durée prévue du projet
  jusqu'au matériel.
- L'écosystème dont dépend ce projet (portages ROS 2 de FAST-LIO2, `octomap_server2`,
  `ros_gz`, la documentation PX4) est **validé et documenté contre Jazzy**. Lyrical a trois
  mois : les paquets tiers ne sont pas encore portés, la documentation PX4 ne le référence
  pas, et les problèmes rencontrés n'auront pas de réponses.
- Choisir la distribution la plus récente sur un projet qui dépend de dix paquets
  communautaires, c'est s'infliger le travail de portage d'autrui.

Le chemin de migration vers Lyrical est prévu en P8, quand l'écosystème aura rattrapé et
que le code sera stabilisé. Le coût est contenu si on respecte les conventions ROS 2
standard, ce que le dossier impose.

Pourquoi **Fast DDS** en défaut : c'est le RMW par défaut de Jazzy, celui contre lequel
tout est testé en amont, et celui qu'utilise l'agent Micro XRCE-DDS de PX4. Un seul
fournisseur DDS dans le graphe réduit la surface de problèmes de découverte.
**Cyclone DDS** (`rmw_cyclonedds_cpp`) reste une bascule d'une variable d'environnement,
documentée, à essayer si des problèmes de découverte ou de débit apparaissent — c'est un
mode de secours connu de la communauté PX4.

*Révision si* : problèmes de découverte DDS persistants (→ Cyclone), ou stabilisation de
l'écosystème sur Lyrical (→ migration P8).

Détail : [ADR-0003](../adr/0003-distribution-ros2.md), [04 — Architecture ROS 2](04-architecture-ros2.md).

---

## 4. Interface avec l'autopilote

**Décision : uXRCE-DDS pour le chemin embarqué (calculateur ↔ PX4). MAVLink réservé à la
station sol et à QGroundControl.**

C'est une distinction qui structure tout le système et que beaucoup de projets ratent.

```
Calculateur embarqué  ──uXRCE-DDS (DDS natif, typé, ~100 Hz)──▶  PX4     [temps réel, local]
Calculateur embarqué  ──DLP sur CommunicationLink (Protobuf)──▶  Sol     [haut niveau, lent]
PX4                   ──MAVLink tunnelé (optionnel)───────────▶  QGC     [supervision humaine]
```

Pourquoi uXRCE-DDS côté embarqué :

- Les messages uORB de PX4 apparaissent **directement comme des topics ROS 2 typés**
  (`px4_msgs`), sans traduction ni perte sémantique. On accède à `VehicleOdometry`,
  `SensorCombined`, `VehicleStatus`, `FailsafeFlags` avec leur structure d'origine.
- C'est le mécanisme officiel depuis PX4 v1.14 ; `micrortps_agent` est mort.
- Il supporte le débit et la latence requis pour le mode offboard (setpoints à ≥ 2 Hz
  obligatoires, 20-50 Hz en pratique) et pour l'injection d'odométrie externe dans EKF2
  (30-50 Hz requis).
- La bibliothèque **`px4_ros2_cpp`** fournit une couche d'abstraction au-dessus (gestion
  des modes, arbitrage offboard, vérification de santé) qui évite de réimplémenter la
  machinerie de mode. On l'utilise plutôt que de publier des `OffboardControlMode` à la main.

Pourquoi **pas MAVLink** comme protocole applicatif de la liaison radio : MAVLink est
excellent pour ce pour quoi il est conçu — piloter et superviser un véhicule depuis une
station sol standard. Il est mauvais pour transporter les objets métier de ce projet :
une mission « cartographier ce polygone avec ces contraintes », un delta de carte 3D, une
carte de risque sémantique. Les forcer dans des `MISSION_ITEM` ou des messages `TUNNEL`
produit un protocole illisible et fragile. On définit donc un protocole propre (DLP,
voir [09](09-communication.md)) et on garde MAVLink comme canal parallèle optionnel pour
QGroundControl — parce que sur le vrai drone, un pilote humain doit pouvoir reprendre la
main avec un outil standard.

Détail : [05 — Intégration PX4](05-integration-px4.md).

---

## 5. Localisation et SLAM

**Décision : architecture d'estimation à deux étages.**

```
Étage 1  — EKF2 dans PX4  (250-1000 Hz, contrôleur de vol, sécuritaire)
           fusionne : IMU + GNSS + baromètre + magnétomètre + odométrie externe
           produit  : l'état utilisé par le contrôleur de vol

Étage 2  — FAST-LIO2 + graphe de poses sur le calculateur embarqué (10-50 Hz)
           fusionne : LiDAR + IMU (+ ancrage GNSS en facteurs souples)
           produit  : l'odométrie externe injectée dans l'étage 1, et la carte
```

**Odométrie LiDAR-inertielle : FAST-LIO2.**

Pourquoi :

- **Coût de calcul le plus bas** de sa catégorie à précision équivalente, grâce à un
  filtre de Kalman itératif à gain formulé dans l'espace d'état et à un arbre k-d
  incrémental (iKD-Tree) qui évite la reconstruction de l'arbre à chaque scan. C'est
  décisif : sur le drone réel, ce calcul partage un Jetson avec la perception.
- **Méthode directe** (pas d'extraction de features géométriques) : robuste dans les
  environnements non structurés — végétation, terrain, ruines — là où les méthodes à
  features type LOAM/LIO-SAM se dégradent. C'est exactement le cas d'usage « zone
  inconnue ».
- **Validé sur quadricoptère agressif** : les auteurs démontrent un suivi sans dérive
  pendant des retournements à 1200 °/s. La robustesse aux mouvements rapides est une
  propriété non négociable sur un drone.
- Portages ROS 2 disponibles et maintenus.

**Rejeté :**

| Alternative | Pourquoi non |
|---|---|
| **slam_toolbox** | **2D uniquement.** Cité dans le brief, mais fondamentalement inadapté : il construit une grille d'occupation planaire. Un drone vole en 3D et doit connaître l'occupation au-dessus et en dessous de lui. Non négociable. |
| **Cartographer** | Conçu autour du 2D ; son mode 3D est peu utilisé, le portage ROS 2 est en retard, et la maintenance amont s'est réduite. |
| **LIO-SAM** | Bonne précision, mais suppose une IMU 9 axes bien calibrée et alignée, extrait des features (fragile en milieu non structuré), et coûte plus cher en calcul. Sa vraie force — le graphe de facteurs avec fermeture de boucle et facteurs GPS — est justement ce qu'on ajoute séparément à l'étage 2. |
| **KISS-ICP** | Élégant et robuste, mais **n'utilise pas l'IMU**. Sur un drone, ignorer l'IMU pendant les mouvements rapides est un handicap gratuit. |
| **GLIM** | Techniquement supérieur (optimisation globale, fermeture de boucle, multi-capteurs, accélération GPU). Mais plus lourd, plus jeune, et son accélération GPU passe par CUDA. **Retenu comme option d'évolution en P4** si la dérive mesurée l'exige. |
| **RTAB-Map** | Généraliste, très complet, mais orienté RGB-D et robots au sol ; surdimensionné et mal adapté au LiDAR aérien rapide. |

**Fermeture de boucle et dérive** : FAST-LIO2 est une odométrie, pas un SLAM complet — il
n'a pas de fermeture de boucle et dérive donc lentement. La correction se fait à l'étage 2
par un graphe de poses (GTSAM) qui reçoit les poses clés de FAST-LIO2, des contraintes de
fermeture de boucle et des **facteurs GNSS souples** quand le GPS est disponible. La
correction est appliquée sur la transformation `map → odom`, jamais par téléportation de
`base_link` — conformément à REP-105. Détail : [06](06-slam-et-cartographie.md).

*Révision si* : la dérive mesurée sur trajectoire longue (> 1 km) dépasse le budget d'erreur
fixé en P2 → bascule vers GLIM.

Détail : [ADR-0004](../adr/0004-slam.md).

---

## 6. Représentation de la carte 3D

**Décision : OctoMap comme carte d'occupation persistante + ESDF local roulant pour la
planification. Open3D en post-traitement hors ligne.**

Pourquoi OctoMap :

- **Il représente explicitement l'inconnu.** C'est la propriété décisive et elle est
  souvent négligée : un octree OctoMap distingue *libre*, *occupé* et *non observé*. Sans
  cette troisième valeur, la détection de frontières — le cœur de l'exploration autonome —
  est impossible. Une carte qui ne modélise pas l'ignorance ne peut pas guider l'exploration.
- Modèle probabiliste (log-odds) : robuste au bruit capteur et aux obstacles mobiles.
- Compression par octree : une zone de plusieurs km³ tient en mémoire.
- Portage ROS 2 (`octomap_server2`) et affichage RViz natifs.

Pourquoi un **ESDF local séparé** : OctoMap répond à « cette cellule est-elle occupée ? »
mais pas à « à quelle distance est l'obstacle le plus proche ? ». Or c'est cette seconde
question que le planificateur et l'optimiseur de trajectoire posent des milliers de fois
par seconde. On maintient donc une grille de distance euclidienne signée sur une fenêtre
roulante centrée sur le drone (typiquement 40 × 40 × 20 m à 0,2 m), recalculée
incrémentalement. Local, borné, rapide.

**Rejeté :**

| Alternative | Pourquoi non |
|---|---|
| **nvblox** | Excellent (TSDF + ESDF sur GPU), mais **CUDA obligatoire**. Éliminé par le matériel. |
| **Voxblox** | La référence historique TSDF+ESDF, mais le portage ROS 2 est faible et la maintenance amont a cessé. |
| **wavemap** (ETH, 2023) | Très intéressant : représentation hiérarchique multi-résolution, bien plus économe en mémoire qu'OctoMap, ROS 2 natif, et modélise l'inconnu. **Sérieux candidat d'évolution en P4** si OctoMap devient le goulot mémoire ou CPU. Pas en V1 : moins de vécu, moins de documentation, et OctoMap suffit à la première boucle. |
| **Open3D comme carte en ligne** | Erreur de catégorie. Open3D est une bibliothèque de traitement de nuages de points, pas une structure de carte incrémentale temps réel. **Conservé, mais à sa place** : post-traitement hors ligne après mission (filtrage, reconstruction de maillage par Poisson, export). |
| **Grille d'occupation dense** | Consommation mémoire prohibitive en 3D sur de grands volumes. |

Détail : [06 — SLAM et cartographie](06-slam-et-cartographie.md).

---

## 7. Navigation et planification

**Décision : pile de planification maison à trois niveaux. Nav2 n'est pas utilisé.**

```
Niveau 3  Planificateur global   A* / JPS sur voxels OctoMap        ~1 Hz, replanification à la demande
Niveau 2  Générateur de traj.    Lissage polynomial + gradients ESDF ~10 Hz
Niveau 1  Filtre réactif         Contrôle par le nuage brut          ~30 Hz, déterministe, autorité de veto
```

**Pourquoi pas Nav2** — c'est un rejet important, expliqué en détail dans
[ADR-0005](../adr/0005-pas-de-nav2.md), résumé ici :

Nav2 est une excellente pile, mais elle est construite autour d'un modèle qui ne
correspond pas : un robot au sol se déplaçant sur une **grille de coût 2D**, avec des
contrôleurs (DWB, MPPI, RPP) qui raisonnent en `(x, y, θ)` et des plugins de costmap
fondamentalement planaires. Un quadricoptère se déplace dans `SE(3)`, doit passer au-dessus
et en dessous d'obstacles, et n'a pas de contrainte non holonome à respecter. Utiliser
Nav2 impose de projeter le monde en 2,5D — on jette précisément l'information 3D que le
LiDAR et OctoMap viennent de produire. On paierait la complexité de Nav2 pour perdre en
capacité.

Ce qu'on **garde** de l'écosystème Nav2 : **BehaviorTree.CPP** pour l'exécutif de mission.
C'est la partie réellement réutilisable et indépendante du 2D — un moteur d'arbres de
comportement mature, introspectable, avec un outil de visualisation (Groot). La logique de
mission (« décoller, puis explorer tant que la batterie le permet, sinon rentrer ») s'y
exprime lisiblement et se modifie sans recompiler.

Pourquoi **A*/JPS sur voxels** en V1 plutôt qu'un échantillonnage type RRT* (OMPL) :
déterministe (P3), rapide sur un volume borné, résultat reproductible pour les tests, et
un chemin trouvé est un chemin optimal sur la grille. OMPL/RRT* devient pertinent si les
volumes deviennent grands et clairsemés — noté comme évolution, pas comme V1.

*Révision si* : la planification globale dépasse son budget temps sur de grands volumes
(→ RRT* + raccourcissement), ou si les trajectoires générées sont trop conservatrices
(→ optimiseur type EGO-Planner avec B-splines).

Détail : [07 — Exploration](07-exploration.md).

---

## 8. Runtime d'inférence IA

**Décision : PyTorch pour l'entraînement, export ONNX, exécution par ONNX Runtime avec
fournisseur d'exécution enfichable.**

C'est le choix qui résout élégamment la contrainte matérielle **et** prépare le drone réel.

```
PyTorch (entraînement / fine-tuning, hors ligne)
        │
        ▼  export
    modèle .onnx  ─── artefact versionné, indépendant du matériel
        │
        ▼  ONNX Runtime — fournisseur d'exécution sélectionné au démarrage
        ├── CPUExecutionProvider      Ryzen 5950X, WSL2 ou natif — défaut de développement
        ├── ROCMExecutionProvider     RX 6900 XT sous Linux natif (HSA_OVERRIDE_GFX_VERSION=10.3.0)
        ├── DmlExecutionProvider      DirectML sur Windows — repli si besoin
        └── TensorrtExecutionProvider Jetson Orin — cible du drone réel
```

Pourquoi c'est le bon choix ici :

- **Le code de perception ne connaît pas le matériel.** Le même nœud ROS 2 tourne sur CPU
  en développement et sur TensorRT à bord. Le fournisseur est une ligne de configuration.
  C'est l'application directe du principe P6.
- **Il contourne l'absence de CUDA sans compromis structurel.** L'inférence de perception
  tourne à 2-5 Hz — pas 30 Hz : une décision d'exploration ne se prend pas à la fréquence
  vidéo. À 5 Hz, un YOLO-nano quantifié tourne confortablement sur un 5950X. Le GPU est un
  confort, pas une nécessité.
- **Il force la discipline du modèle figé.** Un artefact ONNX versionné est reproductible,
  ce qu'un checkpoint PyTorch chargé dynamiquement n'est pas.

**Rejeté :** PyTorch en inférence directe (dépendance lourde, pas de chemin TensorRT
propre, non déterministe entre versions) ; TensorRT seul (verrouille sur NVIDIA dès le
développement) ; OpenVINO (excellent sur CPU Intel, sans intérêt sur Ryzen et sans chemin
Jetson).

Détail : [08 — IA](08-ia.md).

---

## 9. Protocole et liaison radio

**Décision : interface `CommunicationLink` + protocole applicatif DLP encodé en Protobuf.**

Pourquoi **Protobuf** plutôt que JSON ou CBOR : le budget est de **50 kbps partagés**
entre télémétrie, événements, deltas de carte et commandes. À ce débit, la compacité n'est
pas un détail d'optimisation, c'est une contrainte de conception. Protobuf apporte en plus
ce dont un protocole inter-langages a besoin : un schéma unique qui génère le code Python
(drone et serveur), TypeScript (dashboard) et C++ (nœuds ROS 2 critiques) — donc
impossibilité structurelle de divergence entre les trois. Le coût est un générateur de code
dans la chaîne de build ; c'est un coût qu'un projet sérieux paie volontiers.

CBOR aurait été plus simple à mettre en place mais sans schéma partagé — on aurait
réinventé la validation à la main dans trois langages.

Le détail du cadrage, des classes de priorité, du store-and-forward et du modèle de canal
radio est dans [09 — Communication](09-communication.md) et
[ADR-0007](../adr/0007-protocole-radio.md).

---

## 10. Serveur

**Décision : monolithe modulaire FastAPI (Python 3.12) + PostgreSQL avec TimescaleDB et
PostGIS.**

L'architecture en microservices proposée dans le brief (API Gateway, Mission Service,
Drone Service, Telemetry Service, Map Service, AI Service, Event Bus) est **la bonne
architecture logique et la mauvaise architecture de déploiement pour une V1.** Les
frontières de responsabilité qu'elle décrit sont justes et on les conserve — comme
**modules Python à frontières explicites** à l'intérieur d'un seul processus.

Le raisonnement : les microservices achètent l'indépendance de déploiement, la mise à
l'échelle différenciée et l'isolation des pannes. Ces trois bénéfices sont nuls quand il
y a un développeur, un serveur et dix drones. En face, ils coûtent : sept dépôts ou
répertoires à construire et déployer, un bus de messages à opérer, la cohérence
distribuée à gérer, le traçage distribué à mettre en place pour déboguer. C'est un
mauvais échange, et c'est la réponse directe à la question posée dans le brief.

La règle qui rend l'extraction possible plus tard : **aucun module n'importe le modèle de
données d'un autre ni ne requête sa table.** Les modules communiquent par appels de
service typés et par événements publiés sur un bus interne. Le jour où `telemetry` doit
devenir un service séparé, on remplace l'appel de fonction par un appel réseau et le bus
en mémoire par Redis. Rien d'autre ne bouge.

Pourquoi **Python/FastAPI** : cohérence avec le reste de la stack (ROS 2 Python, PyTorch,
outillage de simulation) — un seul langage côté serveur et côté IA, donc les modèles de
données Pydantic et le code Protobuf généré sont partagés littéralement. FastAPI apporte
l'async natif (indispensable pour les WebSockets de télémétrie), la validation Pydantic et
l'OpenAPI généré, qui produit à son tour le client TypeScript du dashboard.

Pourquoi **une seule base** : PostgreSQL joue trois rôles avec deux extensions —
relationnel (missions, drones, utilisateurs), séries temporelles via **TimescaleDB**
(télémétrie : hypertables, compression, agrégats continus) et géospatial via **PostGIS**
(zones de mission, trajectoires, géorepérage). Une base à opérer au lieu de trois. InfluxDB
et MongoDB ajouteraient de l'exploitation sans capacité supplémentaire à cette échelle.

Détail : [ADR-0006](../adr/0006-serveur-monolithe-modulaire.md), [11 — Serveur](11-serveur.md).

---

## 11. Dashboard

**Décision : React 19 + TypeScript + Vite, rendu géospatial et 3D par MapLibre GL JS +
deck.gl.**

Le choix intéressant est celui du moteur de rendu, parce qu'il faut afficher deux choses
de natures différentes : une situation géographique (position, trajectoire, zone de
mission, portée radio sur une carte) et une carte 3D volumétrique (voxels d'occupation,
nuages de points, frontières).

**deck.gl sur MapLibre GL** fait les deux avec un seul moteur WebGL2 : `PointCloudLayer`
pour les nuages, `ColumnLayer`/`PolygonLayer` pour les voxels et les zones, `PathLayer`
pour les trajectoires, `TripsLayer` pour l'animation temporelle — le tout géoréférencé et
capable d'afficher des centaines de milliers de points sans effort.

**Rejeté :** **CesiumJS** (excellent en géospatial 3D avec terrain réel, mais lourd, API
plus complexe, et son écosystème de terrain pousse vers Cesium ion, service tiers — contre
la contrainte open source et hors ligne) ; **three.js / react-three-fiber** seul (superbe
en 3D libre, mais tout le géoréférencement serait à écrire) ; **les deux ensemble** (deux
moteurs WebGL dans une page = complexité et mémoire doublées pour un gain marginal).

Détail : [12 — Dashboard](12-dashboard.md).

---

## Tableau récapitulatif

| Domaine | Retenu | Principal rejeté |
|---|---|---|
| Plateforme | Ubuntu 24.04 natif | WSL2 (gardé en dégradé), achat GPU NVIDIA |
| Simulateur | Gazebo Sim Harmonic + PX4 SITL | Isaac Sim (CUDA), AirSim (mort), Gazebo Classic (EOL) |
| Middleware | ROS 2 Jazzy | Lyrical Luth (trop jeune), ROS 1 (mort) |
| Lien autopilote | uXRCE-DDS + `px4_ros2_cpp` | MAVLink comme protocole applicatif |
| Odométrie | FAST-LIO2 | LIO-SAM, KISS-ICP, Cartographer, slam_toolbox (2D !) |
| Optimisation globale | GTSAM (graphe de poses) | GLIM (gardé en évolution) |
| Carte 3D | OctoMap + ESDF local | nvblox (CUDA), Voxblox (mort), wavemap (évolution) |
| Planification | A*/JPS + lissage + filtre réactif | Nav2 (2D au sol), OMPL (évolution) |
| Exécutif | BehaviorTree.CPP | FSM monolithique, SMACH |
| Inférence | ONNX Runtime multi-EP | PyTorch direct, TensorRT seul |
| Radio | `CommunicationLink` + DLP/Protobuf | MAVLink, JSON, CBOR |
| Serveur | FastAPI monolithe modulaire | Microservices (V1), NestJS, Go |
| Base | PostgreSQL + TimescaleDB + PostGIS | InfluxDB, MongoDB |
| Dashboard | React + MapLibre + deck.gl | CesiumJS, three.js seul |
| Infra | Docker Compose à profils | Kubernetes (V1) |
