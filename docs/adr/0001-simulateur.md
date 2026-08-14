# ADR-0001 — Gazebo Sim Harmonic plutôt qu'Isaac Sim

- **Statut** : accepté
- **Date** : 2026-08-14
- **Contexte** : choix du simulateur pour l'ensemble du projet

## Décision

**Gazebo Sim Harmonic (gz-sim 8) avec PX4 SITL.** Isaac Sim n'est pas utilisé.

## Contexte

Le brief demande explicitement d'évaluer si NVIDIA Isaac Sim « apporte une vraie valeur ».
Le matériel disponible est une AMD Radeon RX 6900 XT.

## Facteur décisif

**Isaac Sim exige CUDA et des cœurs RT NVIDIA.** Il ne fonctionne pas sur du matériel AMD.
Ce seul point clôt la question pour la configuration actuelle.

Mais même en supposant l'achat d'un GPU NVIDIA, la décision resterait la même à ce stade du
projet.

## Raisons indépendantes du matériel

**L'intégration PX4 de Gazebo est officielle ; celle d'Isaac Sim est communautaire.** Les
modèles de drones, les plugins capteurs et les scripts de lancement Gazebo sont maintenus
dans le dépôt `PX4-Autopilot` lui-même, testés par l'amont à chaque release. Isaac Sim
demande une couche d'intégration tierce, moins testée, qui casse aux montées de version.
Cette couche est exactement l'endroit où un projet de ce type s'enlise.

**La valeur d'Isaac Sim est réelle mais tardive.** Elle tient en deux points : rendu
photoréaliste (utile pour entraîner un modèle de perception destiné au monde réel) et
simulation massivement parallèle (utile pour le reinforcement learning). Ce sont des
besoins de phase P7 et au-delà. Adopter Isaac Sim en P1 coûterait plusieurs semaines
d'intégration pour un bénéfice nul pendant six mois.

**Ce qui compte pour la transposabilité, Gazebo le fait déjà.** PX4 SITL exécute le vrai
code de l'autopilote — vrai EKF2, vrais contrôleurs, vrais failsafes. La fidélité de
l'autopilote pèse infiniment plus lourd, pour ce projet, que la fidélité du rendu ou de
l'aérodynamique fine. Les couches que ce projet valide (SLAM, exploration, résilience)
dépendent de la géométrie, des capteurs, des latences et de la logique — que Gazebo
reproduit correctement.

## Alternatives examinées

| Alternative | Rejet |
|---|---|
| **Isaac Sim** | CUDA obligatoire ; intégration PX4 communautaire ; valeur concentrée sur des besoins de phase tardive |
| **Gazebo Classic** | Fin de vie janvier 2025. Dette immédiate. |
| **AirSim** | Archivé par Microsoft en 2022. Le successeur Project AirSim est propriétaire et payant — élimine la contrainte open source. |
| **Webots** | Bon simulateur, mais intégration PX4 moins mature et modèle LiDAR moins riche. Aucun avantage compensatoire. |
| **jMAVSim** | Pas de capteurs de perception. Inadapté. |
| **Flightmare, FlightGoggles** | Orientés recherche RL, communautés réduites, non alignés sur le rythme de PX4. |

## Conséquences

**Positives** : intégration PX4 sans friction ; `ros_gz_bridge` officiel ; fonctionne sur
GPU AMD ; documentation abondante ; capteurs suffisants.

**Négatives** : rendu moins réaliste — limite l'entraînement de modèles de vision destinés
au réel (atténué en utilisant des modèles pré-entraînés sur données réelles, voir
[08 §4](../architecture/08-ia.md)) ; pas de parallélisation massive — le RL à grande
échelle est hors de portée (accepté, voir [07 §7](../architecture/07-exploration.md)) ;
le motif de scan non répétitif du Livox Mid-360 n'est pas reproduit (documenté et compensé
par une réduction du nombre de rayons, [03 §4](../architecture/03-simulation.md)).

## Condition de révision

Réexaminer si l'une de ces conditions est remplie :

1. Le sim-to-real de la perception visuelle devient bloquant en P7 **et** un GPU NVIDIA est
   disponible ;
2. Le RL passe de piste de recherche à besoin réel (barre fixée en
   [07 §7](../architecture/07-exploration.md)) ;
3. L'intégration PX4 d'Isaac Sim devient officielle et maintenue par l'amont.

Dans tous les cas, Isaac Sim viendrait **en complément** pour l'entraînement hors ligne,
jamais en remplacement de Gazebo pour la validation système — celle-ci reste liée à PX4
SITL.
