# 00 — Vision, principes et périmètre

## 1. Le problème à résoudre

Faire voler un quadricoptère dans une zone inconnue, sans carte préalable, sans
opérateur, sans garantie de liaison radio, et en rapporter une carte 3D exploitable —
puis rentrer se poser.

Trois sous-problèmes réellement difficiles se cachent derrière cet énoncé :

1. **Savoir où l'on est** quand le GPS disparaît et que la seule référence est un nuage
   de points qui dérive.
2. **Décider où aller** quand personne ne le dit, avec une batterie qui se vide et une
   carte partielle.
3. **Rester en vie** quand un capteur ment, quand la radio tombe, quand le serveur ne
   répond plus.

Tout le reste (API, dashboard, Docker) est de l'infrastructure. Utile, nécessaire, mais
pas là où se joue la difficulté. Le dossier consacre son effort en conséquence.

## 2. Principes d'ingénierie non négociables

Ces sept principes tranchent tous les arbitrages du projet. Quand un choix technique est
ambigu, on remonte à ces principes.

### P1 — La sécurité est locale, l'intelligence est distante

Aucune boucle de sécurité ne traverse la radio. Le serveur peut être éteint pendant toute
la mission sans conséquence sur la sécurité du vol. Corollaire : tout composant qui a
autorité sur la sécurité tourne sur le calculateur embarqué, et son échec doit dégrader
vers un état sûr, pas vers l'arrêt.

### P2 — Défense en profondeur, avec une couche indépendante en dernier recours

Le superviseur de sécurité tourne sur le calculateur embarqué (Linux, non temps réel,
faillible). Les failsafes PX4 tournent sur le contrôleur de vol (RTOS, indépendant,
alimentation séparée sur le vrai drone). Si le calculateur embarqué meurt — kernel panic,
OOM, deadlock — PX4 détecte la perte du flux offboard et déclenche son propre failsafe.
**Le calculateur embarqué n'est jamais la seule couche de sécurité.**

### P3 — Déterminisme là où c'est critique

Contrôle, estimation d'état, évitement réflexe, machine à états de sécurité : code
classique, latence bornée, comportement reproductible, testable exhaustivement. Pas de
réseau de neurones dans une boucle dont la défaillance fait tomber le drone. L'IA est
autorisée là où une erreur produit une *mauvaise décision d'exploration*, pas un
*crash*.

### P4 — Toute source d'incertitude est injectable et rejouable

Vent, bruit capteur, latence, perte de paquets, perte GPS, panne LiDAR : chaque
perturbation est un composant explicite, paramétré par YAML, piloté par un générateur
pseudo-aléatoire **à graine fixée**. Une exécution de test est reproductible bit à bit.
Sans cela, aucun test de résilience n'est crédible.

### P5 — Les frontières entre composants sont des contrats, pas des conventions

Chaque frontière majeure (radio, inférence IA, carte, planificateur) passe par une
interface explicite avec au moins deux implémentations dès la V1 — dont une factice pour
les tests. Une interface avec une seule implémentation est une interface non validée.

### P6 — Le simulateur est un citoyen de première classe

Le code embarqué ne sait pas s'il est en simulation. Aucun `if simulation:` dans la
logique métier. La simulation se substitue aux pilotes de capteurs et au canal radio,
rien d'autre. C'est ce qui rend le passage au réel possible.

### P7 — Pragmatisme : la V1 minimale doit voler

Chaque phase produit quelque chose qui décolle, cartographie ou survit à une panne. On
n'écrit pas six mois d'infrastructure avant le premier vol simulé. Le premier décollage
autonome est en semaine 3.

## 3. Contraintes du brief, traduites en décisions

| Contrainte | Traduction architecturale |
|-----------|---------------------------|
| Pas de dépendance 4G | `CommunicationLink` abstrait ; l'implémentation par défaut cible un modem série 868 MHz à 50 kbps. Le budget de bande passante est dimensionné sur ce pire cas. |
| Pas de contrôle moteur depuis le serveur | Le protocole radio n'a **aucun** message de type attitude/rate/moteur. C'est une propriété du schéma Protobuf, vérifiée par un test. |
| Autonomie en cas de perte réseau | Le drone n'attend jamais une réponse serveur pour progresser. Toute commande serveur est un *événement*, pas une *dépendance*. |
| Simulable à 100 % | Aucun composant ne dépend d'un service SaaS, d'une clé API ou d'un matériel spécifique pour tourner. |
| Extensible à plusieurs drones | Tout est namespacé par `drone_id` dès la V1 : topics ROS 2 (`/drone_1/...`), adressage DLP, schéma de base de données. Le coût est nul maintenant, prohibitif plus tard. |
| Déterminisme des composants critiques | Voir P3. Documenté composant par composant dans [08 — IA](08-ia.md). |
| IA là où elle apporte de la valeur | Voir [08 — IA](08-ia.md), section « justification composant par composant ». |
| Open source | Toute la stack est OSS. Seules exceptions tolérées plus tard : pilotes de capteurs propriétaires (Livox SDK est OSS, RealSense aussi). |
| Évolutif vers le réel | Voir P6 et [16 — Matériel](16-materiel.md). |
| Sécurité dès le début | Le superviseur de sécurité est livré en P1, avant l'exploration, pas après. |

## 4. Découpage en sous-projets

**Le projet tel que décrit est trop gros pour une seule spécification d'implémentation.**
Il contient au moins six systèmes indépendants qui, dans une équipe, seraient six
personnes. Tenter de tout spécifier au même niveau de détail avant d'écrire une ligne
serait du gaspillage : les décisions des phases tardives dépendent de mesures que seules
les phases précoces produisent.

Le dossier définit donc **l'architecture complète** (les contrats entre systèmes, qui ne
doivent pas bouger) mais **spécifie finement seulement les premières phases**. Chaque
sous-projet reçoit sa propre spécification détaillée au moment de son implémentation.

```
                      ARCHITECTURE (ce dossier — stable)
                                   │
   ┌───────────┬───────────┬───────┴────┬───────────┬───────────┬──────────┐
   │           │           │            │           │           │          │
  SP1         SP2         SP3          SP4         SP5         SP6        SP7
 Socle      Percep-      Naviga-     Explora-     Liaison     Superv.      IA
 de vol     tion &       tion &      tion        & serveur    & résil.   perception
            SLAM         évitement   autonome
   │           │           │            │           │           │          │
   └───────────┴───────────┴─────┬──────┴───────────┴───────────┴──────────┘
                                 │
                          dépendances fortes ──▶
```

| Sous-projet | Périmètre | Dépend de | Phase |
|---|---|---|---|
| **SP1 — Socle de vol** | Gazebo + PX4 SITL + pont ROS 2 + TF + décollage/atterrissage/waypoint offboard | — | P0-P1 |
| **SP2 — Perception & SLAM** | LiDAR simulé, FAST-LIO2, fusion EKF2, OctoMap, RViz | SP1 | P2 |
| **SP3 — Navigation & évitement** | Planificateur A*/JPS, suivi de trajectoire, filtre réactif | SP2 | P3 |
| **SP4 — Exploration autonome** | Frontières, points de vue, scoring, NBV, exécutif de mission | SP3 | P4 |
| **SP5 — Liaison & serveur** | `CommunicationLink`, DLP, station sol, API, base, dashboard | SP1 | P5 |
| **SP6 — Supervision & résilience** | Machine à états de sécurité, injecteur de pannes, scénarios | SP4, SP5 | P6 |
| **SP7 — IA perception** | Détection, segmentation, carte de risque, sites d'atterrissage | SP2 | P7 |

SP5 est délibérément **parallélisable** avec SP2-SP4 : il ne dépend que du socle de vol.
C'est le seul parallélisme réel du projet ; le reste est une chaîne.

## 5. Ce qui est explicitement hors périmètre

Le dire maintenant évite d'y consacrer du temps par inadvertance.

- **Certification aéronautique** (DO-178C, SORA complet). On applique les bonnes pratiques
  (déterminisme, traçabilité, tests) sans viser une certification.
- **Vol en essaim coordonné.** L'architecture supporte *plusieurs drones supervisés*, pas
  *un essaim coopératif* (partage de carte, allocation de tâches distribuée). C'est un
  projet en soi.
- **Sécurité informatique de niveau production** en V1 : chiffrement et authentification
  de la liaison radio sont conçus (voir [09](09-communication.md)) mais implémentés en P8.
- **Optimisation temps réel dur** sur le calculateur embarqué (PREEMPT_RT, isolation CPU).
  Prévu au passage matériel, pas avant.
- **Cartographie photogrammétrique / maillage texturé haute résolution.** On produit une
  carte d'occupation exploitable par le planificateur, plus un export nuage de points. La
  reconstruction fine est un post-traitement hors ligne (Open3D), pas un objectif temps réel.
