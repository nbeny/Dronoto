# 10 — Failsafe et résilience

## 1. Défense en profondeur

Trois couches indépendantes, du plus intelligent au plus fiable. Chacune couvre les
défaillances de la précédente.

```
┌────────────────────────────────────────────────────────────────────────┐
│ COUCHE 3 — mission_executive (BehaviorTree)                            │
│ Intelligent, contextuel, faillible.                                    │
│ « La batterie est basse et la zone est presque couverte : je termine   │
│   cette frontière puis je rentre. »                                    │
└────────────────────────────────────────────────────────────────────────┘
                                    ▼ si défaillant ou dépassé
┌────────────────────────────────────────────────────────────────────────┐
│ COUCHE 2 — safety_supervisor (FSM, calculateur embarqué)                │
│ Déterministe, borné, testable exhaustivement.                          │
│ « Batterie sous le seuil de retour. Mission annulée. Retour immédiat. »│
│ Autorité de veto sur toutes les consignes.                             │
└────────────────────────────────────────────────────────────────────────┘
                                    ▼ si le calculateur embarqué meurt
┌────────────────────────────────────────────────────────────────────────┐
│ COUCHE 1 — Failsafes PX4 (contrôleur de vol, RTOS, indépendant)         │
│ Simple, éprouvé, matériellement séparé.                                │
│ « Plus de flux offboard. Retour à la base. »                           │
│ Fonctionne même si Linux a planté.                                     │
└────────────────────────────────────────────────────────────────────────┘
                                    ▼ si tout échoue
┌────────────────────────────────────────────────────────────────────────┐
│ COUCHE 0 — Reprise en main manuelle (RC directe, drone réel uniquement)│
│ Liaison physique séparée. Autorité humaine ultime.                     │
└────────────────────────────────────────────────────────────────────────┘
```

**La propriété qui compte : l'indépendance.** La couche 1 ne dépend d'aucun code de ce
projet. Le calculateur embarqué peut faire un kernel panic, saturer sa mémoire, se bloquer
sur un deadlock — PX4 constate l'absence de consignes et agit. Sur le drone réel, PX4 est
même alimenté séparément.

Une architecture où le calculateur embarqué serait la seule couche de sécurité serait une
architecture où un bug logiciel dans du code Python entraîne une chute. C'est inacceptable,
et c'est l'erreur la plus courante dans les projets de ce type.

## 2. Machine à états de sécurité

### États

| État | Signification | Consignes autorisées |
|---|---|---|
| `BOOT` | Initialisation, capteurs non prêts | Aucune |
| `IDLE` | Au sol, désarmé, sain | Aucune |
| `PREFLIGHT` | Vérifications en cours | Aucune |
| `READY` | Vérifications passées, armement possible | Aucune |
| `ARMED` | Armé, au sol | Décollage uniquement |
| `TAKEOFF` | Montée | Mode PX4 natif |
| `NOMINAL` | Vol nominal, tous systèmes sains | Toutes |
| `DEGRADED` | Vol avec capacité réduite | Limitées (vitesse, portée) |
| `HOLD` | Vol stationnaire, en attente de résolution | Maintien de position seulement |
| `RETURNING` | Retour à la base en cours | Consignes de trajectoire de retour |
| `LANDING` | Descente contrôlée sur site choisi | Consignes de descente |
| `EMERGENCY_LAND` | Descente immédiate sur place | Descente verticale |
| `TERMINATED` | Arrêt (au sol ou coupure) | Aucune |

### Transitions

```
BOOT ──▶ IDLE ──▶ PREFLIGHT ──▶ READY ──▶ ARMED ──▶ TAKEOFF ──▶ NOMINAL
                       │                                            │  ▲
                       │ échec                                      │  │ résolu
                       ▼                                            ▼  │
                     IDLE                                        DEGRADED
                                                                  │  ▲
                                                    dégradation ▼  │  │ résolu
                                                              HOLD ─┘
                                                                │
                            ┌───────────────────────────────────┤
                            │                                   │
          batterie / fin ▼  │  perte critique persistante  ▼    │
                    RETURNING ──────▶ LANDING ──▶ TERMINATED    │
                            │                                   │
                            └──▶ EMERGENCY_LAND ────────────────┘
                                (depuis N'IMPORTE quel état en vol)
```

`EMERGENCY_LAND` est atteignable depuis **tous** les états en vol, sans condition
intermédiaire. Une machine à états de sécurité dont l'état le plus sûr n'est pas
immédiatement atteignable n'est pas une machine à états de sécurité.

### Priorité des conditions

Les conditions sont évaluées à 10 Hz **dans cet ordre**, et la première qui s'applique
gagne. L'ordre est la spécification : il encode la hiérarchie des dangers.

```
1.  Failsafe PX4 actif (drapeau matériel)     ──▶ suivre PX4, ne pas lutter
2.  Batterie critique (< 10 % ou tension mini)──▶ EMERGENCY_LAND
3.  Perte d'estimation d'état (EKF2 divergé)  ──▶ EMERGENCY_LAND
4.  Géorepérage franchi                       ──▶ RETURNING
5.  Batterie sous réserve de retour           ──▶ RETURNING
6.  Localisation POOR persistante (> 10 s)    ──▶ HOLD puis EMERGENCY_LAND
7.  Perte de tous les capteurs de perception  ──▶ HOLD puis RETURNING
8.  Consignes périmées (> 500 ms)             ──▶ HOLD
9.  Perte de liaison + politique RTH          ──▶ RETURNING
10. Localisation DEGRADED                     ──▶ DEGRADED
11. Perte d'un capteur non critique           ──▶ DEGRADED
12. Aucune condition                          ──▶ NOMINAL
```

Le point 1 mérite un commentaire : quand PX4 a déclenché son propre failsafe, le
superviseur **cesse d'émettre des consignes** au lieu de tenter de reprendre la main. Deux
autorités qui se disputent le contrôle produisent un comportement pire que n'importe
laquelle des deux. On cède.

## 3. Matrice de dégradation

Que perd-on, et que fait-on. Chaque ligne correspond à un test automatisé.

| Panne | Détection | Impact | Réaction | Mission |
|---|---|---|---|---|
| **Perte GNSS** | `vehicle_gps_position` + `estimator_status_flags` | Pas de position absolue | EKF2 bascule sur EV ; vitesse max 12→6 m/s ; RTH par trajectoire enregistrée ; `w_l` ↑ dans le scoring | **Continue** |
| **GNSS dégradé** | HDOP, nb satellites | Position imprécise | Pondération réduite ; marges de sécurité augmentées | Continue |
| **GNSS trompeur** (multi-trajets) | Test d'innovation + verrou applicatif 5 s | Position fausse et confiante | Rejet, journalisation, cohérence exigée avant réintégration | Continue |
| **Perte LiDAR** | Âge du topic > 1 s | Plus de SLAM ni de carte 3D | Si GNSS ok : mode dégradé caméra+IMU, vitesse 3 m/s, pas de nouvelle exploration → RTH. Si GNSS perdu aussi : `EMERGENCY_LAND` | **Suspendue** |
| **LiDAR dégradé** | Taux de points, santé de l'odométrie | Carte bruitée | Confiance réduite, vitesse réduite | Continue dégradée |
| **Perte caméra** | Âge du topic | Pas de sémantique | Perception IA désactivée, poids sémantiques à zéro | **Continue** |
| **Divergence SLAM** | `odometry_health` : résidus, dégénérescence | Position dérivante | `HOLD`, tentative de remontée pour retrouver de la structure ; si > 10 s → `EMERGENCY_LAND` | Suspendue |
| **Perte radio** | 5 s sans trame valide | Pas de supervision | Spool + émission aveugle + politique de mission | **Continue** (défaut) |
| **Serveur indisponible** | Pas d'acquittement applicatif | Aucun | **Aucune.** Le drone ne dépend pas du serveur. | Continue |
| **Batterie < réserve retour** | Calcul dynamique d'énergie | Autonomie insuffisante | `RETURNING` | **Annulée** |
| **Batterie < 10 %** | SoC + tension sous charge | Danger immédiat | `EMERGENCY_LAND` sur le meilleur site connu | Annulée |
| **Consignes périmées** | Âge > 500 ms dans `px4_interface` | Amont défaillant | `HOLD` + maintien de position | Suspendue |
| **Surcharge CPU** | Budgets de latence dépassés | Latences hors budget | Dégradation ordonnée : IA désactivée, puis résolution de carte réduite, puis fréquence de replanification | Continue dégradée |
| **Panne moteur** | `FailsafeFlags` PX4 | Contrôle réduit | PX4 gère ; superviseur demande atterrissage immédiat | Annulée |
| **Géorepérage** | Position vs polygone | Sortie de zone autorisée | `RETURNING` immédiat | Annulée |

### Le cas le plus intéressant : perte simultanée GNSS + LiDAR

C'est le seul cas où le drone perd toute référence de position absolue **et** relative
fiable. Il ne reste que l'IMU (dérive en quelques secondes) et le baromètre (altitude
seulement).

Réaction, dans l'ordre :

1. Tentative de survie par odométrie visuelle si la caméra fonctionne (mode dégradé,
   quelques dizaines de secondes de validité).
2. Sinon : **descente verticale contrôlée immédiate**, à vitesse réduite, sur la position
   actuelle. Pas de tentative de retour — se déplacer sans savoir où l'on est est plus
   dangereux que de se poser à l'aveugle sur un site inconnu.
3. Utilisation de la dernière carte connue pour évaluer le sol sous le drone si elle est
   encore valide.

Ce raisonnement — **immobiliser plutôt que déplacer quand la position est inconnue** — est
la règle générale. Un drone qui bouge sans savoir où il est est un projectile.

## 4. Chiens de garde

`health_monitor` surveille tous les flux critiques. Chaque topic a un âge maximal et une
fréquence minimale ; leur violation produit un événement typé et alimente les conditions du
superviseur.

| Surveillé | Seuil | Action au dépassement |
|---|---|---|
| `sensors/imu` | > 100 ms | Critique — `EMERGENCY_LAND` |
| `sensors/lidar/points` | > 1 s | Voir matrice |
| `state/odometry` | > 200 ms | Critique — `HOLD` puis `EMERGENCY_LAND` |
| `state/battery` | > 3 s | `DEGRADED` + supposer le pire cas |
| `control/setpoint_final` | > 500 ms | `HOLD` |
| `map/octomap_binary` | > 5 s | `DEGRADED`, exploration suspendue |
| `safety/state` | > 500 ms | Le superviseur lui-même est mort → PX4 prend le relais |
| Boucle du superviseur | jitter > 50 ms | Alerte de surcharge, dégradation préventive |

Le cas « le superviseur lui-même est mort » se résout par construction : `px4_interface`
cesse de publier des consignes s'il ne reçoit plus `safety/state`, ce qui provoque en 500 ms
la sortie du mode offboard par PX4, donc son failsafe. **Le mécanisme de sécurité de
dernier recours ne dépend pas du composant de sécurité.**

## 5. Retour à la base

### Calcul de la réserve

```
distance_retour  = longueur du chemin planifié vers la base
                   (à défaut de carte : distance euclidienne × 1,3)
énergie_retour   = distance_retour / v_croisière × P_moyenne
énergie_manoeuvre= énergie de montée à l'altitude de croisière de retour
énergie_atterr.  = 30 s de stationnaire + descente
énergie_vent     = pénalité si vent de face estimé au retour

réserve_requise  = (énergie_retour + manoeuvre + atterrissage + vent) × 1,4
```

Le facteur 1,4 couvre l'imprécision du modèle énergétique, la dégradation de la batterie
avec les cycles, et l'imprévu. Recalculé **en continu**, pas une fois au décollage : la
réserve requise change à chaque mètre parcouru.

### Stratégies selon le contexte

| Contexte | Stratégie de retour |
|---|---|
| GNSS + carte disponibles | Chemin planifié dans la carte, altitude de croisière sûre |
| GNSS ok, carte partielle | Montée à une altitude sûre (au-dessus du plus haut obstacle connu + 15 m) puis ligne droite |
| GNSS perdu, carte disponible | **Suivi de la trajectoire enregistrée à l'envers** (breadcrumb) — chemin connu comme franchissable |
| GNSS perdu, SLAM dégradé | Pas de retour. `EMERGENCY_LAND` sur place. |
| Batterie critique en route | Abandon du retour, atterrissage sur le meilleur site à portée |

Le **suivi de trajectoire inverse** en GNSS perdu est important : une ligne droite vers la
base en position dérivante peut traverser un obstacle. Le chemin déjà parcouru est, lui,
garanti franchissable — c'est l'information la plus fiable disponible dans cette situation.

## 6. Le scénario de référence

Le scénario du brief, formalisé comme test automatisé de bout en bout (`L3`,
`tests/scenarios/full_resilience.yaml`). Il est le critère de sortie de la phase P6.

| # | Étape | Injection | Assertion |
|---|---|---|---|
| 1 | Décollage | — | Altitude atteinte < 20 s, état `NOMINAL` |
| 2 | GNSS disponible | — | `estimator_status_flags` : GPS fusionné |
| 3 | Radio disponible | — | `LinkStatus.state == CONNECTED` |
| 4 | Mission de cartographie | Mission envoyée par le sol | Mission acceptée, exploration démarrée |
| 5 | Perte GNSS | `gnss_loss` à t=120 s | Bascule EV < 2 s ; `NAV_QUALITY == SLAM_ONLY` |
| 6 | SLAM seul | — | ATE < 2 m sur 200 m ; **mission continue** |
| 7 | Perte radio | `radio_blackout` à t=180 s | Détection < 5 s ; spool actif ; **mission continue** |
| 8 | Autonomie | — | Couverture progresse pendant la coupure |
| 9 | Batterie faible | Décharge accélérée à t=300 s | `RETURNING` déclenché par la réserve dynamique |
| 10 | Retour | — | Suivi de trajectoire inverse (GNSS perdu) ; pas de collision |
| 11 | Atterrissage | — | Site géométriquement valide ; vitesse d'impact < 1 m/s ; distance à la base < 5 m |
| 12 | Reconnexion | Fin du `radio_blackout` | Poignée de main < 10 s |
| 13 | Synchronisation | — | Spool vidé ; **aucun événement perdu** ; carte cohérente au sol |

**Assertions globales** valables sur toute la durée : distance minimale à un obstacle
> 1,5 m ; aucune sortie de géorepérage ; aucun message du plan de contrôle sur la radio ;
aucune violation de budget de latence sur la chaîne critique.

## 7. Ce que la simulation ne teste pas

Honnêteté sur les limites, parce qu'un système validé en simulation qu'on croit validé tout
court est un système dangereux.

| Non couvert | Pourquoi | Atténuation |
|---|---|---|
| Défaillances matérielles physiques | Pas de modèle (soudure, connecteur, vibration) | Redondance matérielle, essais au sol, checklists |
| Interférences électromagnétiques | Non modélisées | Essais réels, blindage, séparation des câbles |
| Vibrations sur les IMU | Gazebo ne les modélise pas | Montage amorti, validation en vol réel |
| Thermique (throttling Jetson) | Non modélisé | Bancs thermiques, marges de calcul |
| Effet de sol, vortex ring state | Hors modèle aérodynamique | PX4 les gère ; validation en vol réel |
| Défaillances d'alimentation | Non modélisées | Redondance BEC, surveillance de tension |
| Bugs de PX4 lui-même | On utilise PX4 tel quel | Version épinglée, suivi des correctifs amont |

Ces éléments définissent le contenu du programme d'essais matériels de la phase P9
([17 — Roadmap](17-roadmap.md)). La simulation valide la **logique** ; le vol réel valide
la **physique**. Les deux sont nécessaires et aucun ne remplace l'autre.
