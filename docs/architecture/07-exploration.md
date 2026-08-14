# 07 — Exploration autonome

## 1. Le problème

Le drone reçoit : *« cartographie ce polygone entre 10 et 60 m d'altitude »*. Il doit
décider, en boucle, **où aller ensuite** — sans liste de waypoints, sans opérateur, avec
une carte partielle et une batterie qui se vide.

C'est un problème d'optimisation séquentielle sous incertitude. Il n'a pas de solution
exacte praticable (l'optimum global est NP-difficile et suppose de connaître ce qu'on n'a
pas encore observé). L'approche retenue est gloutonne à horizon court, ce qui est le bon
compromis : les approches à horizon long paient une complexité importante pour un gain
faible quand la carte change à chaque instant.

## 2. Pipeline

```
Mission (polygone, bande d'altitude, contraintes)
   │
   ▼
┌─────────────────────┐
│ frontier_detector   │  voxels libres adjacents à l'inconnu, groupés, filtrés
└──────────┬──────────┘  → N clusters de frontière
           ▼
┌─────────────────────┐
│ viewpoint_sampler   │  autour de chaque cluster, poses observables et sûres
└──────────┬──────────┘  → M candidats (position + cap), M ≈ 50-200
           ▼
┌─────────────────────┐
│    SCORING          │  U(v) = w_ig·IG − w_d·Coût − w_r·Risque − w_l·PerteLoc − w_b·Batterie
└──────────┬──────────┘  contrainte dure : faisabilité batterie
           ▼
┌─────────────────────┐
│   nbv_selector      │  argmax U, avec hystérésis anti-oscillation
└──────────┬──────────┘  → objectif unique
           ▼
    global_planner  →  trajectory_generator  →  follower  →  PX4
```

## 3. Échantillonnage des points de vue

Une frontière n'est pas un objectif. Voler **sur** un voxel-frontière est souvent inutile,
parfois dangereux. Ce qu'on cherche, c'est une pose d'**observation** de la frontière.

Pour chaque cluster de frontière, on génère des candidats sur une couronne autour du
centroïde :

```
distances       : {5, 10, 15, 20} m       (portée utile du LiDAR)
azimuts         : 8 directions             (privilégiant l'opposé de la normale du cluster)
altitudes       : {z_cluster −5, z_cluster, z_cluster +5}, bornées par la mission
cap             : orienté vers le centroïde du cluster
```

Filtres appliqués immédiatement, avant tout scoring (une élimination précoce vaut mieux
qu'un score bas) :

| Filtre | Seuil | Raison |
|---|---|---|
| Voxel libre dans l'OctoMap | requis | Ne pas viser un point occupé ou inconnu |
| Distance à l'obstacle le plus proche | ≥ 2,5 m (ESDF) | Marge de sécurité |
| Dans le polygone de mission | requis | Contrainte opérationnelle et réglementaire |
| Dans la bande d'altitude | requis | idem |
| Ligne de vue vers le cluster | requis | Sinon on n'observera rien depuis là |
| Atteignable par l'espace libre | requis | Évite les objectifs derrière un mur |

Il reste typiquement 50 à 200 candidats. C'est le bon ordre de grandeur : assez pour que le
meilleur soit bon, assez peu pour que l'évaluation coûteuse (lancer de rayons) tienne dans
le budget temps.

## 4. Scoring

```
U(v) = w_ig · IG(v)                     gain d'information       [+]
     − w_d  · Coût(v)                   coût du trajet           [−]
     − w_r  · Risque(v)                 danger                   [−]
     − w_l  · PerteLoc(v)               dégradation attendue de la localisation [−]
     − w_b  · Batterie(v)               coût énergétique relatif [−]

sous contrainte dure :  E(trajet vers v) + E(retour base depuis v) · 1,4  ≤  E_restante
```

Chaque terme est normalisé sur [0, 1] avant pondération — sans quoi les poids ne sont pas
comparables et le réglage devient de la sorcellerie.

### 4.1 Gain d'information — `IG(v)`

Nombre de voxels inconnus qui deviendraient observés depuis `v`, estimé par lancer de
rayons dans l'OctoMap selon le FOV du LiDAR.

Coût réel : c'est l'opération dominante du pipeline. Trois optimisations nécessaires :

- **Sous-échantillonnage angulaire** : 1 rayon tous les 2° au lieu du motif complet du
  capteur (~64× moins de rayons, estimation suffisante pour comparer des candidats).
- **Arrêt anticipé** : un rayon s'arrête au premier voxel occupé.
- **Cache par cluster** : les candidats du même cluster partagent une estimation grossière
  du volume inconnu adjacent, affinée seulement pour les mieux classés (évaluation en deux
  passes : grossière sur tous, fine sur le top 20).

Raffinement : pondérer les voxels inconnus par leur **valeur** plutôt que de les compter à
l'identique. Un voxel inconnu au bord d'un grand volume inconnu vaut plus qu'un voxel isolé
dans un recoin déjà cartographié — parce que l'atteindre ouvre l'accès à davantage. C'est
une heuristique de propagation simple, pas de l'apprentissage.

### 4.2 Coût de trajet — `Coût(v)`

Distance du chemin **réel** (pas euclidienne — un mur change tout) plus le coût de rotation
pour aligner le cap.

Compromis assumé : lancer le planificateur A* complet sur 200 candidats est trop coûteux.
On procède en deux temps — distance euclidienne pondérée par un facteur de tortuosité
estimé pour le classement grossier, puis planification réelle sur les **5 meilleurs**
candidats. Si le meilleur s'avère infaisable, on descend dans la liste.

### 4.3 Risque — `Risque(v)`

```
Risque(v) = α · proximité_obstacles(v)        depuis l'ESDF
          + β · densité_inconnu_local(v)      voler dans l'inconnu est risqué en soi
          + γ · risque_sémantique(v)          couche IA : eau, foule, ligne électrique
          + δ · exposition_au_vent(v)         altitude et dégagement
```

Le terme `densité_inconnu_local` mérite d'être noté : il crée une **tension productive**
avec le gain d'information. L'exploration veut aller vers l'inconnu ; la sécurité veut
l'éviter. L'équilibre entre `w_ig` et `w_r` est donc le réglage central du comportement du
drone — un drone prudent et lent, ou un drone rapide et exposé. Ce n'est pas un bug du
modèle, c'est le vrai arbitrage du problème rendu explicite et réglable.

### 4.4 Perte de qualité de localisation — `PerteLoc(v)`

C'est le terme qui distingue une exploration naïve d'une exploration robuste, et il est
souvent absent des implémentations académiques.

```
PerteLoc(v) = pénalité si la zone autour de v est géométriquement dégénérée
                (grande étendue plane, absence de structure verticale, espace ouvert)
            + pénalité si le GNSS y est probablement occulté (géométrie des bâtiments)
            + pénalité si v est loin de toute zone déjà bien cartographiée
```

Conséquence comportementale : quand le GNSS est perdu, `w_l` est **automatiquement
augmenté** par le superviseur. Le drone préfère alors explorer le long de structures
riches en features plutôt que de traverser un grand espace vide où il dériverait. Il
préserve activement sa capacité à savoir où il est.

C'est un exemple de comportement qui paraît « intelligent » et qui ne demande aucun
apprentissage — juste un terme de coût bien choisi. Ce constat gouverne tout le
chapitre [08 — IA](08-ia.md).

### 4.5 Batterie et faisabilité

La **contrainte dure** est plus importante que le terme de score :

```
E_aller(v)     = distance_chemin(v)     / v_croisière × P_moyenne
E_retour(v)    = distance_base_depuis_v / v_croisière × P_moyenne
E_requise(v)   = E_aller(v) + E_retour(v) × 1,4        (marge de sécurité 40 %)

si E_requise(v) > E_restante :  candidat ÉLIMINÉ (pas pénalisé — éliminé)
```

La marge de 40 % couvre le vent de face au retour, l'imprécision de l'estimation d'énergie,
et la réserve d'atterrissage. `E_retour` utilise la distance de chemin **planifié**, pas la
distance à vol d'oiseau, dès que la carte le permet.

Cette contrainte produit naturellement le bon comportement : le rayon d'exploration se
contracte à mesure que la batterie baisse, jusqu'à ce qu'aucun candidat ne soit faisable —
et c'est alors le retour à la base. **Il n'y a pas besoin d'un seuil « batterie < 20 % →
RTH »** : le retour émerge de la contrainte de faisabilité. Le seuil fixe existe quand
même, comme filet de sécurité indépendant dans le superviseur ([10](10-failsafe.md)) —
défense en profondeur.

### 4.6 Poids par défaut et profils

| Poids | Nominal | GNSS perdu | Batterie faible | Prudent |
|---|---|---|---|---|
| `w_ig` gain d'information | 1,00 | 0,70 | 0,50 | 0,60 |
| `w_d` coût de trajet | 0,35 | 0,45 | 0,80 | 0,40 |
| `w_r` risque | 0,50 | 0,60 | 0,60 | 1,00 |
| `w_l` perte de localisation | 0,20 | **0,90** | 0,30 | 0,50 |
| `w_b` coût énergétique | 0,25 | 0,30 | 0,70 | 0,30 |

Les profils sont commutés par le superviseur selon l'état du système, et modifiables à
chaud depuis le dashboard via `exploration/set_weights`. Les valeurs initiales sont des
points de départ à régler empiriquement en P4 — et ce réglage est mesurable, puisque les
tests L3 produisent des métriques de couverture par unité d'énergie.

## 5. Stabilité de la décision

Un sélecteur glouton pur oscille : la carte change, le meilleur candidat change, le drone
part dans une direction puis fait demi-tour. C'est un comportement observé sur toutes les
implémentations naïves et il gaspille énormément d'énergie.

Trois mécanismes, dans cet ordre :

**Hystérésis.** Le nouvel objectif doit dépasser le score courant réévalué de 15 % pour
provoquer un changement. En dessous, on conserve le cap.

**Engagement minimal.** Une fois un objectif choisi, il est conservé au moins 3 s ou
jusqu'à 5 m de la cible — sauf veto de sécurité, qui outrepasse toujours.

**Bonus de continuité.** Un candidat aligné avec la direction de vol courante reçoit un
petit bonus. Cela produit des trajectoires en balayage plutôt qu'en zigzag, ce qui est à la
fois plus efficace énergétiquement et plus lisible pour un opérateur qui regarde le
dashboard.

## 6. Exécutif de mission (BehaviorTree)

L'arbre de comportement orchestre le tout. Il est chargé depuis un XML, donc modifiable
sans recompilation, et inspectable en direct avec Groot.

```
ReactiveFallback  "Mission"
├── Sequence "Sécurité d'abord"                   ← évalué en PREMIER à chaque tick
│   ├── Condition: SafetyStateRequiresAction
│   └── SubTree: HandleSafetyState                (RTH / atterrissage / stationnaire)
│
└── Sequence "Mission nominale"
    ├── Action: PreflightChecks
    ├── Action: Arm
    ├── Action: Takeoff(altitude)
    ├── ReactiveSequence "Boucle d'exploration"
    │   ├── Condition: BatterySufficientForExploration
    │   ├── Condition: FrontiersRemaining
    │   ├── Condition: CoverageBelowTarget
    │   └── Fallback
    │       ├── Action: SelectAndReachNextViewpoint
    │       └── Action: HoverAndRetry(max 3)      ← échec de planification n'est pas fatal
    ├── Action: ReturnHome
    ├── Action: FindSafeLandingSite
    └── Action: Land
```

Le point structurant : `ReactiveFallback` réévalue la branche « Sécurité d'abord » **à
chaque tick**, pas seulement à l'entrée. Une condition de sécurité qui devient vraie
interrompt donc immédiatement la mission en cours, quel que soit son avancement. La
sécurité n'attend pas la fin d'une action.

## 7. Où l'IA est pertinente ici — et où elle ne l'est pas

Le brief demande explicitement d'éviter de « mettre du machine learning partout
inutilement ». Analyse composant par composant.

| Composant | IA ? | Raison |
|---|---|---|
| Détection de frontières | **Non** | Algorithme exact, déterministe, rapide. Un modèle appris ferait la même chose, moins bien, sans garantie. Il n'y a rien à apprendre : la définition est exacte. |
| Échantillonnage de points de vue | **Non** | Géométrie pure. Contrainte par le FOV et la sécurité. |
| Calcul du gain d'information | **Non en V1, peut-être en P5** | Le lancer de rayons est exact mais coûteux. Un réseau prédisant le gain à partir d'un voisinage de carte serait 10 à 50× plus rapide. **C'est le seul endroit du pipeline d'exploration où l'IA a un argument solide** — accélération d'un calcul coûteux dont l'approximation est sans danger (un mauvais gain estimé donne une exploration sous-optimale, pas un crash). Conditionné à une mesure : si le lancer de rayons ne dépasse pas son budget temps, l'IA n'apporte rien. |
| Estimation de l'intérêt sémantique d'une zone | **Oui** | « Cette zone contient-elle probablement quelque chose d'intéressant ? » n'a pas de définition algorithmique. C'est appris ou codé en dur ; appris est meilleur. Alimente `IG` par une pondération sémantique. |
| Risque sémantique | **Oui** | Reconnaître eau, foule, lignes électriques depuis l'image relève de la perception. Voir [08](08-ia.md). |
| Scoring et sélection | **Non** | Somme pondérée transparente. Décision explicable, réglable, testable. Un réseau qui sortirait un score serait un système dont on ne peut pas expliquer pourquoi il a envoyé le drone quelque part — inacceptable pour un composant de décision de mission. |
| Planification de chemin | **Non** | A* est optimal sur la grille et déterministe. Aucun apprentissage ne fait mieux, et un planificateur appris peut produire des chemins traversant des obstacles. |
| Politique d'exploration globale (RL) | **Non, et probablement jamais** | Voir ci-dessous. |

### Sur le reinforcement learning

Franchise nécessaire : un RL pour la politique d'exploration serait intellectuellement
séduisant et pratiquement mauvais, pour quatre raisons cumulatives.

1. **Le gain potentiel est faible.** Le pipeline frontières + NBV + scoring est proche de
   l'optimal pour la couverture. La littérature montre des gains de RL de l'ordre de 5 à
   15 % sur des environnements proches de l'entraînement — et des dégradations sur les
   autres.
2. **Le coût est élevé.** Millions d'épisodes, donc simulation massivement parallèle, donc
   Isaac Lab, donc CUDA, donc matériel non disponible et changement de simulateur.
3. **La généralisation est mauvaise** précisément là où on en a besoin : une zone inconnue
   ressemble par définition peu au jeu d'entraînement.
4. **La non-explicabilité est disqualifiante** pour un composant qui décide où vole un
   aéronef de 2,5 kg. On ne peut ni auditer la décision, ni garantir un comportement, ni
   expliquer un incident.

Position retenue : le RL reste un **sujet de recherche de phase P9**, en comparaison
mesurée contre la ligne de base classique, et il ne remplacera le pipeline que s'il
démontre un gain net supérieur à 20 % sur des environnements **hors distribution
d'entraînement**. Cette barre est haute délibérément.

## 8. Métriques d'exploration

Mesurées à chaque test L3, suivies dans le temps, affichées au dashboard :

| Métrique | Définition | Cible P4 |
|---|---|---|
| Couverture | volume observé / volume explorable | ≥ 90 % |
| Efficacité de couverture | m³ observés / Wh consommé | maximiser, suivi de tendance |
| Efficacité de trajet | distance parcourue / distance minimale théorique | < 2,0 |
| Temps de décision | latence du cycle `nbv_selector` | < 500 ms (p95) |
| Taux d'oscillation | changements d'objectif par minute | < 4 |
| Frontières mortes | frontières poursuivies puis abandonnées | < 5 % |
| Marge de retour | énergie restante à l'atterrissage | 15-25 % (ni gaspillage ni risque) |
