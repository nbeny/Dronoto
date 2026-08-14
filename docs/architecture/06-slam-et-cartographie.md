# 06 — Localisation, SLAM et cartographie 3D

## 1. La chaîne complète

```
LiDAR (10 Hz, ~20 000 pts)      IMU (250 Hz)        GNSS (5 Hz)
     │                              │                    │
     └──────────┬───────────────────┘                    │
                ▼                                        │
        ┌───────────────────┐                            │
        │   FAST-LIO2       │  déskew + iEKF + iKD-Tree  │
        │   ODOMÉTRIE       │                            │
        └───────┬───────────┘                            │
                │ pose @10 Hz (continue, dérive lente)   │
                │ + covariance + santé                   │
        ┌───────┴────────────────────┬───────────────────┼──────────────┐
        ▼                            ▼                   ▼              ▼
   TF odom→base_link      nuage recalé dans map    ┌──────────────┐  PX4 EKF2
   (continu, 10 Hz)              │                 │ pose_graph   │  (odom. externe
                                 │                 │   GTSAM      │   30-50 Hz)
                                 ▼                 │ + boucles    │
                        ┌─────────────────┐        │ + facteurs   │
                        │    OctoMap      │        │   GNSS       │
                        │ libre/occupé/   │        └──────┬───────┘
                        │    INCONNU      │               │
                        └────┬───────┬────┘        TF map→odom (lent, lissé)
                             │       │
              ┌──────────────┘       └──────────────┐
              ▼                                     ▼
      ┌───────────────┐                    ┌─────────────────┐
      │ esdf_builder  │                    │frontier_detector│
      │ distance aux  │                    │ libre ∩ voisin  │
      │  obstacles    │                    │   d'inconnu     │
      └───────┬───────┘                    └────────┬────────┘
              ▼                                     ▼
        PLANIFICATEUR                          EXPLORATION
```

## 2. Pourquoi FAST-LIO2

Le raisonnement complet est dans [ADR-0004](../adr/0004-slam.md) ; les trois arguments
décisifs :

**Le coût de calcul est le critère dominant, pas la précision.** Toutes les méthodes
LiDAR-inertielles modernes atteignent une précision du même ordre sur des trajectoires
courtes. Ce qui les sépare en pratique embarquée, c'est la charge CPU : sur le Jetson Orin
NX cible, FAST-LIO2 doit cohabiter avec l'insertion OctoMap, l'ESDF, la planification et
l'inférence ONNX. Son arbre k-d incrémental (pas de reconstruction par scan) et son filtre
formulé pour éviter l'inversion d'une matrice de dimension du nombre de points lui donnent
un avantage structurel, pas marginal.

**La méthode directe résiste aux environnements non structurés.** Les méthodes à features
(LOAM, LIO-SAM) extraient arêtes et surfaces planes. Dans une forêt, sur un terrain
accidenté, dans des ruines, ces primitives sont rares ou instables et la méthode se
dégrade. FAST-LIO2 apparie les points bruts contre la carte locale — moins élégant,
nettement plus robuste dans le cas d'usage « zone inconnue quelconque ».

**Le mouvement d'un drone est agressif.** Les auteurs démontrent un suivi maintenu pendant
des retournements à 1200 °/s. Cette marge n'est pas gratuite : elle vient de la
compensation de distorsion (déskew) par intégration IMU à chaque point du scan. Pour un
capteur qui balaie pendant 100 ms sur un véhicule qui tourne, ignorer cette distorsion
produit des scans tordus et une carte floue.

**Ce que FAST-LIO2 ne fait pas** : pas de fermeture de boucle, pas d'optimisation globale,
pas de fusion GNSS. C'est une **odométrie**, pas un SLAM complet. On ajoute ces trois
choses au-dessus plutôt que de choisir un système monolithique qui les intègre mal.

## 3. Fusion IMU + LiDAR + GNSS

La question « comment fusionner ces trois capteurs » a une mauvaise réponse évidente et une
bonne réponse moins évidente.

**La mauvaise réponse** : un seul filtre (par exemple `robot_localization`) qui prend IMU,
LiDAR et GNSS et sort une pose. Problèmes : on duplique l'EKF2 de PX4 qui fait déjà ce
travail mieux et à plus haute fréquence ; on crée deux estimations d'état concurrentes,
donc la question insoluble de savoir laquelle le contrôleur doit croire ; on doit gérer
soi-même les sauts GNSS, les latences et les repères.

**La bonne réponse : trois fusions à trois échelles de temps différentes, chacune à sa
place.**

| Niveau | Où | Fréquence | Fusionne | Produit | Nature |
|---|---|---|---|---|---|
| **Serré** | FAST-LIO2 (iEKF) | 10 Hz sur scans, propagé à 250 Hz par IMU | LiDAR + IMU | Odométrie locale continue | Couplage serré, déskew |
| **Vol** | EKF2 dans PX4 | 250 Hz+ | IMU + GNSS + baro + mag + odométrie externe | État du contrôleur | Sécuritaire, éprouvé, gère les reprises |
| **Global** | `pose_graph` (GTSAM) | ~1 Hz | Poses clés + fermetures de boucle + facteurs GNSS | Correction `map → odom` | Lent, cohérence globale |

Chaque niveau a une responsabilité claire et aucun ne duplique un autre. Le contrôleur
n'écoute qu'EKF2. La carte n'écoute que la chaîne SLAM. Il n'y a jamais d'ambiguïté sur
l'estimation faisant autorité.

### Le GNSS comme facteur souple, jamais comme réinitialisation

Dans le graphe de poses, le GNSS entre comme un **facteur avec une covariance**, aux côtés
des contraintes d'odométrie et de fermeture de boucle. L'optimiseur arbitre. Deux
protections supplémentaires :

- **Noyau robuste** (Huber ou Cauchy) sur les facteurs GNSS : une mesure aberrante — le cas
  du multi-trajets — voit son influence bornée au lieu de tirer toute la trajectoire.
- **Rejet par test du χ²** avant insertion : une mesure trop incohérente avec l'estimation
  courante est écartée et journalisée comme événement.

Ce qu'on ne fait **jamais** : recaler la position sur le GNSS à réception d'un fix. Cela
produit un saut de position, donc un saut de la commande, donc une embardée physique. La
correction passe exclusivement par `map → odom`, appliquée par interpolation sur ~2 s.

## 4. Gestion de la dérive

FAST-LIO2 dérive. Toute odométrie dérive. La question n'est pas de l'éviter mais de la
borner et de la mesurer. Quatre mécanismes, du plus fréquent au plus rare :

**1. Contraintes de carte locale (continu).** L'appariement se fait contre la carte locale
accumulée, pas contre le scan précédent. La dérive croît donc en racine de la distance,
pas linéairement — c'est déjà un gain d'un ordre de grandeur sur une odométrie
scan-à-scan.

**2. Ancrage GNSS (quand disponible).** Facteurs souples dans le graphe. En extérieur avec
un bon fix, la dérive absolue reste bornée par la précision GNSS. C'est le mécanisme
dominant en pratique pour ce cas d'usage.

**3. Fermeture de boucle (opportuniste).** Détection en deux temps : candidats par
proximité spatiale (poses anciennes proches de la position courante mais éloignées dans le
temps), vérification par recalage ICP avec un seuil de score strict. Une fermeture acceptée
déclenche une optimisation globale.

Le compromis assumé : **on préfère manquer une fermeture de boucle qu'en accepter une
fausse.** Une fausse fermeture replie la carte sur elle-même et est catastrophique — bien
pire que la dérive qu'elle prétendait corriger. Les seuils sont donc conservateurs.

**4. Détection de dégénérescence (protection).** Dans un couloir sans structure ou face à
un mur uniforme, le problème d'appariement est mal conditionné : la contrainte le long
d'un axe est faible ou nulle. On détecte cette situation par la valeur propre minimale de
la matrice d'information et on réagit :

- augmenter la covariance dans la direction dégénérée (EKF2 pondère moins) ;
- signaler `NAV_QUALITY = POOR` au superviseur ;
- influencer l'exploration : le scoring pénalise les zones dégénérées, donc le drone
  préfère spontanément des zones riches en structure. **C'est un exemple où la
  localisation et la décision d'exploration sont couplées** — et où ce couplage produit un
  comportement intelligent sans IA.

### Budget de dérive (critère de sortie de P2)

| Condition | Erreur de trajectoire absolue (ATE) | Mesure |
|---|---|---|
| GNSS nominal, trajectoire 500 m | < 0,5 m RMS | vs `sim/ground_truth` |
| GNSS perdu, trajectoire 200 m | < 2,0 m RMS | idem |
| GNSS perdu, trajectoire 500 m | < 8,0 m RMS | idem |
| Intérieur, boucle 100 m avec fermeture | < 1,0 m après fermeture | idem |

Ces chiffres sont mesurés automatiquement à chaque exécution de test L3 et suivis dans le
temps. Une régression est visible immédiatement. Si le budget n'est pas tenu en P2, c'est
le signal de bascule vers GLIM prévu dans [ADR-0004](../adr/0004-slam.md).

## 5. Construction de la carte 3D

### Pourquoi l'inconnu est la valeur qui compte

Un OctoMap classe chaque voxel en trois catégories, et c'est la troisième qui rend
l'exploration autonome possible :

| État | Probabilité log-odds | Signification |
|---|---|---|
| **Libre** | < seuil bas | Traversé par au moins un rayon sans retour |
| **Occupé** | > seuil haut | Point de retour LiDAR observé |
| **Inconnu** | jamais observé | **Aucun rayon n'y est passé** |

Une carte qui ne distingue que libre et occupé confond « je sais que c'est vide » et « je
n'ai pas regardé ». C'est exactement la distinction dont l'exploration a besoin : la
frontière entre le connu et l'inconnu **est** la cible de l'exploration. Sans troisième
état, il faut inventer un mécanisme de suivi de couverture parallèle et fragile.

### Insertion

À chaque nuage recalé (10 Hz) :

1. **Sous-échantillonnage** en grille voxel à la résolution de la carte — plusieurs points
   dans le même voxel n'apportent rien et coûtent cher.
2. **Filtrage de portée** : les points au-delà de 35 m sont écartés (bruit dominant à la
   portée max du Mid-360 sur cible peu réfléchissante).
3. **Lancer de rayons** de l'origine du capteur vers chaque point : les voxels traversés
   passent en libre, le voxel terminal en occupé.
4. **Mise à jour log-odds** avec bornes de saturation (permet à la carte d'oublier : un
   voxel saturé ne pourrait plus jamais changer d'état, ce qui casse la gestion des
   obstacles mobiles).

Paramètres : résolution 0,25 m ; `prob_hit` 0,7 ; `prob_miss` 0,4 ; seuils de clamp
0,12 / 0,97.

Le lancer de rayons est l'opération coûteuse. Optimisation retenue : sous-échantillonner
les rayons de l'espace **libre** (un rayon sur trois) tout en insérant **tous** les points
occupés. L'espace libre est massivement redondant ; les obstacles ne le sont pas. Gain
mesuré attendu : facteur 2 à 2,5 sur le temps d'insertion sans dégradation exploitable.

### Bornes mémoire

Un octree croît. Trois mécanismes de contrôle :

- **Élagage** : les nœuds dont les huit enfants sont dans le même état fusionnent. Natif à
  OctoMap, très efficace sur les grands volumes libres.
- **Fenêtre glissante** : au-delà d'un rayon configurable (300 m par défaut) autour de la
  position courante, les régions sont sérialisées sur disque et retirées de la mémoire
  vive, rechargeables si le drone revient.
- **Résolution adaptative** (P4) : 0,25 m près du drone, 0,5 m au-delà de 50 m. La
  précision fine n'est utile que là où on planifie.

### Couche sémantique

Séparée de l'occupation, alignée sur la même grille. Chaque voxel peut porter des étiquettes
issues de la perception IA (`bâtiment`, `végétation`, `sol`, `eau`, `véhicule`, `personne`)
avec une confiance. **Volontairement découplée** : le planificateur fonctionne sans elle
(l'IA peut échouer ou être désactivée sans casser la navigation), et elle enrichit le
scoring d'exploration et la sélection de site d'atterrissage quand elle est disponible.
C'est l'application du principe « l'IA améliore, ne conditionne pas ».

## 6. Détection des zones inconnues

L'algorithme est simple, et c'est une qualité — il est déterministe, rapide et exhaustif.

```
Pour chaque voxel v de l'octree :
    si v est LIBRE :
        si un voisin (26-connexité) de v est INCONNU :
            v est un voxel-frontière

Regrouper les voxels-frontières par union-find sur la 26-connexité
Écarter les clusters de moins de N voxels (N = 8 par défaut)
Pour chaque cluster : calculer centroïde, normale moyenne, taille, volume inconnu adjacent
```

Trois raffinements qui font la différence entre un algorithme correct et un algorithme
utilisable :

**Frontières atteignables uniquement.** Un voxel-frontière situé à l'intérieur d'un
bâtiment fermé, ou hors du polygone de mission, ou sous l'altitude minimale, est une
frontière **inatteignable**. La filtrer par un test d'accessibilité (propagation dans
l'espace libre depuis la position courante) évite au drone de poursuivre des objectifs
impossibles — un mode de blocage classique des implémentations naïves d'exploration par
frontières.

**Mise à jour incrémentale.** Recalculer toutes les frontières à chaque insertion est
prohibitif sur une grande carte. On ne réévalue que le voisinage des voxels modifiés
depuis la dernière passe, en maintenant l'ensemble des frontières entre deux passes. Coût
proportionnel au changement, pas à la taille de la carte.

**Frontière « de bord » vs frontière « d'occlusion ».** Distinction subtile mais utile :
une frontière au bord du volume observé (le LiDAR n'a simplement pas porté assez loin) se
résout en avançant ; une frontière derrière un obstacle (occlusion) demande de contourner.
Les deux appellent des points de vue différents ; on les étiquette pour que
`viewpoint_sampler` échantillonne intelligemment.

### Complétude de la couverture

Une mission « cartographier cette zone » a besoin d'un critère d'arrêt. Il combine :

```
couverture = volume_observé_dans_la_zone / volume_total_de_la_zone
```

où `volume_total` exclut l'intérieur des obstacles connus. La mission s'achève quand
`couverture ≥ seuil` (90 % par défaut) **ou** qu'il n'existe plus de frontière atteignable
**ou** que la contrainte batterie l'impose. Les trois conditions sont réelles ; la
deuxième est celle qui se produit le plus souvent en pratique.

## 7. Persistance et export

| Artefact | Format | Usage |
|---|---|---|
| Carte d'occupation | `.bt` (OctoMap binaire) | Reprise, rechargement, transfert |
| Carte complète | `.ot` (OctoMap avec données) | Analyse, couche sémantique |
| Nuage de points | `.pcd` / `.las` | Post-traitement Open3D, livrable client |
| Trajectoire | TUM / CSV | Analyse, comparaison ATE |
| Maillage | `.ply` | Reconstruction Poisson hors ligne (Open3D) |
| Résumé de mission | JSON | Serveur, dashboard, base |

**Transmission par radio.** La carte complète ne passe évidemment pas dans 50 kbps. Ce qui
est transmis :

1. **Deltas d'occupation** (`map/occupancy_delta`) : seuls les voxels ayant changé d'état,
   encodés en RLE sur clés de Morton. Priorité basse, transmis quand la bande passante le
   permet.
2. **Résumé de couverture** (`exploration/coverage`) : quelques dizaines d'octets, priorité
   haute — l'opérateur voit la progression même à débit très dégradé.
3. **Carte complète** : jamais par radio. Récupérée au sol après atterrissage, ou par un
   lien haut débit si présent.

Cette hiérarchie est le principe de dégradation gracieuse appliqué à la donnée : à débit
nul on perd tout, à débit faible on garde l'essentiel, à débit correct on a la carte
approximative, et la carte exacte arrive au retour.
