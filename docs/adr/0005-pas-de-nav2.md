# ADR-0005 — Ne pas utiliser Nav2 pour la navigation 3D

- **Statut** : accepté
- **Date** : 2026-08-14

## Décision

**Nav2 n'est pas utilisé comme pile de navigation.** On implémente une pile à trois niveaux
adaptée au vol libre en `SE(3)`.

**BehaviorTree.CPP est conservé** — c'est la partie de l'écosystème Nav2 réellement
réutilisable.

## Contexte

Le brief demande d'évaluer « Nav2 si pertinent ». Nav2 est la pile de navigation de
référence de ROS 2 : mature, documentée, largement déployée. Le réflexe est de l'adopter.

## Raisonnement

**Nav2 est construit autour d'un modèle qui ne correspond pas au problème.**

| Hypothèse de Nav2 | Réalité d'un quadricoptère |
|---|---|
| Le robot se déplace sur une surface | Il se déplace dans un volume |
| L'environnement est une **grille de coût 2D** | L'environnement est un volume d'occupation 3D |
| L'état est `(x, y, θ)` | L'état est `SE(3)` : position 3D + orientation |
| Les obstacles sont projetés au sol | Il faut passer **au-dessus** et **en dessous** |
| Contraintes non holonomes (différentiel, Ackermann) | Holonome, ou presque |
| L'altitude n'existe pas | L'altitude est une variable de décision majeure |

Les contrôleurs de Nav2 (DWB, MPPI, Regulated Pure Pursuit) raisonnent en vitesses
`(vx, vy, ωz)` sur un plan. Les plugins de costmap sont fondamentalement planaires.

**Conséquence concrète** : utiliser Nav2 impose de projeter la carte 3D en 2,5D. On jette
précisément l'information que le LiDAR et OctoMap viennent de produire. On paierait la
complexité de Nav2 (arbre de comportement, serveurs d'action, système de plugins, cycle de
vie des nœuds gérés) **pour perdre en capacité**. C'est le pire des deux mondes.

Un exemple suffit à l'illustrer : franchir un bâtiment par le dessus, comportement banal
pour un drone d'exploration, est inexprimable dans une costmap 2D. Il faudrait le
contourner ou le coder en dehors de Nav2 — auquel cas Nav2 ne sert plus à rien.

## Ce qui est fait à la place

```
Niveau 3   global_planner        A*/JPS sur voxels OctoMap        ~1 Hz
Niveau 2   trajectory_generator  Polynômes contraints par l'ESDF  ~10 Hz
Niveau 1   reactive_avoidance    Nuage brut, repère capteur       ~30 Hz, veto
```

Détail : [07 — Exploration](../architecture/07-exploration.md) et
[02 §5](../architecture/02-architecture-systeme.md).

Coût estimé de l'implémentation : environ deux semaines (P3). C'est moins que le temps qui
serait passé à contourner les hypothèses 2D de Nav2, et le résultat est adapté au problème.

## Ce qui est conservé de l'écosystème Nav2

**BehaviorTree.CPP** pour l'exécutif de mission. C'est un moteur d'arbres de comportement
mature, performant, introspectable, avec un outil de visualisation (Groot), et
**indépendant de toute hypothèse 2D**. La logique de mission s'y exprime lisiblement et se
modifie sans recompiler.

C'est la bonne granularité de réutilisation : prendre le composant générique, laisser la
pile spécialisée.

## Alternatives examinées

| Alternative | Rejet |
|---|---|
| **Nav2 tel quel** | Modèle 2D incompatible. Voir ci-dessus. |
| **Nav2 avec plugins 3D personnalisés** | Il faudrait remplacer costmap, planificateur et contrôleur — c'est-à-dire tout Nav2 sauf le squelette. On garderait la complexité sans le bénéfice. |
| **MoveIt** | Conçu pour les bras manipulateurs et la planification en espace de configuration articulaire. Inadapté. |
| **OMPL seul** | Bonne bibliothèque de planification par échantillonnage. Non retenu en V1 (non déterministe, plus lent qu'A* sur un volume borné) mais **noté comme évolution** si les volumes deviennent grands et clairsemés. |
| **EGO-Planner / Fast-Planner** | Excellents planificateurs de trajectoire pour quadricoptères, issus de la recherche. Portages ROS 2 partiels, code de recherche difficile à maintenir. **Source d'inspiration pour le niveau 2**, pas dépendance directe. |

## Conséquences

**Positives** : la navigation exploite réellement la 3D ; pile simple et compréhensible en
entier ; le veto réactif est indépendant de la carte et de la localisation, ce que Nav2 ne
permettrait pas structurellement ; pas de dépendance à un système de plugins étranger.

**Négatives** : plus de code à écrire et à maintenir ; on ne bénéficie pas des correctifs
amont de Nav2 ; il faut réimplémenter des choses connues (A*, raccourcissement de chemin).
Accepté : ces algorithmes sont bien documentés et leur implémentation est un travail borné,
contrairement au contournement d'hypothèses architecturales.

## Condition de révision

Réexaminer si Nav2 introduit un jour une costmap 3D de première classe et des contrôleurs
`SE(3)`. Cela impliquerait une refonte de Nav2 lui-même — improbable à court terme, mais
le suivi amont fait partie de la veille du projet.
