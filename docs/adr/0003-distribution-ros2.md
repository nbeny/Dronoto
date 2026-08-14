# ADR-0003 — ROS 2 Jazzy plutôt que Lyrical Luth

- **Statut** : accepté
- **Date** : 2026-08-14

## Décision

**ROS 2 Jazzy Jalisco sur Ubuntu 24.04**, avec Gazebo Harmonic. RMW par défaut : Fast DDS.

Migration vers Lyrical Luth évaluée en P8.

## Contexte

Deux distributions LTS sont disponibles en août 2026 :

| | Jazzy Jalisco | Lyrical Luth |
|---|---|---|
| Sortie | mai 2024 | **22 mai 2026** (3 mois) |
| Support | mai 2029 | mai 2031 |
| Ubuntu | 24.04 Noble | 26.04 Resolute |
| Gazebo apparié | Harmonic | Jetty |

Le réflexe serait de prendre la plus récente : plus de support restant, plus moderne.

## Raisonnement

**Ce projet dépend d'une dizaine de paquets communautaires, pas seulement du cœur ROS 2.**
La liste : portage ROS 2 de FAST-LIO2, `octomap_server2`, `ros_gz`, `BehaviorTree.CPP` +
`btcpp_ros2_interfaces`, `px4_msgs` et `px4_ros2_cpp`, `vision_msgs`, GTSAM, les bindings
ONNX Runtime.

Ces paquets sont maintenus par des tiers, à des rythmes variables. Trois mois après une
release ROS 2, la plupart ne sont pas encore portés, testés, ni documentés contre elle.
**Choisir la distribution la plus récente, c'est s'attribuer le travail de portage
d'autrui** — un travail qui n'apporte aucune valeur au projet et qui consomme le temps de
ses phases les plus incertaines.

Deuxième argument : **la documentation PX4 référence Jazzy.** Quand un problème
d'intégration surgit — et il en surgira —, l'existence de réponses publiques est ce qui
fait la différence entre deux heures et deux jours.

Troisième argument : **le support de Jazzy dépasse l'horizon du projet.** Mai 2029, soit
près de trois ans. La roadmap prévoit neuf mois de logiciel plus les essais matériels. Le
support supplémentaire de Lyrical achète de la marge dont on n'a pas l'usage aujourd'hui.

## Choix du RMW

**Fast DDS** en défaut : c'est le RMW par défaut de Jazzy, celui contre lequel l'amont teste,
et celui qu'utilise l'agent Micro XRCE-DDS de PX4. Un seul fournisseur DDS dans le graphe
réduit la surface des problèmes de découverte.

**Cyclone DDS** (`rmw_cyclonedds_cpp`) est documenté comme mode de secours, activable par
variable d'environnement. Une partie de la communauté PX4 le préfère pour la stabilité de
découverte. On ne l'adopte pas par anticipation, mais on sait qu'il existe.

## Conséquences

**Positives** : tous les paquets tiers disponibles et testés ; documentation PX4 alignée ;
réponses publiques aux problèmes rencontrés ; Gazebo Harmonic apparié et stable.

**Négatives** : une migration sera nécessaire un jour. Coût contenu si le code respecte les
conventions ROS 2 standard — ce que ce dossier impose (pas d'API interne, pas de
contournement).

## Condition de révision

Réexaminer en P8, quand :

1. les paquets tiers de la liste ci-dessus sont portés et publiés sur Lyrical ; **et**
2. la documentation PX4 la référence ; **et**
3. la base de code est stabilisée (la migration se fait sur du code figé, pas en pleine
   construction).

La migration serait alors une phase dédiée, avec la campagne L4 complète comme critère
d'acceptation.
