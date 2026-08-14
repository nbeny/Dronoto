# ADR-0007 — Protocole propre (DLP) en Protobuf, pas MAVLink

- **Statut** : accepté
- **Date** : 2026-08-14

## Décision

**Un protocole applicatif propre — DLP (Drone Link Protocol) — encodé en Protobuf**, au-dessus
de l'interface `CommunicationLink`.

MAVLink est conservé, mais **uniquement** comme canal parallèle de basse priorité pour
QGroundControl.

## Contexte

Deux questions à trancher : quel protocole applicatif, et quel encodage.

## Question 1 — Pourquoi pas MAVLink comme protocole applicatif

MAVLink est excellent pour ce pour quoi il est conçu : piloter et superviser un véhicule
depuis une station sol standard, avec un vocabulaire commun à tout l'écosystème.

Il est mauvais pour transporter les objets métier de ce projet :

| Objet à transporter | En MAVLink |
|---|---|
| « Cartographier ce polygone entre 10 et 60 m, cible 90 %, politique de perte de liaison CONTINUE » | Aucun message adapté. Il faudrait détourner `MISSION_ITEM` ou empaqueter du binaire dans `TUNNEL`. |
| Delta d'occupation 3D (voxels modifiés) | Aucun message. `TUNNEL` avec un format maison — donc on réinvente un protocole, mais encapsulé dans un autre. |
| Carte de risque sémantique | Idem. |
| Score des candidats d'exploration | Idem. |
| Politique de failsafe applicative | Idem. |

Forcer ces objets dans MAVLink produit un protocole illisible : des messages `TUNNEL`
contenant un format propriétaire non documenté. On aurait la complexité de MAVLink **plus**
celle d'un protocole maison, sans les bénéfices de l'un ni de l'autre.

Deuxième raison, tout aussi importante : **MAVLink contient des messages de contrôle
temps réel** (`SET_ATTITUDE_TARGET`, `MANUAL_CONTROL`, `SET_ACTUATOR_CONTROL_TARGET`).
L'adopter comme protocole radio rendrait *possible* d'envoyer du contrôle moteur depuis le
sol — précisément ce que la contrainte n°2 du brief interdit. Avec un protocole propre, la
contrainte devient **structurellement invérifiable à violer** : le schéma ne définit aucun
message de ce type, et un test le vérifie
([09 §1](../architecture/09-communication.md)).

Une interdiction garantie par la structure vaut mieux qu'une interdiction garantie par la
discipline.

## Question 2 — Pourquoi Protobuf

Le budget est de **50 kbps partagés**. À ce débit, la compacité est une contrainte de
conception, pas une optimisation.

| Encodage | Taille d'une trame télémétrie | Schéma | Codegen multi-langages |
|---|---|---|---|
| JSON | ~300 o | Non | Non |
| MessagePack | ~150 o | Non | Non |
| CBOR | ~140 o | Optionnel (CDDL) | Faible |
| **Protobuf** | **~50 o** | **Oui** | **Oui : Python, TS, C++** |
| Format binaire maison | ~40 o | Manuel | Manuel, à écrire 3 fois |

Protobuf apporte deux choses décisives au-delà de la taille :

**Un schéma unique génère le code des trois langages.** Python (drone et serveur),
TypeScript (dashboard), C++ (nœuds critiques). Il devient structurellement impossible que
les trois divergent. Sans cela, on maintient à la main trois définitions de la même
structure de télémétrie, et elles finissent par différer — c'est une certitude, et le bug
résultant est silencieux et se manifeste en vol.

**L'évolution est gérée.** Les numéros de champ ne sont jamais réutilisés, les champs
supprimés sont marqués `reserved`. Un drone embarquant une ancienne version du protocole
reste compréhensible par un serveur récent — propriété indispensable dès qu'un drone est
sur le terrain et pas trivialement reflashable.

Le format binaire maison serait légèrement plus compact et coûterait trois implémentations
à maintenir, sans gestion de version. Mauvais échange.

## Ce qui est perdu, et comment on le récupère

En n'adoptant pas MAVLink, on perd la compatibilité immédiate avec l'écosystème
(QGroundControl, Mission Planner, MAVSDK).

**On la récupère par un canal parallèle** : le flux MAVLink de PX4 est tunnelé dans un canal
DLP de priorité `BULK`. Un opérateur peut donc connecter QGroundControl pour la
configuration, la calibration, l'analyse de logs et la reprise en main manuelle. Étant en
`BULK`, ce flux ne peut jamais affamer la télémétrie de sécurité.

C'est la bonne répartition : **DLP pour la mission autonome, MAVLink pour l'humain.** Les
deux usages ont des besoins différents et méritent des canaux différents.

## Conséquences

**Positives** : protocole adapté aux objets métier réels ; 50 kbps largement suffisants
(plan de gestion à 2,4 % du budget) ; interdiction du contrôle moteur garantie par la
structure ; typage bout en bout du drone au navigateur ; évolution de schéma maîtrisée.

**Négatives** : un générateur de code dans la chaîne de build (`tools/generate_proto.sh`,
vérifié en CI) ; le protocole doit être documenté et testé — ce que fait
[09](../architecture/09-communication.md), avec une couverture L0 > 90 % ; pas de
compatibilité immédiate avec les stations sol tierces sans passer par le tunnel MAVLink.

## Condition de révision

Aucune condition de révision identifiée. C'est une décision fondatrice : le protocole est au
centre de l'architecture, et en changer impliquerait de reprendre le drone, la station sol,
le serveur et le dashboard.

La seule évolution envisagée est **additive** : ajouter des types de messages au schéma,
ce que Protobuf gère nativement.
