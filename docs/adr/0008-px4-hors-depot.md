# ADR-0008 — PX4-Autopilot cloné hors du dépôt, pas en sous-module

- **Statut** : accepté
- **Date** : 2026-08-16
- **Contexte** : découvert pendant l'exécution de P0 ; remplace la décision implicite du
  plan initial d'utiliser un sous-module git

## Décision

PX4-Autopilot est **cloné hors du dépôt** par `infrastructure/scripts/install_px4.sh`,
à l'emplacement `$DRONOTO_PX4_DIR` (défaut `~/px4/PX4-Autopilot`).

La version reste verrouillée, dans `simulation/px4/PX4_VERSION`. Le script refuse de
continuer si la version locale diffère.

## Contexte

Le plan d'implémentation prévoyait `git submodule add` dans
`simulation/px4/PX4-Autopilot`. Deux mesures faites sur l'environnement réel l'ont
invalidé.

**Mesure 1 — coût d'E/S sur `/mnt/c`.** Le dépôt vit sur un disque Windows, monté dans
WSL via 9p. Sur 300 fichiers :

| | Écriture | Lecture |
|---|---|---|
| Système de fichiers natif WSL | 298 ms | 15 ms |
| `/mnt/c` (9p) | 761 ms | 758 ms |

**Les lectures sont ~50× plus lentes.** Pour nos paquets (quelques dizaines de fichiers),
c'est négligeable en absolu. Pour PX4 — plus de 30 000 fichiers, dont chaque compilation
relit des milliers d'en-têtes — c'est rédhibitoire.

**Mesure 2 — volume.** Le clone récursif de PX4 dépasse 1 Go et 30 000 fichiers. Les
placer sous `git status` alourdit chaque opération git du dépôt, y compris sur une machine
Linux native.

## Raisonnement

Un sous-module git est justifié quand on modifie le code amont ou qu'on a besoin que la
version soit atomiquement liée à un commit du dépôt. **Aucun des deux ne s'applique ici** :

- On ne modifie pas PX4. Nos personnalisations (airframe, modèle Gazebo) vivent dans
  `simulation/px4/{airframes,models}` — dans **notre** dépôt — et sont liées par lien
  symbolique dans l'arborescence PX4 par `install_px4_overlays.sh`. C'est déjà la
  conception retenue, et elle rend le sous-module inutile.
- Le verrouillage de version, qui est le vrai besoin, est assuré tout aussi bien par un
  fichier de version que le script vérifie. La propriété qui compte — « on ne dérive pas
  de version par accident » — est préservée.

Ce qu'on perd : `git clone --recursive` ne suffit plus, il faut lancer un script. C'est
acceptable, d'autant qu'un script était de toute façon nécessaire pour installer les
dépendances PX4, poser les surcouches et compiler SITL.

## Conséquences

**Positives** : les opérations git du dépôt restent rapides ; sur WSL, la compilation de
PX4 se fait sur le système de fichiers natif ; l'emplacement de PX4 est configurable, donc
plusieurs versions peuvent coexister pour comparer ; le dépôt reste petit.

**Négatives** : une étape d'installation explicite est nécessaire (`install_px4.sh`) ; le
lien entre un commit du dépôt et une version de PX4 est un fichier texte plutôt qu'un
pointeur git — donc vérifié par script plutôt que garanti par git. Le script étant appelé
par la CI, l'écart serait détecté.

## Condition de révision

Repasser en sous-module si le projet devait un jour **modifier** PX4 (correctif amont,
pilote spécifique). Le besoin de suivre nos propres commits sur PX4 justifierait alors son
coût.
