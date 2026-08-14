# ADR-0002 — Ubuntu natif ; pas d'achat de GPU NVIDIA

- **Statut** : accepté
- **Date** : 2026-08-14
- **Contexte** : le développeur propose de passer son PC sous Linux et/ou d'acheter un GPU
  NVIDIA d'entrée de gamme

## Décision

1. **Passer en Ubuntu 24.04 LTS natif (dual-boot).** Recommandé, coût nul, gain immédiat.
2. **Ne pas acheter de GPU NVIDIA maintenant.**
3. Si un achat matériel doit être fait, **c'est 32 Go de RAM supplémentaires (~110 €)**,
   pas un GPU.

## Matériel constaté

```
CPU   AMD Ryzen 9 5950X — 16 cœurs / 32 threads
RAM   32 Go
GPU   AMD Radeon RX 6900 XT — 16 Go de VRAM
Disque 7 To libres sur D:
```

## Raisonnement

### Cette charge de travail est limitée par le CPU

C'est le point contre-intuitif qui gouverne la décision. Composant par composant :

| Composant | Ressource dominante |
|---|---|
| Gazebo Sim — physique, pas de simulation, capteurs | **CPU** |
| PX4 SITL | **CPU**, mono-thread intensif |
| FAST-LIO2 (SLAM LiDAR-inertiel) | **CPU**, calcul dense |
| OctoMap — lancer de rayons, insertion | **CPU + mémoire** |
| ESDF, planification A* | **CPU** |
| Campagnes L4 (4 simulations parallèles) | **CPU — 16 cœurs, exactement le bon dimensionnement** |
| Rendu caméra et LiDAR dans Gazebo | GPU, charge légère |
| Inférence ONNX à 2-5 Hz | CPU suffisant ([08 §3](../architecture/08-ia.md)) |

Le Ryzen 9 5950X est donc le composant le mieux dimensionné de la machine pour ce projet.
La RX 6900 XT, avec ses 16 Go de VRAM, dépasse largement les besoins de rendu de Gazebo.

### Ce qu'un GPU NVIDIA débloquerait réellement

| Usage | Phase | Verdict |
|---|---|---|
| Isaac Sim | P7+ | Voir [ADR-0001](0001-simulateur.md) : écarté pour d'autres raisons aussi |
| Entraînement de modèles | P7 | On part de modèles pré-entraînés ; le fine-tuning léger tourne sur CPU ou sur un GPU cloud loué (~0,30 €/h — quelques euros pour tout le projet) |
| nvblox (cartographie GPU) | — | Écarté ; OctoMap suffit |
| Inférence temps réel | P7 | Inutile : la perception tourne à 2-5 Hz par conception |
| RL massivement parallèle | P9+ | Piste de recherche, barre volontairement haute |

**Aucun besoin en P0-P6.** Acheter maintenant, c'est immobiliser 450 € pour un usage
hypothétique dans huit mois — sachant que dans huit mois le rapport prix/performance aura
changé.

### Pourquoi Linux natif plutôt que WSL2

| Aspect | Linux natif | WSL2 |
|---|---|---|
| Rendu Gazebo | Mesa RADV direct | Traduction D3D12 via WSLg, 30-50 % de perte |
| Découverte DDS multicast | Native | Réseau NAT, configuration DDS spécifique, casse le multi-machine |
| ROCm / PyTorch GPU | Fonctionne sur gfx1030 avec `HSA_OVERRIDE_GFX_VERSION=10.3.0` | Non supporté sur RDNA2 |
| Ordonnancement, jitter | Maîtrisé | Couche VM, horloge dérivante |
| Docker | Natif | Docker Desktop, surcouche |
| Accès `/dev/dri`, `/dev/kfd` | Direct | Partiel |

Coût du passage : quelques heures. Bénéfice : permanent, sur toute la durée du projet.

WSL2 reste **documenté et testé en CI** comme mode dégradé : il abaisse la barrière pour un
contributeur sous Windows. Mais les mesures de performance de référence sont prises sous
Linux natif.

### Pourquoi la RAM plutôt que le GPU

La stack complète en fonctionnement — Gazebo, PX4, une dizaine de nœuds ROS 2, FAST-LIO2,
OctoMap, PostgreSQL, le serveur, le dashboard, un navigateur, un IDE — consomme entre 20 et
28 Go. Les campagnes L4 avec 4 simulations parallèles dépassent 32 Go. **La RAM est la
première ressource qui sature**, et c'est ~110 € pour 2 × 32 Go DDR4.

## Conséquences

- Développement sous Ubuntu 24.04 natif ; images Docker Linux natives ; accès GPU AMD par
  `/dev/dri` sans runtime spécial (plus simple que le `nvidia-container-toolkit`).
- ONNX Runtime utilise `CPUExecutionProvider` par défaut, avec `ROCMExecutionProvider`
  comme accélération **opportuniste** — jamais comme dépendance.
- La CI reste valide sur GitHub Actions (runners CPU uniquement), ce qui est cohérent.

## Condition de révision

Acheter un GPU NVIDIA si, en P7 ou P8 :

1. le sim-to-real de la perception visuelle devient bloquant et exige Isaac Sim ; **ou**
2. le RL franchit la barre fixée en [07 §7](../architecture/07-exploration.md) ; **ou**
3. l'entraînement local devient un goulot mesuré (et non supposé).

Point d'entrée recommandé le cas échéant : une carte **16 Go de VRAM** (RTX 5060 Ti 16 Go
ou équivalent, ~450 €). La VRAM prime sur la puissance brute pour l'inférence et
l'entraînement de modèles de vision ; une carte 8 Go serait un faux achat.
