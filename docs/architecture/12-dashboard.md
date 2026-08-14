# 12 — Dashboard

## 1. Ce que le dashboard doit résoudre

Un opérateur regarde un écran pendant qu'un aéronef vole. Il doit pouvoir répondre en une
seconde à trois questions : **le drone va-t-il bien ? où en est la mission ? dois-je
intervenir ?**

Tout le reste — la carte 3D, les graphes, les logs — est secondaire. Un dashboard qui
présente vingt informations au même niveau visuel oblige l'opérateur à chercher, et
chercher pendant qu'un drone vole est un problème. La hiérarchie visuelle est donc une
exigence fonctionnelle, pas esthétique.

## 2. Stack

| Couche | Choix | Raison |
|---|---|---|
| Framework | React 19 + TypeScript | Écosystème, typage bout en bout avec le client généré depuis OpenAPI |
| Build | Vite | Rapide, simple, standard |
| Carte et 3D | **MapLibre GL JS + deck.gl** | Un seul moteur WebGL2 pour le géospatial et le volumétrique. Voir [01 §11](01-choix-technologiques.md) |
| Fond de carte | Tuiles OSM auto-hébergées ou fichier MBTiles | **Doit fonctionner hors ligne** — cohérent avec l'absence de dépendance réseau |
| État serveur | TanStack Query | Cache, revalidation, gestion d'erreur |
| État client | Zustand | Léger, suffisant |
| Temps réel | WebSocket natif + reconnexion exponentielle | Pas de bibliothèque nécessaire |
| UI | Tailwind CSS + shadcn/ui | Rapide, cohérent, personnalisable |
| Graphes | Recharts (séries temporelles simples) + deck.gl (spatial) | — |
| Tests | Vitest + Testing Library + Playwright | — |

Le fond de carte hors ligne mérite d'être souligné : une station sol sur le terrain n'a pas
forcément Internet. Un dashboard qui affiche un carré gris sans connexion est inutilisable
là où on en a le plus besoin.

## 3. Disposition

```
┌────────────────────────────────────────────────────────────────────────────────┐
│  Dronoto     [Drone 1 ▾]   ● CONNECTÉ 42 ms   🔋 68 %   NOMINAL      [⏸][🏠][⏹]│ ← bandeau critique
├──────────────┬─────────────────────────────────────────────┬───────────────────┤
│              │                                             │                   │
│  MISSIONS    │            VUE PRINCIPALE                   │   TÉLÉMÉTRIE      │
│              │                                             │                   │
│ ▸ Zone Nord  │   [ Carte 2D ] [ Carte 3D ] [ Fusionnée ]   │  Altitude   42 m  │
│   active     │                                             │  Vitesse   6,2 m/s│
│   68 %       │    ┌──────────────────────────────────┐     │  Cap        127°  │
│              │    │                                  │     │  Batterie    68 % │
│ ▸ Zone Sud   │    │    trajectoire · drone            │     │  14,8 V   12,3 A  │
│   brouillon  │    │    voxels connus (couleur = z)    │     │  Autonomie 18 min │
│              │    │    frontières (bordure animée)    │     │                   │
│ [+ Nouvelle] │    │    obstacles                      │     │  ── NAVIGATION ── │
│              │    │    zone de mission                │     │  GNSS  12 sat 0,8 │
│              │    │    portée radio (dégradé)         │     │  SLAM  bon        │
│              │    │    site d'atterrissage prévu      │     │  Source EKF GPS+EV│
│              │    │                                  │     │                   │
│              │    └──────────────────────────────────┘     │  ── LIAISON ────  │
│              │                                             │  RSSI    −84 dBm  │
│              │   Couverture ███████████░░░░░░░  68 %       │  Perte      2,1 % │
│              │                                             │  Débit   38 kbps  │
├──────────────┴─────────────────────────────────────────────┴───────────────────┤
│  ÉVÉNEMENTS   [Tous ▾] [⚠ 2]                                                   │
│  14:32:07  ⚠  GNSS_DEGRADED     satellites 12→6, HDOP 0,8→2,4                  │
│  14:31:52  ℹ  FRONTIER_REACHED  point de vue atteint, 12 m³ observés           │
│  14:30:15  ⚠  OBSTACLE_AVOIDED  veto réactif, distance min 1,8 m               │
└────────────────────────────────────────────────────────────────────────────────┘
```

**Le bandeau supérieur est la zone critique.** Connexion, batterie, état de sécurité, et
les trois boutons d'action immédiate (pause, retour à la base, arrêt). Toujours visible,
jamais masqué par un défilement, jamais recouvert par une boîte de dialogue. C'est ce qu'un
opérateur regarde quand quelque chose ne va pas.

## 4. Vues cartographiques

### Vue 2D (par défaut)

Fond de carte, vue de dessus. Ce qu'un opérateur consulte 90 % du temps parce que c'est le
plus lisible.

| Couche | Rendu | Notes |
|---|---|---|
| Drone | Icône orientée selon le cap | Grisée et marquée si `stale` |
| Trajectoire parcourue | `PathLayer`, couleur = altitude | — |
| Trajectoire planifiée | `PathLayer` pointillé | — |
| Zone de mission | `PolygonLayer` semi-transparent | — |
| Couverture | `PolygonLayer` (union des cellules observées) | Distingue visuellement exploré / inexploré |
| Frontières | `ScatterplotLayer` animé | Là où le drone ira |
| Obstacles | `PolygonLayer` (projection des voxels occupés) | — |
| Portée radio | `PolygonLayer` en dégradé | Calculé depuis le modèle de propagation |
| Géorepérage | contour rouge | — |
| Objectif courant | marqueur distinct | Rend la décision d'exploration lisible |
| Détections IA | marqueurs par classe | Filtrables |

### Vue 3D

`deck.gl` en projection perspective. Voxels d'occupation en `ColumnLayer` (colorés par
altitude ou par classe sémantique), nuage de points décimé en `PointCloudLayer`, trajectoire
en 3D.

**Décimation obligatoire.** Une carte OctoMap complète contient des millions de voxels ; un
navigateur en affiche confortablement quelques centaines de milliers. Le serveur sert donc
des niveaux de détail (`/maps/{id}/pointcloud?lod=2`) et le client charge par tuiles selon
la caméra. Sans cela, la vue 3D fige l'onglet — et un dashboard figé pendant un vol est
pire qu'un dashboard absent.

### Vue fusionnée

Vue 2D avec relief et voxels en surimpression légère. Compromis pour la supervision
courante quand on veut le contexte 3D sans perdre la lisibilité de la carte.

## 5. Actions opérateur

| Action | Interaction | Confirmation |
|---|---|---|
| Créer une mission | Dessiner un polygone, régler altitude et contraintes | Non |
| Démarrer | Bouton | **Oui** — affiche l'état du drone, la batterie et la faisabilité estimée |
| Pause / Reprise | Bouton | Non (réversible) |
| Annuler | Bouton | **Oui** — précise que le drone rentrera à la base |
| Modifier en vol | Éditer le polygone ou les contraintes, puis appliquer | **Oui** — montre le diff |
| Waypoint immédiat | Clic sur la carte → « Aller ici » | **Oui** — vérifie la faisabilité batterie |
| Retour à la base | Bouton du bandeau | **Oui** |
| Atterrir | Bouton | **Oui** — avertit si le site n'est pas évalué |
| Acquitter une alerte | Clic sur l'événement | Non |
| Régler les poids d'exploration | Panneau avancé, curseurs | Non (mais journalisé) |

### Le statut des commandes est visible

Toute commande envoyée à un drone affiche son cycle de vie :

```
[Envoi…] → [Transmise] → [Acquittée par le drone] → [Appliquée]
                       ↘ [Échec — pas d'acquittement en 30 s]  [Réessayer]
                       ↘ [Rejetée par le drone : batterie insuffisante]
```

C'est une exigence directe de l'architecture : la liaison est non fiable **par conception**,
donc l'interface doit rendre cette incertitude visible plutôt que la masquer. Un bouton qui
s'enfonce et ne dit rien laisse l'opérateur dans le doute exactement quand il ne devrait pas
y être.

## 6. Comportement en cas de perte de liaison

Ce que fait le dashboard quand le drone se tait — un cas nominal, pas exceptionnel :

1. Le bandeau passe à `● HORS LIAISON — dernier contact il y a 00:47`.
2. Toute la télémétrie est visuellement **atténuée** et horodatée « il y a N secondes ».
3. La position du drone est affichée en pointillés, avec un **cône d'incertitude** qui
   s'élargit avec le temps écoulé (basé sur la dernière vitesse connue).
4. Les boutons de commande restent actifs mais avertissent que la commande sera mise en file.
5. Un bandeau rappelle la politique de perte de liaison en vigueur, pour que l'opérateur
   sache ce que le drone est en train de faire : *« Politique : CONTINUE — le drone poursuit
   sa mission de manière autonome. »*

Le point 5 est le plus important : sans lui, l'opérateur ne sait pas si le drone continue,
attend ou rentre, et son inquiétude produit de mauvaises décisions. Afficher la politique
transforme une situation angoissante en situation comprise.

## 7. Rejeu de mission

Vue dédiée pour l'après-vol :

- Curseur temporel sur toute la durée de la mission.
- Rejeu synchronisé : position, télémétrie, construction progressive de la carte, événements.
- Superposition possible de plusieurs missions pour comparer.
- Export vidéo ou GIF pour rapport.

C'est l'outil de débogage le plus efficace du projet — plus utile en pratique que la plupart
des logs. Beaucoup de comportements ne se comprennent qu'en voyant *quand* le drone a décidé
quoi, et pourquoi le score d'un candidat a basculé.

## 8. Performance

| Contrainte | Cible |
|---|---|
| Chargement initial | < 2 s |
| Taux de rafraîchissement de la carte | 30 FPS minimum |
| Latence WebSocket → affichage | < 100 ms |
| Points affichables en 3D | ≥ 500 000 avec LOD |
| Missions simultanées supervisables | ≥ 10 drones sans dégradation |

Le dashboard ne recalcule jamais rien de coûteux : la couverture, la portée radio et les
niveaux de détail sont calculés côté serveur et servis prêts à l'affichage. Le navigateur
affiche, il ne calcule pas — un choix qui garde l'interface fluide même sur une machine
modeste de station sol sur le terrain.
