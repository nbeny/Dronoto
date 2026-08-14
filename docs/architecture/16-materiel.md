# 16 — Matériel futur

> **Prix indicatifs (août 2026), hors taxes et port.** Les tarifs de l'électronique
> embarquée bougent vite et varient fortement selon les revendeurs. Ils servent à
> dimensionner un budget, pas à passer commande.

## 1. Principe de sélection

L'objectif est **pro/robuste avec un rapport coût/performance raisonnable**. Trois règles
ont guidé les choix :

1. **Privilégier ce qui est le mieux supporté logiciellement**, pas ce qui a les meilleures
   spécifications. Un capteur avec un pilote ROS 2 mature et une communauté active vaut
   mieux qu'un capteur supérieur sans intégration — le temps d'ingénierie coûte plus cher
   que le matériel.
2. **Ne pas payer pour de la précision inutilisable.** Un LiDAR à 6 000 € apporte une
   densité que le SLAM et l'OctoMap à 0,25 m n'exploitent pas.
3. **Cohérence avec la simulation.** Chaque composant a son équivalent modélisé dans
   Gazebo ([03](03-simulation.md)). C'est ce qui rend la validation en simulation
   pertinente.

## 2. Nomenclature

| Poste | Retenu | Prix indicatif | Justification |
|---|---|---|---|
| **Contrôleur de vol** | Holybro Pixhawk 6X (kit standard) | 320-400 € | Plateforme de référence PX4. STM32H7 double cœur, **IMU triple redondante isolée en température**, alimentation redondante, port Ethernet. La redondance IMU n'est pas un luxe : c'est ce qui permet à PX4 de survivre à une défaillance capteur en vol. |
| **Calculateur embarqué** | Jetson Orin NX 16 Go + carte porteuse | 650-900 € (module) + 200-300 € (porteuse) | 16 Go nécessaires pour SLAM + OctoMap + ESDF + inférence simultanés. TensorRT est le fournisseur ONNX Runtime cible ([08](08-ia.md)). Enveloppe 10-40 W configurable. |
| **LiDAR 3D** | Livox Mid-360 | ~700 € | **Le meilleur rapport capacité/prix du marché pour un drone.** 360° × 59°, 40 m à 10 % de réflectivité, 265 g, IP67, SDK ROS 2 officiel. C'est le capteur de référence des quadricoptères de recherche modernes, donc la littérature et les configurations FAST-LIO2 existent déjà. |
| **Caméra** | Luxonis OAK-D Pro W | 400-500 € | RGB global shutter + stéréo + VPU embarqué. Le VPU peut décharger une partie de l'inférence du Jetson. Grand angle adapté à l'inspection. Alternative : Intel RealSense D456 (IP65) si la robustesse prime. |
| **GNSS RTK** | Holybro H-RTK F9P Helical (rover) | 250-320 € | u-blox ZED-F9P bi-bande. Précision centimétrique avec correction. **L'antenne hélicoïdale compte** : bien meilleure réjection des multi-trajets que les patchs, ce qui est directement le problème traité en [05 §6](05-integration-px4.md). |
| **Base RTK ou NTRIP** | H-RTK F9P base, ou abonnement NTRIP | 250 € ou ~0-300 €/an | En France, un service NTRIP (Centipède, réseau gratuit) évite l'achat d'une base. |
| **Modem radio** | RFD 868x (paire) — **bande EU** | 350-450 € la paire | 868 MHz, bande ISM européenne libre. Portée > 40 km en vue directe, débit configurable jusqu'à ~250 kbps, chiffrement AES matériel. ⚠ **Ne pas prendre le RFD 900x** : il opère en 902-928 MHz, bande non autorisée en Europe. |
| **Antenne drone** | Dipôle 868 MHz 2 dBi × 2 (diversité) | 40-60 € | Omnidirectionnelle : l'orientation du drone change en permanence. |
| **Antenne sol** | Yagi 9 dBi ou patch 60° sur trépied | 80-150 € | Directive : c'est elle qui donne la portée. Un suiveur motorisé (+200 €) devient utile au-delà de 5 km. |
| **Cellule** | Holybro X500 V2 (kit ARF) | 300-380 € | Standard de recherche, 500 mm, fibre de carbone, bras pliables, platines de montage prévues. Pièces détachées disponibles — ce qui compte quand on casse un bras. |
| **Motorisation** | Incluse au kit X500 V2 (2216 880 KV + ESC 20 A) | — | Rapport poussée/poids ~2,3:1 à 2,5 kg. Suffisant avec de la marge de manœuvre. |
| **Batterie** | 6S Li-ion 8 000-10 000 mAh (cellules Molicel P42A) | 200-280 € | **Li-ion plutôt que LiPo** : densité énergétique supérieure d'environ 30 %, durée de vie bien meilleure. Contrepartie : courant de décharge plus faible — acceptable pour un vol de cartographie, pas pour de l'acrobatie. Autonomie estimée 28-35 min. |
| **Module de puissance** | Holybro PM03D | 40-60 € | Mesure tension/courant, alimentation redondante. |
| **RC de secours** | ExpressLRS 868/915 MHz TX+RX | 120-180 € | **Liaison indépendante du modem de mission** ([05 §7](05-integration-px4.md)). Autorité humaine ultime. Non négociable pour des essais réels. |
| **Divers** | Câblage, connecteurs, amortisseurs, impression 3D des supports | 150-250 € | Les amortisseurs de l'IMU comptent réellement pour la qualité de l'estimation. |

### Budget

| Configuration | Total indicatif |
|---|---|
| **Complète** (tout ci-dessus) | **3 500 – 4 600 €** |
| **Économique** (Pixhawk 6C, Orin Nano Super 8 Go, GNSS non-RTK) | **2 300 – 2 900 €** |
| **Haut de gamme** (Orin AGX, Ouster OS0-32, Microhard pMDDL2450) | 9 000 – 12 000 € |

La configuration économique est un vrai point d'entrée, avec deux compromis à connaître :
l'Orin Nano 8 Go sera juste en mémoire quand SLAM, carte et IA tourneront ensemble (il
faudra réduire la résolution de carte ou l'inférence), et sans RTK la précision de position
absolue passe de quelques centimètres à 1-2 mètres — ce qui dégrade l'ancrage cartographique
mais ne casse rien.

## 3. Ce qui n'est pas retenu, et pourquoi

| Alternative | Rejet |
|---|---|
| **Raspberry Pi 5 comme calculateur** | Pas d'accélération d'inférence utilisable, mémoire limitée, pas de TensorRT. Suffirait pour le SLAM seul, pas pour SLAM + carte + IA. Économie de ~600 € qui coûte une capacité entière. |
| **Ouster OS0-32 / OS1** | Excellents (~6 000 €), densité supérieure. Le SLAM et l'OctoMap à 0,25 m n'en exploitent pas la moitié. Mauvais emploi de 5 000 €. |
| **LiDAR 2D (RPLidar, LDS)** | 2D. Éliminé par le besoin de carte 3D. |
| **Caméra seule, sans LiDAR** | Le VIO seul est bien moins robuste : sensible à l'éclairage, aux surfaces sans texture, aux mouvements rapides. Le LiDAR est ce qui rend le SLAM fiable en zone inconnue — c'est le composant qui justifie le plus son prix. |
| **4G/5G comme liaison principale** | Exclu par le brief, et à raison : couverture non garantie hors zones habitées, abonnement, latence variable. Utilisable en **complément** via `MultiPathLink` là où le réseau existe. |
| **Microhard pMDDL2450** (~1 200 €/paire) | 20 Mbps, permettrait de streamer la carte en direct. Séduisant mais : 2,4 GHz donc portée bien moindre, consommation supérieure, et le budget de 50 kbps a été démontré suffisant ([09 §3](09-communication.md)). À reconsidérer si le streaming vidéo devient une exigence. |
| **Batterie LiPo** | Moins chère, plus de courant, mais moins d'énergie par kilo et durée de vie nettement inférieure. Pour un drone de cartographie, l'autonomie prime sur le courant de pointe. |
| **Pixhawk sans redondance IMU (6C)** | Retenu seulement dans la configuration économique. La redondance IMU est ce qui distingue une plateforme professionnelle d'une plateforme de loisir. |

## 4. Bilan de masse et d'autonomie

| Élément | Masse |
|---|---|
| Cellule X500 V2 (avec moteurs, ESC, hélices) | 1 100 g |
| Pixhawk 6X + module de puissance + câblage | 180 g |
| Jetson Orin NX + porteuse + dissipateur | 220 g |
| Livox Mid-360 + support | 300 g |
| OAK-D Pro W | 130 g |
| GNSS RTK hélicoïdal + mât | 120 g |
| Modem RFD 868x + antennes | 90 g |
| Récepteur ELRS | 10 g |
| Batterie 6S 10 Ah Li-ion | 1 100 g |
| Visserie, amortisseurs, supports imprimés | 150 g |
| **Total en ordre de vol** | **≈ 3 400 g** |

Note : c'est **plus lourd que les 2,5 kg modélisés en simulation**. Le modèle Gazebo doit
être recalé sur cette valeur avant les essais réels — l'écart change la consommation, la
réserve de retour et les marges de manœuvre. C'est une tâche explicite de la phase P9.

**Autonomie estimée** : consommation en stationnaire ≈ 320 W à 3,4 kg, plus ~30 W
d'avionique et de calcul. Sur 10 Ah × 22,2 V = 222 Wh utilisables à 85 % :
**≈ 32 minutes en stationnaire**, **25-28 minutes en mission** avec déplacements. Réserve de
retour comprise, cela donne environ 18-20 minutes de mission utile.

## 5. Contraintes réglementaires (Europe / France)

À traiter comme des contraintes de conception, pas comme des formalités de fin de projet.

| Contrainte | Implication |
|---|---|
| **Masse > 900 g** | Sort de la sous-catégorie ouverte A1. Vol en **A3** (loin des personnes, 150 m des zones habitées) ou en **catégorie spécifique** avec autorisation. |
| **Identification à distance (Remote ID)** | Obligatoire. Module conforme requis, ou fonction intégrée au contrôleur de vol. |
| **Enregistrement exploitant** | Obligatoire (AlphaTango en France). |
| **Formation télépilote** | A3 : formation en ligne + attestation. Catégorie spécifique : formation renforcée. |
| **BVLOS (hors vue directe)** | **Le point dur.** Une liaison de 20 km implique du vol hors vue directe, qui exige une autorisation en catégorie spécifique avec analyse de risque **SORA**. Ce n'est pas une formalité — c'est un dossier substantiel. |
| **Bande 868 MHz** | Rapport cyclique ≤ 10 %, PIRE ≤ 25-500 mW selon la sous-bande. Le modèle radio simulé applique la limite de rapport cyclique ([09 §4](09-communication.md)). |
| **Assurance** | Responsabilité civile aéronautique obligatoire. |
| **Zones interdites** | Géorepérage à jour (Géoportail). Le géorepérage logiciel du superviseur ([10](10-failsafe.md)) doit être chargé avec les zones réelles. |

**Recommandation pratique** : les premiers essais réels se font en **vue directe, en zone
A3, à quelques centaines de mètres**. La portée de 20 km est une capacité de conception du
système radio, pas un mode opératoire à viser d'emblée. Concevoir pour 20 km et voler à
300 m est le bon ordre ; l'inverse ne l'est pas.

## 6. Chemin de transition simulation → matériel

Ordonné du moins risqué au plus risqué. Chaque étape est un point d'arrêt.

| Étape | Contenu | Risque |
|---|---|---|
| **H1 — Banc capteurs** | LiDAR + IMU + GNSS sur une table, alimentés. Vérifier les pilotes, les fréquences, la calibration des bras de levier. **Aucun vol.** | Nul |
| **H2 — SLAM à la main** | Porter l'ensemble capteurs à la main, marcher dans un bâtiment et dehors. Vérifier que FAST-LIO2 converge sur des données réelles, mesurer l'ATE contre un parcours connu. | Nul |
| **H3 — HITL** | Pixhawk réel connecté à Gazebo (hardware-in-the-loop). Valide le firmware, les paramètres et le lien uXRCE-DDS réel. | Faible |
| **H4 — Essais au sol** | Drone assemblé, hélices retirées, moteurs armés. Vérifier armement, modes, failsafes, liaison radio, consommation. | Faible |
| **H5 — Vol manuel** | Vol en mode stabilisé sous contrôle du pilote. Valider la mécanique, les vibrations, le réglage PX4, l'autonomie réelle. | Moyen |
| **H6 — Vol assisté** | Position hold, RTL PX4 natif. Valider EKF2 avec les capteurs réels et l'odométrie externe. | Moyen |
| **H7 — Offboard simple** | Waypoints via notre stack, **pilote prêt à reprendre la main à tout instant**. | Moyen-élevé |
| **H8 — Exploration autonome contrainte** | Petite zone, faible altitude, vue directe, pilote en veille. | Élevé |
| **H9 — Mission complète** | Zone réelle, mission complète, portée progressive. | Élevé |

Les étapes H1 et H2 sont réalisables **avant même d'avoir un drone** — il suffit d'acheter
le LiDAR, une IMU et le Jetson, et de valider la chaîne de perception à la main. C'est le
moyen le moins cher et le moins risqué de dérisquer la partie la plus incertaine du
projet : elles peuvent démarrer en parallèle de la phase P4, bien avant P9.
