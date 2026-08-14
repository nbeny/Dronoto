# ADR-0006 — Monolithe modulaire plutôt que microservices

- **Statut** : accepté
- **Date** : 2026-08-14

## Décision

**Un seul processus FastAPI** avec des modules à frontières strictes, plus **un processus
séparé** pour la passerelle station sol.

Les frontières logiques proposées dans le brief (mission, drone, telemetry, map, ai,
event bus) sont **conservées comme modules**, pas comme services.

## Contexte

Le brief propose :

```
API Gateway
     ├── Mission Service
     ├── Drone Service
     ├── Telemetry Service
     ├── Map Service
     ├── AI Service
     └── Event Bus
```

et demande explicitement si cette architecture est pertinente ou inutilement complexe pour
une première version.

## Réponse

**C'est la bonne architecture logique et la mauvaise architecture de déploiement.**

Les microservices achètent trois choses :

| Bénéfice | Valeur ici |
|---|---|
| Déploiement indépendant | **Nulle** — un développeur, un déploiement |
| Mise à l'échelle différenciée | **Nulle** — 10 drones, ~10 messages/s ; un seul processus absorbe 1000× cela |
| Isolation des pannes | **Faible** — si le serveur tombe, aucun service n'a de sens sans les autres |

Ils coûtent :

- sept unités à construire, versionner, déployer et surveiller ;
- un courtier de messages à opérer (Redis, RabbitMQ ou Kafka) ;
- la cohérence distribuée à gérer (transactions réparties, ordre des événements) ;
- du traçage distribué à installer pour pouvoir déboguer une seule requête ;
- de la latence réseau sur des appels qui seraient des appels de fonction.

Mauvais échange, sans ambiguïté.

**Le point souvent oublié** : la difficulté des microservices n'est pas de les créer, c'est
de définir correctement les frontières. Cette difficulté-là existe aussi dans un monolithe
modulaire — et on la traite. Ce qu'on évite, c'est la complexité opérationnelle qui vient
par-dessus.

## La règle qui rend l'extraction possible

> **Aucun module n'importe les modèles internes d'un autre module ni ne requête ses
> tables.** La communication passe par des appels de service typés et par des événements
> publiés sur le bus.

Cette règle est **vérifiée mécaniquement** par `import-linter` en CI. Sans vérification
automatique, elle sera violée dans les trois semaines — ce n'est pas de la méfiance, c'est
l'expérience de tout projet ayant tenté la discipline par convention.

Le jour où `telemetry` doit devenir un service séparé (par exemple à 500 drones), il suffit
de remplacer l'appel de fonction par un appel réseau et `InMemoryEventBus` par
`RedisEventBus`. Le reste ne bouge pas.

## Pourquoi la station sol est un processus séparé

Seule exception au monolithe, et pour trois raisons concrètes :

1. **Elle possède du matériel** (port série, modem radio). Un processus qui tient un
   descripteur de périphérique ne doit pas redémarrer à chaque déploiement d'API.
2. **Elle a des contraintes temporelles** que le serveur applicatif n'a pas : lecture du
   port série sans perte, gestion des délais de réémission.
3. **Elle doit survivre à l'indisponibilité du serveur.** Si le serveur redémarre, la
   passerelle continue de recevoir la télémétrie et la met en tampon. Le drone ne voit pas
   la différence.

C'est une frontière justifiée par les contraintes physiques, pas par le découpage
fonctionnel. C'est la bonne raison de séparer un processus.

## Pourquoi Python/FastAPI

Cohérence avec le reste de la stack : ROS 2 Python, PyTorch, outillage de simulation, et
surtout la bibliothèque `drone/communication/` réutilisée **telle quelle** par la
passerelle sol. Une seule implémentation du protocole aux deux extrémités élimine par
construction toute divergence entre l'émetteur et le récepteur.

FastAPI apporte l'async natif (indispensable pour les WebSockets de télémétrie), la
validation Pydantic, et l'OpenAPI généré qui produit à son tour le client TypeScript du
dashboard — donc le typage bout en bout du drone au navigateur.

**Alternatives écartées** : NestJS/TypeScript (excellent, mais sépare le backend de
l'écosystème Python de l'IA et de la robotique, et imposerait de réimplémenter le protocole
DLP une seconde fois) ; Go (très performant, mais aucune intégration naturelle avec
PyTorch, ONNX ou ROS 2, et le débit requis ne justifie pas ce compromis).

## Base de données

**PostgreSQL avec TimescaleDB et PostGIS.** Une seule base pour trois rôles : relationnel
(missions, drones), séries temporelles (télémétrie — hypertables, compression, agrégats
continus) et géospatial (zones, trajectoires, géorepérage).

Écartés : InfluxDB (une base de plus à opérer pour une capacité que TimescaleDB fournit
déjà) ; MongoDB (pas de bénéfice sur des données fortement relationnelles et géographiques).

## Conséquences

**Positives** : un processus à lancer et déboguer ; transactions locales ; latence
inter-modules nulle ; frontières préservées pour l'avenir ; typage bout en bout.

**Négatives** : mise à l'échelle horizontale impossible sans extraction préalable (accepté —
l'échelle ne le demande pas) ; le respect des frontières dépend d'un outil de vérification
(accepté — l'outil est en CI).

## Condition de révision

Extraire un module en service séparé si l'un de ces seuils est franchi :

1. **> 100 drones simultanés** et la télémétrie devient le goulot mesuré ;
2. plusieurs équipes travaillent sur des modules distincts avec des rythmes de livraison
   incompatibles ;
3. un module a un profil de ressources radicalement différent (typiquement `ai`, s'il
   nécessitait un GPU dédié).

Aucun de ces seuils n'est proche.
