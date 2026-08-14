# 11 — Serveur

## 1. Réponse à la question posée

Le brief propose une architecture en microservices et demande si elle est pertinente ou
inutilement complexe pour une première version.

**Elle est inutilement complexe comme architecture de déploiement, et pertinente comme
architecture logique.** On garde les frontières, on jette la distribution.

Les microservices achètent trois choses : déploiement indépendant, mise à l'échelle
différenciée, isolation des pannes. Avec un développeur, un serveur et dix drones, ces
trois bénéfices valent zéro. En face, ils coûtent : sept unités à construire, versionner et
déployer, un courtier de messages à opérer, la cohérence distribuée à gérer, le traçage
distribué à installer pour pouvoir déboguer une requête. Mauvais échange.

**Décision : monolithe modulaire.** Un processus FastAPI, des modules à frontières
strictes, un bus d'événements en mémoire derrière une interface. Extraction possible plus
tard, sans réécriture, à condition de respecter une règle unique et non négociable :

> **Aucun module n'importe les modèles internes d'un autre module ni ne requête ses
> tables.** La communication se fait par appels de service typés et par événements.

Cette règle est vérifiée automatiquement (`import-linter` en CI). Sans vérification
mécanique, elle sera violée en trois semaines — ce n'est pas de la méfiance, c'est
l'expérience.

## 2. Structure

```
server/
├── api/                        couche HTTP — routes, schémas, dépendances
│   ├── v1/
│   │   ├── missions.py
│   │   ├── drones.py
│   │   ├── telemetry.py
│   │   ├── maps.py
│   │   ├── events.py
│   │   └── ws.py               WebSocket temps réel
│   └── deps.py
│
├── modules/                    LOGIQUE MÉTIER — frontières strictes
│   ├── missions/               service.py · models.py · repository.py · schemas.py
│   ├── drones/
│   ├── telemetry/
│   ├── maps/
│   ├── events/
│   └── ai/                     analyse hors ligne, planification assistée
│
├── core/
│   ├── bus.py                  interface EventBus (mémoire → Redis sans changer les appelants)
│   ├── db.py
│   ├── config.py
│   ├── security.py
│   └── protocol/               code Protobuf généré (partagé avec le drone)
│
└── groundstation/              PROCESSUS SÉPARÉ
    ├── bridge.py               possède le CommunicationLink côté sol
    ├── decoder.py
    └── ipc.py                  ZeroMQ vers le serveur
```

### Pourquoi la station sol est un processus séparé

Ce n'est pas un détail d'organisation. Trois raisons :

1. **Elle possède du matériel** (port série, modem). Un processus qui tient un descripteur
   de fichier matériel ne doit pas redémarrer quand on déploie une correction d'API.
2. **Elle a des contraintes temporelles** que le serveur applicatif n'a pas : lire le port
   série sans perte, gérer les délais de réémission.
3. **Elle doit survivre à l'indisponibilité du serveur.** Si le serveur redémarre, la
   passerelle continue de recevoir la télémétrie et la met en tampon. Le drone ne voit pas
   la différence — cohérent avec le principe que la disponibilité au sol n'affecte pas le
   drone.

L'IPC entre les deux est ZeroMQ (PUB/SUB pour la télémétrie montante, REQ/REP pour les
commandes descendantes) ou une socket UNIX. Le point important est le découplage, pas la
technologie.

## 3. Modèles de données

### Mission

```python
class MissionType(str, Enum):
    SURVEY_AREA   = "survey_area"      # cartographier un polygone
    WAYPOINT      = "waypoint"         # suivre une liste de points
    INSPECT_POINT = "inspect_point"    # observer une cible sous plusieurs angles
    RETURN_HOME   = "return_home"

class MissionStatus(str, Enum):
    DRAFT = "draft"; QUEUED = "queued"; UPLOADING = "uploading"
    ACTIVE = "active"; PAUSED = "paused"
    COMPLETED = "completed"; FAILED = "failed"; CANCELLED = "cancelled"

class MissionArea(BaseModel):
    polygon:     list[GeoPoint]        # ≥ 3 points, WGS84, non auto-intersectant
    min_alt_m:   float = 10.0          # AGL
    max_alt_m:   float = 80.0
    exclusions:  list[list[GeoPoint]] = []   # zones interdites internes

class MissionConstraints(BaseModel):
    max_speed_ms:            float = 8.0
    coverage_target_pct:     float = 90.0
    max_duration_s:          int   = 1800
    battery_reserve_pct:     float = 25.0
    link_loss_policy:        LinkLossPolicy = LinkLossPolicy.CONTINUE
    link_loss_timeout_s:     int = 600
    geofence:                MissionArea | None = None
    scoring_profile:         str = "nominal"

class Mission(BaseModel):
    id:            UUID
    name:          str
    type:          MissionType
    status:        MissionStatus
    drone_id:      UUID | None
    area:          MissionArea | None
    waypoints:     list[Waypoint] = []
    constraints:   MissionConstraints
    home:          GeoPoint | None
    created_at:    datetime
    started_at:    datetime | None
    completed_at:  datetime | None
    map_id:        UUID | None
    stats:         MissionStats | None
    version:       int = 1              # incrémenté à chaque modification
```

Le champ `version` sert la modification de mission en vol : le drone acquitte une version
précise, et le serveur sait donc exactement quelle version le drone exécute. Sans lui, une
modification perdue par la radio produit une divergence silencieuse entre ce que l'opérateur
croit et ce que le drone fait — précisément le genre de désynchronisation qui cause des
incidents.

### État du drone

```python
class Drone(BaseModel):
    id:                UUID
    name:              str
    model:             str
    serial:            str | None
    status:            DroneStatus            # OFFLINE/IDLE/ARMED/FLYING/RETURNING/EMERGENCY
    last_seen:         datetime | None
    current_mission:   UUID | None
    capabilities:      DroneCapabilities      # capteurs présents, autonomie, portée
    home:              GeoPoint | None

class DroneState(BaseModel):
    """Instantané courant — écrasé, pas historisé (l'historique est dans telemetry)."""
    drone_id:      UUID
    timestamp:     datetime
    position:      GeoPoint
    alt_rel_m:     float
    velocity:      Vector3
    heading_deg:   float
    battery:       BatteryInfo
    flight_mode:   str
    safety_state:  str
    nav_quality:   NavQuality             # GOOD/DEGRADED/SLAM_ONLY/POOR
    link:          LinkInfo               # état, RSSI, latence, perte, débit
    gnss:          GnssInfo
    coverage_pct:  float | None
    sensors_ok:    dict[str, bool]
    stale:         bool                   # aucune donnée fraîche → tout est périmé
```

Le champ `stale` évite un piège d'interface classique : quand la liaison est perdue, le
dashboard doit afficher la dernière position **connue** en la marquant comme telle, pas la
présenter comme actuelle. Un opérateur qui croit voir une position en direct alors qu'elle
a trois minutes prend de mauvaises décisions.

### Télémétrie (série temporelle)

```sql
CREATE TABLE telemetry (
    time          TIMESTAMPTZ      NOT NULL,
    drone_id      UUID             NOT NULL,
    mission_id    UUID,
    position      GEOGRAPHY(POINT, 4326),
    alt_rel_m     REAL,
    alt_amsl_m    REAL,
    vx, vy, vz    REAL,
    heading_deg   REAL,
    battery_pct   REAL,
    battery_v     REAL,
    battery_a     REAL,
    flight_mode   SMALLINT,
    safety_state  SMALLINT,
    nav_quality   SMALLINT,
    satellites    SMALLINT,
    hdop          REAL,
    link_rssi     REAL,
    link_loss     REAL,
    link_latency  REAL,
    coverage_pct  REAL,
    flags         INTEGER,
    received_at   TIMESTAMPTZ      NOT NULL,   -- ≠ time si le point vient du spool
    PRIMARY KEY (drone_id, time)
);

SELECT create_hypertable('telemetry', 'time', chunk_time_interval => INTERVAL '1 day');
ALTER TABLE telemetry SET (timescaledb.compress, timescaledb.compress_segmentby = 'drone_id');
SELECT add_compression_policy('telemetry', INTERVAL '7 days');
SELECT add_retention_policy('telemetry', INTERVAL '1 year');
```

La distinction `time` / `received_at` est essentielle pour ce système : les points issus du
spool après une coupure arrivent longtemps après leur production. Sans les deux colonnes,
on ne peut ni ordonner correctement, ni détecter les trous, ni afficher honnêtement ce qui
était connu en direct et ce qui a été récupéré après coup.

### Carte

```python
class MapArtifact(BaseModel):
    id:              UUID
    mission_id:      UUID
    drone_id:        UUID
    kind:            MapKind          # OCCUPANCY / POINTCLOUD / MESH / SEMANTIC
    format:          str              # "octomap-bt" / "pcd" / "ply"
    storage_uri:     str              # fichier local ou S3/MinIO
    size_bytes:      int
    resolution_m:    float
    bounds:          BoundingBox3D
    origin:          GeoPoint         # ancrage du repère map en WGS84
    coverage_pct:    float
    voxel_count:     int
    complete:        bool             # false = assemblée depuis des deltas partiels
    created_at:      datetime
    checksum:        str
```

Les cartes ne vont **pas** en base : elles vont dans un stockage objet (système de fichiers
en V1, MinIO ensuite), la base ne stockant que les métadonnées et l'URI. Mettre des
mégaoctets de binaire dans PostgreSQL dégrade tout le reste.

### Événement

```python
class Event(BaseModel):
    id:          UUID
    drone_id:    UUID
    mission_id:  UUID | None
    timestamp:   datetime          # horloge du drone
    received_at: datetime
    severity:    Severity          # DEBUG/INFO/WARNING/ERROR/CRITICAL
    category:    EventCategory     # SAFETY/NAV/COMM/MISSION/PERCEPTION/SYSTEM
    code:        str               # "GNSS_LOST", "RTH_TRIGGERED", "OBSTACLE_AVOIDED"
    message:     str
    data:        dict              # contexte structuré
    position:    GeoPoint | None
    acknowledged: bool = False
```

Les événements portent un `code` machine **et** un `message` humain. Le code permet le
filtrage, l'alerte et l'analyse statistique ; le message est pour l'opérateur. Se contenter
d'un texte libre rend l'exploitation impossible.

## 4. API REST

Base : `/api/v1`. OpenAPI généré automatiquement, source du client TypeScript du dashboard.

### Missions

| Méthode | Chemin | Description |
|---|---|---|
| `POST` | `/missions` | Créer (statut `draft`), valide la géométrie et la faisabilité |
| `GET` | `/missions` | Lister — filtres `status`, `drone_id`, `from`, `to`, pagination |
| `GET` | `/missions/{id}` | Détail complet |
| `PATCH` | `/missions/{id}` | Modifier — incrémente `version`, propage si active |
| `DELETE` | `/missions/{id}` | Supprimer (draft uniquement) |
| `POST` | `/missions/{id}/assign` | Affecter à un drone, avec vérification des capacités |
| `POST` | `/missions/{id}/start` | Démarrer — téléverse au drone, attend l'acquittement |
| `POST` | `/missions/{id}/pause` | Suspendre — le drone passe en stationnaire |
| `POST` | `/missions/{id}/resume` | Reprendre |
| `POST` | `/missions/{id}/cancel` | Annuler — le drone rentre à la base |
| `GET` | `/missions/{id}/progress` | Couverture, temps écoulé, énergie, estimation de fin |
| `GET` | `/missions/{id}/events` | Événements de la mission |

### Drones

| Méthode | Chemin | Description |
|---|---|---|
| `GET` | `/drones` | Liste avec état courant |
| `GET` | `/drones/{id}` | Détail et capacités |
| `GET` | `/drones/{id}/state` | Instantané courant (avec drapeau `stale`) |
| `GET` | `/drones/{id}/telemetry` | Historique — `from`, `to`, `resolution` (agrégats continus) |
| `GET` | `/drones/{id}/track` | Trajectoire en GeoJSON |
| `POST` | `/drones/{id}/commands/goto` | Envoyer un waypoint immédiat |
| `POST` | `/drones/{id}/commands/rth` | Demander le retour à la base |
| `POST` | `/drones/{id}/commands/hold` | Vol stationnaire |
| `POST` | `/drones/{id}/commands/land` | Atterrir |
| `GET` | `/drones/{id}/commands/{cmd_id}` | Statut d'une commande (envoyée / acquittée / expirée / rejetée) |
| `GET` | `/drones/{id}/link` | Qualité de liaison détaillée et historique |

### Cartes et événements

| Méthode | Chemin | Description |
|---|---|---|
| `GET` | `/maps` | Lister — filtres `mission_id`, `drone_id` |
| `GET` | `/maps/{id}` | Métadonnées |
| `GET` | `/maps/{id}/download` | Fichier brut |
| `GET` | `/maps/{id}/tiles/{z}/{x}/{y}` | Tuiles pour affichage progressif |
| `GET` | `/maps/{id}/pointcloud` | Nuage décimé pour le web (LOD) |
| `GET` | `/events` | Filtres `severity`, `category`, `drone_id`, `from`, `to` |
| `POST` | `/events/{id}/ack` | Acquitter une alerte |

### Sur le statut des commandes

`POST /drones/{id}/commands/goto` retourne `202 Accepted` avec un identifiant de commande —
**pas `200 OK`**. La distinction est sémantiquement importante : le serveur a accepté de
transmettre la commande, il ne garantit pas que le drone l'a reçue ni acceptée. Le client
suit le statut réel via `GET /commands/{cmd_id}` ou par WebSocket. Retourner `200` laisserait
croire à l'opérateur que la commande est appliquée alors qu'elle est peut-être perdue dans
une coupure radio.

## 5. WebSocket

`/api/v1/ws?drone_id=...&topics=state,events,map_delta`

Messages poussés :

```jsonc
{ "type": "state",     "drone_id": "...", "data": { /* DroneState */ } }   // 1-2 Hz
{ "type": "event",     "drone_id": "...", "data": { /* Event */ } }        // événementiel
{ "type": "map_delta", "drone_id": "...", "data": { /* voxels */ } }       // 0,5 Hz
{ "type": "link",      "drone_id": "...", "data": { /* LinkInfo */ } }     // 1 Hz
{ "type": "mission",   "mission_id": "...", "data": { /* progression */ } }// 0,5 Hz
```

Le serveur **limite le débit par client** et abandonne les messages `map_delta` si le client
prend du retard — un navigateur lent ne doit pas faire gonfler les tampons du serveur.
Les événements ne sont jamais abandonnés : ils sont mis en file et retransmis. Même
hiérarchie que sur la radio, appliquée au sol.

## 6. Bus d'événements

```python
class EventBus(ABC):
    @abstractmethod
    async def publish(self, topic: str, payload: dict) -> None: ...
    @abstractmethod
    def subscribe(self, topic: str, handler: Callable) -> Subscription: ...
```

V1 : `InMemoryEventBus` (asyncio). V2 : `RedisEventBus`. Les appelants ne changent pas.

C'est le point de découplage qui rend l'extraction en microservices possible sans
réécriture. Il coûte une trentaine de lignes aujourd'hui et évite une refonte plus tard —
un des rares cas où l'abstraction anticipée est rentable, parce que l'interface est
minuscule et l'implémentation triviale.

Sujets : `telemetry.received`, `drone.state_changed`, `mission.status_changed`,
`event.raised`, `map.updated`, `link.state_changed`, `command.acked`.

## 7. Sécurité

| Aspect | V1 | V2 (P8) |
|---|---|---|
| Authentification API | Jeton statique en variable d'environnement | OAuth2 / OIDC, JWT à durée limitée |
| Autorisation | Aucune (mono-utilisateur) | RBAC : `viewer`, `operator`, `admin` |
| Liaison radio | Authentification HMAC | + chiffrement ChaCha20-Poly1305 |
| Base | Mot de passe local | Secrets externalisés, TLS |
| Journal d'audit | Journalisation applicative | Table d'audit immuable |

Les commandes qui affectent un aéronef en vol (`goto`, `rth`, `land`, `cancel`) sont
**toujours journalisées avec leur auteur**, dès la V1, même sans authentification (auteur =
`"anonymous"`). Après un incident, savoir qui a envoyé quoi et quand n'est pas négociable —
et rétro-ajouter un journal d'audit signifie qu'on n'a pas les traces de la période où on en
avait besoin.

## 8. Module IA du serveur

Ce que le serveur fait avec de l'IA, par opposition au bord :

| Fonction | Pourquoi au sol plutôt qu'à bord |
|---|---|
| Post-traitement de carte (reconstruction de maillage, nettoyage) | Coûteux, non temps réel, Open3D |
| Détection d'objets haute résolution sur images récupérées | Modèles lourds, GPU disponible, pas de contrainte énergétique |
| Analyse de mission (efficacité, anomalies, recommandations) | Nécessite l'historique complet et le recul |
| Planification pré-mission (découpage de zone, estimation de durée) | Avant le vol, sans contrainte de latence |
| Entraînement et réentraînement de modèles | Évidemment |

Le principe reste le même qu'à bord : le serveur ne prend jamais de décision temps réel.
Son IA produit des analyses et des recommandations qu'un opérateur valide, ou des artefacts
(modèles, plans) chargés à bord avant le vol.
