# 09 — Communication radio

## 1. Ce que la liaison transporte — et ne transporte pas

**Transporté** : missions, commandes haut niveau, waypoints, télémétrie, état du drone,
batterie, position, événements, alertes, deltas de carte, résumés de couverture.

**Jamais transporté** : consignes d'attitude, de taux angulaires, de poussée, sorties
moteur, boucles de contrôle, retour capteur temps réel.

Ce n'est pas une convention mais une **propriété structurelle vérifiée** : le schéma
Protobuf ne définit aucun message du plan de contrôle. Le test
`test_protocol_has_no_control_plane_messages` inspecte le descripteur Protobuf compilé et
échoue si un champ contenant `attitude`, `rate`, `thrust`, `motor` ou `actuator` apparaît.
On ne peut donc pas violer la contrainte par inadvertance — il faudrait modifier le schéma
et supprimer le test.

## 2. L'interface `CommunicationLink`

Le point d'abstraction demandé par le brief, avec les précisions qu'un usage réel impose.

```python
# drone/communication/link/base.py

class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTING   = auto()
    CONNECTED    = auto()
    DEGRADED     = auto()   # passe des données, mais hors des seuils nominaux
    FAILED       = auto()   # panne matérielle, pas une simple perte de portée


@dataclass(frozen=True)
class LinkQuality:
    rssi_dbm:          float | None   # None si le support ne le remonte pas
    snr_db:            float | None
    packet_loss_ratio: float          # [0,1], fenêtre glissante 10 s
    latency_ms:        float          # aller-retour estimé
    jitter_ms:         float
    bandwidth_bps:     int            # débit utile estimé, pas nominal
    score:             float          # [0,1], indicateur agrégé
    measured_at:       float          # horodatage monotone


class CommunicationLink(ABC):
    """Transport orienté messages, non fiable, entre le drone et le sol.

    Contrat :
      - send() est NON BLOQUANT et peut échouer. Il ne garantit RIEN sur la remise.
      - receive() est non bloquant, retourne None s'il n'y a rien.
      - Aucune méthode ne lève d'exception en fonctionnement normal ; une panne
        se signale par l'état, jamais par une exception. Le code appelant ne
        doit pas avoir à envelopper chaque appel dans un try/except.
      - Aucune méthode ne bloque plus de 10 ms. Ce transport est appelé depuis
        une boucle qui a d'autres choses à faire.
    """

    @abstractmethod
    def send(self, payload: bytes, priority: Priority) -> bool:
        """Met en file d'émission. True = accepté, False = file pleine ou lien mort.
        Un True ne signifie PAS que le message est arrivé."""

    @abstractmethod
    def receive(self) -> bytes | None:
        """Prochain message reçu, ou None."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Raccourci : state in (CONNECTED, DEGRADED)."""

    @abstractmethod
    def get_state(self) -> ConnectionState: ...

    @abstractmethod
    def get_quality(self) -> LinkQuality:
        """Qualité mesurée. Toujours disponible, même déconnecté (valeurs périmées
        avec measured_at ancien — au consommateur de juger)."""

    @abstractmethod
    def get_mtu(self) -> int:
        """Taille utile max d'un message. 50-250 octets sur modem série,
        ~1400 sur IP. La couche supérieure DOIT fragmenter au-delà."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...
```

### Écarts assumés par rapport au brief

Le brief propose `getLatency()`, `getSignalQuality()`, `getBandwidth()` séparément. Trois
modifications, chacune motivée par un problème réel :

**Une seule méthode `get_quality()` renvoyant un objet cohérent.** Trois appels séparés
renvoient des mesures prises à des instants différents ; on peut lire une latence d'avant
la coupure et un débit d'après. Un instantané atomique horodaté supprime cette classe de
bug.

**Ajout de `get_mtu()`.** Un modem série 868 MHz a une charge utile de l'ordre de 50 à
250 octets. Une couche supérieure qui l'ignore produit des messages tronqués découverts en
vol. Exposer le MTU force la fragmentation à être conçue, pas découverte.

**Ajout de `Priority` dans `send()`.** Sans priorité, une rafale de deltas de carte peut
retarder une alerte de batterie critique. Sur un canal à 50 kbps, c'est un scénario
courant, pas un cas limite.

### Implémentations

| Classe | Support | Phase | Usage |
|---|---|---|---|
| `VirtualRadioLink` | UDP + modèle de canal | **P5** | Simulation, tests |
| `LoopbackLink` | Appel direct en mémoire | **P5** | Tests unitaires, débogage |
| `SerialRadioLink` | `pyserial` — RFD 868x, Microhard | P9 | Drone réel |
| `IPLink` | UDP/TCP — WiFi, Ethernet, 4G | P8 | Portée courte, banc d'essai |
| `SatelliteLink` | Iridium SBD, Swarm | Futur | Secours mondial, très bas débit |
| `MultiPathLink` | Composite | Futur | Bascule et duplication entre liens |

`MultiPathLink` mérite un mot : il implémente `CommunicationLink` **et** agrège plusieurs
`CommunicationLink`. Il route selon la priorité et la disponibilité — messages de sécurité
dupliqués sur tous les liens, télémétrie de volume sur le plus rapide. Le fait que cette
composition soit possible sans modifier quoi que ce soit d'autre est la preuve que
l'abstraction est correcte.

## 3. Protocole DLP (Drone Link Protocol)

### Trame

```
┌────────┬────────┬─────────┬────────┬────────┬──────────┬─────────┬──────────┬───────┐
│ MAGIC  │ VER    │ FLAGS   │ SRC    │ DST    │ SEQ      │ LEN     │ PAYLOAD  │ CRC32 │
│ 2 o    │ 1 o    │ 1 o     │ 2 o    │ 2 o    │ 2 o      │ 2 o     │ n o      │ 4 o   │
└────────┴────────┴─────────┴────────┴────────┴──────────┴─────────┴──────────┴───────┘
   0xD8      1      priorité   drone   station  compteur   taille    Protobuf   sur
   0x0A             frag       _id     _id      cyclique             sérialisé  l'entête
                    ack req                                                     + charge
```

En-tête de 16 octets. Sur des messages de 100 à 200 octets, c'est 8 à 16 % de surcoût —
acceptable et non compressible davantage sans perdre en robustesse. Le CRC32 est
indispensable : un modem radio délivre parfois des trames corrompues que le contrôle
d'erreur de la couche physique n'a pas rattrapées, et une trame corrompue interprétée comme
une commande est un incident.

### Classes de priorité

| Priorité | Contenu | Politique |
|---|---|---|
| `SAFETY` (0) | Alertes critiques, batterie critique, demande d'atterrissage d'urgence, déclenchement de failsafe | **Jamais abandonnée.** Réémission jusqu'à acquittement. Court-circuite la file. |
| `COMMAND` (1) | Missions, waypoints, commandes de contrôle de mission | Acquittement requis, réémission avec repli exponentiel, expiration à 60 s |
| `TELEMETRY` (2) | Télémétrie périodique, état | Best effort. Une trame perdue est remplacée par la suivante. |
| `BULK` (3) | Deltas de carte, logs, MAVLink tunnelé, images vignettes | Abandonnée en premier. Transmise uniquement sur bande passante excédentaire. |

Ordonnancement par **file de priorité stricte avec anti-famine** : `BULK` reçoit une part
minimale garantie (5 %) pour éviter qu'un flux continu de télémétrie ne bloque
indéfiniment le transfert de carte.

### Messages Protobuf

```protobuf
// interfaces/proto/dronoto/v1/link.proto
syntax = "proto3";
package dronoto.v1;

message Envelope {
  uint32 drone_id     = 1;
  uint64 timestamp_us = 2;   // horloge monotone du drone
  uint32 sequence     = 3;
  oneof payload {
    Telemetry        telemetry         = 10;
    Event            event             = 11;
    MapDelta         map_delta         = 12;
    CoverageSummary  coverage          = 13;
    CommandAck       command_ack       = 14;
    // ── sol → drone ──
    MissionCommand   mission_command   = 20;
    WaypointCommand  waypoint_command  = 21;
    ControlCommand   control_command   = 22;   // pause/reprise/annulation/RTH — PAS de contrôle moteur
    ParameterUpdate  parameter_update  = 23;
    TimeSync         time_sync         = 24;
  }
}

message Telemetry {
  // Position en entiers : latitude/longitude en 1e-7 degré, altitude en cm.
  // Le float64 coûterait 3× plus cher pour une précision inutile.
  sint32 lat_1e7        = 1;
  sint32 lon_1e7        = 2;
  sint32 alt_amsl_cm    = 3;
  sint32 alt_rel_cm     = 4;
  sint32 vx_cms         = 5;
  sint32 vy_cms         = 6;
  sint32 vz_cms         = 7;
  uint32 heading_cdeg   = 8;    // centidegrés
  uint32 battery_pct    = 9;
  uint32 battery_mv     = 10;
  sint32 battery_ma     = 11;
  FlightMode  mode      = 12;
  SafetyState safety    = 13;
  NavQuality  nav       = 14;
  uint32 satellites     = 15;
  uint32 hdop_cm        = 16;
  uint32 coverage_pct   = 17;
  uint32 mission_id     = 18;
  uint32 flags          = 19;   // champ de bits : armé, offboard, GPS ok, LiDAR ok...
}
```

**Encodage entier partout.** Une trame `Telemetry` fait ainsi 45 à 60 octets contre 300+
en JSON. À 50 kbps partagés, ce facteur 5 décide de la faisabilité — il permet 1 Hz de
télémétrie tout en laissant de la place pour la carte.

### Budget de bande passante à 50 kbps

```
Débit brut                                     50 000 bps
− surcoût radio (préambule, FEC, ~25 %)       −12 500
= débit utile                                  37 500 bps  ≈ 4 700 octets/s

Répartition nominale :
  Télémétrie   1 Hz × 60 o   =    60 o/s   (1,3 %)
  Événements   ~0,2 Hz × 80 o=    16 o/s   (0,3 %)
  Commandes    sporadique     ≈   20 o/s   (0,4 %)
  Acquittements               ≈   15 o/s   (0,3 %)
  ─────────────────────────────────────────────────
  Sous-total plan de gestion  =   111 o/s  (2,4 %)
  DISPONIBLE pour BULK        ≈ 4 589 o/s  (97 %)  → deltas de carte
```

Le résultat est confortable, et ce n'est pas un hasard : c'est la conséquence directe
d'avoir refusé de faire transiter des données de contrôle ou des flux capteur. Une
architecture qui aurait mis du contrôle temps réel sur la radio ne tiendrait pas dans ce
budget, et c'est une raison technique — pas seulement doctrinale — de ne pas le faire.

## 4. Modèle de canal radio simulé

`VirtualRadioLink` relaie entre deux sockets UDP en appliquant un modèle physique. Le drone
et le sol ne savent pas qu'ils sont dégradés : ils voient un lien qui perd des paquets,
exactement comme un vrai.

### Modèle de propagation

```
Perte de parcours (log-distance) :
    PL(d) = PL(d₀) + 10·n·log₁₀(d/d₀) + X_σ

    n     = exposant d'atténuation (2,0 espace libre ; 2,7-3,5 avec obstacles)
    X_σ   = évanouissement lognormal, σ = 4-8 dB, corrélé dans le temps (τ ≈ 2 s)

RSSI = P_tx + G_tx + G_rx − PL(d) − pertes_câbles
SNR  = RSSI − plancher_de_bruit

Taux d'erreur paquet, dérivé du SNR par une sigmoïde calée sur le seuil du modem :
    PER = 1 / (1 + exp((SNR − SNR_seuil) / k))

Débit utile, adaptatif selon la modulation du modem :
    bande_passante = bande_nominale × f(SNR)   par paliers, comme un vrai modem

Latence = latence_base + temps_transmission(taille, bande_passante) + gigue
    gigue ~ loi exponentielle, moyenne croissante avec la charge de la file
```

Enrichissements qui font la différence entre un modèle jouet et un modèle utile :

- **Occlusion géométrique** : si le terrain ou un bâtiment coupe la ligne de vue
  drone-station, on ajoute une atténuation par diffraction. Le lien tombe donc derrière une
  colline et revient après — comportement réel, très différent d'une simple dégradation
  avec la distance.
- **Diagramme d'antenne** : gain dépendant de l'angle. Une antenne directive au sol crée un
  cône de couverture ; sortir du cône dégrade même à courte distance. Le drone incliné en
  vol rapide perd du gain sur son antenne dipôle — effet réel, souvent surprenant.
- **Asymétrie** : montant et descendant sont modélisés séparément (puissances d'émission et
  antennes différentes). Le cas « je reçois la télémétrie mais mes commandes ne passent
  pas » est fréquent en réel et doit être testable.
- **Corrélation temporelle** : l'évanouissement est un processus corrélé, pas un tirage
  indépendant par paquet. Les pertes arrivent donc en rafales, ce qui teste bien plus
  durement les mécanismes de réémission qu'une perte uniforme.

### Configuration

```yaml
# simulation/config/radio_rfd868x.yaml
radio:
  profile: "RFD 868x, antenne dipôle drone + Yagi 9 dBi au sol"
  tx_power_dbm: 27
  antenna_gain_drone_dbi: 2.0
  antenna_gain_ground_dbi: 9.0
  antenna_pattern_ground: "yagi_9dbi_60deg"
  noise_floor_dbm: -110
  snr_threshold_db: 6.0
  nominal_bandwidth_bps: 50000
  base_latency_ms: 80
  jitter_ms: { mean: 15, model: exponential }
  path_loss_exponent: 2.7
  shadowing_sigma_db: 6.0
  shadowing_correlation_s: 2.0
  max_range_m: 20000
  mtu_bytes: 200
  geometric_occlusion: true
  seed: 42                      # reproductibilité — obligatoire
```

**La graine est obligatoire.** Sans elle, un test de résilience qui échoue une fois sur dix
est inexploitable : on ne sait pas si le correctif fonctionne ou si on a eu de la chance.

## 5. Perte de liaison et reprise

### Détection

Trois signaux combinés, parce qu'aucun n'est suffisant seul :

1. **Absence de réception** : aucune trame valide depuis `T_silence` (défaut 5 s).
2. **Absence d'acquittement** : les messages `SAFETY` et `COMMAND` ne sont plus acquittés.
3. **Qualité effondrée** : `LinkQuality.score` sous le seuil pendant une durée soutenue.

Le drone déclare `LINK_LOST` après 5 s, `LINK_DEGRADED` bien avant — ce qui déclenche
l'adaptation de débit avant la perte totale, pas après.

### Comportement pendant la perte

```
Perte de liaison détectée
      │
      ├─▶ La mission CONTINUE  (comportement par défaut, configurable par politique)
      │
      ├─▶ Toute la télémétrie part dans le SPOOL (tampon circulaire sur disque)
      │       · 30 minutes de télémétrie à 1 Hz  ≈ 110 ko
      │       · tous les événements, jamais abandonnés
      │       · deltas de carte, jusqu'à un quota (50 Mo)
      │
      ├─▶ Le comm_manager continue d'ÉMETTRE en aveugle à cadence réduite
      │       Sans écoute, on ne détecte pas le retour du lien.
      │       Cadence réduite à 0,2 Hz pour économiser l'énergie du modem.
      │
      └─▶ Une horloge de perte de liaison démarre  →  politique après expiration
```

### Politiques de perte de liaison

Configurables par mission, avec un défaut délibérément peu agressif :

| Politique | Comportement | Cas d'usage |
|---|---|---|
| `CONTINUE` **(défaut)** | Poursuit la mission indéfiniment, rentre à la base à la fin ou sur batterie faible | Cartographie autonome — la liaison est un confort, pas une nécessité |
| `CONTINUE_THEN_RTH` | Poursuit `T` minutes (défaut 10), puis rentre | Compromis quand la supervision compte |
| `RTH_IMMEDIATE` | Retour immédiat | Opérations sensibles, zones réglementées |
| `HOLD_THEN_RTH` | Stationnaire `T` s pour laisser une chance à la reprise, puis rentre | Perte de liaison brève attendue |
| `LAND` | Atterrissage sur place | Vol confiné, intérieur |

`CONTINUE` par défaut est un choix délibéré et cohérent avec le brief : le drone est conçu
pour être autonome, la liaison est un canal de supervision. Rentrer à chaque coupure
annulerait l'intérêt du système. Le retour à la base reste garanti par la contrainte
batterie, qui elle ne dépend de rien d'extérieur.

### Reprise et resynchronisation

```
Trame valide reçue après une perte
      │
      ▼
Poignée de main : HELLO(drone_id, boot_id, dernière_seq_reçue, taille_spool)
      │
      ▼
Le sol répond : HELLO_ACK(dernière_seq_reçue_par_le_sol, horloge)
      │
      ├──▶ Synchronisation d'horloge (compensation de dérive)
      │
      ├──▶ Vidage du spool par priorité, en tâche de fond :
      │       1. événements (tous, dans l'ordre)
      │       2. télémétrie (décimée si le volume est important — au-delà
      │          d'une certaine ancienneté, 1 point sur 5 suffit à tracer la trajectoire)
      │       3. deltas de carte (fusionnés avant envoi : un voxel modifié
      │          cinq fois pendant la coupure n'est envoyé qu'une fois dans son état final)
      │
      └──▶ Le sol réconcilie : détection de trous, marquage des périodes sans données
```

La **fusion des deltas de carte avant vidage** est l'optimisation qui rend la reprise
viable : après 10 minutes de coupure, le spool brut pourrait contenir des mégaoctets de
mises à jour redondantes, alors que l'état final tient dans une fraction de ce volume.

Le `boot_id` détecte le redémarrage du drone en vol : le sol sait alors que les compteurs
de séquence ont été réinitialisés et que l'état antérieur n'est plus valide.

## 6. Adaptation de débit

Le `comm_manager` ajuste en continu ce qu'il émet selon la bande passante réellement
disponible :

| Qualité de liaison | Télémétrie | Deltas de carte | Événements | Vignettes |
|---|---|---|---|---|
| Excellente (> 0,8) | 2 Hz | Pleine résolution | Tous | Oui |
| Bonne (0,5-0,8) | 1 Hz | Pleine résolution | Tous | Non |
| Dégradée (0,2-0,5) | 0,5 Hz | Décimés 1/4 | Tous | Non |
| Mauvaise (< 0,2) | 0,2 Hz | Suspendus | Critiques seulement | Non |
| Perdue | Spool | Spool | Spool | Non |

**Les événements critiques ne sont jamais dégradés.** C'est la ligne qui ne bouge pas :
quand tout se dégrade, ce qui doit passer est l'information de sécurité.

## 7. Sécurité de la liaison

Conçue maintenant, implémentée en P8. La concevoir tard obligerait à changer le format de
trame, donc tout retester.

- **Authentification** : HMAC-SHA256 tronqué à 8 octets sur chaque trame, clé pré-partagée.
  Coût : 8 octets par trame, soit ~5 % de surcoût. Acceptable.
- **Anti-rejeu** : le compteur de séquence et l'horodatage sont couverts par le HMAC ; une
  fenêtre glissante rejette les trames trop anciennes ou déjà vues.
- **Chiffrement** : ChaCha20-Poly1305 sur la charge utile, optionnel — le chiffrement de la
  télémétrie n'est pas toujours nécessaire et coûte du CPU sur un calculateur contraint.
  **L'authentification, elle, n'est pas optionnelle** : accepter une commande non
  authentifiée est un défaut de sécurité, pas un arbitrage.
- **Restrictions réglementaires** : en bande ISM 868 MHz européenne, le chiffrement est
  autorisé, mais le rapport cyclique (duty cycle ≤ 10 %) et la PIRE sont limités. Le modèle
  de canal simulé applique la limite de rapport cyclique pour que les tests reflètent la
  réalité opérationnelle.
