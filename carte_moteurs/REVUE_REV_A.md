# Carte moteurs — revue du schéma et du PCB (rev A)

Revue effectuée le 2026-09-27 sur `carte_moteurs.kicad_sch` et `carte_moteurs.kicad_pcb`.

Méthode : extraction de la netlist et exécution de l'ERC via
`kicad-cli sch export netlist` et `kicad-cli sch erc --severity-all`,
puis analyse des largeurs de pistes et des plans du `.kicad_pcb`.
L'ERC remonte **82 violations**.

## Contexte de la carte

| | |
|---|---|
| MCU principal | `A1` — ESP32 DevKit V1 DOIT 30 broches |
| MCU secondaire | `U1` — ESP32-C3 SuperMini (nœud WiFi, une seule broche utilisée) |
| Drivers | `U2`/`U3`/`U5`/`U4` — TMC2209 SilentStepStick, moteurs 1/2/3/4 |
| Alimentation | `VR1` — module LM2596 réglable, BAT → 5V |
| Entrée | XT30 `BAT_GND1`, arrêt d'urgence en série sur BAT+ via `J5` |
| Sorties | 4× header 2,54 mm 4 pts (phases moteur), XT30 `12V_OUT1`, OLED I²C `J1`, bus série `SERIE_IN1`/`SERIE_OUT1` |
| PCB | 136 × 55,5 mm, 2 couches, plans GND sur F.Cu et B.Cu |

---

# 🔴 Bloquants

## B1 — L'ESP32 DevKit (`A1`) n'est pas alimenté

`VIN` (broche 30) n'est reliée à rien. Le net `/3V3` ne contient que
`A1.1` (la *sortie* 3,3 V du DevKit) et `J1.2` (alimentation de l'OLED).
Le 5 V du LM2596 va au SuperMini et aux `VIO` des drivers,
mais **jamais au DevKit**.

Conséquence : la carte ne fonctionne que si le DevKit est alimenté par son
port USB. Sur batterie, rien ne démarre.

Confirmé par l'ERC :

```
@(49.53 mm, 93.98 mm) : Symbol A1 Pin 30 [VIN, Power input] — Pin not connected
@(49.53 mm, 93.98 mm) : Symbol A1 Pin 30 [VIN, Power input] — not driven
@(39.37 mm, 93.98 mm) : Symbol A1 Pin 1  [3.3V, Power input] — not driven
```

**Correction :** relier le net `5V` (VR1 broche 4, OUT+) à `A1.VIN`.

## B2 — `STEP_M1` est câblé sur GPIO35, qui est *input-only*

`A1.20` porte le nom `ADC1_CH7/D35`, soit **GPIO35**. Sur l'ESP32 classique,
GPIO34 / 35 / 36 / 39 ne disposent d'aucun étage de sortie : ils ne peuvent
pas être configurés en sortie, quel que soit le code.

Conséquence : **le moteur 1 ne tournera jamais.** Aucune impulsion STEP ne
peut être émise.

**Correction :** déplacer `STEP_M1` sur une broche capable de sortie.
Voir le mapping proposé plus bas.

## B3 — `DIR_M3` est câblé sur GPIO12, broche de *strapping*

`A1.27` = `ADC2_CH5/D12` = **GPIO12 / MTDI**. Cette broche est lue au reset
pour choisir la tension d'alimentation de la flash. Si elle est au niveau
haut au démarrage, l'ESP32 configure la flash en 1,8 V et **ne démarre pas**.

Elle est ici reliée à l'entrée `DIR` d'un TMC2209. Selon l'état de cette
entrée au moment du reset, la carte peut refuser de booter de façon
intermittente — le pire type de panne à diagnostiquer.

**Correction :** libérer GPIO12 (à laisser en l'air). À défaut, pull-down
10 kΩ vers GND, mais la réaffectation est préférable.

## B4 — `VIO` des TMC2209 alimenté en 5 V alors que les GPIO sont en 3,3 V

Net `5V` : `U2.2`, `U3.2`, `U4.2`, `U5.2` (`VIO`) + `VR1.4` + `U1.5V`.

Les seuils logiques d'entrée du TMC2209 sont relatifs à `VIO` :
V_IH ≈ 0,7 × VIO, soit **3,5 V** avec VIO = 5 V. Les GPIO de l'ESP32
sortent 3,3 V au maximum — on est **sous le seuil garanti par le
constructeur** sur STEP, DIR et EN.

En pratique cela peut fonctionner à température ambiante sur un exemplaire
donné, puis devenir aléatoire en chauffant ou sur une autre série : pas
manqués, direction erratique, driver qui ne s'active pas.

**Correction :** alimenter `VIO` en **3,3 V** (depuis `A1.3V3`), pas en 5 V.
Le 5 V reste pour le SuperMini uniquement.

---

# 🟠 Importants

## I1 — Aucun pull-up sur `EN`

Net `EN` : `A1.28` (GPIO13) + les quatre `~{EN}` des drivers, sans aucune
résistance.

`EN` est actif à l'état bas. Entre la mise sous tension et l'initialisation
du firmware — et pendant toute la durée d'un reset — la ligne est
**flottante**, donc dans un état indéterminé. Les quatre drivers peuvent
s'activer avant que le programme ne contrôle quoi que ce soit.

Sur un robot mobile, cela signifie un départ intempestif à la mise sous
tension. C'est le point le plus sérieux du point de vue sécurité.

**Correction :** résistance **10 kΩ de `EN` vers `VIO`**, pour que l'état
par défaut au repos soit « drivers désactivés ».

## I2 — I²C de l'OLED sur des broches non standard

| Net | Broche `A1` | GPIO |
|---|---|---|
| `OLED_SDA` | 11 — `D21/SDA` | 21 ✔ |
| `OLED_SCK` | 9 — `D18` | **18** ✘ |

`A1.14` = `D22/SCL`, la broche SCL par défaut, est laissée **non connectée**.
Le code doit obligatoirement faire `Wire.begin(21, 18)`, sinon rien ne
répond sur le bus.

Vraisemblablement une erreur de câblage plutôt qu'un choix.

**Correction :** `OLED_SCL` → GPIO22. Vérifier par ailleurs que le module
OLED embarque bien ses résistances de tirage (aucune sur la carte).

## I3 — Erreur dans le symbole du DevKit : deux broches nommées `D32`

Dans la définition du symbole `ESP32_DevKit_V1_DOIT_30GPIOs`, les broches
**21 et 22 portent toutes deux le nom `ADC1_CH4/D32`**.

Sur un DOIT V1 30 broches, la rangée droite est, du bas vers le haut :
VIN(30), GND(29), D13(28), D12(27), D14(26), D27(25), D26(24), D25(23),
**D33(22)**, D32(21), D35(20), D34(19), VN(18), VP(17), EN(16).

La broche 22 est donc **GPIO33**, pas GPIO32.

Sans conséquence électrique (les deux sont capables de sortie), mais
`DIR_M2` est physiquement sur **GPIO33** : le firmware qui utiliserait
GPIO32 pilotera la mauvaise broche.

**Correction :** corriger le symbole dans la bibliothèque
`PCM_SL_Development_Board`.

## I4 — Bus série multi-point sans aucune protection

Net `/WROOM_RX`, 4 nœuds :

```
A1.6            (RX2 / GPIO16)
SERIE_IN1.1
SERIE_OUT1.1
SW_WIFI_CABLE1.2  (cavalier vers le TX du ESP32-C3)
```

`SERIE_IN1` et `SERIE_OUT1` sont câblés en parallèle (broche 1 sur
`/WROOM_RX`, broche 2 sur `GND`) : c'est un bus à chaînage, pas une entrée
et une sortie distinctes.

Deux problèmes :

1. **Court-circuit franc** si deux émetteurs push-pull se retrouvent sur le
   bus en même temps (par exemple une autre carte sur `SERIE_IN` pendant que
   le cavalier `SW_WIFI_CABLE1` est en place). Aucune résistance série ne
   limite le courant.
2. **Entrée flottante** : cavalier retiré et connecteurs débranchés,
   GPIO16 flotte. L'UART reçoit des trames parasites.

Par ailleurs `A1.7` (`TX2` / GPIO17) est non connectée : la liaison est
**unidirectionnelle**, la carte ne peut pas répondre.

**Correction :** 100–330 Ω en série sur chaque TX, pull-up 10 kΩ sur la
ligne RX, et passage des connecteurs série en 3 points (GND / RX / TX).

## I5 — Aucune protection sur l'entrée batterie

Le net `BAT` part directement de l'XT30 (via l'arrêt d'urgence) vers les
quatre `VM` et l'entrée du LM2596. Il n'y a **ni fusible, ni protection
contre l'inversion de polarité, ni écrêteur de surtension**.

Une XT30 branchée à l'envers détruit les quatre TMC2209 et le module
LM2596 en une seule fois.

**Correction :** MOSFET canal P en série sur BAT+ (anti-inversion,
quelques mΩ), TVS type SMBJ18A, et porte-fusible.

## I6 — Régulateur réglable sans garde-fou

`VR1` est un module LM2596 à sortie **ajustable par potentiomètre**. Rien
n'empêche d'envoyer 12 V sur tout le logique si le potentiomètre n'a pas été
réglé avant montage, ou s'il bouge.

**Correction :** module à sortie fixe 5 V. Si le LM2596 est conservé :
procédure de réglage en sérigraphie + écrêteur zener 5,6 V en sortie.

---

# 🟡 Mineurs, BOM et documentation

## M1 — `C1` à `C4` n'ont aucune valeur

Les quatre condensateurs ont pour champ `Value` la chaîne `C_Polarized`,
c'est-à-dire le nom du symbole. **Ni capacité, ni tension de service.**
L'empreinte est `CP_Radial_D8.0mm_P5.00mm`.

Ils sont sur le rail `BAT` (≈ 12 V), donc 25 V de tenue au minimum.
En l'état la BOM est inexploitable.

**Correction :** 100 µF / 25 V faible ESR (ou 220 µF / 35 V), **un par
driver**, placé à quelques millimètres de chaque broche `VM`, avec un
100 nF céramique à côté. Ajouter aussi 22 µF + 100 nF en sortie du 5 V
et sur le 3,3 V.

## M2 — `MS1`, `MS2`, `SPRD`, `PDN_UART` en l'air sur les quatre drivers

Les quatre drivers tournent en mode autonome avec toutes leurs broches de
configuration flottantes :

- `MS1`/`MS2` : le micro-pas dépend des tirages internes du composant, il
  n'est pas choisi explicitement.
- `SPRD` : mode de hachage figé.
- `PDN_UART` : aucune configuration logicielle possible. Le courant moteur
  ne se règle **qu'au potentiomètre**, au tournevis, sur chaque driver.

Ce n'est pas une faute, c'est un choix par défaut — mais il faut savoir ce
qu'on perd : pas de StallGuard exploitable (le seuil `SGTHRS` n'est pas
accessible sans UART), pas de réduction du courant à l'arrêt, pas de
diagnostic.

**Correction minimale :** câbler `MS1`/`MS2` en dur pour un micro-pas
déterministe.
**Correction complète :** voir la section « mode UART » plus bas.

## M3 — Les quatre `DIAG` et `INDEX` sont non connectés

`U2.18`, `U3.18`, `U4.18`, `U5.18` (`DIAG`) et les `INDEX` ne vont nulle
part. Aucune remontée de défaut driver (surchauffe, court-circuit) vers le
MCU, et aucune détection de blocage possible.

## M4 — `12V_OUT1` ne délivre pas du 12 V régulé

`12V_OUT1.2` est sur le net `BAT`, c'est-à-dire la **batterie brute** en
aval de l'arrêt d'urgence. Sur un pack 3S, la tension va de 12,6 V chargé
à environ 10,5 V déchargé.

Le nom du connecteur laisse croire à un rail 12 V régulé. Risque
d'endommager ce qu'on y branchera.

**Correction :** renommer `BAT_OUT`, et l'indiquer en sérigraphie.

## M5 — Références et repérage incohérents

- `BAT_GND1` a pour valeur `BAT_IN` : le nom de la référence et sa valeur
  se contredisent.
- Les références `J3`, `J6`, `J7`, `J8` sont absentes de la numérotation
  (présents : J1, J2, J4, J5, J9, J10).
- **`U4` correspond au moteur 4 et `U5` au moteur 3** : l'ordre des
  références ne suit pas l'ordre des moteurs. Source d'erreur d'assemblage
  et de câblage.

## M6 — Orientation du couple symbole / empreinte TMC2209 à vérifier

L'empreinte `MODULE_TMC2209_SILENTSTEPSTICK` (import SnapEDA) place :

- pastilles 1→8 en x = −6,35 mm, de y = −8,89 à +8,89 :
  GND, VIO, M1B, M1A, M2A, M2B, GND, VM
- pastilles 9→16 en x = +6,35 mm :
  DIR, STEP, PDN, UART, SPRD, MS2, MS1, EN
- pastilles 17, 18 en bas : INDEX, DIAG

Les groupes de signaux sont cohérents entre eux et correspondent bien à un
StepStick (rangée puissance / moteur d'un côté, rangée logique de l'autre),
mais l'ensemble est **tourné de 180°** par rapport à l'orientation
habituelle du module. Électriquement sans effet ; à revérifier contre le
repère de la broche 1 sur la sérigraphie du module réel avant refabrication.

## M7 — Absences diverses

- Pas de LED d'alimentation ni de LED d'état.
- Pas de point de test (GND, 3,3 V, 5 V, BAT).
- `H1`/`H2` utilisent `MountingHole` sans pastille : pas de reprise de masse
  du châssis.
- Pas de connecteur I²C supplémentaire pour capteurs (télémètre, IMU).
- Rien n'avertit qu'il ne faut **jamais débrancher un moteur sous tension** :
  cela détruit un TMC2209.

---

# 🔧 Problèmes de routage (PCB)

Largeurs de pistes relevées dans `carte_moteurs.kicad_pcb` :

| Net | Largeur | Verdict |
|---|---|---|
| Phases moteur (`Net-(J2-Pin_1)` … `Net-(J10-Pin_4)`) | **0,2 mm** | ✘ très insuffisant |
| `5V` | **0,2 mm** | ✘ insuffisant |
| Signaux logiques (STEP, DIR, EN, I²C, série) | 0,2 mm | ✔ |
| `BAT`, `Net-(BAT_GND1-Pin_2)` | 2 mm | ~ acceptable |

## R1 — Les pistes de phases moteur sont en 0,2 mm

C'est le défaut PCB le plus sérieux. Ces seize pistes transportent le
**courant de bobine** : 1,4 A RMS, 2 A crête pour un TMC2209. Une piste de
0,2 mm en cuivre 35 µm tient environ 0,5 A pour 10 °C d'élévation.

Conséquence : échauffement de la piste, chute de tension, couple perdu, et
risque de destruction de la piste à pleine charge.

**Correction :** **≥ 1,0 mm**, idéalement 1,2 mm.

## R2 — `5V` en 0,2 mm

Alimente les quatre `VIO` plus le SuperMini. Le courant reste modeste, mais
0,2 mm sur un rail d'alimentation est fragile.

**Correction :** 0,6 mm minimum.

## R3 — `BAT` en piste plutôt qu'en plan

2 mm passe pour le courant d'entrée, mais un **plan `BAT` sur la couche
inférieure** jusqu'aux quatre `VM` réduirait l'inductance de boucle et le
bruit commuté.

## R4 — Retours de puissance

Prévoir un routage en étoile des retours de puissance vers un point unique
côté connecteur batterie, afin que le courant des bobines ne traverse pas le
plan de masse sous l'ESP32.

---

# Correctifs proposés pour la rev B

## Réaffectation des broches de l'ESP32

Broches évitées : *input-only* (34, 35, 36, 39), *strapping* (0, 2, 12, 15),
flash (6 à 11), console USB (1, 3). Les broches qui émettent un parasite au
boot (5, 14) ne portent que du `DIR`, jamais un `STEP` ni `EN`.

```
STEP     M1=25   M2=27   M3=32   M4=13
DIR      M1=26   M2=33   M3=4    M4=14
EN commun = 19            (+ pull-up 10 kΩ vers 3V3)
OLED I2C  SDA=21  SCL=22
Série ext RX=16   TX=17
Réserve   5, 15
Interdits 12 (strapping), 1 et 3 (console), 6-11 (flash)
```

Si le mode UART est retenu, ajouter :

```
TMC UART  TX=23 (via 1 kΩ série)   RX=18
DIAG     M1=36  M2=39  M3=34  M4=35   (broches input-only, inutilisables autrement)
LED état = 2 (LED bleue déjà présente sur le module)
```

## Mode UART des TMC2209 — ce que cela change

À noter : **le mode UART ne fait gagner aucune broche**, il en coûte deux.
STEP et DIR restent nécessaires pour le mouvement. L'UART sert à la
configuration et au diagnostic.

Ce qu'il apporte :

| | Autonome (rev A) | UART |
|---|---|---|
| Courant moteur | potentiomètre, réglé une fois | `IRUN`, modifiable à chaud |
| Courant à l'arrêt | identique à la marche | `IHOLD` réduit → moins de chauffe, plus d'autonomie |
| CoolStep | non | courant adapté à la charge réelle |
| StallGuard | **non exploitable** (`SGTHRS` inaccessible) | homing sans fin de course, détection de collision |
| StealthChop / SpreadCycle | figé par `SPRD` | bascule selon la vitesse (`TPWMTHRS`) |
| Diagnostic | DIAG en tout-ou-rien | surchauffe, court-circuit, circuit ouvert, présence du driver |

Les deux gains décisifs : **StallGuard** (suppression de fins de course,
détection de blocage) et **`IHOLD`** (un moteur qui tient à pleine puissance
à l'arrêt chauffe et vide la batterie pour rien).

Câblage : les quatre `PDN_UART` reliés ensemble en un seul fil, `TX` via
1 kΩ, `RX` en direct, pull-up 10 kΩ vers 3,3 V. L'adressage se fait par
`MS1`/`MS2` câblés en dur :

| Driver | MS1 | MS2 | Adresse |
|---|---|---|---|
| Moteur 1 | GND | GND | 0 |
| Moteur 2 | 3V3 | GND | 1 |
| Moteur 3 | GND | 3V3 | 2 |
| Moteur 4 | 3V3 | 3V3 | 3 |

Précautions StallGuard : ne fonctionne qu'en StealthChop, dépend fortement
de la vitesse (régler `TCOOLTHRS`, faire le homing à vitesse fixe),
inexploitable à l'arrêt, **incohérent pendant l'accélération** (masquer
`DIAG` pendant ~100 ms après tout changement de consigne, sinon faux
positif à chaque départ), et le seuil dépend de la tension batterie —
calibrer pack chargé **et** déchargé.

Si le choix n'est pas tranché : router le bus vers deux GPIO et ne pas
peupler. Deux pistes et une résistance ne coûtent rien sur une carte à
refaire, et la décision se prend au fer à souder.

## Arrêt d'urgence

Le câblage actuel est correct dans son principe — `J5` coupe bien BAT+ en
amont de tout :

```
BAT_GND1.2 → J5.1 → [bouton] → J5.2 → net BAT → VM des drivers + LM2596
```

Deux améliorations :

- Alimenter le logique **en amont** de la coupure et ajouter un pont
  diviseur vers une entrée ADC : le robot s'arrête, mais l'ESP32 reste
  vivant et *sait* qu'il est en arrêt d'urgence (affichage, télémétrie,
  reprise propre).
- Ajouter un cavalier de contournement pour les essais au banc : aujourd'hui
  si `J5` est débranché, rien ne fonctionne et la cause n'est pas évidente.

## Connecteurs

Les phases moteur sont sur des headers 2,54 mm non détrompés : un moteur
peut être branché à l'envers ou décalé d'une broche. Passer en **JST-XH
4 points détrompés**.

---

# Récapitulatif

| # | Problème | Gravité |
|---|---|---|
| B1 | `A1.VIN` non connectée — le DevKit n'est pas alimenté | 🔴 |
| B2 | `STEP_M1` sur GPIO35, *input-only* — moteur 1 inopérant | 🔴 |
| B3 | `DIR_M3` sur GPIO12, *strapping* — boot aléatoire | 🔴 |
| B4 | `VIO` en 5 V piloté en 3,3 V — seuils logiques non garantis | 🔴 |
| I1 | Pas de pull-up sur `EN` — départ intempestif au reset | 🟠 |
| I2 | `OLED_SCL` sur GPIO18 au lieu de GPIO22 | 🟠 |
| I3 | Symbole DevKit : broche 22 nommée `D32` au lieu de `D33` | 🟠 |
| I4 | Bus série sans résistance série ni pull-up ; `TX2` non câblée | 🟠 |
| I5 | Aucune protection d'entrée batterie | 🟠 |
| I6 | Régulateur réglable sans garde-fou | 🟠 |
| M1 | `C1`–`C4` sans valeur ni tension | 🟡 |
| M2 | `MS1`/`MS2`/`SPRD`/`PDN` flottants | 🟡 |
| M3 | `DIAG` et `INDEX` non connectés | 🟡 |
| M4 | `12V_OUT1` délivre la batterie brute | 🟡 |
| M5 | Références incohérentes, `U4`/`U5` inversés vs moteurs 3/4 | 🟡 |
| M6 | Orientation empreinte TMC2209 à revérifier | 🟡 |
| M7 | Pas de LED, points de test, reprise de masse châssis | 🟡 |
| R1 | Phases moteur en 0,2 mm — il faut ≥ 1,0 mm | 🔴 |
| R2 | `5V` en 0,2 mm | 🟠 |
| R3 | `BAT` en piste plutôt qu'en plan | 🟡 |
| R4 | Retours de puissance non séparés du plan logique | 🟡 |

**À corriger en priorité avant toute refabrication :** B1, B2, B3, B4, I1, R1.
