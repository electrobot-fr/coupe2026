# Carte PAMI — revue de conception

**Version analysée :** rev A (état des cartes fabriquées)
**Fichiers :** `pami.kicad_sch`, `pami.kicad_pcb` — KiCad 9.0
**Date de la revue :** 2026-09-20
**Statut de la carte :** fonctionnelle. Aucun des points ci-dessous n'est bloquant *aujourd'hui* ;
plusieurs sont en revanche marginaux, c'est-à-dire qu'ils passent sur l'établi et peuvent lâcher en match.

Le netlist utilisé pour cette revue a été extrait de `pami.kicad_pcb` (netlist calculé par KiCad),
pas du schéma, afin de refléter exactement ce qui a été fabriqué.

---

## 1. Rappel de l'architecture

### Arbre d'alimentation

```
J3 (LiPo 2S "LP E10" 7,4 V)
  └─ J5 (arrêt d'urgence, NC)
       └─ net BAT (7,4–8,4 V)
            ├─ C1, C2 (réservoirs)
            ├─ U2.8  VM  (TMC2209 moteur 1)
            ├─ U3.8  VM  (TMC2209 moteur 2)
            └─ VR1 IN+ (LM2596)
                 └─ net 5V
                      ├─ U1.5V  (ESP32-C3 SuperMini)
                      ├─ U2.2  VIO   <-- voir §2.1
                      ├─ U3.2  VIO   <-- voir §2.1
                      ├─ J7.2 (servo 1)
                      └─ J8.2 (servo 2)

U1.3V3 ── J1.2 (écran OLED)   [seule charge du 3,3 V]
```

Conséquence à noter : **l'arrêt d'urgence coupe aussi l'ESP32.** Conforme au règlement,
mais le robot reboote au relâchement. À confirmer comme choix volontaire.

### Correspondance des références

Les empreintes du PCB ont été renommées, elles ne portent pas les mêmes références que le schéma.

| Schéma | PCB | Fonction |
|---|---|---|
| J1 | `Ecran` | écran OLED I2C (4 pts) |
| J2 | `J2` | moteur pas-à-pas 1 |
| J3 | `Batterie` | LiPo 2S 7,4 V |
| J4 | `J4` | moteur pas-à-pas 2 |
| J5 | `Arret Urgence` | bouton d'arrêt d'urgence |
| J6 | `Tirette` | tirette de départ |
| J7 | `Servo` | servo 1 (GPIO5) |
| J8 | `Servo` | servo 2 (GPIO6) |

⚠️ **Deux empreintes portent la référence `Servo`.** KiCad s'y retrouve par UUID, mais toute
réannotation ou tout export de BOM posera problème. Ces noms doivent aller dans le champ *Value*
ou en texte de sérigraphie, pas dans *Reference*.

### Affectation des GPIO (ESP32-C3)

| GPIO | Usage | Remarque |
|---|---|---|
| 0 | `~EN` de U2 **et** U3 | enable commun aux deux drivers |
| 1 | STEP moteur 2 (U3.10) | |
| 2 | DIR moteur 2 (U3.9) | ⚠️ **broche de strapping** — §2.2 |
| 3 | STEP moteur 1 (U2.10) | |
| 4 | DIR moteur 1 (U2.9) | |
| 5 | signal servo 1 (J7.3) | |
| 6 | signal servo 2 (J8.3) | |
| 7 | — | libre |
| 8 | I2C écran (J1.4) | ⚠️ **strapping** + LED embarquée du SuperMini — §2.3 |
| 9 | I2C écran (J1.3) | ⚠️ **strapping boot** — §2.3 |
| 10 | tirette (J6.1) | |
| 20, 21 | — | libres (UART0 console) |

Broches TMC2209 non câblées sur les deux drivers : `MS1`, `MS2`, `SPRD`, `PDN`, `UART`, `INDEX`, `DIAG`.
Les drivers tournent donc en mode *standalone* step/dir, courant réglé au potentiomètre.

---

## 2. Problèmes critiques

### 2.1 — VIO des TMC2209 alimenté en 5 V alors que l'ESP32 sort du 3,3 V

**C'est le point le plus important de cette revue.**

`U2.2` et `U3.2` (VIO) sont sur le net `5V`. Or le TMC2209 référence **tous** ses seuils d'entrée
logique sur VIO :

```
V_IH = 0,7 × VIO = 0,7 × 5 V = 3,5 V
```

L'ESP32-C3 sort au maximum 3,3 V. **Les niveaux hauts de STEP, DIR et EN sont donc sous le seuil
garanti.** C'est le grand classique du SilentStepStick câblé sur un MCU 3,3 V.

Symptômes attendus : pas perdus erratiques, sens de rotation aléatoire, aggravation à chaud —
et **aucun message d'erreur**, puisque rien ne détecte la faute.

Que ça marche aujourd'hui ne contredit pas le diagnostic : on est dans la marge non garantie
du composant, qui dérive avec la température et d'un exemplaire à l'autre.

**Correction :** VIO → 3V3. La broche `3.3` de U1 est déjà disponible et n'alimente que l'écran.
Le courant VIO d'un TMC2209 est de quelques mA, le LDO du SuperMini (500 mA) absorbe ça sans problème.

### 2.2 — GPIO2 (DIR moteur 2) est une broche de strapping

Sur l'ESP32-C3, **GPIO2 doit être au niveau haut au moment du reset** et ne possède pas de
pull-up interne par défaut. Les entrées du TMC2209 ont un pull-down interne.

Risque : non-démarrage intermittent de l'ESP32.

**Correction :** pull-up 10 kΩ vers 3V3 sur GPIO2, ou déplacer DIR sur **GPIO7** (libre).
Pour la rev B, GPIO7 est la bonne réponse.

### 2.3 — I2C de l'écran sur GPIO8 + GPIO9, les deux autres broches de strapping

- **GPIO9 bas au reset = mode download.** Le robot ne démarre pas.
- **GPIO8 ne doit pas être tiré bas au boot.**
- De plus, **la LED embarquée du SuperMini est sur GPIO8** (son réseau agit comme un pull-up faible).

Le scénario de panne concret : un bus I2C qui se bloque avec SDA maintenu à 0 (cas classique
quand on reset le MCU au milieu d'une transaction) empêche le redémarrage. Idem si le module
écran est débranché, les deux lignes flottent.

**Correction :**
- Court terme : pull-ups **4,7 kΩ vers 3V3 sur la carte** — ne pas dépendre de ceux du module OLED.
- Rev B : déplacer l'I2C hors des broches de strapping (GPIO7 / GPIO20 / GPIO21).

### 2.4 — Aucun découplage sur le rail 5 V

`C1` et `C2` sont tous les deux sur le net `BAT`. **Le 5 V n'a aucun réservoir**, alors qu'il
alimente simultanément l'ESP32 et les deux servos.

Un servo en butée tire un pic de plusieurs ampères → chute du 5 V → brown-out du MCU
en pleine phase de match.

**Correction :** 470–1000 µF / 16 V + 100 nF sur le 5 V, au plus près des connecteurs servo.

---

## 3. Problèmes de routage (PCB)

### 3.1 — Tout le net 5 V est routé en pistes de 0,2 mm

Relevé dans `pami.kicad_pcb` : **100 % des segments du net `5V` font 0,2 mm de large**
(le net `BAT`, lui, est correctement en 2 mm).

0,2 mm en cuivre 35 µm ≈ **0,5 A maximum** (IPC-2221, élévation 10 °C). Ce rail alimente
l'ESP32 + les deux VIO + **les deux servos**. C'est très probablement le facteur limitant
réel de la carte aujourd'hui, avant même le problème de VIO.

Topologie relevée (tout est en guirlande depuis VR1) :

```
VR1 OUT+ (174,85 ; 53,35)
  → (177,5 ; 56) → (177,5 ; 73)
      ├─ (179,5 ; 73) → (187,5 ; 81) → ... → servos
      └─ (176,4 ; 74,1) → (169,5 ; 74,1)
             ├─ (169,35 ; …) → (167 ; 96,66)  U1.5V
             └─ (154,1 ; 74,1) ─┬─ (153,6 ; 74,1) → (148,65 ; 79,04)  U3 VIO
                                └─ (153,5 ; 74,1) → (132,2 ; 74,1) → (127,3 ; 79,04)  U2 VIO
```

Note utile pour le rework : les deux pads VIO sont des **feuilles** de l'arbre, pas des maillons.
Les isoler ne coupe l'alimentation de rien d'autre.

**Correction rev B :** 5 V en 0,8–1 mm minimum, et alimentation des servos en étoile depuis VR1
plutôt qu'en guirlande derrière l'ESP32.

### 3.2 — Plans de masse

Zones `GND` sur `F.Cu` et `B.Cu`, carte 2 couches. Rien à signaler.

---

## 4. Problèmes sérieux

### 4.1 — C1 et C2 n'ont pas de valeur

Le champ *Value* des deux condensateurs est resté à `C_Polarized`. **La BOM est inexploitable.**

Spécification nécessaire : ≥ 100 µF par driver, **16 V minimum** (une LiPo 2S chargée fait 8,4 V ;
un 10 V est trop juste). L'empreinte `CP_Radial_D8.0mm_P5.00mm` correspond à du 100–220 µF.

Polarité vérifiée : `C1.1`/`C2.1` (+) sur `BAT`, `C1.2`/`C2.2` sur `GND`. Correct.

### 4.2 — MS1 et MS2 flottants sur les deux drivers

Sur le TMC2209, `MS1`/`MS2` sélectionnent la résolution de micropas. Laissés en l'air, on hérite
du **1/8 de pas** via les pull-downs internes — valeur non choisie, et deux pistes flottantes
sensibles au bruit juste à côté des sorties de puissance.

**Correction :** les tirer explicitement à GND ou VIO (straps, jumpers, ou résistances).

### 4.3 — Pas d'état sûr garanti sur EN au démarrage

`GPIO0` pilote `~EN` des deux drivers. Au reset, l'ESP32 place ses GPIO en haute impédance :
l'état des drivers dépend alors uniquement du pull-up interne du TMC2209.

**Correction :** pull-up 10 kΩ explicite de EN vers VIO → moteurs garantis désactivés
tant que le firmware n'a pas pris la main. C'est une mesure de sécurité, pas une optimisation.

### 4.4 — Aucune protection sur l'entrée batterie

Ni fusible, ni anti-inversion. Une LiPo branchée à l'envers détruit VR1 et les deux drivers.

Atténuation : le JST-XH est détrompé, donc le risque réel se limite à un câble mal serti.

**Correction :** fusible en ligne dans le fil batterie (hors carte, immédiat).
Rev B : MOSFET P en série pour l'anti-inversion.

### 4.5 — Alimenter la broche 5V du SuperMini depuis le LM2596

Sur ces modules clones, il n'y a généralement **pas de diode** entre la broche `5V` et le VBUS USB.
Brancher l'USB pour flasher alors que la batterie est allumée met le LM2596 et le port USB
du PC en opposition.

**Correction :** Schottky (SS34) en série VR1 → U1.5V. En attendant : jamais USB et batterie
en même temps.

### 4.6 — Budget courant des connecteurs

Tout le courant du robot passe par un **JST-XH 2 points** (3 A par contact) et par le contact
du bouton d'arrêt d'urgence. Deux steppers plus deux servos en pic, c'est juste.
À vérifier avec les courants réellement réglés sur les potentiomètres des drivers.

---

## 5. Points mineurs

- **Pas d'interrupteur ON/OFF** distinct de l'arrêt d'urgence.
- **Tirette sans pull-up externe ni filtrage** : simple contact vers GND sur GPIO10. Le pull-up
  interne suffit, mais sur un fil long un RC (10 kΩ + 100 nF) évite les faux départs.
- **La broche `3V3` du symbole ESP32 est typée `power_in`** : l'ERC signalera un conflit
  quand elle alimente l'écran. Cosmétique.
- **Mapping des bobines moteur** : `J2`/`J4` → `M1B`/`M1A`/`M2A`/`M2B` sur les pins 1/2/3/4,
  identique pour les deux moteurs. Correct et cohérent.
- **Brochage servo** : pin 1 = GND, pin 2 = 5 V, pin 3 = signal. Conforme au standard. Correct.
- **PDN/UART non câblés** → mode standalone. Acceptable pour un PAMI, mais on perd StallGuard
  et le réglage du courant par logiciel.

---

## 6. Procédure de rework sur les cartes existantes

Environ 30 minutes, entièrement réversible. Coordonnées relevées dans `pami.kicad_pcb` (en mm).

### ① VIO → 3V3  *(indispensable)*

**Ne couper aucune piste.** Les TMC2209 sont sur supports : déplier la **broche 2 (VIO)** de
chaque module pour qu'elle n'entre pas dans le connecteur, puis câbler en fil à wrapper :

| De | Vers | Longueur |
|---|---|---|
| `U1` pad `3.3` (167,00 ; 91,58) | `U3` VIO (148,65 ; 79,04) | ≈ 22 mm |
| `U3` VIO | `U2` VIO (127,30 ; 79,04) | ≈ 21 mm |

⚠️ Sur les deux modules, **le pad 1 est GND, à 2,54 mm juste au-dessus du pad VIO.**
Attention au pont de soudure. Et vérifier que le fil est réellement soudé : VIO en l'air = driver muet.

### ② Renfort du rail 5 V

Fil de 0,5 mm² : `VR1` OUT+ (174,85 ; 53,35) → pad `5V` du connecteur servo (187,50 ; 93,54).

### ③ Réservoir sur le 5 V

Chimique 470–1000 µF / 16 V soudé **directement en travers du connecteur servo** :
pad 1 GND (187,50 ; 91,00) et pad 2 5V (187,50 ; 93,54) — pas de 2,54 mm, ça tombe pile.

### ④ Pull-up 10 kΩ GPIO0 → 3V3

Moteurs désactivés au boot. Les deux pads sont dans la même colonne au dos de `U1` :
(167,00 ; 78,88) et (167,00 ; 91,58).

### ⑤ MS1 / MS2  *(optionnel)*

Si le 1/8 de pas convient, laisser tel quel. Sinon, fil vers le plan de masse depuis
`U2` MS1 (140,00 ; 91,74) et MS2 (140,00 ; 89,20), `U3` MS1 (161,35 ; 91,74) et MS2 (161,35 ; 89,20).

### ⑥ Pull-up 10 kΩ GPIO2 → 3V3  *(seulement si besoin)*

(167,00 ; 83,96) → (167,00 ; 91,58). À faire uniquement en cas de non-démarrage constaté.

### Si un seul rework

Faire le **①**.

---

## 7. Checklist rev B

- [ ] `VIO` des deux TMC2209 sur **3V3**, plus sur 5 V
- [ ] Pistes du net `5V` en **0,8–1 mm** minimum
- [ ] Alimentation des servos **en étoile** depuis VR1, pas en guirlande derrière l'ESP32
- [ ] **470–1000 µF / 16 V + 100 nF** sur le 5 V près des servos
- [ ] `DIR` moteur 2 déplacé de **GPIO2 vers GPIO7**
- [ ] I2C écran sorti des broches de strapping (GPIO20 / GPIO21), sinon **pull-ups 4,7 kΩ** sur la carte
- [ ] **Pull-up 10 kΩ sur EN** vers VIO (état sûr au boot)
- [ ] `MS1` / `MS2` tirés explicitement (GND ou VIO)
- [ ] **Valeurs de C1 et C2 renseignées** (≥ 100 µF / 16 V)
- [ ] **Schottky SS34** en série VR1 → U1.5V (protection USB)
- [ ] **MOSFET P anti-inversion** + emplacement fusible sur l'entrée batterie
- [ ] **RC sur l'entrée tirette** (10 kΩ + 100 nF)
- [ ] Références PCB dupliquées `Servo` corrigées → noms dans *Value* ou en sérigraphie
- [ ] 100 nF de découplage sur chaque rail
- [ ] Décision assumée sur le périmètre de l'arrêt d'urgence (coupe-t-il la logique ?)
