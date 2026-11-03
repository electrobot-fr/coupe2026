# Asservissement en cap — base roulante

Notes de travail sur l'asservissement en cap de la base roulante avec le BNO055 :
état actuel, réglages, procédure de test, et pistes d'amélioration.

## État actuel

### Capteur

- **BNO055 en UART** sur le connecteur `J1` de `carte_moteurs` (prévu pour l'OLED).
- Le sens TX/RX est détecté au démarrage : la base essaie RX=GPIO18 / TX=GPIO21, puis inverse si le BNO ne répond pas.
- Côté BNO055 : **PS1 à 3V3, PS0 à GND** (mode UART).
- **Mode IMU** (`0x08`) : gyro + accéléromètre, sans magnétomètre. Le cap 0 correspond à l'orientation du robot au démarrage de la base.

| `J1` | Signal carte | BNO055 |
|---|---|---|
| 1 (pastille carrée) | GND | GND |
| 2 | 3V3 | VIN |
| 3 | GPIO18 | SCL |
| 4 | GPIO21 | SDA |

### Consigne

- La **télécommande** intègre le joystick de rotation (A0) en un **cap absolu**, en degrés [-180, 180[, envoyé dans `message.z`.
- Joystick à fond = 90°/s (`ROT_RATE`), zone morte ±50 autour du centre (`ROT_DEAD_ZONE`).
- Consigne = 0 au démarrage de la télécommande.

### Correcteur (`base_roulante/src/main.cpp`)

- PD à 50 Hz (`CTRL_PERIOD` = 20 ms) sur l'erreur `wrap180(consigne - cap)`. Le robot tourne toujours par le plus court chemin.
- La correction est la vitesse de rotation passée à `move()`. X/Y restent pilotés directement par le joystick, dans le repère du robot.
- Le watchdog (200 ms sans paquet) arrête les moteurs et suspend l'asservissement.
- Si le BNO ne répond pas au démarrage, la rotation reste à 0.

| Constante | Valeur | Rôle |
|---|---|---|
| `KP` | 120 | (steps/s) par degré d'erreur |
| `KD` | 4 | (steps/s) par degré/s, dérivée de l'erreur |
| `ANGLE_DEADBAND` | 1° | pas de correction en dessous |
| `MAX_ROT` | `MAX_SPEED` (3000) | plafond de la correction |
| `ROT_SIGN` | -1 | sens de la correction |

### Retours visuels

- **LED intégrée (GPIO2)** :
  - elle change d'état à chaque cycle d'asservissement réussi, donc un créneau à 25 Hz ;
  - éteinte : pas de paquet ou pas de BNO ;
  - figée pendant que la télécommande envoie : les lectures du BNO échouent.
- **`DEBUG_SERIAL`** (ligne 4) : affiche au format Teleplot `>t`, `>consigne`, `>mesure` (ramenée entre −180° et +180°) et `>reponse`.
  - **Activé dans le commit `c6939f8`** : à commenter pour le match.
  - L'asservissement, et donc le debug, ne tourne que si la télécommande envoie des paquets.

### Problèmes constatés

- **Retour mou** : environ 2 s entre le relâchement du joystick et l'arrêt du mouvement.
- **Saccades** pendant la rotation.
- Essai abandonné : un D calculé sur la mesure, avec une zone morte progressive. Retiré parce qu'il n'apportait pas d'amélioration nette (« bof pas ouf »).

## Procédure de test

### Enregistrer un log

Depuis `base_roulante`, avec `DEBUG_SERIAL` activé :

```bash
PYTHONUNBUFFERED=1 pio device monitor -b 115200 --raw | tee asserv.log
```

- `PYTHONUNBUFFERED=1` est **indispensable**. Sans lui, la sortie de pio reste en mémoire et le log est vide après Ctrl+C.
- Appuie sur EN/RST après avoir lancé la commande, pour avoir l'init dans le log.
- Ne pas commiter `asserv.log`.

### Séquence de mouvements

Robot au sol, télécommande allumée, environ 40 s :

1. **Repos 3 s** sans toucher à rien. Ça donne une référence pour le bruit et les vibrations.
2. **Coup bref à droite** : joystick Z à fond environ 0,5 s, relâche, puis attends 3 s.
3. **Coup bref à gauche**, pareil, puis attends 3 s.
4. **Maintien à fond à droite pendant 3 s**, relâche, puis attends 4 s.
5. **Maintien à fond à gauche pendant 3 s**, relâche, puis attends 4 s.
6. **Joystick à moitié à droite pendant 3 s**, relâche, puis attends 4 s.
7. **Perturbation** : tourne le robot à la main d'environ 45°, lâche-le, puis attends 4 s.
8. **Ctrl+C**

Note à quelle étape apparaît chaque saccade ou dépassement.

### Lire les courbes

- **`reponse` bloquée à ±3000 pendant un mouvement** : le robot ne peut pas tourner plus vite. Baisse `ROT_RATE` dans la télécommande.
- **`reponse` loin du max mais `mesure` en retard** : monte `KP`, ou ajoute l'anticipation (fix B).
- **`reponse` qui fait des marches toutes les 50 ms** : ce sont les paliers de consigne. Fixes A, C et D.
- **Le robot dépasse la consigne ou oscille** : augmente `KD`, ou baisse `KP`.
- **Vibrations à l'arrêt** : `KP` est trop haut pour la mécanique.

## Pistes de correction de l'asservissement

Causes probables, à confirmer par un log :

- **Retard** : la consigne avance à 90°/s pendant que le joystick est tenu. Un correcteur P traîne derrière une consigne qui bouge, et on a pu toucher la limite de vitesse de rotation.
- **Saccades** : la consigne arrive en marches de 50 ms. Le D calculé sur l'erreur réagit à chaque marche par un pic de vitesse.

| # | Fix | Ce que ça corrige | Effort |
|---|---|---|---|
| **A** | **Prendre le D sur le gyro du BNO055** (vitesse de rotation autour de Z, registre 0x18). On lit 0x18 à 0x1B d'un coup (gyro Z + cap), donc ça ne coûte pas plus cher. | Le pic du terme D à chaque palier. Le D devient propre et sans bruit. | Faible |
| **B** | **Anticipation (feedforward)** : la base estime la vitesse de la consigne à partir des paquets et l'envoie directement aux moteurs. Le P ne corrige plus que le petit reste. | Le retard pendant et après un mouvement de joystick. | Moyen |
| **C** | **Lisser la consigne côté base** entre deux paquets (une interpolation, ou une limite de vitesse). | Les à-coups. | Faible |
| **D** | **Envoyer plus souvent** : `SEND_INTERVAL` passe de 50 à 20 ms sur la télécommande. Il faut vérifier que la liaison radio suit. | Des marches plus petites, moins de latence. | Très faible |
| **E** | **Remettre `lastError` à zéro** quand l'asservissement reprend après le watchdog. | Un coup de rotation parasite quand la télécommande se rallume. | Très faible |
| **F** | **Boucle en cascade** : l'angle donne une vitesse en °/s, convertie en steps/s par une constante liée à la géométrie (diamètre des roues, entraxe, micro-pas). | Des gains en unités physiques, et un réglage qui reste valable si le micro-pas change. | Moyen |

**Priorité : A + B + C**, qui peuvent être codés sans le robot. Ensuite, on règle les gains aux essais.

## Améliorations du comportement et du pilotage

- **Pilotage dans le repère du terrain** : on fait tourner (X, Y) du joystick du cap mesuré. « Avant » reste alors toujours le même côté du terrain, quelle que soit l'orientation du robot.
- **Recalage au premier paquet** : aujourd'hui, si la base ou la télécommande redémarre seule, le robot pivote brutalement vers l'ancien 0. La base peut aligner la consigne sur son cap actuel quand la liaison revient.
- **Bouton « zéro cap »** sur la télécommande : il remet la consigne et le cap à 0, par exemple pour se caler contre une bordure. Il faut un champ dans le `STRUCT`, ou une case libre de `compteur[]`.
- **Limiter l'avance de la consigne** côté base : au-delà de 180° d'avance sur le robot, celui-ci repart dans l'autre sens. Plafonner la vitesse de la consigne suivie évite ce cas.

## Choix de l'organe de commande du cap

La question à se poser : est-ce que le pilote commande une **vitesse de rotation** ou une **orientation** ?

| Organe | Ce qu'il commande | + | − |
|---|---|---|---|
| **Joystick** (actuel) | Une vitesse de rotation, que la télécommande intègre | Intuitif pour « tourner un peu », déjà câblé | La consigne court pendant qu'on le tient et prend de l'avance sur le robot ; difficile de viser un angle précis ; légère dérive si le centre bouge |
| **Potentiomètre** | Un angle absolu | Simple : une position du bouton = un cap | Course limitée à environ 270°, donc pas de tour complet ; ou 270° étirés sur 360°, et ce n'est plus du 1:1 |
| **Codeur rotatif à crans** (type KY-040) | Des pas d'angle (ex. 1 cran = 15°) | Tours infinis, précis, reproductible, aucune dérive ; le bouton-poussoir intégré peut servir de zéro cap | Ne dit pas où il « pointe » ; il faut compter les crans |
| **Bouton magnétique absolu** (AS5600 + aimant) | Un angle absolu sur 360° | Vrai 1:1 : le robot regarde là où pointe le bouton, tours infinis | Un aimant à fixer sur l'axe ; I²C |

### Recommandation : le codeur rotatif à crans

- Il permet de se mettre à 0°, 90° ou 180° vite et juste.
- Il n'y a plus de consigne qui prend de l'avance.
- Le zéro cap est gratuit, grâce au bouton-poussoir intégré.
- Il coûte environ 1 €.

**Broches libres sur la télécommande (Uno)** :
- **A1** : déclarée en entrée mais jamais lue ;
- **A4 et A5** : libres, et ce sont aussi les broches I²C pour un AS5600 ;
- D2 et D3 (les entrées d'interruption) sont prises par l'afficheur TM1637. On lit donc le codeur en scrutation ou par interruption sur changement de broche.

### Variantes en gardant le joystick

- **Joystick + boutons ±90°** : le joystick pour l'ajustement fin, deux boutons pour les quarts de tour.
- **Joystick en position** : la direction du joystick donne le cap, comme dans un jeu vidéo à deux sticks. Il faut un joystick 2 axes dédié et un seuil.

**Changer d'organe ne règle pas tout côté base.** Avec des crans, la consigne fait des sauts nets. Il faut quand même les fixes A et C, sinon chaque cran donne un à-coup.

## Robustesse du BNO055

- **Lectures UART bloquantes** : jusqu'à 100 ms par échec, ce qui fige toute la boucle, moteurs compris. On peut réduire le délai d'attente (une trame dure environ 1 ms), réessayer une fois, puis réinitialiser le BNO après N échecs.
- **Vérifier le calibrage** (registre 0x35, `CALIB_STAT`) : le gyro doit être calibré, robot immobile au démarrage. On peut afficher l'état sur la LED pendant l'init.
- **Dérive du mode IMU** (sans magnétomètre) : quelques degrés par minute, acceptable pour un match.

## Multithread (FreeRTOS) ?

**Pas pour l'instant.** Le mou et les saccades viennent de la loi de commande, pas d'un manque de temps de calcul.

### Ce qui tourne déjà en parallèle

- **La génération des pas moteurs** : FastAccelStepper la confie au matériel de l'ESP32 et à sa propre tâche. Même si `loop()` ralentit, les pas restent réguliers.
- **`loop()`** : un paquet SerialTransfer à décoder, une lecture du BNO d'environ 1 ms toutes les 20 ms, quelques calculs. Le processeur (240 MHz, 2 cœurs) est occupé à moins de 10 %.

### Ce que ça apporterait

| Gain | Ça compte ici ? |
|---|---|
| **Une période d'asservissement exacte** (`vTaskDelayUntil`) | Peu. La boucle n'est pas bloquante, la variation est d'environ 1 ms. À vérifier avec `>t:` dans le log. |
| **Le BNO ne bloque plus la réception des paquets** | Oui, mais seulement en cas d'échec de lecture : jusqu'à 100 ms de blocage, pour un watchdog à 200 ms. |
| **Lire le BNO à 100 Hz**, indépendamment de l'asservissement | Faible gain. |

### Ce que ça coûterait

- **Des variables partagées entre tâches** (cap, gyro, consigne) : il faut un mutex ou une section critique. Sinon, on lit des valeurs à moitié écrites, et ces bugs sont intermittents.
- **FastAccelStepper ne doit être appelé que depuis une seule tâche.**
- **`Serial1` réservé à une seule tâche.**
- **Un débogage plus dur** pendant le réglage de l'asservissement.

### L'alternative, plus simple : lecture BNO non bloquante

1. Au cycle N, on envoie la requête `0xAA 0x01 0x18 0x04` et on repart aussitôt.
2. Au cycle N+1, on lit la réponse si elle est arrivée. Sinon, on garde la dernière valeur et on compte une erreur.

La donnée a un cycle de retard (20 ms), ce qui est négligeable. On n'a plus aucun blocage, et pas de mutex.

### Ordre de travail

1. **A + B + C** : ça règle le comportement.
2. **La lecture BNO non bloquante** : ça règle la robustesse.
3. **Le multithread** : seulement si `>t:` montre une période irrégulière, ou si on ajoute des traitements lourds sur la base (odométrie, écran…).

## Système global

- **`STRUCT` partagé** : il est copié dans chaque projet. `ecureuil/src/main.cpp:162` lit encore `message.z` comme un joystick brut (0–1023), donc il est déjà incohérent avec la consigne d'angle. Un en-tête commun (`lib/` partagé via `lib_extra_dirs`) plus un octet de version dans le paquet éviteraient ces décalages.
- **Latence** télécommande → émetteur → ESP-NOW → récepteur → base : à mesurer au moins une fois. Le watchdog de 200 ms ne laisse passer que 4 paquets perdus à 50 ms.
- **Carte moteurs, révision B** (voir `carte_moteurs/REVUE_REV_A.md`) :
  - **tirage de `EN` vers `VIO`** pour que les drivers soient désactivés par défaut. C'est un point de sécurité : aujourd'hui, le robot peut démarrer tout seul à la mise sous tension ;
  - `OLED_SCL` sur GPIO22 au lieu de GPIO18 ;
  - un vrai connecteur dédié à l'IMU au lieu de celui de l'OLED.
- **Hygiène** :
  - couper `DEBUG_SERIAL` avant le match ;
  - ajouter `.vscode/` et `*.log` au `.gitignore` ;
  - supprimer l'une des deux installations de PlatformIO (6.1.18 et 6.2.0).
