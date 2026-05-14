Coupe de France de Robotique Junior 2026

## Compilation

Chaque sous-dossier est un projet PlatformIO indépendant. Le `Makefile` à la racine fournit des raccourcis.

### Cartes standards

```bash
make base_roulante              # compiler
make upload-base_roulante       # compiler + flasher
make all                        # compiler toutes les cartes (sauf PAMI)
make clean                      # nettoyer toutes les cartes
```

Cartes disponibles : `base_roulante`, `carte_actionneurs`, `emetteur`, `recepteur`, `telecommande`.

### PAMI (5 variantes)

Le code du PAMI est commun mais le comportement est sélectionné à la compilation par le flag `-DPAMI_VARIANT=N`. Une cible Makefile existe par variante :

| Cible | Variante | Description |
|---|---|---|
| `make pami-ninja` | NINJA | Séquence longue avec accélérations/décélérations |
| `make pami-1` | PAMI1 | Séquence courte (2 virages) |
| `make pami-2` | PAMI2 | Délai 6s puis avance + virage |
| `make pami-3` | PAMI3 | Délai 3s puis longue avance |
| `make pami-4` | PAMI4 | Délai 8.5s puis ligne droite |

Pour flasher, préfixer par `upload-` :

```bash
make upload-pami-ninja
make upload-pami-1
# etc.
```

⚠️ Compiler directement avec `cd pami && pio run` échoue : `PAMI_VARIANT` n'est pas défini et le code lève un `#error`. Toujours passer par le Makefile.

### Ajouter une nouvelle variante de PAMI

1. Ajouter un `#define` dans `pami/src/main.cpp` (ex. `#define PAMI5 5`)
2. Ajouter le bloc `#if PAMI_VARIANT == PAMI5 ... #endif` avec la séquence
3. Ajouter la variante dans le `Makefile` :
   - Ajouter `5` à `PAMI_VARIANTS`
   - Ajouter `PAMI_FLAG_5 = 5`
