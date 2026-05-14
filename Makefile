PROJECTS = base_roulante carte_actionneurs emetteur recepteur telecommande
PAMI_VARIANTS = ninja 1 2 3 4

# Map variant name -> PAMI_VARIANT macro value (must match #defines in pami/src/main.cpp)
PAMI_FLAG_ninja = 0
PAMI_FLAG_1     = 1
PAMI_FLAG_2     = 2
PAMI_FLAG_3     = 3
PAMI_FLAG_4     = 4

# Default DECOMPTE_INITIAL (seconds) per variant. Override on command line: `make pami-1 DECOMPTE=30`
DECOMPTE_ninja = 5
DECOMPTE_1     = 87
DECOMPTE_2     = 87
DECOMPTE_3     = 87
DECOMPTE_4     = 87

.PHONY: all clean $(PROJECTS)

all: $(PROJECTS)

$(PROJECTS):
	cd $@ && pio run

upload-%:
	cd $* && pio run --target upload

# Generate one build + upload target per PAMI variant
define PAMI_RULE
.PHONY: pami-$(1) upload-pami-$(1)
pami-$(1):
	cd pami && PLATFORMIO_BUILD_FLAGS="-DPAMI_VARIANT=$$(PAMI_FLAG_$(1)) -DDECOMPTE_INITIAL=$$(or $$(DECOMPTE),$$(DECOMPTE_$(1)))" pio run
upload-pami-$(1):
	cd pami && PLATFORMIO_BUILD_FLAGS="-DPAMI_VARIANT=$$(PAMI_FLAG_$(1)) -DDECOMPTE_INITIAL=$$(or $$(DECOMPTE),$$(DECOMPTE_$(1)))" pio run --target upload
endef
$(foreach v,$(PAMI_VARIANTS),$(eval $(call PAMI_RULE,$(v))))

clean:
	@for p in $(PROJECTS) pami; do cd $$p && pio run --target clean && cd ..; done
