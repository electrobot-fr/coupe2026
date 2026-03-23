PROJECTS = base_roulante carte_actionneurs emetteur pami recepteur telecommande

.PHONY: all clean $(PROJECTS)

all: $(PROJECTS)

$(PROJECTS):
	cd $@ && pio run

upload-%:
	cd $* && pio run --target upload

clean:
	@for p in $(PROJECTS); do cd $$p && pio run --target clean && cd ..; done
