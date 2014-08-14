all:
	$(MAKE) -C src empfs
	if [ -f src/empfs ]; then cp src/empfs empfs; fi;

clean:
	$(MAKE) -C src clean
	if [ -f ./empfs ]; then rm empfs; fi;
