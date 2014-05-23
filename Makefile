all:
	$(MAKE) -C src vsfs
	if [ -f src/vsfs ]; then cp src/vsfs vsfs; fi;

clean:
	$(MAKE) -C src clean
	if [ -f ./vsfs ]; then rm vsfs; fi;
