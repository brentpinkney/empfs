major := 0
version := $(major).0.0
name := empfs
pkg := $(name).deb

install: install-stamp

install-stamp: uninstall $(pkg) 
	sudo dpkg -i $(pkg)
	touch $@

uninstall:
	sudo dpkg -r $(name)
	rm -f install-stamp

package: $(pkg)

$(pkg): control debian-binary prerm postinst empfs
	mkdir -m 755 -p tmp tmp/DEBIAN tmp/usr/bin
	cp -f control       tmp/DEBIAN
	cp -f debian-binary tmp/DEBIAN
	cp -f prerm         tmp/DEBIAN
	cp -f postinst      tmp/DEBIAN
	chmod +x            tmp/DEBIAN/prerm
	chmod +x            tmp/DEBIAN/postinst
	cp -f empfs         tmp/usr/bin
	chmod 750           tmp/usr/bin/empfs
	dpkg-deb --build tmp $(pkg)

empfs : empfs.o log.o
	gcc -g -o empfs empfs.o log.o `pkg-config fuse --libs`

%.o: %.c params.h log.h
	gcc -g -Wall `pkg-config fuse --cflags` -c $<

%: %-preedit
	sed 's/@VERSION@/$(version)/g' $< > $@

clean:
	rm -f empfs control prerm postinst *.o
	rm -rf tmp *.deb *-stamp *~

