empfs : empfs.o log.o
	gcc -g -o empfs empfs.o log.o `pkg-config fuse --libs`

empfs.o : empfs.c params.h
	gcc -g -Wall `pkg-config fuse --cflags` -c empfs.c

log.o : log.c log.h params.h
	gcc -g -Wall `pkg-config fuse --cflags` -c log.c

clean:
	rm -f empfs *.o
