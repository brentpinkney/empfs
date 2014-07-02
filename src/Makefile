vsfs : vsfs.o log.o
	gcc -g -o vsfs vsfs.o log.o `pkg-config fuse --libs`

vsfs.o : vsfs.c params.h
	gcc -g -Wall `pkg-config fuse --cflags` -c vsfs.c

log.o : log.c log.h params.h
	gcc -g -Wall `pkg-config fuse --cflags` -c log.c

clean:
	rm -f vsfs *.o
