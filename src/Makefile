vsfs : vsfs.o
	gcc -g -o vsfs vsfs.o `pkg-config fuse --libs`

vsfs.o : vsfs.c params.h
	gcc -g -Wall `pkg-config fuse --cflags` -c vsfs.c

clean:
	rm -f vsfs *.o
