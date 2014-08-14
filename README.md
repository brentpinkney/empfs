## The Environment Variable Mount Point Filesystem (EMPFS)
======

The Environment Variable Mount Point Filesystem (EMPFS) is a FUSE filesystem
that reads the specified environment variable from a process's
`proc/${PID}/environ` file.  The environment variable should contain a full
directory path. This path will then become the mount point to which requests to
the FUSE mount point will be redirected.

### Building

empfs requires the following packages in order to build on Ubuntu:
* libfuse-dev

### Usage
```
empfs -m mount-point -e VARIABLE
   
    mount-point  - The mount directory
    VARIABLE     - The environment variable to the actual directory
```

### Example

Using bash as the shell:
<br><br>

```
$ cd /tmp
$ mkdir wendy; echo "wendy" > wendy/a.txt
$ mkdir peter; echo "peter" > peter/a.txt
$ mkdir floating
```
<br>

Show the current listing of the mount point:
```
$ ls /tmp/floating
```
<br>

Start the EMPFS dæmon, specifying the mountpoint and the environment variable:
```
$ empfs -m /tmp/floating -e FLOAT
```
<br>

Set the environment variable, and re-exec bash to force the
*/proc/${PID}/environ* file to reload pointing to wendy.
```
$ export FLOAT=/tmp/wendy
$ exec bash
$ cat /tmp/floating/a.txt
wendy
```
<br>

Float to the peter directory:
```
$ export FLOAT=/tmp/peter
$ exec bash
$ cat /tmp/floating/a.txt
peter
```
<br>

To unmount the filesystem
```
$ cd                     # anywhere outside /tmp/floating
$ fusermount -u /tmp/floating
```

### Thanks

I would like to thanks Paul Jolly and his [blog entry about trying to implement
variant symlinks using Go]
(http://blog.myitcv.org.uk/2014/03/18/using-process-namespaces-to-implement-variant-symlinks.html).
This gave us the idea of using a FUSE filesystem to solve our problems.

Secondly I would like to thank Joseph J. Pfeiffer for his ["Writing a FUSE
Filesystem: a tutorial"](http://www.cs.nmsu.edu/~pfeiffer/fuse-tutorial/).

Almost everything I now know about FUSE and implementing a FUSE filesystem came
from this tutorial.

