## The Variant Symlink Filesystem
======

The variant symlink filesystem is a FUSE filesystem that reads the specified environment variable from a processes `proc/${PID}/environ` file.
The contents of this environment variable should be a full path. This path will then become the root path upon which all requests to the FUSE 
mount point will be appended.

### Building

vsfs requires the following packages in order to build on ubuntu
* libfuse-dev

### Usage
```
vsfs -m mount-point -e ENVIRONMENT_VARIABLE
   
    mount-point           - The directory to which the variant symlink will be applied.
    ENVIRONMENT_VARIABLE  - The environment variable from which the root path will be obtained.
```

### Example

Our examples will use bash as the shell.
<br><br>

Set the environment variable, and re-exec bash to force the */proc/${PID}/environ* file to reload and contain the ROOT_PATH environment variable
```
$ export ROOT_PATH=/home/cole
$ exec bash
$
```
<br>

Show the current listing of /home/cole
```
$ ls
Desktop  Documents  Downloads  lxc-test  Music  Pictures  Public  research  Templates  Videos  work
$
```
<br>

Show the current listing of mountdir (the mountpoint we will use)
```
$ cd mountdir
$ ls
$
```
<br>

Start the Variant Symlink Filesystem and specify the mountpoint and the environment variable to read the path from
```
$ vsfs -m mountdir -e ENVIRONMENT_VARIABLE
$
```
<br>

Change to the mountpoint and show that it is now a variant symlink based on the ROOT_PATH environment variable
```
$ cd mountdir
$ ls
Desktop  Documents  Downloads  lxc-test  Music  Pictures  Public  research  Templates  Videos  work
$
```
<br>

To unmount the filesystem
```
$ cd ~ # or any location not inside the mount point 
$ fusermount -u mountdir
```

### Thanks

I would like to thanks Paul Jolly and his [blog entry about trying to implement variant symlinks using Go](http://blog.myitcv.org.uk/2014/03/18/using-process-namespaces-to-implement-variant-symlinks.html). This gave us the idea of using a FUSE filesystem to solve our problems.

Secondly I would like to thank Joseph J. Pfeiffer for his ["Writing a FUSE Filesystem: a tutorial"](http://www.cs.nmsu.edu/~pfeiffer/fuse-tutorial/). Almost everything I now know about FUSE and implementing a FUSE filesystem came from this tutorial.
