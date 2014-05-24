/*
 * Variant Symlink Filesystem
*/

#ifndef _PARAMS_H_
#define _PARAMS_H_

// Fuse API version
#define FUSE_USE_VERSION 26

// USe this to get pwrite(). Use setvbuf() instead of setlinebuf().
#define _XOPEN_SOURCE 500

// maintain vsfs state in here
#include <limits.h>
#include <stdio.h>
struct vs_state {
    FILE *logfile;
    char *rootdir;
    char *env_variable;
};
#define VS_DATA ((struct vs_state *) fuse_get_context()->private_data)

#endif
