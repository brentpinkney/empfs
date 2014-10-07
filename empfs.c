/*
 * The Environment Variable Mount Point Filesystem (EMPFS)
 *
 * Copyright (c) 2014, Cole Minnaar and ﬀrøøt limited (ffroot)
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. All advertising materials mentioning features or use of this software
 *     must display the following acknowledgement:
 *     This product includes software developed by Cole Minnaar and ffroot.
 *  4. Neither the name ffroot nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without specific
 *     prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY COLE MINNAAR ''AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 *  NO EVENT SHALL COLE MINNAAR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include "params.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <fuse.h>
#include <libgen.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/xattr.h>

#include "log.h"

void log_enter_function( const char * const function )
{
	log_msg( "ENTER %s\n", function );
	VS_DATA->tab_count += 1;
}

void log_leave_function( const char * const function )
{
	VS_DATA->tab_count -= 1;
	log_msg( "LEAVE %s\n", function );

	if( VS_DATA->tab_count == 0 )
	{
		log_msg( "\n" );
	}
}

// Report errors to logfile and give -errno to caller
static int empfs_error( char * const str )
{
	log_msg( "ERROR %s: %s\n", str, strerror( errno ) );
	return -errno;
}

// Get the parent pid using /proc/${PID}/status
pid_t get_parent_pid( pid_t pid )
{
	log_enter_function( "get_parent_pid" );

	char   environ[ PATH_MAX ];
	FILE * fl = NULL;
	long   fl_size = 500;
	char * buffer = NULL;
	size_t res = 0;
	long   total = 0;
	char * find_var = "PPid";
	size_t length = strlen( find_var );
	pid_t  ppid = 0;

	snprintf( environ, sizeof( environ ), "/proc/%d/status", (int) pid );
	printf( "environ = [%s]\n", environ );

	fl = fopen( environ, "r" );
	if( fl == NULL )
	{
		fprintf( stderr, "Error opening %s\n", environ );
		exit( 1 );
	}

	buffer = (char *) malloc( sizeof( char ) * fl_size );
	if( buffer == NULL )
	{
		fprintf( stderr, "Error not enough memory\n" );
		exit( 2 );
	}

	while( !feof( fl ) )
	{
		res = fread( buffer + total, 1, fl_size - total, fl );
		total += res;

		printf( "fl_size = %ld\n", fl_size );
		printf( "total = %ld\n", total );

		if( total == fl_size )
		{
			fl_size = fl_size * 2;
			buffer = (char *) realloc( buffer, sizeof( char ) * fl_size );
			if( buffer == NULL )
			{
				fprintf( stderr, "Error not enough memory\n" );
				exit( 3 );
			}
		}
	}

	buffer[total] = 0;
	const char * delim = "\n";
	char * save;
	char * p;

	for( p = strtok_r( buffer, delim, &save ); p; p = strtok_r( NULL, delim, &save ) )
	{
		if( strncmp( find_var, p, strlen( find_var ) ) == 0 )
		{
			ppid = (pid_t) atoi( p + length + 1 );
			break;
		}
	}

	fclose( fl );
	free( buffer );

	log_leave_function( "get_parent_pid" );
	return ppid;
}

// Take a PID and examine its /proc/pid/environ file searching for the matching
// environment parameter. If found, the value is copied to the variable and
// returned.
static int get_env_variable( char variable[ PATH_MAX ], pid_t pid )
{
	log_enter_function( "get_env_variable" );

	char    environ[ PATH_MAX ];
	FILE *  fl = NULL;
	long    fl_size = 4096;
	char *  buffer  = NULL;
	char *  next_var = NULL;
	char *  find_var = VS_DATA->env_variable;
	size_t  res   = 0;
	long    total = 0;
	int     found = 0;
	static size_t length = 0;

	if( length == 0 )
	{
		length = strlen( find_var );
	}

	snprintf( environ, sizeof( environ ), "/proc/%d/environ", (int) pid );

	fl = fopen( environ, "r" );
	if( fl == NULL )
	{
		log_msg( "Error opening %s\n", environ );
		fprintf( stderr, "Error opening %s\n", environ );
		log_leave_function( "get_env_variable" );
		return found;
	}

	buffer = (char*) malloc( sizeof( char ) * fl_size );
	if( buffer == NULL )
	{
		log_msg( "Error not enough memory\n" );
		fprintf( stderr, "Error not enough memory\n" );
		exit( 2 );
	}

	while( !feof( fl ) )
	{
		res = fread( buffer + total, 1, fl_size - total, fl );
		total += res;

		if( total == fl_size )
		{
			fl_size = fl_size * 2;
			buffer = (char*) realloc( buffer, sizeof( char ) * fl_size );
			if( buffer == NULL )
			{
				fprintf( stderr, "Error not enough memory\n" );
				exit( 3 );
			}
		}
	}
	buffer[ total ] = 0;
	next_var = buffer;

	while( next_var < ( buffer + total ) )
	{
		if( strncmp( next_var, find_var, length ) == 0 )
		{
			strncpy( variable, next_var + length + 1, PATH_MAX );
			found = 1;
			break;
		}
		next_var += strlen( next_var ) + 1;
	}

	fclose( fl );
	free( buffer );

	log_leave_function( "get_env_variable" );
	return found;
}

// 1. Get the environment variable from /proc/PID/environ
// 2. Append the path received from fuse to the value from the variable.
// 3. Return 1 if the variable was found.
static int empfs_fullpath( char fpath[ PATH_MAX ], const char * const path )
{
	log_enter_function( "empfs_fullpath" );

	char  variable[ PATH_MAX ];
	pid_t ppid = 0;
	pid_t pid  = fuse_get_context( )->pid;
	int   found = 0;

	memset( variable, 0, sizeof( char ) * PATH_MAX );

	log_msg( "empfs_fullpath\n" );
	log_msg( "path: %s\n", path );
	log_msg( "pid: %d\n", pid );
	
	ppid = get_parent_pid( pid );
	log_msg( "ppid: %d\n", ppid );
	if( ppid != 0 )
	{
		found = get_env_variable( variable, ppid );
		log_msg( "parent variable: %s\n", found ? variable : "(not found)" );
	}

	if( found == 0 )
	{
		found = get_env_variable( variable, pid );
		log_msg( "variable: %s\n", found ? variable : "(not found)" );
	}

	if( found )
	{
		strcpy( fpath, variable );
		strncat( fpath, path, PATH_MAX );
	}

	log_leave_function( "empfs_fullpath" );
	return found;
}

// All the prototype functions come from fuse.h
// Most of them are nothing more than wrappers around the normal functions

/** Get file attributes.
 *
 * Similar to stat().  The 'st_dev' and 'st_blksize' fields are
 * ignored.  The 'st_ino' field is ignored except if the 'use_ino'
 * mount option is given.
 */
int empfs_getattr( const char * path, struct stat * statbuf )
{
	log_enter_function( "empfs_getattr" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = lstat( fpath, statbuf );
		if( retstat != 0 )
		{
			retstat = empfs_error( "empfs_getattr lstat" );
		}
	}
	log_leave_function( "empfs_getattr" );
	return retstat;
}

/** Read the target of a symbolic link
 *
 * The buffer should be filled with a null terminated string.  The
 * buffer size argument includes the space for the terminating
 * null character.  If the linkname is too long to fit in the
 * buffer, it should be truncated.  The return value should be 0
 * for success.
 */

// System readlink() will lose the terminating null because it truncates it.
// So pass size - 1 to system readlink.
int empfs_readlink( const char * path, char * link, size_t size )
{
	log_enter_function( "empfs_readlink" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = readlink( fpath, link, size - 1 );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_readlink readlink" );
		}
		else
		{
			link[ retstat ] = 0;
			retstat = 0;
		}
	}
	log_leave_function( "empfs_readlink" );
	return retstat;
}

/** Create a file node
 *
 * There is no create() operation, mknod() will be called for
 * creation of all non-directory, non-symlink nodes.
 */
int empfs_mknod( const char * path, mode_t mode, dev_t dev )
{
	log_enter_function( "empfs_mknod" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		// On Linux this could just be 'mknod(path, mode, rdev)'
		// but this is more portable.
		if( S_ISREG( mode ) )
		{
			retstat = open( fpath, O_CREAT | O_EXCL | O_WRONLY, mode );
			if( retstat < 0 )
			{
				retstat = empfs_error( "empfs_mknod open" );
			}
			else
			{
				retstat = close( retstat );
				if( retstat < 0 )
				{
					retstat = empfs_error( "empfs_mknod close" );
				}
			}
		}
		else
		{
			if( S_ISFIFO( mode ) )
			{
				retstat = mkfifo( fpath, mode );
				if( retstat < 0 )
				{
					retstat = empfs_error( "empfs_mknod mkfifo" );
				}
			}
			else
			{
				retstat = mknod( fpath, mode, dev );
				if( retstat < 0 )
				{
					retstat = empfs_error( "empfs_mknod mknod" );
				}
			}
		}
	}
	log_leave_function( "empfs_mknod" );
	return retstat;
}

/** Create a directory */
int empfs_mkdir( const char * path, mode_t mode )
{
	log_enter_function( "empfs_mkdir" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = mkdir( fpath, mode );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_mkdir mkdir" );
		}
	}
	log_leave_function( "empfs_mkdir" );
	return retstat;
}

/** Remove a file */
int empfs_unlink( const char * path )
{
	log_enter_function( "empfs_unlink" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = unlink( fpath );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_unlink unlink" );
		}
	}
	log_leave_function( "empfs_unlink" );
	return retstat;
}

/** Remove a directory */
int empfs_rmdir( const char * path )
{
	log_enter_function( "empfs_rmdir" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = rmdir( fpath );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_rmdir rmdir" );
		}
	}
	log_leave_function( "empfs_rmdir" );
	return retstat;
}

/** Create a symbolic link */
// 'path' is where the link points to.
// 'link' is the link itself
// So leave path unaltered but add link to the mounted directory
int empfs_symlink( const char * path, const char * link )
{
	log_enter_function( "empfs_symlink" );

	int  retstat = -ENOENT;
	char flink[ PATH_MAX ];

	empfs_fullpath( flink, link );
	{
		retstat = symlink( path, flink );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_symlink symlink" );
		}
	}
	log_leave_function( "empfs_symlink" );
	return retstat;
}

/** Rename a file */
// both path and newpath are fs-relative
int empfs_rename( const char * path, const char * newpath )
{
	log_enter_function( "empfs_rename" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];
	char fnewpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) && empfs_fullpath( fnewpath, newpath ) )
	{
		retstat = rename( fpath, fnewpath );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_rename rename" );
		}
	}
	log_leave_function( "empfs_rename" );
	return retstat;
}

/** Create a hard link to a file */
int empfs_link( const char * path, const char * newpath )
{
	log_enter_function( "empfs_link" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ], fnewpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) && empfs_fullpath( fnewpath, newpath ) )
	{
		retstat = link( fpath, fnewpath );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_link link" );
		}
	}
	log_leave_function( "empfs_link" );
	return retstat;
}

/** Change the permission bits of a file */
int empfs_chmod( const char * path, mode_t mode )
{
	log_enter_function( "empfs_chmod" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = chmod( fpath, mode );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_chmod chmod" );
		}
	}
	log_leave_function( "empfs_chmod" );
	return retstat;
}

/** Change the owner and group of a file */
int empfs_chown( const char * path, uid_t uid, gid_t gid )
{
	log_enter_function( "empfs_chown" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = chown( fpath, uid, gid );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_chown chown" );
		}
	}
	log_leave_function( "empfs_chown" );
	return retstat;
}

/** Change the size of a file */
int empfs_truncate( const char * path, off_t newsize)
{
	log_enter_function( "empfs_truncate" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = truncate( fpath, newsize );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_truncate truncate" );
		}
	}
	log_leave_function( "empfs_truncate" );
	return retstat;
}

/** Change the access and/or modification times of a file */
int empfs_utime( const char * path, struct utimbuf * ubuf )
{
	log_enter_function( "empfs_utime" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = utime( fpath, ubuf );
		if( retstat < 0)
		{
			retstat = empfs_error( "empfs_utime utime" );
			/* ignore file not found errors, so tar can create symlinks */
			if( retstat == -ENOENT ) retstat = 0;
		}
	}
	log_leave_function( "empfs_utime" );
	return retstat;
}

/** File open operation
 *
 * No creation, or truncation flags (O_CREAT, O_EXCL, O_TRUNC)
 * will be passed to open().  Open should check if the operation
 * is permitted for the given flags.  Optionally open may also
 * return an arbitrary filehandle in the fuse_file_info structure,
 * which will be passed to all file operations.
 *
 * Changed in version 2.2
 */
int empfs_open( const char * path, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_open" );

	int  retstat = -ENOENT;
	int  fd;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		fd = open( fpath, fi->flags );
		if( fd < 0 )
		{
			retstat = empfs_error( "empfs_open open" );
		}
		else
		{
			retstat = 0;
			fi->fh = fd;
		}
	}
	log_leave_function( "empfs_open" );
	return retstat;
}

/** Read data from an open file
 *
 * Read should return exactly the number of bytes requested except
 * on EOF or error, otherwise the rest of the data will be
 * substituted with zeroes.  An exception to this is when the
 * 'direct_io' mount option is specified, in which case the return
 * value of the read system call will reflect the return value of
 * this operation.
 *
 * Changed in version 2.2
 */
int empfs_read( const char * path, char * buf, size_t size, off_t offset, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_read" );

	int retstat = -ENOENT;

	retstat = pread( fi->fh, buf, size, offset );
	if( retstat < 0 )
	{
		retstat = empfs_error( "empfs_read read" );
	}

	log_leave_function( "empfs_read" );
	return retstat;
}

/** Write data to an open file
 *
 * Write should return exactly the number of bytes requested
 * except on error.  An exception to this is when the 'direct_io'
 * mount option is specified (see read operation).
 *
 * Changed in version 2.2
 */
int empfs_write( const char * path, const char * buf, size_t size, off_t offset, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_write" );

	int retstat = -ENOENT;

	retstat = pwrite( fi->fh, buf, size, offset );
	if( retstat < 0 )
	{
		retstat = empfs_error( "empfs_write pwrite" );
	}

	log_leave_function( "empfs_write" );
	return retstat;
}

/** Get file system statistics
 *
 * The 'f_frsize', 'f_favail', 'f_fsid' and 'f_flag' fields are ignored
 * Replaced 'struct statfs' parameter with 'struct statvfs' in
 * version 2.5
 */
int empfs_statfs( const char * path, struct statvfs * statv )
{
	log_enter_function( "empfs_statfs" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		// get stats for underlying filesystem
		retstat = statvfs( fpath, statv );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_statfs statvfs" );
		}
	}
	log_leave_function( "empfs_statfs" );
	return retstat;
}

/** Possibly flush cached data
 *
 * BIG NOTE: This is not equivalent to fsync().  It's not a
 * request to sync dirty data.
 *
 * Flush is called on each close() of a file descriptor.  So if a
 * filesystem wants to return write errors in close() and the file
 * has cached dirty data, this is a good place to write back data
 * and return any errors.  Since many applications ignore close()
 * errors this is not always useful.
 *
 * NOTE: The flush() method may be called more than once for each
 * open().  This happens if more than one file descriptor refers
 * to an opened file due to dup(), dup2() or fork() calls.  It is
 * not possible to determine if a flush is final, so each flush
 * should be treated equally.  Multiple write-flush sequences are
 * relatively rare, so this shouldn't be a problem.
 *
 * Filesystems shouldn't assume that flush will always be called
 * after some writes, or that if will be called at all.
 *
 * Changed in version 2.2
 */
int empfs_flush( const char * path, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_flush" );
	int retstat = 0;

	log_leave_function( "empfs_flush" );
	return retstat;
}

/** Release an open file
 *
 * Release is called when there are no more references to an open
 * file: all file descriptors are closed and all memory mappings
 * are unmapped.
 *
 * For every open() call there will be exactly one release() call
 * with the same flags and file descriptor.  It is possible to
 * have a file opened more than once, in which case only the last
 * release will mean, that no more reads/writes will happen on the
 * file.  The return value of release is ignored.
 *
 * Changed in version 2.2
 */
int empfs_release( const char * path, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_release" );
	int retstat = -ENOENT;

	// We need to close the file.  Had we allocated any resources
	// (buffers etc) we'd need to free them here as well.
	retstat = close( fi->fh );

	log_leave_function( "empfs_release" );
	return retstat;
}

/** Synchronize file contents
 *
 * If the datasync parameter is non-zero, then only the user data
 * should be flushed, not the meta data.
 *
 * Changed in version 2.2
 */
int empfs_fsync( const char * path, int datasync, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_fsync" );

	int retstat = -ENOENT;

	if( datasync )
	{
		retstat = fdatasync( fi->fh );
	}
	else
	{
		retstat = fsync( fi->fh );
	}

	if( retstat < 0 )
	{
		retstat = empfs_error( "empfs_fsync fsync" );
	}

	log_leave_function( "empfs_fsync" );
	return retstat;
}

/** Set extended attributes */
int empfs_setxattr( const char * path, const char * name, const char * value, size_t size, int flags )
{
	log_enter_function( "empfs_setxattr" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = lsetxattr( fpath, name, value, size, flags );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_setxattr lsetxattr" );
		}
	}
	log_leave_function( "empfs_setxattr" );
	return retstat;
}

/** Get extended attributes */
int empfs_getxattr( const char * path, const char * name, char * value, size_t size )
{
	log_enter_function( "empfs_getxattr" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	log_msg( "path = [%s], name = [%s], value = 0x%08x, size = %d\n", path, name, value, size );

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = lgetxattr( fpath, name, value, size );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_getxattr lgetxattr" );
		}
	}
	log_leave_function( "empfs_getxattr" );
	return retstat;
}

/** List extended attributes */
int empfs_listxattr( const char * path, char * list, size_t size )
{
	log_enter_function( "empfs_listxattr" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = llistxattr( fpath, list, size );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_listxattr llistxattr");
		}
	}
	log_leave_function( "empfs_listxattr" );
	return retstat;
}

/** Remove extended attributes */
int empfs_removexattr( const char * path, const char * name )
{
	log_enter_function( "empfs_removexattr" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) ) 
	{
		retstat = lremovexattr( fpath, name );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_removexattr lrmovexattr" );
		}
	}
	log_leave_function( "empfs_removexattr" );
	return retstat;
}

/** Open directory
 *
 * This method should check if the open operation is permitted for
 * this  directory
 *
 * Introduced in version 2.3
 */
int empfs_opendir( const char * path, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_opendir" );

	DIR * dp;
	int   retstat = -ENOENT;
	char  fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		dp = opendir( fpath );
		if( dp == NULL )
		{
			retstat = empfs_error( "empfs_opendir opendir" );
		}
		else
		{
			retstat = 0;
			fi->fh = (intptr_t) dp;
		}
	}
	log_leave_function( "empfs_opendir" );
	return retstat;
}

/** Read directory
 *
 * This supersedes the old getdir( ) interface.  New applications
 * should use this.
 *
 * The filesystem may choose between two modes of operation:
 *
 * 1) The readdir implementation ignores the offset parameter, and
 * passes zero to the filler function's offset.  The filler
 * function will not return '1' ( unless an error happens), so the
 * whole directory is read in a single readdir operation.  This
 * works just like the old getdir( ) method.
 *
 * 2) The readdir implementation keeps track of the offsets of the
 * directory entries.  It uses the offset parameter and always
 * passes non-zero offset to the filler function.  When the buffer
 * is full ( or an error happens) the filler function will return
 * '1'.
 *
 * Introduced in version 2.3
 */
int empfs_readdir( const char * path, void * buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_readdir" );

	int    retstat = 0;
	DIR *  dp;
	struct dirent * de;

	// once again, no need for fullpath -- but note that I need to cast fi->fh
	dp = (DIR *) (uintptr_t) fi->fh;

	de = readdir( dp );
	if( de == NULL )
	{
		retstat = empfs_error( "empfs_readdir readdir" );
	}

	do
	{
		if( filler( buf, de->d_name, NULL, 0 ) != 0 )
		{
			retstat = -ENOMEM;
			break;
		}
	}
	while( ( de = readdir( dp ) ) != NULL );

	log_leave_function( "empfs_readdir" );
	return retstat;
}

/** Release directory
 *
 * Introduced in version 2.3
 */
int empfs_releasedir( const char * path, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_releasedir" );

	closedir( (DIR *) (uintptr_t) fi->fh );

	log_leave_function( "empfs_releasedir" );
	return 0;
}

/** Synchronize directory contents
 *
 * If the datasync parameter is non-zero, then only the user data
 * should be flushed, not the meta data
 *
 * Introduced in version 2.3
 */
int empfs_fsyncdir( const char * path, int datasync, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_fsyncdir" );

	log_leave_function( "empfs_fsyncdir" );
	return 0;
}

/**
 * Initialize filesystem
 *
 * The return value will passed in the private_data field of
 * fuse_context to all file operations and as a parameter to the
 * destroy( ) method.
 *
 * Introduced in version 2.3
 * Changed in version 2.6
 */
void * empfs_init( struct fuse_conn_info * conn )
{
	log_enter_function( "empfs_init" );
	log_leave_function( "empfs_init" );
	return VS_DATA;
}

/**
 * Clean up filesystem
 * Called on filesystem exit.
 * Introduced in version 2.3
 */
void empfs_destroy( void * userdata )
{
	log_enter_function( "empfs_destroy" );
	log_leave_function( "empfs_destroy" );
	return;
}

/**
 * Check file access permissions
 *
 * This will be called for the access( ) system call.  If the
 * 'default_permissions' mount option is given, this method is not
 * called.
 *
 * This method is not called under Linux kernel versions 2.4.x
 *
 * Introduced in version 2.5
 */
int empfs_access( const char * path, int mask )
{
	log_enter_function( "empfs_access" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];

	if( empfs_fullpath( fpath, path ) )
	{
		retstat = access( fpath, mask );
		if( retstat < 0 )
		{
			retstat = empfs_error( "empfs_access access" );
		}
	}
	log_leave_function( "empfs_access" );
	return retstat;
}

/**
 * Create and open a file
 *
 * If the file does not exist, first create it with the specified
 * mode, and then open it.
 *
 * If this method is not implemented or under Linux kernel
 * versions earlier than 2.6.15, the mknod( ) and open( ) methods
 * will be called instead.
 *
 * Introduced in version 2.5
 */
int empfs_create( const char * path, mode_t mode, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_create" );

	int  retstat = -ENOENT;
	char fpath[ PATH_MAX ];
	int  fd;

	if( empfs_fullpath( fpath, path ) )
	{
		fd = creat( fpath, mode );
		if( fd < 0 )
		{
			retstat = empfs_error( "empfs_create creat" );
		}
		else
		{
			retstat = 0;
			fi->fh = fd;
		}
	}
	log_leave_function( "empfs_create" );
	return retstat;
}

/**
 * Change the size of an open file
 *
 * This method is called instead of the truncate( ) method if the
 * truncation was invoked from an ftruncate( ) system call.
 *
 * If this method is not implemented or under Linux kernel
 * versions earlier than 2.6.15, the truncate( ) method will be
 * called instead.
 *
 * Introduced in version 2.5
 */
int empfs_ftruncate( const char * path, off_t offset, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_ftruncate" );

	int retstat = -ENOENT;

	retstat = ftruncate( fi->fh, offset );
	if( retstat < 0 )
	{
		retstat = empfs_error( "empfs_ftruncate ftruncate" );
	}

	log_leave_function( "empfs_ftruncate" );
	return retstat;
}

/**
 * Get attributes from an open file
 *
 * This method is called instead of the getattr( ) method if the
 * file information is available.
 *
 * Currently this is only called after the create( ) method if that
 * is implemented ( see above).  Later it may be called for
 * invocations of fstat( ) too.
 *
 * Introduced in version 2.5
 */
int empfs_fgetattr( const char * path, struct stat * statbuf, struct fuse_file_info * fi )
{
	log_enter_function( "empfs_fgetattr" );

	int retstat = -ENOENT;

	retstat = fstat( fi->fh, statbuf );
	if( retstat < 0 )
	{
		retstat = empfs_error( "empfs_fgetattr fstat" );
	}

	log_leave_function( "empfs_fgetattr" );
	return retstat;
}

struct fuse_operations empfs_oper =
{
	.getattr     = empfs_getattr,
	.readlink    = empfs_readlink,
	// no .getdir -- deprecated
	.getdir      = NULL,
	.mknod       = empfs_mknod,
	.mkdir       = empfs_mkdir,
	.unlink      = empfs_unlink,
	.rmdir       = empfs_rmdir,
	.symlink     = empfs_symlink,
	.rename      = empfs_rename,
	.link        = empfs_link,
	.chmod       = empfs_chmod,
	.chown       = empfs_chown,
	.truncate    = empfs_truncate,
	.utime       = empfs_utime,
	.open        = empfs_open,
	.read        = empfs_read,
	.write       = empfs_write,
	/** Just a placeholder, don't set */ // huh???
	.statfs      = empfs_statfs,
	.flush       = empfs_flush,
	.release     = empfs_release,
	.fsync       = empfs_fsync,
	.setxattr    = empfs_setxattr,
	.getxattr    = empfs_getxattr,
	.listxattr   = empfs_listxattr,
	.removexattr = empfs_removexattr,
	.opendir     = empfs_opendir,
	.readdir     = empfs_readdir,
	.releasedir  = empfs_releasedir,
	.fsyncdir    = empfs_fsyncdir,
	.init        = empfs_init,
	.destroy     = empfs_destroy,
	.access      = empfs_access,
	.create      = empfs_create,
	.ftruncate   = empfs_ftruncate,
	.fgetattr    = empfs_fgetattr
};

void empfs_usage( )
{
	fprintf( stderr, "usage:  empfs [FUSE and options] -m <mount point> -e <environment variable>\n" );
	fprintf( stderr, "\noptions:\n" );
	fprintf( stderr, "    -d    Enable logging of debug messages to empfs.log\n" );
	exit( 0 );
}

int main( int argc, char * argv[ ] )
{
	int    c;
	char * fuse_argv[ 5 ];
	struct vs_state * empfs_data;
	
	empfs_data = malloc( sizeof( struct vs_state ) );
	if( empfs_data == NULL )
	{
		perror( "malloc failed" );
		return 1;
	}

	empfs_data->debug        = 0;
	empfs_data->rootdir      = NULL;
	empfs_data->logfile      = NULL;
	empfs_data->tab_count    = 0;
	empfs_data->env_variable = NULL;

	// fuse_argv passes parameters to FUSE
	fuse_argv[ 0 ] = argv[ 0 ];	// empfs
	fuse_argv[ 1 ] = NULL;		// mount point 
	fuse_argv[ 2 ] = "-o";		// http://unix.stackexchange.com/questions/19595/
	fuse_argv[ 3 ] = "allow_other";	// how-can-i-create-a-unionfs-fuse-mount-that-is-readable-by-all
	fuse_argv[ 4 ] = NULL;		// null terminated list

	while( ( c = getopt( argc, argv, "hdm:e:" ) ) != -1 )
	{
		switch( c )
		{
			case 'd':
				empfs_data->debug = 1;
				break;
			case 'e':
				empfs_data->env_variable = optarg;
				break;
			case 'h':
				empfs_usage( );
				break;
			case 'm':
				empfs_data->rootdir = realpath( optarg, NULL );
				fuse_argv[ 1 ] = optarg;
				break;
			case '?':
				if( optopt == 'm' )
					fprintf( stderr, "Option -m requires a mount point.\n" );
				else if( optopt == 'v' )
					fprintf( stderr, "Option -e requires an environment variable.\n" );
				else if( isprint( optopt ) )
					fprintf( stderr, "Uknown option `-%c'.\n", optopt );
				else
					fprintf( stderr, "Unknown option character `\\x%x'.\n", optopt );
				return 1;
				break;
			default:
				empfs_usage( );
				break;
		}
	}

	if( empfs_data->rootdir == NULL )
	{
		fprintf( stderr, "Mount directory has not been specified.\n" );
		empfs_usage( );
	}

	if( empfs_data->env_variable == NULL )
	{
		fprintf( stderr, "Environment variable has not been specified.\n" );
		empfs_usage( );
	}

	if( empfs_data->debug )
	{
		printf( "empfs_data->env_variable: %s\n", empfs_data->env_variable );
		printf( "empfs_data->rootdir: %s\n", empfs_data->rootdir );
		printf( "fuse_argv[ 0 ]: %s\n", fuse_argv[ 0 ] );
		printf( "fuse_argv[ 1 ]: %s\n", fuse_argv[ 1 ] );
		printf( "fuse_argv[ 2 ]: %s\n", fuse_argv[ 2 ] );
		printf( "fuse_argv[ 3 ]: %s\n", fuse_argv[ 3 ] );
		printf( "fuse_argv[ 4 ]: %s\n", fuse_argv[ 4 ] );

		empfs_data->logfile = log_open( empfs_data->env_variable );
	}

	return fuse_main( 4, fuse_argv, &empfs_oper, empfs_data );
}

