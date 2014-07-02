/*
 * Variant Symlink Filesystem
 *
 * This fuse filesystem reads the specified environment variable from a processes /proc/${pid}/environ file.
 * The environment variable specified should point to a real fully qualified path. This path is then used as the 
 * root path for all requests received by this fuse filesystem.
 *
 * Copyright (c) 2014, Cole Minnaar and ﬀrøøt limited
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
 *     This product includes software developed by the Cole Minnaar and ﬀrøøt limited.
 *  4. Neither the name ﬀrøøt nor the names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY COLE MINNAAR ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL COLE MINNAAR BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
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

void log_enter_function( const char *function )
{
	log_msg( "ENTER %s\n", function );
	VS_DATA->tab_count += 1;
}

void log_leave_function( const char *function )
{
	VS_DATA->tab_count -= 1;
	log_msg( "LEAVE %s\n", function );

	if( VS_DATA->tab_count == 0 )
		log_msg( "\n" );
}

// Report errors to logfile and give -errno to caller
static int vs_error( char *str)
{
	int ret = -errno;

	log_msg( "ERROR %s: %s\n", str, strerror( errno ) );

	return ret;
}

// Function to get the parent pid using /proc/${PID}/status
pid_t get_parent_pid( pid_t pid )
{
	log_enter_function( "get_parent_pid" );

	char environ[PATH_MAX];
	FILE *fl = NULL;
	long fl_size = 500;
	char *buffer = NULL;
	size_t res = 0;
	long total = 0;
	char *find_var = "PPid";
	size_t length = strlen( find_var );
	pid_t ppid = 0;

	snprintf( environ, sizeof(environ), "/proc/%d/status", (int) pid );
	printf( "environ=[%s]\n", environ );

	fl = fopen( environ, "r" );
	if( fl == NULL ){
		fprintf( stderr, "Error opening %s\n", environ );
		exit( 1 );
	}

	buffer = ( char * )malloc( sizeof( char ) * fl_size );
	if( buffer == NULL ){
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
			buffer = ( char * )realloc( buffer, sizeof( char ) * fl_size );
			if( buffer == NULL )
			{
				fprintf( stderr, "Error not enough memory\n" );
				exit( 3 );
			}
		}
	}

	buffer[total] = '\0';

	const char *delim = "\n";
	char *save;
	char *p;

	for( p = strtok_r(buffer, delim, &save); p; p = strtok_r( NULL, delim, &save) )
	{
		if( strncmp( find_var, p, strlen( find_var ) ) == 0 )
		{
			ppid = (pid_t)atoi( p + length + 1 );
			break;
		}
	}

	fclose( fl );
	free( buffer );

	log_leave_function( "get_parent_pid" );

	return ppid;
}

// This function will take a pid and examine its /proc/pid/environ file
// searching for the matching environment parameter. If found, the value
// is copied to the variable and returned.

static void get_env_variable( char variable[PATH_MAX], pid_t pid )
{
	log_enter_function( "get_env_variable" );

	char environ[PATH_MAX];
	FILE *fl = NULL;
	long fl_size = 4096;
	char *buffer = NULL;
	char *next_var = NULL;
	char *find_var = VS_DATA->env_variable;
	size_t length = strlen( find_var );
	size_t res = 0;
	long total = 0;

	snprintf( environ, sizeof( environ ), "/proc/%d/environ", (int) pid );

	fl = fopen( environ, "r" );
	if( fl == NULL ){
		log_msg( "Error opening %s\n", environ );
		fprintf( stderr, "Error opening %s\n", environ );
		log_leave_function( "get_env_variable" );
		//exit( 1 );
		return;
	}

	buffer = ( char * )malloc( sizeof( char ) * fl_size );
	if( buffer == NULL ){
		log_msg( "Error not enough memory\n" );
		fprintf( stderr, "Error not enough memory\n" );
		exit( 2 );
	}

	while( !feof( fl ) )
	{
		res = fread( buffer + total, 1, fl_size - total, fl );
		total += res;

		if( total == fl_size ){
			fl_size = fl_size * 2;
			buffer = ( char * )realloc( buffer, sizeof( char ) * fl_size );
			if( buffer == NULL ){
				fprintf( stderr, "Error not enough memory\n" );
				exit( 3 );
			}
		}
	}
	buffer[total] = '\0';
	next_var = buffer;

	while( next_var < ( buffer + total ) )
	{
		if( strncmp( next_var, find_var, length ) == 0 )
		{
			strncpy( variable, next_var + length + 1, PATH_MAX );
			break;
		}
		next_var += strlen( next_var ) + 1;
	}

	fclose( fl );
	free( buffer );

	log_leave_function( "get_env_variable" );
}


// 1. Get the PQ_CHASSIS environment variable from /proc/PID/environ
// 2. Append the path received from fuse to the value from PQ_CHASSIS
// 3. Return the value
static void vs_fullpath( char fpath[PATH_MAX], const char *path)
{
	log_enter_function( "vs_fullpath" );
	char variable[PATH_MAX];
	pid_t ppid = 0;
	pid_t pid = fuse_get_context( )->pid;

	memset( variable, '\0', sizeof(char) * PATH_MAX );

	log_msg( "vs_fullpath\n" );
	log_msg( "path: %s\n", path );
	log_msg( "pid: %d\n", pid );
	
	ppid = get_parent_pid( pid );
	log_msg( "ppid: %d\n", ppid );
	if( ppid != 0 ){
		get_env_variable( variable, ppid );

		log_msg( "parent variable: %s\n", variable );
	}

	if( strlen( variable ) == 0 ){
		get_env_variable( variable, pid );

		log_msg( "variable: %s\n", variable );
	}

	strcpy( fpath, variable );
	strncat( fpath, path, PATH_MAX );

	log_leave_function( "vs_fullpath" );
}

// All the prototype functions come from fuse.h
// Most of them are nothing more than wrappers around the normal functions

/** Get file attributes.
 *
 * Similar to stat().  The 'st_dev' and 'st_blksize' fields are
 * ignored.  The 'st_ino' field is ignored except if the 'use_ino'
 * mount option is given.
 */
int vs_getattr( const char *path, struct stat *statbuf )
{
	log_enter_function( "vs_getattr" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path);

	retstat = lstat( fpath, statbuf );
	if ( retstat != 0){
		retstat = vs_error( "vs_getattr lstat" );
	}

	log_leave_function( "vs_getattr" );

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
int vs_readlink( const char *path, char *link, size_t size )
{
	log_enter_function( "vs_readlink" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = readlink( fpath, link, size - 1 );
	if ( retstat < 0 ) {
		retstat = vs_error( "vs_readlink readlink" );
	}
	else  {
		link[retstat] = '\0';
		retstat = 0;
	}

	log_leave_function( "vs_readlink" );

	return retstat;
}

/** Create a file node
 *
 * There is no create() operation, mknod() will be called for
 * creation of all non-directory, non-symlink nodes.
 */
int vs_mknod( const char *path, mode_t mode, dev_t dev )
{
	log_enter_function( "vs_mknod" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	// On Linux this could just be 'mknod(path, mode, rdev)' but this
	//  is more portable
	if ( S_ISREG( mode ) ) {
		retstat = open( fpath, O_CREAT | O_EXCL | O_WRONLY, mode );
		if ( retstat < 0 )
		    retstat = vs_error( "vs_mknod open" );
		else {
		    retstat = close( retstat );
		    if ( retstat < 0 )
			retstat = vs_error( "vs_mknod close" );
		}
	} 
	else {
		if ( S_ISFIFO( mode ) ) {
		    retstat = mkfifo( fpath, mode );
		    if ( retstat < 0 )
			retstat = vs_error( "vs_mknod mkfifo" );
		} else {
		    retstat = mknod( fpath, mode, dev );
		    if ( retstat < 0 )
			retstat = vs_error( "vs_mknod mknod" );
		}
	}

	log_leave_function( "vs_mknod" );

	return retstat;
}

/** Create a directory */
int vs_mkdir( const char *path, mode_t mode )
{
	log_enter_function( "vs_mkdir" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = mkdir( fpath, mode );
	if ( retstat < 0 )
		retstat = vs_error( "vs_mkdir mkdir" );

	log_leave_function( "vs_mkdir" );

	return retstat;
}

/** Remove a file */
int vs_unlink( const char *path )
{
	log_enter_function( "vs_unlink" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = unlink( fpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_unlink unlink" );

	log_leave_function( "vs_unlink" );

	return retstat;
}

/** Remove a directory */
int vs_rmdir( const char *path )
{
	log_enter_function( "vs_rmdir" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = rmdir( fpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_rmdir rmdir" );

	log_leave_function( "vs_rmdir" );

	return retstat;
}

/** Create a symbolic link */
// 'path' is where the link points to.
// 'link' is the link itself
// So leave path unaltered but add link to the mounted directory
int vs_symlink( const char *path, const char *link )
{
	log_enter_function( "vs_symlink" );

	int retstat = 0;
	char flink[PATH_MAX];

	vs_fullpath( flink, link );

	retstat = symlink( path, flink );
	if ( retstat < 0 )
		retstat = vs_error( "vs_symlink symlink" );

	log_leave_function( "vs_symlink" );
	
	return retstat;
}

/** Rename a file */
// both path and newpath are fs-relative
int vs_rename( const char *path, const char *newpath )
{
	log_enter_function( "vs_rename" );

	int retstat = 0;
	char fpath[PATH_MAX];
	char fnewpath[PATH_MAX];

	vs_fullpath( fpath, path ); 
	vs_fullpath( fnewpath, newpath );

	retstat = rename( fpath, fnewpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_rename rename" );

	log_leave_function( "vs_rename" );

	return retstat;
}

/** Create a hard link to a file */
int vs_link( const char *path, const char *newpath )
{
	log_enter_function( "vs_link" );

	int retstat = 0;
	char fpath[PATH_MAX], fnewpath[PATH_MAX];

	vs_fullpath( fpath, path );
	vs_fullpath( fnewpath, newpath );

	retstat = link( fpath, fnewpath );
	if ( retstat < 0 ) 
		retstat = vs_error( "vs_link link" );


	log_leave_function( "vs_link" );
	return retstat;
}

/** Change the permission bits of a file */
int vs_chmod( const char *path, mode_t mode )
{
	log_enter_function( "vs_chmod" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = chmod( fpath, mode );
	if ( retstat < 0 )
		retstat = vs_error( "vs_chmod chmod" );

	log_leave_function( "vs_chmod" );

	return retstat;
}

/** Change the owner and group of a file */
int vs_chown( const char *path, uid_t uid, gid_t gid )
{
	log_enter_function( "vs_chown" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = chown( fpath, uid, gid );
	if ( retstat < 0 )
	retstat = vs_error( "vs_chown chown" );

	log_leave_function( "vs_chown" );

	return retstat;
}

/** Change the size of a file */
int vs_truncate( const char *path, off_t newsize)
{
	log_enter_function( "vs_truncate" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = truncate( fpath, newsize );
	if ( retstat < 0 )
		vs_error( "vs_truncate truncate" );

	log_leave_function( "vs_truncate" );
	return retstat;
}

/** Change the access and/or modification times of a file */
/* note -- I'll want to change this as soon as 2.6 is in debian testing */
int vs_utime( const char *path, struct utimbuf *ubuf )
{
	log_enter_function( "vs_utime" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = utime( fpath, ubuf );
	if ( retstat < 0)
		retstat = vs_error( "vs_utime utime" );

	log_leave_function( "vs_utime" );

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
int vs_open( const char *path, struct fuse_file_info *fi )
{
	log_enter_function( "vs_open" );

	int retstat = 0;
	int fd;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	fd = open( fpath, fi->flags );
	if ( fd < 0 )
		retstat = vs_error( "vs_open open" );

	fi->fh = fd;

	log_leave_function( "vs_open" );

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
int vs_read( const char *path, char *buf, size_t size, off_t offset, struct fuse_file_info *fi )
{
	log_enter_function( "vs_read" );

	int retstat = 0;

	retstat = pread( fi->fh, buf, size, offset );
	if ( retstat < 0 )
		retstat = vs_error( "vs_read read" );

	log_leave_function( "vs_read" );
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
int vs_write( const char *path, const char *buf, size_t size, off_t offset, struct fuse_file_info *fi )
{
	log_enter_function( "vs_write" );

	int retstat = 0;

	retstat = pwrite( fi->fh, buf, size, offset );
	if ( retstat < 0 )
		retstat = vs_error( "vs_write pwrite" );

	log_leave_function( "vs_write" );

	return retstat;
}

/** Get file system statistics
 *
 * The 'f_frsize', 'f_favail', 'f_fsid' and 'f_flag' fields are ignored
 *
 * Replaced 'struct statfs' parameter with 'struct statvfs' in
 * version 2.5
 */
int vs_statfs( const char *path, struct statvfs *statv )
{
	log_enter_function( "vs_statfs" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	// get stats for underlying filesystem
	retstat = statvfs( fpath, statv );
	if ( retstat < 0 )
		retstat = vs_error( "vs_statfs statvfs" );

	log_leave_function( "vs_statfs" );

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
int vs_flush( const char *path, struct fuse_file_info *fi )
{
	log_enter_function( "vs_flush" );
	int retstat = 0;

	log_leave_function( "vs_flush" );
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
int vs_release( const char *path, struct fuse_file_info *fi )
{
	log_enter_function( "vs_release" );
	int retstat = 0;

	// We need to close the file.  Had we allocated any resources
	// (buffers etc) we'd need to free them here as well.
	retstat = close( fi->fh );

	log_leave_function( "vs_release" );
	return retstat;
}

/** Synchronize file contents
 *
 * If the datasync parameter is non-zero, then only the user data
 * should be flushed, not the meta data.
 *
 * Changed in version 2.2
 */
int vs_fsync( const char *path, int datasync, struct fuse_file_info *fi )
{
	log_enter_function( "vs_fsync" );

	int retstat = 0;

	if ( datasync )
		retstat = fdatasync( fi->fh );
	else
		retstat = fsync( fi->fh );

	if ( retstat < 0 )
		vs_error( "vs_fsync fsync" );

	log_leave_function( "vs_fsync" );

	return retstat;
}

/** Set extended attributes */
int vs_setxattr( const char *path, const char *name, const char *value, size_t size, int flags )
{
	log_enter_function( "vs_setxattr" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = lsetxattr( fpath, name, value, size, flags );
	if ( retstat < 0 )
		retstat = vs_error( "vs_setxattr lsetxattr" );

	log_leave_function( "vs_setxattr" );

	return retstat;
}

/** Get extended attributes */
int vs_getxattr( const char *path, const char *name, char *value, size_t size )
{
	log_enter_function( "vs_getxattr" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = lgetxattr( fpath, name, value, size );
	if ( retstat < 0 )
		retstat = vs_error( "vs_getxattr lgetxattr" );

	log_leave_function( "vs_getxattr" );

	return retstat;
}

/** List extended attributes */
int vs_listxattr( const char *path, char *list, size_t size )
{
	log_enter_function( "vs_listxattr" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = llistxattr( fpath, list, size );
	if ( retstat < 0 )
		retstat = vs_error( "vs_listxattr llistxattr");

	log_leave_function( "vs_listxattr" );

	return retstat;
}

/** Remove extended attributes */
int vs_removexattr( const char *path, const char *name )
{
	log_enter_function( "vs_removexattr" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path ); 

	retstat = lremovexattr( fpath, name );
	if ( retstat < 0 )
		retstat = vs_error( "vs_removexattr lrmovexattr" );

	log_leave_function( "vs_removexattr" );

	return retstat;
}

/** Open directory
 *
 * This method should check if the open operation is permitted for
 * this  directory
 *
 * Introduced in version 2.3
 */
int vs_opendir( const char *path, struct fuse_file_info *fi )
{
	log_enter_function( "vs_opendir" );

	DIR *dp;
	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	dp = opendir( fpath );
	if ( dp == NULL )
		retstat = vs_error( "vs_opendir opendir" );

	fi->fh = (intptr_t) dp;

	log_leave_function( "vs_opendir" );

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
int vs_readdir( const char *path, void *buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info *fi )
{
	log_enter_function( "vs_readdir" );

	int retstat = 0;
	DIR *dp;
	struct dirent *de;

	// once again, no need for fullpath -- but note that I need to cast fi->fh
	dp = (DIR *) (uintptr_t) fi->fh;

	de = readdir( dp );
	if ( de == 0 ) {
		retstat = vs_error( "vs_readdir readdir" );
		return retstat;
	}

	do {
		if ( filler( buf, de->d_name, NULL, 0 ) != 0 ) {
			return -ENOMEM;
		}
	} while ( ( de = readdir( dp ) ) != NULL );

	log_leave_function( "vs_readdir" );

	return retstat;
}

/** Release directory
 *
 * Introduced in version 2.3
 */
int vs_releasedir( const char *path, struct fuse_file_info *fi )
{
	log_enter_function( "vs_releasedir" );

	int retstat = 0;

	closedir( (DIR *) (uintptr_t) fi->fh );

	log_leave_function( "vs_releasedir" );

	return retstat;
}

/** Synchronize directory contents
 *
 * If the datasync parameter is non-zero, then only the user data
 * should be flushed, not the meta data
 *
 * Introduced in version 2.3
 */
int vs_fsyncdir( const char *path, int datasync, struct fuse_file_info *fi )
{
	log_enter_function( "vs_fsyncdir" );

	int retstat = 0;

	log_leave_function( "vs_fsyncdir" );
	return retstat;
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
void *vs_init( struct fuse_conn_info *conn )
{
	log_enter_function( "vs_init" );
	log_leave_function( "vs_init" );
	return VS_DATA;
}

/**
 * Clean up filesystem
 *
 * Called on filesystem exit.
 *
 * Introduced in version 2.3
 */
void vs_destroy( void *userdata )
{
	log_enter_function( "vs_destroy" );
	log_leave_function( "vs_destroy" );
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
int vs_access( const char *path, int mask )
{
	log_enter_function( "vs_access" );

	int retstat = 0;
	char fpath[PATH_MAX];

	vs_fullpath( fpath, path );

	retstat = access( fpath, mask );

	if ( retstat < 0 )
		retstat = vs_error( "vs_access access" );

	log_leave_function( "vs_access" );
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
int vs_create( const char *path, mode_t mode, struct fuse_file_info *fi )
{
	log_enter_function( "vs_create" );

	int retstat = 0;
	char fpath[PATH_MAX];
	int fd;

	vs_fullpath( fpath, path );

	fd = creat( fpath, mode );
	if ( fd < 0 )
		retstat = vs_error( "vs_create creat" );

	fi->fh = fd;

	log_leave_function( "vs_create" );

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
int vs_ftruncate( const char *path, off_t offset, struct fuse_file_info *fi )
{
	log_enter_function( "vs_ftruncate" );

	int retstat = 0;

	retstat = ftruncate( fi->fh, offset );
	if ( retstat < 0 )
		retstat = vs_error( "vs_ftruncate ftruncate" );

	log_leave_function( "vs_ftruncate" );

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
int vs_fgetattr( const char *path, struct stat *statbuf, struct fuse_file_info *fi )
{
	log_enter_function( "vs_fgetattr" );

	int retstat = 0;

	retstat = fstat( fi->fh, statbuf );
	if ( retstat < 0 )
		retstat = vs_error( "vs_fgetattr fstat" );

	log_leave_function( "vs_fgetattr" );

	return retstat;
}

struct fuse_operations vs_oper = {
	.getattr = vs_getattr,
	.readlink = vs_readlink,
	// no .getdir -- that's deprecated
	.getdir = NULL,
	.mknod = vs_mknod,
	.mkdir = vs_mkdir,
	.unlink = vs_unlink,
	.rmdir = vs_rmdir,
	.symlink = vs_symlink,
	.rename = vs_rename,
	.link = vs_link,
	.chmod = vs_chmod,
	.chown = vs_chown,
	.truncate = vs_truncate,
	.utime = vs_utime,
	.open = vs_open,
	.read = vs_read,
	.write = vs_write,
	/** Just a placeholder, don't set */ // huh???
	.statfs = vs_statfs,
	.flush = vs_flush,
	.release = vs_release,
	.fsync = vs_fsync,
	.setxattr = vs_setxattr,
	.getxattr = vs_getxattr,
	.listxattr = vs_listxattr,
	.removexattr = vs_removexattr,
	.opendir = vs_opendir,
	.readdir = vs_readdir,
	.releasedir = vs_releasedir,
	.fsyncdir = vs_fsyncdir,
	.init = vs_init,
	.destroy = vs_destroy,
	.access = vs_access,
	.create = vs_create,
	.ftruncate = vs_ftruncate,
	.fgetattr = vs_fgetattr
};

void vs_usage( )
{
	fprintf( stderr, "usage:  vsfs [FUSE and options] -m <mount point> -e <environment variable>\n" );
	fprintf( stderr, "\noptions:\n" );
	fprintf( stderr, "    -d    Enable logging of debug messages to vsfs.log\n" );
	exit( -1 );
}

int main( int argc, char *argv[] )
{
	int fuse_stat;
	struct vs_state *vs_data;
	char *new_argv[3];
	int c;
	
	if ( ( getuid( ) == 0 ) || ( geteuid( ) == 0 ) ) {
		fprintf( stderr, "Running VSFS as root opens unnacceptable security holes\n" );
		return 1;
	}

	vs_data = malloc( sizeof( struct vs_state ) );
	if ( vs_data == NULL) {
		perror( "main calloc" );
		abort( );
	}

	vs_data->debug = 0;
	vs_data->rootdir = NULL;
	vs_data->logfile = NULL;
	vs_data->env_variable = NULL;

	// new_argv is used to pass the mountpoint to FUSE.
	new_argv[0] = argv[0];
	new_argv[1] = NULL;
	new_argv[2] = NULL;

	while( ( c = getopt( argc, argv, "hdm:e:" ) ) != -1 )
	{
		switch( c )
		{
			case 'd':
				vs_data->debug = 0;
				break;
			case 'e':
				vs_data->env_variable = optarg;
				break;
			case 'h':
				vs_usage( );
				break;
			case 'm':
				vs_data->rootdir = realpath( optarg, NULL );
				new_argv[1] = optarg;
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
				vs_usage( );
				break;
		}
	}

	if( vs_data->rootdir == NULL ){
		fprintf( stderr, "Mount directory has not been specified.\n" );
		vs_usage( );
	}
	if( vs_data->env_variable == NULL )
	{
		fprintf( stderr, "Environment variable has not been specified.\n" );
		vs_usage( );
	}

	/*
	printf( "vs_data->env_variable: %s\n", vs_data->env_variable );
	printf( "vs_data->rootdir: %s\n", vs_data->rootdir );
	printf( "new_argv[0]: %s\n", new_argv[0] );
	printf( "new_argv[1]: %s\n", new_argv[1] );
	printf( "new_argv[2]: %s\n", new_argv[2] );
	*/

	vs_data->logfile = log_open( );
	vs_data->tab_count = 0;

	// turn over control to fuse
	fprintf( stderr, "vsfs starting, calling fuse_main\n" );
	fuse_stat = fuse_main( 2, new_argv, &vs_oper, vs_data );
	fprintf( stderr, "vsfs ended, fuse_main returned %d\n", fuse_stat );

	return fuse_stat;
}
