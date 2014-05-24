/*
 * Variant Symlink Filesystem
 *
 * This fuse filesystem reads the specified environment variable from a processes /proc/${pid}/environ file.
 * The environment variable specified should point to a real fully qualified path. This path is then used as the 
 * root path for all requests received by this fuse filesystem.
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

// Report errors to logfile and give -errno to caller
static int vs_error( char *str)
{
    int ret = -errno;
    
    return ret;
}

// This function will take a pid and examine its /proc/pid/environ file
// searching for the matching environment parameter. If found, the value
// is copied to the variable and returned.

static void get_env_variable( char variable[PATH_MAX], pid_t pid )
{
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
}


// 1. Get the PQ_CHASSIS environment variable from /proc/PID/environ
// 2. Append the path received from fuse to the value from PQ_CHASSIS
// 3. Return the value
static void pq_fullpath( char fpath[PATH_MAX], const char *path)
{
	char variable[PATH_MAX];
	pid_t pid = fuse_get_context( )->pid;

	get_env_variable( variable, pid );

	strcpy( fpath, variable );
	strncat( fpath, path, PATH_MAX );
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
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path);

	retstat = lstat( fpath, statbuf );
	if ( retstat != 0){
		retstat = vs_error( "vs_getattr lstat" );
	}

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
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = readlink( fpath, link, size - 1 );
	if ( retstat < 0 ) {
		retstat = vs_error( "vs_readlink readlink" );
	}
	else  {
		link[retstat] = '\0';
		retstat = 0;
	}

	return retstat;
}

/** Create a file node
 *
 * There is no create() operation, mknod() will be called for
 * creation of all non-directory, non-symlink nodes.
 */
int vs_mknod( const char *path, mode_t mode, dev_t dev )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

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

	return retstat;
}

/** Create a directory */
int vs_mkdir( const char *path, mode_t mode )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = mkdir( fpath, mode );
	if ( retstat < 0 )
		retstat = vs_error( "vs_mkdir mkdir" );

	return retstat;
}

/** Remove a file */
int vs_unlink( const char *path )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = unlink( fpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_unlink unlink" );

	return retstat;
}

/** Remove a directory */
int vs_rmdir( const char *path )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = rmdir( fpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_rmdir rmdir" );

	return retstat;
}

/** Create a symbolic link */
// 'path' is where the link points to.
// 'link' is the link itself
// So leave path unaltered but add link to the mounted directory
int vs_symlink( const char *path, const char *link )
{
	int retstat = 0;
	char flink[PATH_MAX];

	pq_fullpath( flink, link );

	retstat = symlink( path, flink );
	if ( retstat < 0 )
		retstat = vs_error( "vs_symlink symlink" );

	return retstat;
}

/** Rename a file */
// both path and newpath are fs-relative
int vs_rename( const char *path, const char *newpath )
{
	int retstat = 0;
	char fpath[PATH_MAX];
	char fnewpath[PATH_MAX];

	pq_fullpath( fpath, path ); 
	pq_fullpath( fnewpath, newpath );

	retstat = rename( fpath, fnewpath );
	if ( retstat < 0 )
		retstat = vs_error( "vs_rename rename" );

	return retstat;
}

/** Create a hard link to a file */
int vs_link( const char *path, const char *newpath )
{
	int retstat = 0;
	char fpath[PATH_MAX], fnewpath[PATH_MAX];

	pq_fullpath( fpath, path );
	pq_fullpath( fnewpath, newpath );

	retstat = link( fpath, fnewpath );
	if ( retstat < 0 ) 
		retstat = vs_error( "vs_link link" );

	return retstat;
}

/** Change the permission bits of a file */
int vs_chmod( const char *path, mode_t mode )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = chmod( fpath, mode );
	if ( retstat < 0 )
		retstat = vs_error( "vs_chmod chmod" );

	return retstat;
}

/** Change the owner and group of a file */
int vs_chown( const char *path, uid_t uid, gid_t gid )
  
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = chown( fpath, uid, gid );
	if ( retstat < 0 )
	retstat = vs_error( "vs_chown chown" );

	return retstat;
}

/** Change the size of a file */
int vs_truncate( const char *path, off_t newsize)
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = truncate( fpath, newsize );
	if ( retstat < 0 )
		vs_error( "vs_truncate truncate" );

	return retstat;
}

/** Change the access and/or modification times of a file */
/* note -- I'll want to change this as soon as 2.6 is in debian testing */
int vs_utime( const char *path, struct utimbuf *ubuf )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = utime( fpath, ubuf );
	if ( retstat < 0)
		retstat = vs_error( "vs_utime utime" );

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
	int retstat = 0;
	int fd;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	fd = open( fpath, fi->flags );
	if ( fd < 0 )
		retstat = vs_error( "vs_open open" );

	fi->fh = fd;
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
	int retstat = 0;

	retstat = pread( fi->fh, buf, size, offset );
	if ( retstat < 0 )
		retstat = vs_error( "vs_read read" );

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
	int retstat = 0;

	retstat = pwrite( fi->fh, buf, size, offset );
	if ( retstat < 0 )
		retstat = vs_error( "vs_write pwrite" );

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
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	// get stats for underlying filesystem
	retstat = statvfs( fpath, statv );
	if ( retstat < 0 )
		retstat = vs_error( "vs_statfs statvfs" );

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
	int retstat = 0;

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
	int retstat = 0;

	// We need to close the file.  Had we allocated any resources
	// (buffers etc) we'd need to free them here as well.
	retstat = close( fi->fh );

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
	int retstat = 0;

	if ( datasync )
		retstat = fdatasync( fi->fh );
	else
		retstat = fsync( fi->fh );

	if ( retstat < 0 )
		vs_error( "vs_fsync fsync" );

	return retstat;
}

/** Set extended attributes */
int vs_setxattr( const char *path, const char *name, const char *value, size_t size, int flags )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = lsetxattr( fpath, name, value, size, flags );
	if ( retstat < 0 )
		retstat = vs_error( "vs_setxattr lsetxattr" );

	return retstat;
}

/** Get extended attributes */
int vs_getxattr( const char *path, const char *name, char *value, size_t size )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = lgetxattr( fpath, name, value, size );
	if ( retstat < 0 )
		retstat = vs_error( "vs_getxattr lgetxattr" );

	return retstat;
}

/** List extended attributes */
int vs_listxattr( const char *path, char *list, size_t size )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = llistxattr( fpath, list, size );
	if ( retstat < 0 )
		retstat = vs_error( "vs_listxattr llistxattr");

	return retstat;
}

/** Remove extended attributes */
int vs_removexattr( const char *path, const char *name )
{
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path ); 

	retstat = lremovexattr( fpath, name );
	if ( retstat < 0 )
		retstat = vs_error( "vs_removexattr lrmovexattr" );

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
	DIR *dp;
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	dp = opendir( fpath );
	if ( dp == NULL )
		retstat = vs_error( "vs_opendir opendir" );

	fi->fh = (intptr_t) dp;

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

	return retstat;
}

/** Release directory
 *
 * Introduced in version 2.3
 */
int vs_releasedir( const char *path, struct fuse_file_info *fi )
{
	int retstat = 0;

	closedir( (DIR *) (uintptr_t) fi->fh );

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
	int retstat = 0;

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
	int retstat = 0;
	char fpath[PATH_MAX];

	pq_fullpath( fpath, path );

	retstat = access( fpath, mask );

	if ( retstat < 0 )
		retstat = vs_error( "vs_access access" );

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
	int retstat = 0;
	char fpath[PATH_MAX];
	int fd;

	pq_fullpath( fpath, path );

	fd = creat( fpath, mode );
	if ( fd < 0 )
		retstat = vs_error( "vs_create creat" );

	fi->fh = fd;

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
	int retstat = 0;

	retstat = ftruncate( fi->fh, offset );
	if ( retstat < 0 )
	retstat = vs_error( "vs_ftruncate ftruncate" );

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
	int retstat = 0;

	retstat = fstat( fi->fh, statbuf );
	if ( retstat < 0 )
		retstat = vs_error( "vs_fgetattr fstat" );

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
	fprintf( stderr, "usage:  vsfs [FUSE and mount options] mountPoint environment-variable\n" );
	abort( );
}

int main( int argc, char *argv[] )
{
	int fuse_stat;
	struct vs_state *vs_data;

	if ( ( getuid( ) == 0 ) || ( geteuid( ) == 0 ) ) {
		fprintf( stderr, "Running BBFS as root opens unnacceptable security holes\n" );
		return 1;
	}

	if ( ( argc < 3 ) || ( argv[argc-3][0] == '-' ) || ( argv[argc-2][0] == '-' ) || ( argv[argc-1][0] == '-' ) )
		vs_usage( );

	vs_data = malloc( sizeof( struct vs_state ) );
	if ( vs_data == NULL) {
		perror( "main calloc" );
		abort( );
	}

	// Save the environment variable
	vs_data->env_variable = argv[argc-1];
	// Get the mount directory as a full path
	vs_data->rootdir = realpath( argv[argc-2], NULL );
	// Shift argv down by 1 and NULL out the last parameter.
	argv[argc-1] = NULL;
	argc-=1;

	// turn over control to fuse
	fprintf( stderr, "vsfs starting, calling fuse_main\n" );
	fuse_stat = fuse_main( argc, argv, &vs_oper, vs_data );
	fprintf( stderr, "vsfs ended, fuse_main returned %d\n", fuse_stat );

	return fuse_stat;
}
