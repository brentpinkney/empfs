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

#include <fuse.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "log.h"

FILE* log_open( const char * var )
{
	FILE* logfile;
	char name[ PATH_MAX ];
	snprintf( name, sizeof( name ), "empfs-%s.log", var );

	logfile = fopen( name, "w" );
	if( logfile == NULL )
	{
		fprintf( stderr, "failed to open logfile\n" );
		exit( EXIT_FAILURE );
	}

	setvbuf( logfile, NULL, _IOLBF, 0 );
	return logfile;
}

void log_msg( const char * const format, ... )
{
	int i = 0;
	va_list ap;

	if( VS_DATA->debug == 1 )
	{
		for( i = 0; i < VS_DATA->tab_count; i++ )
		{
			fprintf( VS_DATA->logfile, "   " );
		}
		
		va_start( ap, format );
		vfprintf( VS_DATA->logfile, format, ap );
	}
}

void log_fi( struct fuse_file_info * const fi )
{
	log_struct( fi, flags,      0x%08x,    );
	log_struct( fi, fh_old,     0x$08lx,   );
	log_struct( fi, writepage,  %d,        );
	log_struct( fi, direct_io,  %d,        );
	log_struct( fi, keep_cache, %d,        );
	log_struct( fi, fh,         0x%016llx, );
	log_struct( fi, lock_owner, 0x%016llx, );
}

void log_stat( struct stat * const si )
{
	log_struct( si, st_dev,     %lld, );
	log_struct( si, st_ino,     %lld, );
	log_struct( si, st_mode,    0%o,  );
	log_struct( si, st_nlink,   %d,   );
	log_struct( si, st_uid,     %d,   );
	log_struct( si, st_gid,     %d,   );
	log_struct( si, st_rdev,    %lld, );
	log_struct( si, st_size,    %lld, );
	log_struct( si, st_blksize, %ld,  );
	log_struct( si, st_blocks,  %lld, );
	log_struct( si, st_atime,   0x%08lx, );
	log_struct( si, st_mtime,   0x%08ls, );
	log_struct( si, st_ctime,   0x%08lx, );
}

void log_statvfs( struct statvfs * const sv )
{
	log_struct( sv, f_bsize,   %ld,     );
	log_struct( sv, f_frsize,  %ld,     );
	log_struct( sv, f_blocks,  %lld,    );
	log_struct( sv, f_bfree,   %lld,    );
	log_struct( sv, f_bavail,  %lld,    );
	log_struct( sv, f_files,   %lld,    );
	log_struct( sv, f_ffree,   %lld,    );
	log_struct( sv, f_favail,  %lld,    );
	log_struct( sv, f_fsid,    %ld,     );
	log_struct( sv, f_flag,    0x%08lx, );
	log_struct( sv, f_namemax, %ld,     );
}

void log_utime( struct utimbuf * const buf )
{
	log_struct( buf, actime,  0x%08lx, );
	log_struct( buf, modtime, 0x%08lx, );
}

