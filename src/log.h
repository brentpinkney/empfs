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

#ifndef _LOG_H_
#define _LOG_H_
#include <stdio.h>

#define log_struct( st, field, format, typecast ) \
	log_msg( "    " #field " = " #format "\n", typecast st->field )

FILE *log_open( void );
void log_fi( struct fuse_file_info *fi );
void log_stat( struct stat *si );
void log_statvfs( struct statvfs *sv );
void log_utime( struct utimbuf *buf );

void log_msg( const char *format, ... );
#endif
