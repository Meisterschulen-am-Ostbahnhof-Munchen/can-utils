/*
 * canconnect.c - connect canif with stdin and stdout
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <libgen.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

#define COMMENTSZ 200
#define BUFSZ (sizeof("(1345212884.318850)") + IFNAMSIZ + 4 + CL_CFSZ + COMMENTSZ) /* for one line in the logfile */

extern int optind;
int txidx;
char* ifname;

void print_usage( char *prg ) {
  fprintf( stderr, "\nUsage: %s [interface]\n\n", prg );
}


static void* print( void* s ) {
  struct msghdr msg;
  char ctrlmsg[CMSG_SPACE( sizeof( struct timeval ) ) + CMSG_SPACE( sizeof( __u32 ) )];
  struct iovec iov;
  struct canfd_frame frame;
  struct sockaddr_can addr;
  struct timeval tv;
  fd_set rdfs;
  int ret, nbytes;
  int skt = *( ( int* )s );
  unsigned char view = 0;

  iov.iov_base = &frame;
  msg.msg_name = &addr;
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = &ctrlmsg;

  addr.can_family = AF_CAN;
  addr.can_ifindex = txidx;

  for( ;; ) {
    FD_ZERO( &rdfs );
    FD_SET( skt, &rdfs );
    if ( ( ret = select( skt+1, &rdfs, NULL, NULL, NULL ) ) <= 0 ) {
      continue;
    }

    if ( FD_ISSET( skt, &rdfs ) ) {
      iov.iov_len = sizeof( frame );
      msg.msg_namelen = sizeof( addr );
      msg.msg_controllen = sizeof( ctrlmsg );
      msg.msg_flags = 0;

      nbytes = recvmsg( skt, &msg, 0 );
      if ( nbytes < 0 ) {
        perror( "read" );
        return NULL;
      }

      /* once we detected a EFF frame indent SFF frames accordingly */
      if ( frame.can_id & CAN_EFF_FLAG )
        view |= CANLIB_VIEW_INDENT_SFF;
      /* print CAN frame in log file style to stdout */
      gettimeofday( &tv, NULL );
      printf( "(%010ld.%06ld) ", tv.tv_sec, tv.tv_usec );
      printf( "%*s ", 10, ifname );
      fprint_canframe( stdout, &frame, "\n", 0, CAN_MTU );
      fflush( stdout );
    }
  }
  return NULL;
}

int main( int argc, char **argv ) {
  static char buf[BUFSZ], device[BUFSZ], ascframe[BUFSZ];
  struct sockaddr_can addr;
  static struct canfd_frame frame;
  static struct timeval today_tv, log_tv;
  int s; /* CAN_RAW socket */
  FILE *infile = stdin;
  static int opt;
  int eof, txmtu, devname;
  char *fret;
  struct ifreq ifr;
  pthread_t prt;

  while ( ( opt = getopt( argc, argv, "?" ) ) != -1 ) {
    switch ( opt ) {
      case '?':
      default:
        print_usage( basename( argv[0] ) );
        return 1;
        break;
    }
  }

  if( argc - optind <= 0 ) {
    print_usage( basename( argv[0] ) );
    return 1;
  }

  devname = argc - optind; /* find real number of user assignments */

  /* open socket */
  if ( ( s = socket( PF_CAN, SOCK_RAW, CAN_RAW ) ) < 0 ) {
    perror( "socket" );
    return 1;
  }

  ifname = argv[devname];

  addr.can_family = AF_CAN;
  strcpy( ifr.ifr_name, argv[devname] );
  if ( ioctl( s, SIOCGIFINDEX, &ifr ) < 0 ) perror( "SIOCGIFINDEX" );
  txidx = ifr.ifr_ifindex;
  addr.can_ifindex = ifr.ifr_ifindex;


  if ( bind( s, ( struct sockaddr * )&addr, sizeof( addr ) ) < 0 ) {
    perror( "bind" );
    return 1;
  }


  pthread_create( &prt, NULL, print, &s );

  for( ;; ) {

    /* read first non-comment frame from logfile */
    while ( ( fret = fgets( buf, BUFSZ-1, infile ) ) != NULL && buf[0] != '(' ) {
      if ( strlen( buf ) >= BUFSZ-2 ) {
        fprintf( stderr, "comment line too long for input buffer\n" );
        return 1;
      }
    }

    if ( !fret )
      goto out; /* nothing to read */

    eof = 0;

    if ( sscanf( buf, "(%ld.%ld) %s %s", &log_tv.tv_sec, &log_tv.tv_usec,
                 device, ascframe ) != 4 ) {
      fprintf( stderr, "incorrect line format in logfile\n" );
      return 1;
    }


    while ( !eof ) {

      for( ;; ) {

        if ( strlen( device ) >= IFNAMSIZ ) {
          fprintf( stderr, "log interface name '%s' too long!", device );
          return 1;
        }



        txmtu = parse_canframe( ascframe, &frame );
        if ( !txmtu ) {
          fprintf( stderr, "wrong CAN frame format: '%s'!", ascframe );
          return 1;
        }

        addr.can_family  = AF_CAN;
        addr.can_ifindex = txidx; /* send via this interface */

        if ( sendto( s, &frame, txmtu, 0,	( struct sockaddr* )&addr, sizeof( addr ) ) != txmtu ) {
          perror( "sendto" );
          return 1;
        }


        /* read next non-comment frame from logfile */
        while ( ( fret = fgets( buf, BUFSZ-1, infile ) ) != NULL && buf[0] != '(' ) {
          if ( strlen( buf ) >= BUFSZ-2 ) {
            fprintf( stderr, "comment line too long for input buffer\n" );
            return 1;
          }
        }

        if ( !fret ) {
          eof = 1; /* this file is completely processed */
          break;
        }

        if ( sscanf( buf, "(%ld.%ld) %s %s", &log_tv.tv_sec, &log_tv.tv_usec,
                     device, ascframe ) != 4 ) {
          fprintf( stderr, "incorrect line format in logfile\n" );
          return 1;
        }

      }


      gettimeofday( &today_tv, NULL );

    }

  }

out:

  close( s );
  fclose( infile );
  pthread_join( prt, NULL );
  return 0;
}
