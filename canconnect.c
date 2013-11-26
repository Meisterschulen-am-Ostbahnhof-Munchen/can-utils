/*
 * canconnect.c - replay a compact CAN frame logfile to CAN devices
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

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

#define CHANNELS	20	/* anyone using more than 20 CAN interfaces at a time? */
#define COMMENTSZ 200
#define BUFSZ (sizeof("(1345212884.318850)") + IFNAMSIZ + 4 + CL_CFSZ + COMMENTSZ) /* for one line in the logfile */
#define STDOUTIDX	65536	/* interface index for printing on stdout - bigger than max uint16 */

const int canfd_on = 1;

extern int optind;

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [interface]\n\n", prg);
}


int main(int argc, char **argv)
{
	static char buf[BUFSZ], device[BUFSZ], ascframe[BUFSZ];
	struct sockaddr_can addr;
	static struct canfd_frame frame;
	static struct timeval today_tv, log_tv;
	int s; /* CAN_RAW socket */
	FILE *infile = stdin;
	static int opt;
	int txidx;       /* sendto() interface index */
	int eof, txmtu, devname;
	char *fret;
	struct ifreq ifr;

	while ((opt = getopt(argc, argv, "?")) != -1) {
		switch (opt) {
		case '?':
		default:
			print_usage(basename(argv[0]));
			return 1;
			break;
		}
	}

	devname = argc - optind; /* find real number of user assignments */

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;
	strcpy(ifr.ifr_name, argv[devname]);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
		perror("SIOCGIFINDEX");
	txidx = ifr.ifr_ifindex;
	addr.can_ifindex = ifr.ifr_ifindex;


	/* disable unneeded default receive filter on this RAW socket */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* try to switch the socket into CAN FD mode */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}


	while ( 1 ) {

		/* read first non-comment frame from logfile */
		while ((fret = fgets(buf, BUFSZ-1, infile)) != NULL && buf[0] != '(') {
			if (strlen(buf) >= BUFSZ-2) {
				fprintf(stderr, "comment line too long for input buffer\n");
				return 1;
			}
		}

		if (!fret)
			goto out; /* nothing to read */

		eof = 0;

		if (sscanf(buf, "(%ld.%ld) %s %s", &log_tv.tv_sec, &log_tv.tv_usec,
			   device, ascframe) != 4) {
			fprintf(stderr, "incorrect line format in logfile\n");
			return 1;
		}


		while (!eof) {

		  	for( ;; ) {

				if (strlen(device) >= IFNAMSIZ) {
					fprintf(stderr, "log interface name '%s' too long!", device);
					return 1;
				}



				txmtu = parse_canframe(ascframe, &frame);
				if (!txmtu) {
					fprintf(stderr, "wrong CAN frame format: '%s'!", ascframe);
					return 1;
				}

				addr.can_family  = AF_CAN;
				addr.can_ifindex = txidx; /* send via this interface */

				if (sendto(s, &frame, txmtu, 0,	(struct sockaddr*)&addr, sizeof(addr)) != txmtu) {
					perror("sendto");
					return 1;
				}


				/* read next non-comment frame from logfile */
				while ((fret = fgets(buf, BUFSZ-1, infile)) != NULL && buf[0] != '(') {
					if (strlen(buf) >= BUFSZ-2) {
						fprintf(stderr, "comment line too long for input buffer\n");
						return 1;
					}
				}

				if (!fret) {
					eof = 1; /* this file is completely processed */
					break;
				}

				if (sscanf(buf, "(%ld.%ld) %s %s", &log_tv.tv_sec, &log_tv.tv_usec,
					   device, ascframe) != 4) {
					fprintf(stderr, "incorrect line format in logfile\n");
					return 1;
				}

			}


			gettimeofday(&today_tv, NULL);

		} /* while (!eof) */

	} /* while */

out:

	close(s);
	fclose(infile);
	return 0;
}
