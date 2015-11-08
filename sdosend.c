/*
 * sdosend.c - simple command line tool to send SDO frames via socketcan
 *
 * Copyright (c) 2015 Maximilian Pachl
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
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define SDO_CMD_UPLOAD		0x40
#define SDO_DATA_LEN		4
#define SDO_ERROR		0x80

#define SDO_REQ_OFFSET		0x600
#define SDO_RESP_OFFSET		0x580

typedef struct _SDO_abort_code {
      uint32_t code;
      char *description;
} SDO_abort_code_t;
static SDO_abort_code_t sdo_errors[] = {
    {0x05030000, "Toggle bit not alternated" },
    {0x05040000, "SDO protocol timed out" },
    {0x05040001, "Client/Server command specifier not valid or unknown" },
    {0x05040002, "Invalid block size (Block Transfer mode only)" },
    {0x05040003, "Invalid sequence number (Block Transfer mode only)" },
    {0x05030004, "CRC error (Block Transfer mode only)" },
    {0x05030005, "Out of memory"},
    {0x06010000, "Unsupported access to an object"},
    {0x06010001, "Attempt to read a write-only object"},
    {0x06010002, "Attempt to write a read-only object"},
    {0x06020000, "Object does not exist in the Object Dictionary"},
    {0x06040041, "Object can not be mapped to the PDO"},
    {0x06040042, "The number and length of the objects to be mapped would exceed PDO length"},
    {0x06040043, "General parameter incompatibility reason"},
    {0x06040047, "General internal incompatibility in the device"},
    {0x06060000, "Object access failed due to a hardware error"},
    {0x06060010, "Data type does not match, lengh of service parameter does not match"},
    {0x06060012, "Data type does not match, lengh of service parameter is too high"},
    {0x06060013, "Data type does not match, lengh of service parameter is too low"},
    {0x06090011, "Sub-index does not exist"},
    {0x06090030, "Value range of parameter exceeded (only for write access)"},
    {0x06090031, "Value of parameter written too high"},
    {0x06090032, "Value of parameter written too low"},
    {0x06090036, "Maximum value is less than minimum value"},
    {0x08000000, "General error"},
    {0x08000020, "Data can not be transferred or stored to the application"},
    {0x08000021, "Data can not be transferred or stored to the application because of local control"},
    {0x08000022, "Data can not be transferred or stored to the application because of the present device state"},
    {0x08000023, "Object Dictionary dynamic generation fails or no Object Dictionary is present (e.g. OD is generated from file and generation fails because of a file error)"},
    {0x0000000, NULL}
};

char *sdo_error(uint32_t code)
{
	/* lookup the string corepsonding to the error code */
	SDO_abort_code_t *error = sdo_errors;
	while (error->code)
	{
		if (error->code == code)
			return error->description;

		error++;
	}

	return "unknown error";
}

int parse_sdo(char *direction, char *str, struct can_frame *cf, int *nid, int *idx, int *subidx)
{
	int node = -1, index = -1, subindex = -1, datalen = 0;
	uint8_t data[SDO_DATA_LEN];
	char *tokens = ":#.";
	char *token;

 	/* init the canframe */
	memset(cf, 0, sizeof(*cf));

	/* walk through the tokens of the string representation of the sdo req */
	token = strtok(str, tokens);
	while (token != NULL) 
	{
    	if (node == -1)	/* nodeid */
    		node = strtol(token, NULL, 16);

    	else if (index == -1) /* index */
    		index = strtol(token, NULL, 16);

    	else if (subindex == -1) /* subindex */
    		subindex = strtol(token, NULL, 16);

    	else { /* payload */
    		if (datalen >= SDO_DATA_LEN)
    			return 0;
    		else
    			data[datalen++] = (uint8_t)strtol(token, NULL, 16);
    	}

		token = strtok(NULL, tokens);
	}

	/* ensure that a valid direction is supplied */
	if (strcmp(direction, "download") == 0) { /* download request */
		if (datalen > 0 && datalen <= SDO_DATA_LEN)
			cf->data[0] = 0x20 | ((SDO_DATA_LEN - datalen) << 2) | 0x03;
		else
			return 0;

	} else if (strcmp(direction, "upload") == 0) { /* upload request */
		cf->data[0] = SDO_CMD_UPLOAD;

		/* we do not want to send payload in upload request */
		datalen = 0;

	} else { /* invalid direction */
		return 0;
	}

	/* these three values are obligatory! */
	if (node == -1 || index == -1 || subindex == -1)
		return 0;

	if (nid != NULL)
		*nid = node;

	if (idx != NULL)
		*idx = index;

	if (subidx != NULL)
		*subidx = subindex;

	/* fill meta data */
	cf->can_id = SDO_REQ_OFFSET + node;
	cf->can_dlc = 8;

	/* fill the canframe with the obligatory parameters */
	cf->data[1] = index & 0xFF;
	cf->data[2] = (index >> 8) & 0xFF;
	cf->data[3] = subindex & 0xFF;

	if (datalen > 0)
		memcpy(&cf->data[4], data, datalen);

	return 1;
}

int main(int argc, char **argv)
{
	int s, nodeid, index, subindex;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame request, response;
	fd_set set;
  	struct timeval timeout;
  	int rv;

	/* check command line options */
	if (argc != 4) {
		fprintf(stderr, "Usage: %s <device> <upload|download> <node>:<index>:<subindex>#data\n", argv[0]);
		return 1;
	}

	/* parse the string represenatation of the sdo request. */
	if (!parse_sdo(argv[2], argv[3], &request, &nodeid, &index, &subindex))
	{
		fprintf(stderr, "Wrong CAN-frame format!\n");
		return -1;
	}

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	/* find the interface */
	strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return 1;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* send request */
	if (write(s, &request, sizeof(request)) != sizeof(request)) {
		perror("write");
		return 1;
	}

	/* timed wait for the response */
	FD_ZERO(&set);
	FD_SET(s, &set);

	timeout.tv_sec = 0;
  	timeout.tv_usec = 200000;

	/* wait for the right resposne canframe */
	while (response.can_id != (SDO_RESP_OFFSET + nodeid) &&
	       *((uint16_t*)&response.data[1]) != index &&
	       *((uint16_t*)&response.data[3]) != subindex)
	{
		/* timeout exeeceded -> exit */
		if (timeout.tv_sec == 0 && timeout.tv_usec == 0)
		{
			fprintf(stderr, "request exeeded the timout\n");
			return -1;
		}

		rv = select(s + 1, &set, NULL, NULL, &timeout);
		if(rv == -1) {
			perror("select"); /* an error accured */
			return -1;

		} else if(rv == 0) { /* timeout occoured */
			fprintf(stderr, "request exeeded the timout\n");
			return -1;

		} else {	/* a new canframe is available -> read resposne */
			if (read(s, &response, sizeof(response)) != sizeof(response)) {
				perror("read");
				return 1;
			}
		}
	}

	uint32_t payload = *((uint32_t*)&response.data[4]);

	/* uuuupps an sdo error occoured */
	if (response.data[0] == SDO_ERROR) {
		fprintf(stderr, "error while executing request: %s\n", sdo_error(payload));
		return -1;

	/* everything is file, display the results */
	} else {
		if (strcmp(argv[2], "download") == 0)
			printf("download successfull\n");

		else {
			printf("0x%x 0x%x:0x:%x = 0x%x\n", nodeid, index, subindex, payload);
		}

	}


	close(s);

	return 0;
}
