/*
 * Copyright (c) 2013, Institute of Computer Aided Automation
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *      Most parts are based on the iotsys-server by Markus Jung
 * \author
 *      Lukas Hartung <lukas.hartung@student.tuwien.ac.at>
 */

#ifndef IOTSYS_H_
#define IOTSYS_H_


// Group communication definition
#define MAX_GC_HANDLERS 2
#define MAX_GC_GROUPS 5

//size of payload_buffer
#define PUT_BUFFER_SIZE 140

#define CHUNKS_TOTAL        1024


typedef void (*gc_handler) (char*);

// Data structure for storing group communication assignments.
// It is intended to store only the group identifier
// of a transient link-local scope multicast address (FF:12::XXXX)
typedef struct {
	int group_identifier;
	gc_handler handlers[MAX_GC_HANDLERS];
} gc_handler_t;

#if GROUP_COMM_ENABLED
	static struct simple_udp_connection broadcast_connection;
#endif

void iotsys_send(void* request, void* response, uint8_t *buffer, uint16_t preferred_size,  int32_t *offset, char *message, uint8_t size_msg);

/**
 * Returns a pointer to the payload. The payload is only returned for PUT and POST requests
**/
char * iotsys_process_request(void* request, gc_handler groupCommHandler);

void send_group_update(char* payload, size_t msgSize, gc_handler handler );


#endif /*IOTSYS_H_*/
