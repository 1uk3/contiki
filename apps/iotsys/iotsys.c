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

#include "contiki.h"
#include "contiki-net.h"
#include <string.h>
#include <stdio.h>
#include "iotsys.h"


/* For CoAP-specific example: not required for normal RESTful Web service. */
#if WITH_COAP == 3
#include "er-coap-03.h"
#elif WITH_COAP == 7
#include "er-coap-07.h"
#elif WITH_COAP == 12
#include "er-coap-12.h"
#elif WITH_COAP == 13
#include "er-coap-13.h"
#include "er-coap-13-engine.h"
#else
#warning "IoTSyS server example"
#endif /* CoAP-specific example */



#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif




char payload_buffer[PUT_BUFFER_SIZE];

gc_handler_t gc_handlers[MAX_GC_GROUPS];

coap_packet_t request;

static int msgid = 0;


int get_bool_value_obix(char* obix_object){

	if(strstr(obix_object, "true") != NULL){
		return 1;
	}
	return 0;
}

void send_message(const char* message, const uint16_t size_msg, void *request,
		void *response, uint8_t *buffer, uint16_t preferred_size,
		int32_t *offset) {
	PRINTF("Send Message: Size = %u, Offset = %ld\n", size_msg, *offset);
	PRINTF("Preferred Size: %d\n", preferred_size);

	uint16_t length;
	char *err_msg;

	length = size_msg - *offset;

	printf("length is: %d\n", length);

	if (length <= 0) {
		REST.set_response_status(response, REST.status.INTERNAL_SERVER_ERROR);
		err_msg = "calculation of message length error";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}

	if (preferred_size < 0 || preferred_size > REST_MAX_CHUNK_SIZE) {
		preferred_size = REST_MAX_CHUNK_SIZE;
		PRINTF(
				"Preferred size set to REST_MAX_CHUNK_SIZE = %d\n", preferred_size);
	}

	if (length > preferred_size) {
		PRINTF("Message still larger then preferred_size, truncating...\n");
		length = preferred_size;
		PRINTF("Length is now %u\n", length);

		memcpy(buffer, message + *offset, length);

		/* Truncate if above CHUNKS_TOTAL bytes. */
		if (*offset + length > CHUNKS_TOTAL) {
			PRINTF("Reached CHUNKS_TOTAL, truncating...\n");
			length = CHUNKS_TOTAL - *offset;
			PRINTF("Length is now %u\n", length);
			PRINTF("End of resource, setting offset to -1\n");
			*offset = -1;
		} else {
			/* IMPORTANT for chunk-wise resources: Signal chunk awareness to REST engine. */
			*offset += length;
			PRINTF("Offset refreshed to %ld\n", *offset);
		}
	} else {
		memcpy(buffer, message + *offset, length);
		*offset = -1;
	}

	PRINTF(
			"Sending response chunk: length = %u, offset = %ld\n", length, *offset);

	REST.set_header_etag(response, (uint8_t *) &length, 1);
	REST.set_response_payload(response, buffer, length);
}


char * iotsys_process_request(void* request, gc_handler groupCommHandler)
{
	const uint8_t *incoming = NULL;
	static size_t payload_len = 0;
	payload_buffer[0]=0;

	const char *uri_path = NULL;
	int len = REST.get_url(request, &uri_path);

	// for PUT and POST request we need to process the payload content
	if( REST.get_method_type(request) == METHOD_PUT || REST.get_method_type(request) == METHOD_POST){
		payload_len = REST.get_request_payload(request, &incoming);
		memcpy(payload_buffer, incoming, payload_len);
		payload_buffer[payload_len]=0;					//terminate the string!
	}

#if GROUP_COMM_ENABLED
	if(groupCommHandler != NULL){
		uip_ip6addr_t groupAddress;

		int16_t groupIdentifier = 0;

		if(strstr(uri_path, "joinGroup") && REST.get_method_type(request) == METHOD_POST ){
			printf("#### Join Group Called!");
			PRINTF("Join group called.\n");
			get_ipv6_multicast_addr(payload_buffer, &groupAddress);

			// join locally for the multicast address
			uip_ds6_maddr_add(&groupAddress);

			PRINT6ADDR(&groupAddress);
			extract_group_identifier(&groupAddress, &groupIdentifier);
			PRINTF("\n group identifier: %d\n", groupIdentifier);
			join_group(groupIdentifier, groupCommHandler);


		}
		else if(strstr(uri_path, "leaveGroup") && REST.get_method_type(request) == METHOD_POST){
			PRINTF("Leave group called.\n");
			get_ipv6_multicast_addr(payload_buffer, &groupAddress);
			PRINT6ADDR(&groupAddress);
			extract_group_identifier(&groupAddress, &groupIdentifier);
			PRINTF("\n group identifier: %d\n", groupIdentifier);
			leave_group(groupIdentifier,  groupCommHandler);
		}
	}
#endif

    return payload_buffer;
}



void iotsys_send(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset, char *message, uint8_t size_msg)
{
	
	char *err_msg;

	// Check the offset for boundaries of the resource data.
	if (*offset >= CHUNKS_TOTAL) {
		REST.set_response_status(response, REST.status.BAD_OPTION);
		// A block error message should not exceed the minimum block size (16).
		err_msg = "BlockOutOfScope";
		REST.set_response_payload(response, err_msg, strlen(err_msg));
		return;
	}
	REST.set_header_content_type(response, REST.type.APPLICATION_XML);

	// First block of the message
	if (*offset <= 0) {
		if (size_msg <= 0) {
			PRINTF("ERROR while creating message!\n");
			REST.set_response_status(response,
					REST.status.INTERNAL_SERVER_ERROR);
			err_msg = "ERROR while creating message :\\";
			REST.set_response_payload(response, err_msg, strlen(err_msg));
			return;
		}
	}

	send_message(message, size_msg, request, response, buffer, preferred_size, offset);
}


// creates an IPv6 address from the provided string
// note: the provided string is manipulated.
void get_ipv6_multicast_addr(char* input, uip_ip6addr_t* address){
	// first draft, assume an IPv6 address with explicit notation like FF12:0000:0000:0000:0000:0000:0000:0001

	// in this case the address shall be an char array with 32 (hex chars) + 7 (: delim) + 1 string delimiter
	// replace all : with a whitespace
	char* curChar;
	char* pEnd;
	int addr1, addr2,addr3, addr4, addr5, addr6, addr7, addr8;

	// move to the beginning of the IPv6 address --> assume it starts with FF
	input = strstr(input, "FF");

	curChar = strchr(input, ':');

	while (curChar != NULL)
	{
	   *curChar = ' '; // replace : with space
	   curChar=strchr(curChar+1,':');
	}

	addr1 = strtol(input,&pEnd,16); // FF12 block
	addr2 = strtol(pEnd, &pEnd,16); // 0000 block
	addr3 = strtol(pEnd, &pEnd,16); // 0000 block
	addr4 = strtol(pEnd, &pEnd,16); // 0000 block
	addr5 = strtol(pEnd, &pEnd,16); // 0000 block
	addr6 = strtol(pEnd, &pEnd,16); // 0000 block
	addr7 = strtol(pEnd, &pEnd,16); // 0000 block
	addr8 = strtol(pEnd, &pEnd,16); // 0000 block

	// create ipv6 address with 16 bit words
	uip_ip6addr(address,addr1, addr2, addr3, addr4, addr5, addr6, addr7, addr8); // 0001 block
}





void send_coap_multicast(char* payload, size_t msgSize, uip_ip6addr_t* mc_address){
	 coap_init_message(&request, COAP_TYPE_NON, COAP_PUT, msgid++ );
	 coap_set_payload(&request, (uint8_t *)payload, msgSize);
	 coap_set_header_uri_path(&request, "");
	 coap_simple_request(mc_address, 5683, &request);
	 printf("\n--Done--\n");
}

void send_group_update(char* payload, size_t msgSize, gc_handler handler ){
	PRINTF("sending group update\n");
	int i,l=0;
	uip_ip6addr_t gc_address;

	for(i = 0; i < MAX_GC_GROUPS; i++){
		// adding gc handler
		for(l=0; l < MAX_GC_HANDLERS; l++){
			if(gc_handlers[i].handlers[l] == handler ){
				printf("Sending update to group identifier %d", gc_handlers[i].group_identifier);
				uip_ip6addr(&gc_address, 0xff15, 0, 0, 0, 0, 0, 0, gc_handlers[i].group_identifier);
				send_coap_multicast(payload, msgSize, &gc_address);
			}
		}

	}
}

void extract_group_identifier(uip_ip6addr_t* ipv6Address, uint16_t* groupIdentifier ){
	*groupIdentifier = 0;
	*groupIdentifier =  ((uint8_t *)ipv6Address)[14];
	*groupIdentifier <<= 8;
	*groupIdentifier += ((uint8_t *)ipv6Address)[15];
}

void join_group(int groupIdentifier, gc_handler handler  ){
	int i,l=0;
	// use last 32 bits
	for(i = 0; i < MAX_GC_GROUPS; i++){
		if(gc_handlers[i].group_identifier == 0 || gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot

			gc_handlers[i].group_identifier = groupIdentifier;
			//gc_handlers[i].group_identifier &= (groupAddress.u16[6] << 16);
			printf("Assigned slot: %d\n", gc_handlers[i].group_identifier);

			// adding gc handler
			for(l=0; l < MAX_GC_HANDLERS; l++){
				if(gc_handlers[i].handlers[l] == NULL ||  gc_handlers[i].handlers[l] == &handler ){
					gc_handlers[i].handlers[l] = handler;
					PRINTF("(Re-)Assigned callback on slot %d\n", l);
					break;
				}
			}
			break;
		}
	}
}

void leave_group(int groupIdentifier, gc_handler handler){
	int i,l=0;
	for(i = 0; i < MAX_GC_GROUPS; i++){
		if(gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot

			gc_handlers[i].group_identifier = groupIdentifier;
			//gc_handlers[i].group_identifier &= (groupAddress.u16[6] << 16);
			PRINTF("Found slot: %d\n", gc_handlers[i].group_identifier);

			// adding gc handler
			for(l=0; l < MAX_GC_HANDLERS; l++){

				if(gc_handlers[i].handlers[l] == handler ){
					gc_handlers[i].handlers[l] = NULL;
					PRINTF("Removed callback from slot %d\n", l);
					break;
				}
			}
			break;
		}
	}
}

void group_comm_handler(const uip_ipaddr_t *sender_addr,
         const uip_ipaddr_t *receiver_addr,
         const uint8_t *data,
         uint16_t datalen)
{
	uint16_t groupIdentifier;
	PRINT6ADDR(sender_addr);
	PRINT6ADDR(receiver_addr);
	uint8_t i,l=0;

	groupIdentifier =  ((uint8_t *)receiver_addr)[14];
    groupIdentifier <<= 8;
    groupIdentifier += ((uint8_t *)receiver_addr)[15];
    PRINTF("\n######### Data received on group comm handler with length %d for group identifier %d\n",
		 datalen, groupIdentifier);

    for(i = 0; i < MAX_GC_GROUPS; i++){
    			if(gc_handlers[i].group_identifier == groupIdentifier){ // free slot or same slot
    				for(l=0; l < MAX_GC_HANDLERS; l++){
    					if(gc_handlers[i].handlers[l] != NULL){
    						gc_handlers[i].handlers[l](data);
    					}
    				}
    			}
        	}
}

