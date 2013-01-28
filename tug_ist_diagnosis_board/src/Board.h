/*
* Board.h is a header file defining necessary functionality for the Board.
*
* Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#define STX '2'

class Message
{
public:
   Message();
   ~Message();
   void initMessage();
   void parseBuffer();
   char getBuffer(int *length);
   
      
private:
   void create_threads();
   char delim;
   char command;
   ushort length;
   char n;
   int max_curr;
   int max_vol;
   unsigned char nbuffer[255];
   unsigned char * p;
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   struct hostent *host;
   struct sockaddr_in server_addr;
   bool sendSignal;

};
