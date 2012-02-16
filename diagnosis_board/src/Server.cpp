/* tcpserver.c */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>


int main()
{
        int sock, connected, bytes_recieved , True = 1;  
        char send_data [1024] , recv_data[1024];
        unsigned char nbuffer[255];
        unsigned char * p; 
        struct header
        {
          char delim;
					char command;
					float length;
        };       
        struct header h;
        struct sockaddr_in server_addr,client_addr;    
        int sin_size;
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket");
            exit(1);
        }

        if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&True,sizeof(int)) == -1) {
            perror("Setsockopt");
            exit(1);
        }
        
        server_addr.sin_family = AF_INET;         
        server_addr.sin_port = htons(5000);     
        server_addr.sin_addr.s_addr = INADDR_ANY; 
        bzero(&(server_addr.sin_zero),8); 

        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))
                                                                       == -1) {
            perror("Unable to bind");
            exit(1);
        }

        if (listen(sock, 5) == -1) {
            perror("Listen");
            exit(1);
        }
		
	printf("\nTCPServer Waiting for client on port 5000");
        fflush(stdout);


        while(1)
        {  

            sin_size = sizeof(struct sockaddr_in);

      //      connected = accept(sock, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size);

            printf("\n I got a connection from (%s , %d)",
                   inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
          char delim = 2;
          char command = 0;
          ushort length = 9;
//          memcpy(nbuffer,&delim, sizeof(delim));
//          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
//          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));
					
          p = nbuffer;

          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;


                    

          char n = 1;
          //int max_curr = 28;
          //int max_vol = 50;
          float max_curr = 23.1;
          float max_vol = 50.3;
          printf("SIZE of maxcurr=%d",sizeof(max_curr));

          *p=n;
          p++;

          *((float *)p) = max_curr;
          p+=4;
          *((float *)p) = max_vol;

//          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)], &n, sizeof(n));
//          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n)], &max_curr, sizeof(max_curr));
//          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n)+sizeof(max_curr)], &max_vol, sizeof(max_vol));
            while (1)
            {
              //send(connected,(void*)&nbuffer,sizeof(nbuffer), 0);
              //printf("\n SENT DATA from Server: delim = %c , command = %c , length = %i , Channels = %c , Max_Cur = %d , Max_Vol = %d " , *nbuffer, *(&nbuffer[1]), *(&nbuffer[2]) , *(&nbuffer[4]), *(&nbuffer[5]), *(&nbuffer[9]));
              printf("\n SENT DATA from Server: delim = %i , command = %i , length = %i , Channels = %i , Max_Cur = %f , Max_Vol = %f " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4], *((float *)(nbuffer+5)), *((float *)(nbuffer+9)));
             
             //float value=54.1;
             //char array[4];
             //void* array;
             //*array = &value;
             //memcpy(array, &value, sizeof (value));
             //printf("VAEL %f",(float) *array);

              //bytes_recieved = recv(connected,recv_data,1024,0);
              //printf("\n RECIEVED BYTES = %d " , bytes_recieved);
              recv_data[bytes_recieved] = '\0';

              if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
              {
                close(connected);
                break;
              }
              fflush(stdout);
            }
        }       

      close(sock);
      return 0;
} 

