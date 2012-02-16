/* tcpclient.c */

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
struct header
        {
          char delim;
					char command;
					ushort length;
        };

int main()

{
        char delim;
        char command;
        ushort length;
        void *buffer = malloc(sizeof(delim)+sizeof(command)+sizeof(length));
        char nbuffer[15] = "";
        //void *buffer[3];
        int sock, bytes_recieved;  
        char send_data[1024],recv_data[1024];
        struct hostent *host;
        struct sockaddr_in server_addr;  
        struct header h;
        host = gethostbyname("127.0.0.1");

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket");
            exit(1);
        }

        server_addr.sin_family = AF_INET;     
        server_addr.sin_port = htons(5000);   
        server_addr.sin_addr = *((struct in_addr *)host->h_addr);
        bzero(&(server_addr.sin_zero),8); 

        if (connect(sock, (struct sockaddr *)&server_addr,
                    sizeof(struct sockaddr)) == -1) 
        {
            perror("Connect");
            exit(1);
        }

        while(1)
        {
        
          bytes_recieved=recv(sock,recv_data,1024,0);
          recv_data[bytes_recieved] = '\0';
 
          if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
          {
           close(sock);
           break;
          }
           //printf("\n RECIEVED DATA from Server: delim = %c , command = %c , length = %d , Channels = %c , Max_Cur = %d , Max_Vol = %d " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]) , *(&recv_data[4]), *(&recv_data[5]), *(&recv_data[9]));
           printf("\n RECIEVED DATA from Server: delim = %c , command = %c , length = %i , Channels = %c , Max_Cur = %f , Max_Vol = %f " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]) , *(&recv_data[4]), *(&recv_data[5]), *(&recv_data[9]));
           //printf("\nRecieved data = %s " , recv_data);
           
           printf("\nSEND (q or Q to quit) : ");
           char d[2],c;
           float f;
           gets(d);
           printf("after scanf");
          delim = '2';
          command = '4';
          length = 7;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));            
          if (d!="q")
             send(sock,(void*)&nbuffer,sizeof(nbuffer), 0); 
          else
          {
             send(sock,(void*)&nbuffer,sizeof(nbuffer), 0);  
             close(sock);
           break;printf("\n RECIEVED DATA : delim = %c , command = %c , length = %f " , h.delim,h.command,h.length);
          }
          //printf("\n SENT DATA : delim = %c , command = %c , length = %d  " ,*nbuffer, *(&nbuffer[sizeof(delim)]), *(&nbuffer[sizeof(delim)+sizeof(command)]));
        }   
return 0;
}

