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
int sock, connected, bytes_recieved , True = 1;  
unsigned char send_data[255] , recv_data[255];
unsigned char nbuffer[255];
unsigned char * p;
int channels;
struct board
{
         char on_off[10];
         float max_vol[10];
         float max_cur[10];
         float pr_vol[10];
         float pr_cur[10];
};
struct board board_info; 
struct sockaddr_in server_addr,client_addr;    
int sin_size;
void send_boardSpecifications()
{          
          p = nbuffer;
					channels = 10;
          char delim = 2;
          char command = 0;
          ushort length = 8*channels+1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;


          char n = (char) channels;
          
         *p=n;
          p++;

          int offset = sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n);
          
          for(int channel=0;channel<channels;channel++)
          {
          float max_curr = board_info.max_cur[channel];
          float max_vol = board_info.max_vol[channel];
          *((float *)p) = max_curr;
          p+=4;
          *((float *)p) = max_vol;
          p+=4;
          }
          printf("\n SENT DATA from Server: delim = %i , command = %i , length = %i , Channels = %i " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4]);

          offset = 5;
          for(int channel=0;channel<channels;channel++)
          {
           printf("\n Channel# : %d, Max_Vol= %f, Max_Cur= %f", channel,*((float *)(nbuffer+offset)), *((float *)(nbuffer+offset+4)));
           offset +=8;
          }
}                

void send_boardMeasurments()
{         
          p = nbuffer;
					channels = 10;
          char delim = 2;
          char command = 2;
          ushort length = 9*channels+1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;


          char n = (char) channels;
          
         *p=n;
          p++;

          int offset = sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n);
          
          for(int channel=0;channel<channels;channel++)
          {
          char on_off = board_info.on_off[channel];
          float pr_curr = board_info.pr_cur[channel];
          float pr_vol = board_info.pr_vol[channel];
          *p=on_off;
          p++;
          *((float *)p) = pr_curr;
          p+=4;
          *((float *)p) = pr_vol;
          p+=4;
          }
          printf("\n SENT DATA from Server: delim = %i , command = %i , length = %i , Channels = %i " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4]);

          offset = 5;
          for(int channel=0;channel<channels;channel++)
          {
           printf("\n Channel# : %d, On/Off= %i, Max_Vol= %f, Max_Cur= %f", channel,nbuffer[offset] ,*((float *)(nbuffer+offset)), *((float *)(nbuffer+offset+4)));
           offset +=9;
          }
          
					channels = 10;
          char delim = '2';
          char command = '2';
          ushort length = 9*channels+1;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));
          char n = (char) channels;
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)], &n,sizeof(n));
          int offset = sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n);
          
          for(int channel=0;channel<channels;channel++)
          {
          char on_off = board_info.on_off[channel];
          int pr_vol = board_info.pr_vol[channel];
          int pr_curr = board_info.pr_cur[channel];
          memcpy(&nbuffer[offset], &on_off, sizeof(on_off));
          memcpy(&nbuffer[offset+sizeof(on_off)], &pr_curr, sizeof(pr_curr));
          memcpy(&nbuffer[offset+sizeof(on_off)+sizeof(pr_curr)], &pr_vol, sizeof(pr_vol));
          offset = offset + 9;
          }
          printf("\n\n SENT DATA from Server:\nHeader: delim = %i , command = %i , length = %i , Channels = %i  " , *nbuffer, *(&nbuffer[1]), *(&nbuffer[2]), *(&nbuffer[4]) );
          offset = 5;
                  for(int channel=0;channel<channels;channel++)
                  {
                    printf("\n Channel# : %d, Switch ON/OFF = %c Flowing_Vol= %d, Flowing_Cur= %d", channel,*(&nbuffer[offset]),*(&nbuffer[offset+1]), *(&nbuffer[offset+5]));
                  
                    offset = offset + 9;
                  }//for 
}

void switch_OnOffChannel(char channel, char status)
{
printf("%i channel to %c ",channel,status);
if(channel=='0')
   {
    board_info.on_off[0] = status;
    printf("%c",board_info.on_off[0]);
    }
else if(channel=='1')
   board_info.on_off[1] = status;
else if(channel=='2')
   board_info.on_off[2] = status;
else if(channel=='3')
   board_info.on_off[3] = status;
else if(channel=='4')
   board_info.on_off[4] = status;
else if(channel=='5')
   board_info.on_off[5] = status;
else if(channel=='6')
   board_info.on_off[6] = status;
else if(channel=='7')
   board_info.on_off[7] = status;
else if(channel=='8')
   board_info.on_off[8] = status;
else if(channel=='9')
   board_info.on_off[9] = status;
else
  printf("%c channel does not exist",channel);

}

int main()
{
        
        for(int channel=0;channel<10;channel++)
           {
             if((channel%2)==0)
               { 
                 board_info.on_off[channel] = 0;
                 board_info.max_vol[channel] = channel*10+0.1*channel;
                 board_info.max_cur[channel] = channel*20+0.1*channel;
                 board_info.pr_vol[channel] = channel-1+0.1*channel;
                 board_info.pr_cur[channel] = channel+1+0.1*channel;
              }
             else
                {
                 board_info.on_off[channel] = 1;
                 board_info.max_vol[channel] = channel*5+0.1*channel;
                 board_info.max_cur[channel] = channel*8+0.1*channel;
                 board_info.pr_vol[channel] = channel+0.1*channel;
                 board_info.pr_cur[channel] = channel+2+0.1*channel;
                 }
           }        

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

            connected = accept(sock, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size);

            printf("\n I got a connection from (%s , %d)",
                   inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
            send_boardSpecifications();
            //send_boardMeasurments();
            while (1)
            { 
              send(connected,(void*)&nbuffer,sizeof(nbuffer), 0);
              
              bytes_recieved = recv(connected,recv_data,1024,0);
              recv_data[bytes_recieved] = '\0';
              
              if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
              {
                close(connected);
                break;
              }
              printf("\n RECIEVED from Client:\ndelim = %i , command = %i , length = %d" ,*recv_data, *(&recv_data[1]), *(&recv_data[2]));
              
             
              char command = recv_data[1];
              if(command==3)
                 send_boardMeasurments();
              else if(command==4)
                   switch_OnOffChannel(*(&recv_data[4]),*(&recv_data[5]));
             
              fflush(stdout);
            }
        }       

      close(sock);
      return 0;
} 

