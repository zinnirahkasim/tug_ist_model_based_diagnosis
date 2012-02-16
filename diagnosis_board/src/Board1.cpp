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
char send_data [1024] , recv_data[1024];
char nbuffer[95];
int channels;
struct board
{
         char on_off[10];
         int max_vol[10];
         int max_cur[10];
         int pr_vol[10];
         int pr_cur[10];
};
struct board board_info; 
struct header
{
          char delim;
					char command;
					float length;
};       
struct header h;
struct sockaddr_in server_addr,client_addr;    
int sin_size;
void send_boardSpecifications()
{
					channels = 10;
          char delim = '2';
          char command = '0';
          ushort length = 8*channels+1;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));
          char n = (char) channels;
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)], &n,sizeof(n));
          int offset = sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n);
          
          //float max_curr = 23.1;
          //float max_vol = 50.3;
          for(int channel=0;channel<channels;channel++)
          {
          int max_vol = board_info.max_vol[channel];
          int max_curr = board_info.max_cur[channel];
          memcpy(&nbuffer[offset], &max_curr, sizeof(max_curr));
          memcpy(&nbuffer[offset+sizeof(max_curr)], &max_vol, sizeof(max_vol));
          offset = offset + 8;
          }
          printf("\n\n SENT DATA from Server:\nHeader: delim = %c , command = %c , length = %i , Channels = %i  " , *nbuffer, *(&nbuffer[1]), *(&nbuffer[2]), *(&nbuffer[4]));
                  offset = 5;
                  for(int channel=0;channel<channels;channel++)
                  {
                     printf("\n Channel# : %d, Max_Vol= %d, Max_Cur= %d", channel,*(&nbuffer[offset]),*(&nbuffer[offset+4]));
                     offset +=8;
                  }
}                

void send_boardMeasurments()
{
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
          
          //float max_curr = 23.1;
          //float max_vol = 50.3;
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
          printf("\n\n SENT DATA from Server:\nHeader: delim = %c , command = %c , length = %i , Channels = %i  " , *nbuffer, *(&nbuffer[1]), *(&nbuffer[2]), *(&nbuffer[4]) );
          offset = 5;
                  for(int channel=0;channel<channels;channel++)
                  {
                    printf("\n Channel# : %d, Switch ON/OFF = %c Flowing_Vol= %d, Flowing_Cur= %d", channel,*(&nbuffer[offset]),*(&nbuffer[offset+1]), *(&nbuffer[offset+5]));
                  // printf("\n Channel# : %d, Switch ON/OFF = %c Flowing_Vol= %d, Flowing_Cur= %d", channel,board_info.on_off[channel],board_info.pr_vol[channel], board_info.pr_cur[channel]);
                    offset = offset + 9;
                  }//for 
}

void switch_OnOffChannel(char channel, char status)
{
//int chan = (int) channel;
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
                 board_info.on_off[channel] = '0';
                 board_info.max_vol[channel] = channel*10;
                 board_info.max_cur[channel] = channel*20;
                 board_info.pr_vol[channel] = channel-1;
                 board_info.pr_cur[channel] = channel+1;
              }
             else
                {
                 board_info.on_off[channel] = '1';
                 board_info.max_vol[channel] = channel*5;
                 board_info.max_cur[channel] = channel*8;
                 board_info.pr_vol[channel] = channel;
                 board_info.pr_cur[channel] = channel+2;
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
              
             //float value=54.1;
             //char array[4];
             //void* array;
             //*array = &value;
             //memcpy(array, &value, sizeof (value));
             //printf("VAEL %f",(float) *array);

              bytes_recieved = recv(connected,recv_data,1024,0);
              //printf("\n RECIEVED BYTES = %d " , bytes_recieved);
              recv_data[bytes_recieved] = '\0';
              
              if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
              {
                close(connected);
                break;
              }
              printf("\n RECIEVED from Client:\ndelim = %c , command = %c , length = %d" ,*recv_data, *(&recv_data[1]), *(&recv_data[2]));
              //printf("\n SENT DATA from Server: delim = %c , command = %c , length = %i , Channels = %i  " , *nbuffer, *(&nbuffer[1]), *(&nbuffer[2]) );
             
              char command = *(&recv_data[1]);
              if(command=='3')
                 send_boardMeasurments();
              else if(command=='4')
                   switch_OnOffChannel(*(&recv_data[4]),*(&recv_data[5]));
             
              fflush(stdout);
            }
        }       

      close(sock);
      return 0;
} 

