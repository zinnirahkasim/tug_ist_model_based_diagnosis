/* Diagnosis Board Server*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <boost/thread.hpp>
#include <stdlib.h>

int BUFF_SIZE;
int sock, connected, bytes_recieved , True = 1;  
char send_data[255] , recv_data[255];
unsigned char nbuffer[255];
unsigned char * p;
int channels;
int MAX_channels = 8;
struct board
{
         char on_off[8];
         float max_vol[8];
         float max_cur[8];
         float pr_vol[8];
         float pr_cur[8];
};
struct board board_info; 
struct sockaddr_in server_addr,client_addr;    
int sin_size;

void take_boardSpecifications()
{          
          p = nbuffer;
					channels = MAX_channels;
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
          BUFF_SIZE = 4+1+8*channels;
          printf("\n SENT DATA from Server: delim = %i , command = %i , length = %i , Channels = %i " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4]);
          offset = 5;
          for(int channel=0;channel<channels;channel++)
          {
           printf("\n Channel# : %d, Max_Vol= %f, Max_Cur= %f", channel,*((float *)(nbuffer+offset)), *((float *)(nbuffer+offset+4)));
           offset +=8;
          }
}                

void take_boardMeasurments()
{         
          p = nbuffer;
					channels = 8;
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

          BUFF_SIZE = 4+1+9*channels;
          printf("\n SENT DATA from Server: delim = %i , command = %i , length = %i , Channels = %i " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4]);

          int offset = 5;
          for(int channel=0;channel<channels;channel++)
          {
           printf("\n Channel# : %d, On/Off= %i, Present_Curr= %f, Present_Vol= %f", channel,nbuffer[offset] ,*((float *)(nbuffer+offset+1)), *((float *)(nbuffer+offset+5)));
           offset+=9;
          }
          
					
}

bool check_Command(unsigned char *buf)
{ 
  printf("\n RECIEVED from Client:\ndelim = %i , command = %i , length = %d" ,*recv_data, *(&recv_data[1]), *(&recv_data[2]),*((float *)(nbuffer+5)));
}
void send_Ack()
{         
          //bool correctCommand=check_Command();
          p = nbuffer;
					char delim = 2;
          char command = 5;
          ushort length = 1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;

          char ecode = 0;
          
         *p=ecode;
          
          BUFF_SIZE = 5;
          send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
          printf("\n ACK from Server: delim = %i , command = %i , length = %i , ecode = %i " , nbuffer[0], nbuffer[1], nbuffer[2] , nbuffer[4]);
         
}

void switch_OnOffChannel(char channel, char status)
{
printf("%i channel to %c ",channel,status);
int ch;
ch = (int) channel;
board_info.on_off[ch] = status;
}

void BoradcastWithFrq()
{

while(true)
 {
    
    take_boardMeasurments(); 
    send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
    sleep(0.8);

 }


}



int main()
{

        for(int channel=0;channel<MAX_channels;channel++)
           {
             if((channel%2)==0)
               { 
                 board_info.on_off[channel] = 1;
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

        boost::thread broadcaster;
        
        while(1)
        {  

            sin_size = sizeof(struct sockaddr_in);
 
            connected = accept(sock, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size);

            printf("\n I got a connection from (%s , %d)",
                   inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));

            take_boardSpecifications();
            char command=0;
            while (1)
            { 
              if(command!=4)
              send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
              bytes_recieved = recv(connected,recv_data,255,0);
              
                   
              command = recv_data[1];
              if(command==1)
                   {
                    printf("\n RECIEVED from Client:\ndelim = %i, command = %i,length = %d, BroadCastFrequency = %i" ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *(&recv_data[4]));
                    broadcaster.interrupt();
                    broadcaster = boost::thread(BoradcastWithFrq);
                    //take_boardMeasurments();
                   }
              else if(command==3)
                 { printf("\n RECIEVED from Client:\ndelim = %i, command = %i,length = %i " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *(&recv_data[4]), *(&recv_data[5]));
                   send_Ack();
                   take_boardMeasurments();
                 }
              else if(command==4)
                   {
                     switch_OnOffChannel(*(&recv_data[4]),*(&recv_data[5]));
                     printf("\n RECIEVED from Client:\ndelim = %i, command = %i,length = %d,channel = %i, State = %i " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *(&recv_data[4]), *(&recv_data[5]));
                    send_Ack();
                   }
              
                    
 
              fflush(stdout);

            }
        }       

      close(sock);
      return 0;
} 

