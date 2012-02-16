#include "Client.h"
Client::Client()
{
    host = gethostbyname("127.0.0.1");
    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(5000);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8);
    initiate = false; 
    
}

Client::~Client()
{
  close(sock);
}

void Client::start_stopBroadcasting()
{
    			delim = STX;
          command = '1';
          length = 1;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));            
          send(sock,(void*)&nbuffer,sizeof(nbuffer), 0); 
          printf("\n SENT DATA : delim = %c , command = %c , length = %d  " ,*nbuffer, *(&nbuffer[sizeof(delim)]), *(&nbuffer[sizeof(delim)+sizeof(command)]));
}

void Client::on_offChannel(char channel, char status)
{
  				delim = STX;
          command = '4';
          length = 2;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)], &channel, sizeof(channel));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(channel)], &status, sizeof(status));            
          send(sock,(void*)&nbuffer,sizeof(nbuffer), 0); 
          printf("\n SENT DATA : delim = %c , command = %c , length = %d  , channel#= %c, ON/OFF= %c" ,*nbuffer, *(&nbuffer[sizeof(delim)]), *(&nbuffer[sizeof(delim)+sizeof(command)]),*(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)]),*(&nbuffer[sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(channel)]));
}


void Client::re_initClient()
{
 close(sock);
 Client();
 initClient();
}

void Client::initClient()
{ 
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
         perror("Socket");
         exit(1);
    }
  if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
            perror("Connect");
            exit(1);
    }
  initiate = true;
  receiveFromServer();
}

void Client::receiveFromServer()
{
  //while(1)
    //    {
        
          bytes_recieved=recv(sock,recv_data,1024,0);
          recv_data[bytes_recieved] = '\0';
 
          if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
          {
           close(sock);
            exit(0);
           //break;
          }
           //printf("\n RECIEVED DATA from Server: delim = %c , command = %c , length = %d , Channels = %d , Max_Cur = %d , Max_Vol = %d " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]) , *(&recv_data[4]), *(&recv_data[5]), *(&recv_data[9]));
           printf("\n RECIEVED DATA from Server:\n delim = %c , command = %c , length = %d , Channels = %d " ,*recv_data, *(&recv_data[1]), *(&recv_data[2]) , *(&recv_data[4]));
           char command = *(&recv_data[1]);
           ushort length = *(&recv_data[2]);
           int channels = *(&recv_data[4]);
           //printf("\nCommand = %c, Bytes=%i,Channels=%d", command, length,channels);
          if(command == '0')
          {
             int offset = 5;
             for(int channel=0;channel<channels;channel++)
             {
                printf("\n Channel# : %d, Max_Vol= %d, Max_Cur= %d", channel,*(&recv_data[offset]),*(&recv_data[offset+4]));
                offset = offset + 8;
              }
           }else
                {
                  int offset = 5;
                  for(int channel=0;channel<channels;channel++)
                  {
                    printf("\n Channel# : %d, Switch ON/OFF = %c Flowing_Vol= %d, Flowing_Cur= %d", channel,*(&recv_data[offset]),*(&recv_data[offset+1]), *(&recv_data[offset+5]));
                    offset = offset + 9;
                  }//for
              }//else
          
           
           //printf("\nSEND (q or Q to quit) : ");
           //char d[2],c;
           //float f;
           //gets(d);
          //d = "n";
          /* printf("after scanf");
          delim = '2';
          command = '4';
          length = 7;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));            
          *///if (d!="q")
            // send(sock,(void*)&nbuffer,sizeof(nbuffer), 0); 
          /*else
          {
             send(sock,(void*)&nbuffer,sizeof(nbuffer), 0);  
             close(sock);
           
          }*/
          //printf("\n SENT DATA : delim = %c , command = %c , length = %d  " ,*nbuffer, *(&nbuffer[sizeof(delim)]), *(&nbuffer[sizeof(delim)+sizeof(command)]));
    //       }   
}
 

void Client::requestMeasurments()
{
  				delim = STX;
          command = '3';
          length = 0;
          memcpy(nbuffer,&delim, sizeof(delim));
          memcpy(&nbuffer[sizeof(delim)],&command, sizeof(command));
          memcpy(&nbuffer[sizeof(delim)+sizeof(command)], &length, sizeof(length));            
          send(sock,(void*)&nbuffer,sizeof(nbuffer), 0); 
          printf("\n SENT DATA : delim = %c , command = %c , length = %d  " ,*nbuffer, *(&nbuffer[sizeof(delim)]), *(&nbuffer[sizeof(delim)+sizeof(command)]));
}
