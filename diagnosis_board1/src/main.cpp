#include "Board.h"
//#include "conio.h"

Message *client;

int main( int argc, char **argv)
{
  
	client = new Message();
  client->initMessage();
  printf("mainClient");
  while(1)
   { 
     printf("\n\n****COMMAND MENUE:*****\n");
     printf("a. Start/Stop Broadcasting\nb. Request Measurments\nc. Channel Switch On/Off\n ");
     printf("Enter Choice: ");
     char d[2];
     gets(d);
     if(strcmp(d,"a")==0)
              client->start_stopBroadcasting();
          else if(strcmp(d,"b")==0)
               { client->requestMeasurments();
                 //client->receiveFromServer();
               }
          		 else if(strcmp(d,"c")==0)
                     {
                     char channel[2],status[2];
                     ushort chan;
                     printf("Enter channel#:");
                     gets(channel);
                     //scanf("%i",&chan);
                     printf("Enter Status(0=OFF,1=ON)#:");
                     gets(status);
                     //scanf("%i",&status);
                     printf("ch=%c,status=%c",channel[0],status[0]);
                     //char chn = (char) channel;
                     //char st = (char) status;
                     //printf("ch=%c,status=%i",chn,st);
                     if(strcmp(status,"0")==0)
                        client->on_offChannel(channel[0],'0');
                     else
                        client->on_offChannel(channel[0],'1');
                     
                     }
     
       
    }
    
}
