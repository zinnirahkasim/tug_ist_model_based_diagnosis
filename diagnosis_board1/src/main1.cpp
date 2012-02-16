#include "Client.h"
//#include "conio.h"

Client *client;

int main( int argc, char **argv)
{
  
	client = new Client();
  client->initClient();
  printf("mainClient");
  while(1)
   { 
     printf("\n\n****COMMAND MENUE:*****\n");
     printf("0. Initialization\n1. Start/Stop Broadcasting\n2. Request Measurments\n3. Channel Switch On/Off\n ");
     printf("Enter Choice: ");
     char d[2];
     gets(d);
     if(strcmp(d,"0")==0)
       {client = NULL;
       client = new Client();}
      else if(strcmp(d,"1")==0)
              client->start_stopBroadcasting();
          else if(strcmp(d,"2")==0)
               client->requestMeasurments();
          		 else if(strcmp(d,"3")==0)
                     client->on_offChannel();
     
       
    }
    
}
