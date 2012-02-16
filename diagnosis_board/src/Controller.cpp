/* Controller Class */

#include "Controller.h"

Controller::Controller()
{
    host = gethostbyname("127.0.0.1");
    //host = gethostbyname("192.168.0.70");
    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(5000);   
    //server_addr.sin_port = htons(9760);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8);
    
}

Controller::~Controller()
{

}

void Controller::initController()
{
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
         perror("Socket");
         exit(1);
    }
  if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
            perror("Connect");
            exit(1);
    }
  create_threads();
}

void* run_recv_Thread(void* contrl_ptr){
    static_cast<Controller*>(contrl_ptr)->recv_Thread();
}

void* run_send_Thread(void* contrl_ptr){
    static_cast<Controller*>(contrl_ptr)->send_Thread();
}

void Controller::create_threads(){
    pthread_t r_thread, c_thread;
    r_thread=pthread_create(&r_thread, NULL, run_recv_Thread, this);
    c_thread=pthread_create(&c_thread, NULL, run_send_Thread, this);
}  

void Controller::recv_Thread()
{
while(1)
  {
   int header_length=4,counter=0;
   while(counter<header_length)
     {
       bytes_recieved=recv(sock,buffer,4-counter,0);
       counter+=bytes_recieved;
       //printf("HBytes =%d",bytes_recieved);
     }
     printf("\nRECIEVED:\ndelim = %i,command = %i,length = %i ", buffer[0], buffer[1], buffer[2]);
     char delim = buffer[0];
     char command = buffer[1];
     ushort data_length = buffer[2];
     printf("del=%i,cmd=%i,len=%i",delim,command,data_length);
    counter = 0;
  while(counter<data_length)
    {  bytes_recieved=recv(sock,buffer,data_length-counter,0);
       counter+=bytes_recieved;
       printf("DBytes =%d",bytes_recieved);
     }
    unsigned char * ptr;
    ptr = buffer;
    switch(command)
    {
      case 0:
      			msg = new MessageSpefications(delim,command,data_length,ptr);
      break;
      case 2:
      			msg = new MessageMeasurments(delim,command,data_length,ptr);
      break;
      case 5:
      			msg = new MessageAcknowledgment(delim,command,data_length,ptr);
      break;

    } // switch
      
  }//while(1)
}// recv_Thread

void Controller::send_Thread()
{ 
  while(1)
   {
    
   }
}


