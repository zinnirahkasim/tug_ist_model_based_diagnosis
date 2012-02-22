/* Controller Class */
#include "Controller.h"

Controller::Controller()
{
    ControlExit = false;
    host = gethostbyname("127.0.0.1");
    //host = gethostbyname("192.168.0.70");
    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(5000);   
    //server_addr.sin_port = htons(9760);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8);
    pub_board_msr_ = n_.advertise<diagnosis_msgs::DBoardMeasurments>("/board_measurments",1);
    
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
  ControlExit = true;
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

void Controller::processBuffer(unsigned char *buf,char command)
{      
     std::vector<float> curr;
     int channels = *buf;
     buf++;
     board_msr.o_time = 111.11;
     diagnosis_msgs::Channel channel;
     std::vector<diagnosis_msgs::Channel> msr_vector;
     for(int chnl=0;chnl<channels;chnl++)
      { 
        if(command==2)
         { ushort status;
           printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", chnl,*buf,*((float *)(buf+1)), *((float *)(buf+5)));
           channel.id = chnl;
           status = *buf;
           channel.status =(int) status;
           channel.current = *((float *)(buf+1));
           channel.voltage = *((float *)(buf+5));
           msr_vector.push_back(channel);
           buf+=9;
         }
        else if(command==0)
         {
           printf("\n Channel# : %d, Max_Curr= %f, Max_Vol= %f", chnl,*((float *)(buf)), *((float *)(buf+4)));
           buf+=8;
         }
      }
    board_msr.channel = msr_vector;
    msr_vector.clear();
    pub_board_msr_.publish(board_msr);
    
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
     //printf("del=%i,cmd=%i,len=%i",delim,command,data_length);
    counter = 0;
  while(counter<data_length)
    {  bytes_recieved=recv(sock,buffer,data_length-counter,0);
       counter+=bytes_recieved;
       //printf("DBytes =%d",bytes_recieved);
     }
    unsigned char * bufPtr;
    bufPtr = buffer;
    processBuffer(bufPtr,command);
    switch(command)
    {
      case 0:
      			msg = new MessageSpefications(delim,command,data_length);
            msg->parseBuffer(bufPtr);
      break;
      case 2:
      			msg = new MessageMeasurments(delim,command,data_length);
            msg->parseBuffer(bufPtr);
      break;
      case 5:
            msg = new MessageAcknowledgment(delim,command,data_length);
            msg->parseBuffer(bufPtr);
      break;

    } // switch
      
  }//while(1)
}// recv_Thread



void Controller::send_Thread()
{ 
char d[2];
 while(ControlExit)
{
printf("\na. BroadCasting\nb. Request Measurments\nc. Switch On/Off channel\n ");
printf("Enter Choice: ");
d[0]=' ';
gets(d);
if(strcmp(d,"a")==0)
  {  char s;
     printf("\nBradCasting Start/Stop(0=Stop,n=Start (milisecond)) :");
     scanf("%i",&s);
     unsigned char *p;
     msg = new MessageBroadCasting(s);
     int buf_len;
     p = msg->getBuffer(buf_len);
     send(sock,p,buf_len, 0);
     delete p;
     
  }
else if(strcmp(d,"b")==0)
   { 
     unsigned char *p;
     msg = new MessageRequest();
     int buf_len;
     p = msg->getBuffer(buf_len);
     send(sock,p,buf_len, 0);
     printf("d=%i,c=%i,l=%i,size=%d",*p,*(p+1),*(p+2),buf_len);
     delete p;
   }
else if(strcmp(d,"c")==0)
   {
    char c[2],s;
    printf("\nEnter Channel#");
    gets(c);
    printf("\nEnter Status you want (0=Off,1=On)#");
    scanf("%i",&s);
    printf("\nREAD channel=%c,Status=%i",c[0],s);
    unsigned char *p;
    msg = new MessageChannelOnOff(c[0]-48,s);
    int buf_len;
    p = msg->getBuffer(buf_len);
    send(sock,p,buf_len, 0);
    printf("d=%i,c=%i,l=%i,channel=%i, state=%i, size=%d",*p,*(p+1),*(p+2),*(p+4),*(p+5),buf_len);
    delete p;
  }
}
}


