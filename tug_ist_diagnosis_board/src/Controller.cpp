/*
* Copyright (c).2012. OWNER: Institute for Software Technology TU-Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 3. Neither the name of the <ORGANIZATION> nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
* QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*/

#include "Controller.h"

Controller::Controller(unsigned char frq)
{  
    
    host = gethostbyname("127.0.0.1");
    //host = gethostbyname("192.168.0.70");
    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(5000);   
    //server_addr.sin_port = htons(9760);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8);
    pub_board_msr_ = n_.advertise<tug_ist_diagnosis_msgs::DBoardMeasurments>("/board_measurments",1);
    initFrq = frq;
    
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
  chnl2dev_mapping();
  CallMessageBroadCasting(initFrq);
}

void Controller::chnl2dev_mapping()
{

  chnl2dev_map[0] = "sensor_head";
  chnl2dev_map[1] = "laser_alignment";
  chnl2dev_map[2] = "pc";
  chnl2dev_map[3] = "hokuyo";
  chnl2dev_map[4] = "kinect";
  chnl2dev_map[5] = "router";
  chnl2dev_map[6] = "thermal_camera";
  chnl2dev_map[7] = "jaguar";
  chnl2dev_map[8] = "no_dev1";
  chnl2dev_map[9] = "no_dev2";


}

char Controller::get_chnl_from_map(string dev)
{
   map<char,string>::const_iterator it;
   char key = -1;
   for (it = chnl2dev_map.begin(); it != chnl2dev_map.end(); ++it)
   {
    if (it->second == dev)
     {
      key = it->first;
      break;
     }
   }
  return key;
}

void* run_recv_Thread(void* contrl_ptr){
    static_cast<Controller*>(contrl_ptr)->recv_Thread();
}

void Controller::create_threads(){
    pthread_t r_thread;
    r_thread=pthread_create(&r_thread, NULL, run_recv_Thread, this);
}  

void Controller::processBuffer(unsigned char *buf,char command)
{      
     std::vector<float> curr;
     int channels = *buf;
     buf++;
     board_msr.o_time = 111.11;
     tug_ist_diagnosis_msgs::Channel channel;
     std::vector<tug_ist_diagnosis_msgs::Channel> msr_vector;
     for(int chnl=0;chnl<channels;chnl++)
      { 
        if(command==2)
         { ushort status;
           //printf("\n Channel# : %d, ON/Off= %i, Present_Curr= %f, Present_Vol= %f", chnl,*buf,*((float *)(buf+1)), *((float *)(buf+5)));
           channel.id = chnl;
           channel.dev_connected = chnl2dev_map[(char) chnl];
           status = *buf;
           channel.status =(int) status;
           channel.current = *((float *)(buf+1));
           channel.voltage = *((float *)(buf+5));
           msr_vector.push_back(channel);
           buf+=9;
         }
        else if(command==0)
         {
           //printf("\n Channel# : %d, Max_Curr= %f, Max_Vol= %f", chnl,*((float *)(buf)), *((float *)(buf+4)));
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
     //printf("\nRECIEVED:\ndelim = %i,command = %i,length = %i ", buffer[0], buffer[1], buffer[2]);
     char delim = buffer[0];
     char command = buffer[1];
     ushort data_length = buffer[2];
     //printf("del=%i,cmd=%i,len=%i",delim,command,data_length);
    counter = 0;
  while(counter<data_length)
    {  
       bytes_recieved=recv(sock,buffer,data_length-counter,0);
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


void Controller::CallMessageBroadCasting(unsigned char frq)
{
     unsigned char *p;
     msg = new MessageBroadCasting(frq);
     int buf_len;
     p = msg->getBuffer(buf_len);
     send(sock,p,buf_len, 0);
     delete p;
}

void Controller::CallMessageRequest()
{
     unsigned char *p;
     msg = new MessageRequest();
     int buf_len;
     p = msg->getBuffer(buf_len);
     send(sock,p,buf_len, 0);
     printf("d=%i,c=%i,l=%i,size=%d",*p,*(p+1),*(p+2),buf_len);
     delete p;
}

void Controller::CallMessageChannelOnOff(char chnl, char status)
{
    unsigned char *p;
    msg = new MessageChannelOnOff(chnl,status);
    int buf_len;
    p = msg->getBuffer(buf_len);
    send(sock,p,buf_len, 0);
    printf("d=%i,c=%i,l=%i,channel=%i, state=%i, size=%d",*p,*(p+1),*(p+2),*(p+4),*(p+5),buf_len);
    delete p;
}

