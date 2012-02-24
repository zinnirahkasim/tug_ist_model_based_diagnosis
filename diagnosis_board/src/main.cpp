#include "Controller.h"
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_msgs/BoardAction.h>
typedef actionlib::SimpleActionServer<diagnosis_msgs::BoardAction> Server;

Controller *contl;


void execute(const diagnosis_msgs::BoardGoalConstPtr& goal, Server* as)
{
  int i;
  i = goal->command;
  if(i==3)
   contl->CallMessageRequest();
  else if(i==2)
   { 
     char frq;
     frq = goal->arg1;
     contl->CallMessageBroadCasting(frq);
   }
  else if(i==4)
       { 
         char chnl;
         char status;
         chnl = goal->arg1;
         status = goal->arg2;
         contl->CallMessageChannelOnOff(chnl,status);
         
       }
   printf("Otherthan3");
  as->setSucceeded();
}



int main( int argc, char **argv)
{
ros::init(argc, argv,"board_server");
ros::NodeHandle n;
ROS_INFO("Board Controller trying to Connect....");
contl = new Controller(3);

Server server(n, "board_server", boost::bind(&execute, _1, &server), false);
server.start();

contl->initController();

ros::spin();
/*while(contl->ControlExit)
{
}*/
//printf("Controller Exit!");
/*
contl->SendCmd = 1;
printf("\na. BroadCasting\nb. Request Measurments\nc. Switch On/Off channel\n ");
printf("Enter Choice: ");
char d[2];
gets(d);
if(strcmp(d,"a")==0)
  {  char s;
     printf("\nBradCasting Start/Stop(0=Stop,n=Start (milisecond)) :");
     scanf("%i",&s);
     contl->sendCmd1(s); 
  }
else if(strcmp(d,"b")==0)
   { 
     contl->sendCmd3(); 
   }
else if(strcmp(d,"c")==0)
   {char c[2],s;
    printf("\nEnter Channel#");
    gets(c);
    printf("\nEnter Status you want (0=Off,1=On)#");
    //scanf("%i",&s);
    scanf("%i",&s);
    //gets(s);
    contl->sendCmd4(c[0],s);

  }
} */  
}
