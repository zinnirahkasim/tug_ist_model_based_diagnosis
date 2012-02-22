#include "Controller.h"
typedef actionlib::SimpleActionServer<diagnosis_board::BoardAction> Server;

Controller *contl;


void execute(const diagnosis_board::BoardGoalConstPtr& goal, Server* as)
{
  int i;
  i = goal->goal;
  if(i==3)
   contl->CallMessageRequest();
  else if(i==2)
   { 
    char frq;
    frq = 10; 
    contl->CallMessageBroadCasting(char);;
   }
  else if(i==4)
    contl->CallMessageRequest();
   printf("Otherthan3");
  as->setSucceeded();
}



int main( int argc, char **argv)
{
ros::init(argc, argv,"board_server");
ros::NodeHandle n;
ROS_INFO("Board Controller trying to Connect....");
contl = new Controller();

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
