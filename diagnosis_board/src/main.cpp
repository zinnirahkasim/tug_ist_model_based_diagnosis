#include "Controller.h"
Controller *contl;

int main( int argc, char **argv)
{
contl = new Controller();
contl->initController();	
printf("a. BroadCasting\nb. Request Measurments\nc. Switch On/Off channel\n ");
printf("Enter Choice: ");
char d[2];
gets(d);
if(strcmp(d,"a")==0)
       printf("\na given");
      else if(strcmp(d,"b")==0)
              printf("\nb given");
    
}
