#include <wiringPi.h>
#include <stdio.h>
int main()
{
  wiringPiSetup();
  char val;
  {
    pinMode(7,INPUT);
   }
  
  while(1)
  { 
   val=digitalRead(7);
   if(val==1)
   printf("There are liquid!\n");
   else
   printf("No liquid!\n");
  }	
}
