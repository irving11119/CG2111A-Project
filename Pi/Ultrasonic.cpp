// compile with g++ -Wall -pthread -o ultrasonic ultrasonic.cpp -lpigpio -lrt
// Pins set to 19 and 26 (see code)
// install pigpio first
#include <stdio.h>
#include <pigpio.h>

#define TRIG 19
#define ECHO 26
#define TRIG2 14
#define ECHO2 15

double last_range = 0;

void delay(int ms) {
    gpioDelay(1000*ms);
}

void ping() {

   gpioSetMode(TRIG, PI_OUTPUT);
   gpioSetMode(TRIG2, PI_OUTPUT);
   gpioWrite(TRIG, PI_ON);
   gpioWrite(TRIG2,PI_ON);
   gpioDelay(10);
   gpioWrite(TRIG, PI_OFF);
   gpioWrite(TRIG2, PI_OFF);
   gpioDelay(5);
   gpioSetMode(ECHO2, PI_INPUT);
   gpioSetMode(ECHO, PI_INPUT);



}

void range(int gpio, int level, uint32_t tick) {

   static double startTick, endTick;
   double diffTick;

      if (level == PI_ON) {
        startTick = tick;
      } else

      if (level == PI_OFF) {
         endTick = tick;
         diffTick = (endTick - startTick)*0.01715;

         last_range = diffTick;
          if (diffTick < 600){
            printf("L: ");
            printf("%f", diffTick);
            printf(" cm\n");
            if (diffTick < 20){
                printf("==========\n");
            }
           }
         else{
           printf("L: Out of range\n");
           last_range = 0;
         }

      }
}

void range2(int gpio, int level, uint32_t tick) {

   static double startTick, endTick;
   double diffTick;

      if (level == PI_ON) {
        startTick = tick;
      } else

      if (level == PI_OFF) {
         endTick = tick;
         diffTick = (endTick - startTick)*0.01715;

         last_range = diffTick;
          if (diffTick < 600){
            printf("R: ");
            printf("%f", diffTick);
            printf(" cm\n");
            if(diffTick < 20){
                printf("=========\n");
            }
           }
          
         else{
           printf("R: Out of range\n");
           last_range = 0;
         }

      }
}



void sleep(int t) {
  gpioSleep(PI_TIME_RELATIVE, t, 0);
}
int main(){

   if (gpioInitialise()<0) return 1;

   gpioSetMode(ECHO, PI_INPUT);
   gpioSetMode(ECHO2, PI_INPUT);
   gpioSetAlertFunc(ECHO2,range2);
   gpioSetAlertFunc(ECHO, range);

   while (1) {

      ping();
      sleep(1);
   }
}
