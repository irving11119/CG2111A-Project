// compile with g++ -Wall -pthread -o ultrasonic ultrasonic.cpp -lpigpio -lrt
// Pins set to 19 and 26 (see code)
// install pigpio first
#include <stdio.h>
#include <pigpio.h>

#define TRIGLEFT 19
#define ECHOLEFT 26
#define TRIGRIGHT 14
#define ECHORIGHT 15

double last_range = 0;

/**
 * @brief Sets the system for a delay of specified time 
 * 
 * @param ms An int representing the number of milliseconds for the system to delay
 */
void delay(int ms) {
    gpioDelay(1000*ms);
}

/**
 * @brief Triggers the ultrasonic pulse from the ultrasonic sensor
 * 
 */
void ping() {

   gpioSetMode(TRIGLEFT, PI_OUTPUT);
   gpioSetMode(TRIGRIGHT, PI_OUTPUT);
   gpioWrite(TRIGLEFT, PI_ON);
   gpioWrite(TRIGRIGHT,PI_ON);
   gpioDelay(10);
   gpioWrite(TRIGLEFT, PI_OFF);
   gpioWrite(TRIGRIGHT, PI_OFF);
   gpioDelay(5);
   gpioSetMode(ECHORIGHT, PI_INPUT);
   gpioSetMode(ECHOLEFT, PI_INPUT);
}
/**
 * @brief Function to obtain the distance reading from the left ultrasonic sensor
 * 
 * @param gpio A GPIO Pin whose state changes 
 * @param level A int of range 0-2, representing: 0 = change to low (a falling edge),
 *              1 = change to high (a rising edge), 2 = no level change (a watchdog timeout) 
 * @param tick Integer representing the number of seconds since boot
 */
void rangeLeft(int gpio, int level, uint32_t tick) {

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

/**
 * @brief Function to obtain the distance reading from the right ultrasonic sensor
 * 
 * @param gpio A GPIO Pin whose state changes 
 * @param level A int of range 0-2, representing: 0 = change to low (a falling edge),
 *              1 = change to high (a rising edge), 2 = no level change (a watchdog timeout) 
 * @param tick Integer representing the number of seconds since boot
 */
void rangeRight(int gpio, int level, uint32_t tick) {

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

/**
 * @brief Sets the system to sleep for a specified time
 * 
 * @param t integer representing the time in seconds the System will sleep for
 */
void sleep(int t) {
  gpioSleep(PI_TIME_RELATIVE, t, 0);
}

int main(){

   if (gpioInitialise() < 0) return 1;

   gpioSetMode(ECHOLEFT, PI_INPUT);
   gpioSetMode(ECHORIGHT, PI_INPUT);
   gpioSetAlertFunc(ECHORIGHT,rangeRight);
   gpioSetAlertFunc(ECHOLEFT, rangeLeft);

   while (1) {

      ping();
      sleep(1);
   }
}
