#include <serialize.h>
#include "Arduino.h"
#include <buffer.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <stdarg.h>

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;
volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV      180

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          20.4
#define MULTIPLIER          0.95

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  1 << 5   // Left forward pin
#define LR                  1 << 6   // Left reverse pin
#define RF                  1 << 2  // Right forward pin
#define RR                  1 << 1  // Right reverse pin

/*
 *    Alex's State Variables
 */

#define PI 3.141592654
#define ALEX_LENGTH 16
#define ALEX_BREADTH 6

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks; 


// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

TBuffer *buffer_array;

/*
 * 
 * Alex Communication Routines.
 * 
 */
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it. Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet); 
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
     va_list args;
     char buffer[128];
     va_start(args, format);
     vsprintf(buffer, format, args);
     sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= ~(1 << PIND2 | 1 << PIND3);
  PIND |= (1 << PIND2 | 1 << PIND3);   
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
    //dbprint("rightFowardTicks: %ld", rightForwardTicks);
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. 
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
  sei();
}

// External interrupt ISRs
ISR(INT0_vect){
  leftISR();
}
ISR(INT1_vect){
  rightISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.

void setBaudRate(unsigned long baudrate){
  unsigned int b;
  b = (unsigned int) round(16000000 / (16.0 * baudrate)) - 1;
  UBRR0H = (unsigned char) (b >> 8);
  UBRR0L = (unsigned char) b;
}

void setupSerial()
{
  // To replace later with bare-metal.
  //Serial.begin(9600);
  setBaudRate(9600);
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.
void startSerial()
{
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.
int readSerial(char *buffer)
{
  int count=0;

    /*
  while(Serial.available())
    buffer[count++] = Serial.read();
*/
  TBufferResult result;

  do
  {
    result = readBuffer(buffer_array, &buffer[count]);

    if (result == BUFFER_OK){
      count++;
    } 
  } while (result == BUFFER_OK);

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code
void writeSerial(const char *buffer, int len)
{
  //Serial.write(buffer, len);
  TBufferResult result = BUFFER_OK;
  for(int i = 1; i < len; i += 1){
    result = writeBuffer(buffer_array, buffer[i]);
  }

  UDR0 = buffer[0];

  UCSR0B |= 0b00100000;
}


ISR(USART_RX_vect){
  unsigned char data = UDR0;

  writeBuffer(buffer_array, data);
}

ISR(USART_UDRE_vect){
  unsigned char data;
  TBufferResult result = readBuffer(buffer_array, &data);

  if(result == BUFFER_OK){
    UDR0 = data;
  } else {
    if (result == BUFFER_EMPTY){
      UCSR0B &= 0b11011111;
    }
  }
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2IN - Pin 9, PB1, OC1A
   */
  // set up timer0
  TCNT0 = 0;
  TCCR0A = 0b10100001;
  
  OCR0A = 0;
  OCR0B = 0;
  
  TIMSK0 |= 0b110;
  
  // set up timer1
  TCNT1 = 0;
  TCCR1A = 0b10100001;;
  
  OCR1A = 0;
  OCR1B = 0;
  
  TIMSK1 |= 0b110;

}

// Start the PWM for Alex's motors.
void startMotors()
{
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
  
  DDRD |= (LF | LR);
  DDRB |= (RF | RR);
}

ISR(TIMER0_COMPA_vect){
}
ISR(TIMER0_COMPB_vect){
}
ISR(TIMER1_COMPA_vect){
}
ISR(TIMER1_COMPB_vect){
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(speed);
 
  OCR0A = val;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = val * MULTIPLIER;

}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=reverseDist + deltaDist;
  dir = BACKWARD;
  int val = pwmVal(speed);

  OCR0A = 0;
  OCR0B = val;
  OCR1A = val * MULTIPLIER;
  OCR1B = 0;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed)
{
  if(ang == 0) 
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  dir = LEFT;
  int val = pwmVal(speed);
  
  OCR0A = 0;
  OCR0B = val;
  OCR1A = 0;
  OCR1B = val * MULTIPLIER;

}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  if(ang == 0) 
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  dir = LEFT;
  int val = pwmVal(speed);
  dir = RIGHT;

  OCR0A = val;
  OCR0B = 0;
  OCR1A = val * MULTIPLIER;
  OCR1B = 0;

}

// Stop Alex
void stop()
{
  dir = STOP;

  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}

// Intialize Alex's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        sendOK();
        break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;
 
    default:
      sendBadCommand();
  }
}

// hello! i think we dont use this
void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() 
{
  // put your setup code here, to run once:
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH *
            ALEX_BREADTH));
  AlexCirc = PI  * AlexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    // do we need to do anything w other cases
    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() 
{
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
  else
    if(result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    } 
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if(forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if(dir == BACKWARD) {
      if(reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if(dir == STOP){
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  
  if(deltaTicks > 0) {
    if(dir == LEFT) {
      if(leftReverseTicksTurns >= targetTicks) {
        deltaTicks=0;
        targetTicks=0;
        stop();
      }
    } else if (dir == RIGHT) {
      if(rightReverseTicksTurns >= targetTicks) {
        deltaTicks=0;
        targetTicks=0;
        stop();
      }
    } else if (dir == STOP) {
      deltaTicks=0;
      targetTicks=0;
      stop();
    }
  }
}
