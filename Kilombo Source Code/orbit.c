/* The planet orbit demonstration from the kilobotics-labs
 * https://www.kilobotics.com/labs#lab4-orbit
 *
 * Lightly modified to work in the simulator, in particular:
 * - mydata->variable for global variables
 * - callback function cb_botinfo() to report bot state back to the simulator for display
 * - spin-up motors only when required, using the helper function  smooth_set_motors()
 *
 * Modifications by Fredrik Jansson 2015
 */

#include <math.h>

#include <kilombo.h>

#include "orbit.h"

#ifdef SIMULATOR
#include <stdio.h> // for printf
#else
#include <avr/io.h>  // for microcontroller register defs
//  #define DEBUG          // for printf to serial port
//  #include "debug.h"
#endif

#define JOIN 1
#define OK 2

REGISTER_USERDATA(USERDATA)



void message_rx(message_t *m, distance_measurement_t *d) {
    mydata->new_message = 1;
    mydata->received_msg=*m;
    mydata->dist = *d;
}

void setup_message(uint8_t data)
{
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff; //low byte of ID, currently not really used for anything
  mydata->transmit_msg.data[1] = data;
  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
}

message_t *message_tx() 
{
  return &mydata->transmit_msg;
}

void setup()
{
  mydata->cur_distance = 0;
  mydata->new_message = 0;
  mydata->received_okjoin=0;
  if (kilo_uid == 0)
    set_color(RGB(0,0,0)); // color of the stationary bot
  else
    set_color(RGB(3,0,0)); // color of the moving bot
}


void sendJoinMessage(){
	setup_message(JOIN);
}

void sendOk(){
  setup_message(OK);
  printf("Sent OK\n");
	
}

void waitForJoin(){
	if(mydata->new_message){
            printf("Received a message\n");
		    sendOk();
            mydata->new_message=0;
	}

}

void waitForOk(){
	if(mydata->new_message){
		printf("Kilobot with uid %d has received ok message\n",kilo_uid);
        mydata->cur_distance = estimate_distance(&mydata->dist);
        printf("Current distance is %d\n",mydata->cur_distance);
        mydata->new_message=0;
        mydata->received_okjoin=1;
        
	}
}

void loop() {
    
    // Platooning is stationary. Other bot searches for it and asks to join its tail
    if (kilo_uid == 0){
		waitForJoin();
	} else 
    {
		if (kilo_ticks <32)
        {
			spinup_motors();
			return;
		}
        if(mydata->received_okjoin==0)
        {
            set_motors(kilo_turn_left,kilo_turn_right);
            sendJoinMessage();
            waitForOk();
        }
        else
        {
            waitForOk();
            if(mydata->cur_distance<=50)
                 set_motors(0,0);
            else
                set_motors(kilo_turn_left,kilo_turn_right);
        }        
	}
    
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    if (kilo_uid == 0)
      kilo_message_tx = message_tx;
    
    kilo_start(setup, loop);

    return 0;
}

