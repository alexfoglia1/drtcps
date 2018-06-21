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
#define LEFT 3
#define STRAIGHT 0

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
  mydata->turning = 0;
  mydata->received_okjoin=0;
  if (kilo_uid == 0){
    set_color(RGB(3,0,0)); // color of the leader
    mydata->leader_id = 0;
    }
  else{
    mydata->leader_id = kilo_uid-1;
    set_color(RGB(0,0,0)); // color of the moving bot
    }
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
void doLeader(){
    int ticksm = kilo_ticks % 850;
    if(ticksm <= 850-128){
        set_motors(kilo_turn_left,kilo_turn_right);
        setup_message(STRAIGHT);
    }
    else{
        set_motors(kilo_turn_left,0);
        if(ticksm > 850-25)
            setup_message(LEFT);
    }


}

int passedFourSeconds(){

    if(kilo_ticks - mydata->rcv_ticks < 160) return 0;
    else return 1;


}

int passedOneSecond(){

    if(kilo_ticks - mydata->rcv_ticks < 50) return 0;
    else return 1;


}

void turn_and_send(uint8_t rcv){
    if(rcv == LEFT || mydata->turning){
        printf("rcv = %d\n",rcv);
        printf("turning = %d\n",mydata->turning);
        printf("kiloticks attuali = %d\n",kilo_ticks);
        printf("Differenza = %d\n",kilo_ticks-mydata->rcv_ticks);
        printf("passed four seconds = %d\n",passedFourSeconds());
        if(passedOneSecond()==0) return;
        if(passedFourSeconds()==0)
        {
            set_motors(kilo_turn_left,0); 
            printf("STO GIRANDO A SX\n");
            setup_message(LEFT);
        }
        else
        {
            mydata->turning = 0;
        }
    }
    else{
        set_motors(kilo_turn_left,kilo_turn_right);
        setup_message(STRAIGHT);
    }
    

}

void doFollower(){
    if(mydata->new_message && mydata->received_msg.data[0] == mydata->leader_id)
    {
        uint8_t rcv = mydata->received_msg.data[1];
        if(mydata->turning == 0 && rcv == LEFT){
            mydata->rcv_ticks = kilo_ticks;
            printf("Kiloticks quando ho ricevuto left la prima volta: %d\n",mydata->rcv_ticks);
            mydata->turning =1;
        }
        turn_and_send(rcv);
    }
    mydata->new_message=0;
}
void loop() {
    
   if(kilo_ticks < 32){
        spinup_motors();
        return;
    }
    
    if(kilo_uid == 0)
        doLeader();
    else
        doFollower();
    
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);
    return 0;
}

