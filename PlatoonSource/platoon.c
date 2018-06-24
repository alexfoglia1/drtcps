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

#include "platoon.h"

#ifdef SIMULATOR
#include <stdio.h> // for printf
#include <stdlib.h>
#else
#include <avr/io.h>  // for microcontroller register defs
//  #define DEBUG          // for printf to serial port
//  #include "debug.h"
#endif

#define STRAIGHT 1
#define LEFT 2
#define JOIN 3
#define QUIT 4
#define OK 5
#define SPEED_UP 6
#define SPEED_DOWN 7
#define TURN_LEFT_DELAY 126
#define GO_STRAIGHT_DELAY 700
#define STANDARD_DISTANCE 80
#define SLOW_SPEED 70
#define NORMAL_SPEED 80
#define CRAZY_SPEED 90

REGISTER_USERDATA(USERDATA)



void message_rx(message_t *m, distance_measurement_t *d) {
    mydata->new_message = 1;
    mydata->received_msg=*m;
    mydata->dist = *d;
}

void setup_message(uint8_t data) {
	mydata->transmit_msg.type = NORMAL;
	mydata->transmit_msg.data[0] = kilo_uid & 0xff; //low byte of ID, currently not really used for anything
	mydata->transmit_msg.data[1] = data;
	mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
}

message_t *message_tx() {
  return &mydata->transmit_msg;
}

void setup() {
	mydata->cur_distance = 0;
	mydata->new_message = 0;
	mydata->turning = 0;
	mydata->follower_id = kilo_uid+1;
	if (kilo_uid == 0)
		set_color(RGB(0,0,0)); // color of the stationary bot
	else {
		set_color(RGB(3,0,0)); // color of the moving bot
		mydata->my_leader = kilo_uid-1;
	}
}



// LEADER CODE
/*********************************************************************/

int checkDistance() {
	if (mydata->new_message && mydata->received_msg.data[0] == mydata->follower_id) {
		if (estimate_distance(&mydata->dist) > STANDARD_DISTANCE+3)
			return SPEED_DOWN;
		if (estimate_distance(&mydata->dist) < STANDARD_DISTANCE-3)
			return SPEED_UP;
	}
	return -1;
}

void leader() {
	mydata->myClock = kilo_ticks%(GO_STRAIGHT_DELAY+TURN_LEFT_DELAY);
	int distance = checkDistance();
	if (mydata->myClock < GO_STRAIGHT_DELAY-346) {
		if (distance == SPEED_DOWN)
			set_motors(SLOW_SPEED,SLOW_SPEED);
		else if (distance == SPEED_UP)
			set_motors(CRAZY_SPEED,CRAZY_SPEED);
		else
			set_motors(NORMAL_SPEED,NORMAL_SPEED);
		setup_message(STRAIGHT);
	} else if (mydata->myClock >= GO_STRAIGHT_DELAY -346 && mydata->myClock < GO_STRAIGHT_DELAY){
		set_motors(NORMAL_SPEED,NORMAL_SPEED);
		setup_message(STRAIGHT);	
	} else {
		setup_message(LEFT);
		set_motors(NORMAL_SPEED, 0);	
	}
}

/*********************************************************************/



// FOLLOWER CODE
/*********************************************************************/

int handleMessage() {
	if (mydata->new_message && mydata->received_msg.data[0] == mydata->my_leader) {
		return mydata->received_msg.data[1];
	}
	return 0;
}

int handleTurnLeft() {
	if (kilo_ticks - mydata->message_timestamp < 346) {
		setup_message(STRAIGHT);
		set_motors(NORMAL_SPEED,NORMAL_SPEED);
		return 1;
	} else if (kilo_ticks - mydata->message_timestamp >= 346 && kilo_ticks - mydata->message_timestamp < 346+TURN_LEFT_DELAY){
		setup_message(LEFT);
		set_motors(NORMAL_SPEED, 0);
		return 1;
	} else {
		setup_message(STRAIGHT);
		set_motors(NORMAL_SPEED,NORMAL_SPEED);
		return 0;
	}
}

void follower() {
	int distance = checkDistance();
	int message = handleMessage();
	if (mydata->turning == 0 && message == LEFT){
		mydata->message_timestamp = kilo_ticks;
		mydata->turning = 1;
	}
	if (mydata->turning == 0 && message == STRAIGHT) {
		if (distance == SPEED_DOWN)
			set_motors(SLOW_SPEED,SLOW_SPEED);
		else if (distance == SPEED_UP)
			set_motors(CRAZY_SPEED,CRAZY_SPEED);
		else
			set_motors(NORMAL_SPEED,NORMAL_SPEED);
		setup_message(STRAIGHT);
	} else if (mydata->turning == 1) {
		mydata->turning = handleTurnLeft();
	}
}


void loop() {
	if (kilo_uid == 0) {
		leader();
	} else {
		follower();
	}
}


int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    
    kilo_start(setup, loop);

    return 0;
}
