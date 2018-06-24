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

#include <kilolib.h>
#define JOIN 1
#define OK 2
#define FOLLOWING 3


struct mystruct {
    uint8_t new_message;
    distance_measurement_t dist;
    uint8_t cur_distance;
    message_t transmit_msg;
};

struct mystruct mydata;
void message_rx(message_t *m, distance_measurement_t *d) {
    mydata.new_message = 1;
    mydata.dist = *d;
}

void setup_message(void)
{
  mydata.transmit_msg.type = NORMAL;
  mydata.transmit_msg.data[0] = kilo_uid & 0xff; //low byte of ID, currently not really used for anything
  mydata.transmit_msg.data[1] = JOIN;

  //finally, calculate a message check sum
  mydata.transmit_msg.crc = message_crc(&mydata.transmit_msg);
}

message_t *message_tx() 
{
  return &mydata.transmit_msg;
}

void setup()
{
  mydata.cur_distance = 0;
  mydata.new_message = 0;

  setup_message();

  if (kilo_uid == 0)
    set_color(RGB(0,0,0)); // color of the stationary bot
  else
    set_color(RGB(3,0,0)); // color of the moving bot
}

void sendFollowing(){
    mydata.transmit_msg.type = NORMAL;
    mydata.transmit_msg.data[0] = kilo_uid & 0xff;
    mydata.transmit_msg.data[1] = FOLLOWING;

    mydata.transmit_msg.crc = message_crc(&mydata.transmit_msg);
}

void sendJoinMessage(){
	setup_message();
}

void sendOk(){
  mydata.transmit_msg.type = NORMAL;
  mydata.transmit_msg.data[0] = kilo_uid & 0xff; //low byte of ID, currently not really used for anything
  mydata.transmit_msg.data[1] = OK;

  //finally, calculate a message check sum
  mydata.transmit_msg.crc = message_crc(&mydata.transmit_msg);
	
}

void waitForJoin(){
	if(mydata.new_message){
	    set_color(RGB(0,0,3));
		sendOk();
	}

}

int waitForOk(){
	if(mydata.new_message){
		mydata.cur_distance = estimate_distance(&mydata.dist);
	    set_color(RGB(3,0,0));
	    return 1;
	}
	return 0;
}

int init = 0;

int myRight = 70;
int myLeft = 70;
int following = 0;
uint8_t myDist;
uint8_t t0;

void followTheLeader(){
    if (following == 0){
        t0 = kilo_ticks;
        sendFollowing();
        myDist = mydata.cur_distance;
    }
    if (waitForOk()){
        following = 1;
        uint8_t elapsed = kilo_ticks - t0;
        if (elapsed < 32 * 3){
            set_motors(myLeft,myRight);
            sendFollowing();
        } else if (myDist > mydata.cur_distance) {
            return;
        } else {
    	    myRight = myRight*rand_hard();
    	    myLeft = myLeft*rand_hard();
            following = 0;
        }
    }
}


void follower(){
    if (init == 0){
		spinup_motors();
		init = 1;
		return;
	} else {
    	set_motors(myLeft,myRight);
    	sendJoinMessage();
    	if(waitForOk()){
    	    followTheLeader();
    	} else {
    	    myRight = myRight*rand_hard();
    	    myLeft = myLeft*rand_hard();
    	}
	}
}

void leader(){
    set_color(RGB(0,3,3));
	waitForJoin();
}

void loop() {
    // Update distance estimate with every message
    if (mydata.new_message) {
        mydata.new_message = 0;
        mydata.cur_distance = estimate_distance(&mydata.dist);
    } 
    // bot 0 is stationary. Other bots orbit around it.
    if (kilo_uid == 1){
        leader();
	} else {
		follower();
	}
    
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    
    // bot 0 is stationary and transmits messages. Other bots orbit around it.
    if (kilo_uid == 0)
      kilo_message_tx = message_tx;
    
    kilo_start(setup, loop);

    return 0;
}

