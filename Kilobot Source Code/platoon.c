#include <kilolib.h>

#define RED RGB(3,0,0)
#define GREEN RGB(0,3,0)
#define BLUE RGB(0,0,3)
#define WHITE RGB(3,3,3)
#define STRAIGHT 1
#define LEFT 2
#define JOIN 3
#define QUIT 4
#define OK 5
#define LEAVE 6
#define SPEED_DOWN 7
#define TURN_LEFT_DELAY 126
#define GO_STRAIGHT_DELAY 700
#define JOIN_DELAY 150
#define FOLLOW_DELAY 220
#define STANDARD_DISTANCE 80
#define NORMAL_SPEED 70
#define CAN_LEAVE 2
#define CAN_JOIN 3
#define LEAVE_TIME 2500
#define END_TIME 20000

typedef struct {
	uint8_t cur_distance;
	uint8_t new_message;
	distance_measurement_t dist;
	message_t received_msg;
	message_t transmit_msg;
	uint8_t my_leader;
    uint8_t joining;
    uint8_t following;
	int message_timestamp;
	int turning;
	int myClock;
	int turn_timestamp;
	int follower_id;
    int follow_timestamp;
} USERDATA;

USERDATA* mydata;

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

void setupUserData(){
    mydata->cur_distance = 0;
	mydata->new_message = 0;
	mydata->turning = 0;
    mydata->joining = 0;
    mydata->following = 0;
	mydata->follower_id = kilo_uid+1;
}

void setup() {
	setupUserData();
	if (kilo_uid == 0)
		set_color(WHITE); // color of the leader bot
	else if(kilo_uid == CAN_JOIN)
	{
		mydata->my_leader = 255;
		set_color(BLUE); //color of the joining bot
	}
	else
	{
		set_color(RED); // color of the moving bots
		mydata->my_leader = kilo_uid-1;
	}

}



// LEADER CODE
/*********************************************************************/

int checkDistance() {
	if (mydata->new_message && mydata->received_msg.data[0] == mydata->follower_id) {
		if (estimate_distance(&mydata->dist) > STANDARD_DISTANCE+3)
			return SPEED_DOWN;
	}
	return -1;
}


void speedCorrection(int distance){
    if (distance == SPEED_DOWN)
	set_motors(0,0);
    else
	set_motors(kilo_turn_left, kilo_turn_right);
}

void leader() {
	mydata->myClock = kilo_ticks%(GO_STRAIGHT_DELAY+TURN_LEFT_DELAY);
	if (mydata->myClock < GO_STRAIGHT_DELAY) {
		speedCorrection(checkDistance());
		setup_message(STRAIGHT);
	} else {
		setup_message(LEFT);
		set_motors(kilo_turn_left, 0);
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

int handleOther() {
    if(mydata->new_message && mydata->received_msg.data[0] != mydata->my_leader) {
        return mydata->received_msg.data[1];
    }
    return 0;
}

int goStraight() {
    return (kilo_ticks - mydata->message_timestamp < 346);
}

int goLeft() {
    int timestamp_isok = (kilo_ticks - mydata->message_timestamp >= 346);
    int passed_delay = kilo_ticks - mydata->message_timestamp < 346 +TURN_LEFT_DELAY;
    return (timestamp_isok && passed_delay);
}

int handleTurnLeft() {
	if (goStraight()) {
		setup_message(STRAIGHT);
		set_motors(kilo_turn_left,kilo_turn_right);
		return 1;
	} else if (goLeft()){
		setup_message(LEFT);
		set_motors(kilo_turn_left, 0);
		return 1;
	} else {
		setup_message(STRAIGHT);
		set_motors(kilo_turn_left, kilo_turn_right);
		return 0;
	}
}

void leave() {
    setup_message(LEAVE);
    mydata->my_leader = 255;
    set_color(GREEN);
    set_motors(kilo_turn_left,kilo_turn_right);
}

void join() {
    set_motors(kilo_turn_left,kilo_turn_right);
    setup_message(JOIN);
    if(mydata->new_message && mydata->received_msg.data[1] == OK)
    {
        mydata->my_leader = mydata->received_msg.data[0];
        mydata->joining = 1;
    }
}

void prepareToFollow() {
    mydata->follow_timestamp = kilo_ticks;
    mydata->joining = 0;
    mydata->following = 1;

}

void followPlatoon() {
   if(kilo_ticks< mydata->follow_timestamp + FOLLOW_DELAY) {
       set_motors(kilo_turn_left,kilo_turn_right);
   	   set_color(RED);
   }
   else mydata->following = 0;
}

int checkJoin(){
    if(mydata->following){
        followPlatoon();
        return 1;
    }
    if(mydata->joining){
 	    prepareToFollow();
        return 1;
    }
    if(kilo_ticks == LEAVE_TIME && kilo_uid == CAN_LEAVE) {
        leave();
        return 1;
    }
    if(kilo_ticks >= LEAVE_TIME + JOIN_DELAY && kilo_uid == CAN_JOIN && mydata->my_leader == 255) {
       join();
       return 1;
    }
    if(handleOther() == JOIN) {
	   setup_message(OK);
	   return 1;
    }
    return 0;
}


void follower() {
    if(checkJoin()) return;
    int message = handleMessage();
    if (mydata->turning == 0 && message == LEFT) {
	    mydata->message_timestamp = kilo_ticks;
	    mydata->turning = 1;
    }
    if (mydata->turning == 0 && message == STRAIGHT) {
	    speedCorrection(checkDistance());
	    setup_message(STRAIGHT);
    } else if (mydata->turning == 1) {
	    mydata->turning = handleTurnLeft();
    }

}

/*********************************************************************/



// COMMON CODE
/*********************************************************************/
void loop() {

	if(kilo_ticks >= END_TIME) {
	    set_color(RGB(0,0,0));
	    set_motors(0,0);
	}
	else
	{
        if(kilo_ticks< 32) spinup_motors();
	    if (kilo_uid == 0) leader();
	    else follower();
	}
}

void initMessageFunctions(){
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
}

int main() {
    kilo_init();
    initMessageFunctions();
    kilo_start(setup, loop);
    return 0;
}
