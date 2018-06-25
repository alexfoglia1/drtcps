#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif


// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// declare state variable type
typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
} orbit_state_t;

// declare variables

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

