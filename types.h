#ifndef TYPES_H
#define TYPES_H

typedef unsigned char byte;
typedef unsigned short word ;
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;

/*
//#pragma pack(1)
 __packed typedef struct 
{
	unsigned long abs1;//4	
	unsigned long abs2;//4
	unsigned long abs3;//4
	unsigned long relays;//4
	unsigned char cmd ;	//1
	unsigned long cmddata; //4
}   RFDataFrame  ;

*/

/* of 
//#pragma pack(1)
 typedef struct 
{
	unsigned long abs1;//4	
	unsigned long abs2;//4
	unsigned long abs3;//4
	unsigned long relays;//4
	unsigned char cmd ;	//1
	unsigned long cmddata; //4
}  __attribute((packed))  RFDataFrame  ;

*/

//#pragma pack(4)

typedef struct
{
	ulong homing_speed				;		// delay between steps while homing
	long homing_offset				;		// when the homing switch is detected the actual position will be set to this number.(positive or negative number)
	long home_detection_steplimit	;		// maximum number of steps that can be executed while homing.(i.e. no home switch trigger was detected)
	byte homeswitch_mode				;		// bitmask with possible combinations of homing system.
	
	long travel_max					;		// sets the maximum limit on travel	
	long travel_min					;		// sets the minimum limit on travel (use 0 as default)
	long accel_limit					;		// sets the maximum allowable acceleration/deceleration
	
	unsigned long vmax				;		// sets the maximum allowable speed
	
	byte canid							;
	
	byte invalid						;		// contains 0xff when the device is initially flashed
}systemcfg_t;


typedef struct
{
	long accel ;
	long decel;
	long targetpos;
	long newtargetpos;
	long currentpos;
	byte newposrequest;
	
	
	/***************************************
	*	Configuration parameters.
	****************************************/
	//long homing_max_direction		;		// determines the direction and maximum allowed travel to reach the home (0) position
	//long homing_speed					;		// motor speed when homing
	//long homing_acc_dec				;		// acceleration and decceleration while homing.
	//long homing_offset				;		// offset between the home position (0) and the position of the home detection switch
	
	//long travel_maximum_position	;		// sets the maximum limit on travel	
	//long travel_minimum_position	;		// sets the minimum limit on travel
	//long maximum_stepspeed			;		// sets the maximum speed limit
	

	int backlash;
	int v_max;

	long step_count; //running count

	//! What part of the speed ramp we are in.
	unsigned char run_state ;
	
	//! What step_pos to start decelaration
	long decel_start;
	
	//! Sets deceleration rate.
	long decel_val;
	
	//! Minimum time delay (max speed)
	unsigned long min_delay;

	//! Peroid of next timer delay. At start this value set the accelration rate.
	long step_delay;

	//! Direction stepper motor should move.
	unsigned char dir ;
	
	//! True when stepper motor is running.
	unsigned char running;
	unsigned char stopreq;
	unsigned char homingdone;
	unsigned char recalc_after_stop;

	//! Counter used when accelerateing/decelerateing to calculate step_delay.
	long accel_count;

	//werden vroger telkens opnieuw berekent: niet nodig als accel en vmax niet wijzigd
	unsigned long max_s_lim;	//! Number of steps before we hit max speed.
	unsigned long start_step_delay ;	//ook omdat we niet telkens deze moeten herverekenen
	
	unsigned long toholdpowertimer;

} status_t;

#endif
