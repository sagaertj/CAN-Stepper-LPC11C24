#ifndef STEPPER_H
#define STEPPER_H

#define TRUE 1
#define FALSE 0

//#define DEFAULTACCELERATION 20000 
//#define DEFAULTDECELERATION 20000
#define DEFAULTMINSPEED 50
//#define DEFAULTMAXSPEED 8000
//#define DEFAULTRUNSPEED 1000

#define FULLPOWERDELAY		10
#define TOHOLDPOWERDELAY	1000	//1sec


// Direction of stepper motor movement
enum DIRECTION
{
 	CW=0,
	CCW=1
} ;

enum POWERSTATE
{
 	POWEROFF=0,
	HOLDMODE=1,
	FULLPOWER=2
} ;

// Speed ramp states
enum RAMPSTATE
{
 	STOP	=0,
	ACCEL	=1,
	DECEL	=2,
	RUN	=3,
	DOBACKLASH=4
} ;


// Timer resulution for stepping 1 usec 
#define T1_FREQ  		1000000

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 								200
#define MICROSTEPS						8
#define STEPSPERREVISION 				(FSPR * MICROSTEPS)	 		//  microsteps  (1600)


// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#ifdef USERADIANSUNITS
#define ALPHA ( 2*3.1415926 / STEPSPERREVISION ) 			// 2*pi/spr		=0.005235983333  snelheid in 0.01rads /sec
#else
#define ALPHA ( 0.01 ) 	 							// om snelheid units te hebben in usteps /sec
#endif


// (ALPHA / T1_FREQ)*100
#define A_T_x100 ((long)(ALPHA * T1_FREQ * 100))  // is gelijk aan 1000000 in geval van alpha gelijk aan 0.01
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000


static unsigned long sqrt(unsigned long v);

//void speed_cntr_TIMER1_COMPA_interrupt( void );	//__irq;

void FindHomePosition(void);
void Set_MotorPower(enum POWERSTATE powermode);
void Step_Motor(void);
void DoContineousRun(enum DIRECTION dir);
void StopMotor(void);
void DoMotorRun(void);
void init_motorvars(void);
//void DoMotorRun2(void);

#endif
