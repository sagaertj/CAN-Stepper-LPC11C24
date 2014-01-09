#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LPC11xx.h"                            /* LPC11xx definitions        */

//#include "LPC2103.H"                       /* LPC21xx definitions  */

#include "constan.h"
#include "types.h"
#include "motor.h"
#include "main.h"
//#include "isr.h"

volatile long timer1count;
volatile int  timer1done;


volatile status_t stepstat ;
volatile systemcfg_t systemconfig ;
long val1;

void DoMotorRun(void);

void setup_motorIO(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 10);    /* enable power to Timer32_1 */
	
	LPC_IOCON->R_PIO1_1=0xc3;							//	STEP CT32B1_M0 mode for IO pin P1_1 
	LPC_IOCON->PIO3_3=0xc0;								// DIR output		
	
	LPC_IOCON->PIO3_2=0xf0;								// Home 1 input (hys + pu)
	LPC_IOCON->PIO1_10=0xf0;							// Home 2 input (hys + pu)
	
	LPC_IOCON->PIO3_1=0xc0;								// SLEEP
	LPC_IOCON->PIO2_6=0xc0;								// STPDIS
	
	LPC_IOCON->PIO0_7=0xc0;								// led1 
	LPC_IOCON->PIO3_0=0xc0;								// led2
	LPC_IOCON->PIO1_8=0xc0;								// led3

	LPC_IOCON->PIO2_7=0xf0;								// F1 
	LPC_IOCON->PIO2_8=0xf0;								// F2
	LPC_IOCON->PIO2_10=0xf0;							// F3
	
	LPC_GPIO0->DIR=0x80;
	LPC_GPIO1->DIR=0x102;
	LPC_GPIO2->DIR=0x40;
	LPC_GPIO3->DIR=0xb;
	
		
	//setup timer 0
	// timer 0 clock source=PCLK edge
	LPC_TMR32B1->CTCR=0;
	// timer 0 prescaler register	 ( om te delen door 15 : 14 invullen)
	LPC_TMR32B1->PR=14;	 //resolutie 1us
	
	LPC_TMR32B1->EMR=0 ;	

	// match 02 reset on match
	//T0MCR=MCR_MR2R ;
	
	// reset on match en interrupt
	LPC_TMR32B1->MCR=MCR_MR0R | MCR_MR0I ;
	
	//t0 Match on value clock=15000000
	LPC_TMR32B1->MR0=999;
	
	LPC_TMR32B1->TCR=TCR_RESET ;	//stop & reset 
	
	//VICVectCntl0 = IRQSLOT_EN | IRQ_TIMER0; 
	//VICVectAddr0 = (unsigned long) speed_cntr_TIMER1_COMPA_interrupt; 
	//VICVectAddr0 = (unsigned long) ReloadTimer0; 
	//VICIntEnable = TIMER0  ;    // Enable  Interrupt
	
	//enable TIMER_32_1_IRQ in NVIC
	NVIC_EnableIRQ(TIMER_32_1_IRQn);
}

void init_motorvars(void)
{
	stepstat.homingdone=FALSE;
	stepstat.targetpos=0;
	stepstat.accel=DEFAULT_ACC; 
	stepstat.decel=DEFAULT_DEC;
	stepstat.v_max=DEFAULT_SPEED;
	stepstat.backlash=0;

	stepstat.newposrequest=FALSE;
	stepstat.currentpos=0;
	stepstat.running=FALSE;
	stepstat.stopreq=FALSE;
	stepstat.recalc_after_stop=FALSE;
	
	stepstat.toholdpowertimer=0;
}
void process_serialdata(char * serial_buffer)
{
	char *token;
	char seps[] = " ";
	
	token = strtok((char *) serial_buffer, seps );
	if( token != NULL )
	{
		if (0==strncmp(token,"hm",2))
		{
			printf ("to home\r\n");
			stepstat.targetpos=0;
			DoMotorRun();
		}
		else if (0==strncmp(token,"nh",2))
		{
			printf ("setting current pos new homepos\r\n");
			stepstat.currentpos=0;
			//stepstat.newtargetpos=stepstat.currentpos;
		}
		else if (0==strncmp(token,"init",4))
		{
			printf ("initialize defaults\r\n");
			FindHomePosition();
		}
		else if (0==strncmp(token,"cw",2))
		{
			printf ("run forward (CW)\r\n");
			DoContineousRun(CW);
		}
		else if (0==strncmp(token,"ccw",3))
		{
			printf ("run backward (CCW)\r\n");
			DoContineousRun(CCW);
		}
		else if (0==strncmp(token,"stop",4))
		{
			printf ("stopping\r\n");
			StopMotor();
		}
		else if (0==strncmp(token,"ma",2))				 // move absolte
		{
			long pos;
			token = strtok( NULL, seps );
			if (token==NULL)		  // geen locatie nummer opgegeven
				goto error;
			pos=atol(token);

			if (stepstat.targetpos!=pos || stepstat.targetpos!=stepstat.currentpos ) //we want a new target position 
			{				  
				while(stepstat.newposrequest!=FALSE);	//wacht tot request is verwerkt
				//while(stepstat.recalc_after_stop);
				
				//	if(stepstat.newposrequest==FALSE && stepstat.run_state!=DECEL)
				//	while(stepstat.run_state==DECEL);// && (  (stepstat.currentpos-stepstat.newtargetpos) < -20  ||  (stepstat.currentpos-stepstat.newtargetpos) > 20) );
				//if(stepstat.newposrequest==FALSE)
				//{
				
				//if(stepstat.run_state==DECEL)
				//{
					  //stepstat.newtargetpos=pos;
				//}

				//if ((stepstat.currentpos-pos)<-2  || (stepstat.currentpos-pos)>2)	
				//if(stepstat.run_state!=ACCEL)
				//if(stepstat.run_state!=DECEL)
				//{
					stepstat.recalc_after_stop=FALSE;
					if (stepstat.running)
					{
						//stepstat.recalc_after_stop=FALSE;
						stepstat.newtargetpos=pos;
						//deze laatst zetten (irq)
						stepstat.newposrequest=TRUE;
					}	
					else
					{
						stepstat.newtargetpos=pos;
						stepstat.targetpos=pos;
						DoMotorRun();
					}
				//}
			}
		}
		else if (0==strncmp(token,"sy",2))//continue to last target position
		{
			//printf ("syncing real with last job position\r\n");
			DoMotorRun();
		}
		else if (0==strncmp(token,"j+",2)) //jog 1 step CW
		{
			//printf ("1 step forward\r\n");
			stepstat.targetpos+=1;
			DoMotorRun();
		}
		else if (0==strncmp(token,"j-",2)) //jog 1 step CCW
		{
			//printf ("1 step backward\r\n");
			stepstat.targetpos-=1;
			DoMotorRun();
		}
		else if (0==strncmp(token,"J+",2)) //rel 10 steps CW
		{
			//printf ("1 step forward\r\n");
			stepstat.targetpos+=10;
			DoMotorRun();
		}
		else if (0==strncmp(token,"J-",2))	//rel 10 steps CCW
		{
			//printf ("1 step backward\r\n");
			stepstat.targetpos-=10;
			DoMotorRun();
		}
		else if (0==strncmp(token,"mr",2))	//relative move
		{
			int rel;
			token = strtok( NULL, seps );
			if (token==NULL)
				goto error;		  
			rel=atoi(token);
			stepstat.targetpos+=rel;
			printf ("relative move %i\r\n",rel);
			DoMotorRun();
		}
		else if (0==strncmp(token,"ac",2))	//set acceleration
		{
			int acc;
			token = strtok( NULL, seps );
			if (token==NULL)
				goto error;
			acc=atoi(token);
			if ((acc>500000)||(acc<=0))
			   goto error;
			stepstat.accel=acc;
			//printf ("set acceleration %i\r\n",acc);
		}
		else if (0==strncmp(token,"de",2))	//set decelleration
		{
			int dec;
			token = strtok( NULL, seps );
			if (token==NULL)
				goto error;
			dec=atoi(token);
			if ((dec>500000)||(dec<=0))
			   goto error;
			stepstat.decel=dec;
			printf ("set deceleration %i\r\n",dec);
		}
		else if (0==strncmp(token,"ad",2))	// set accel and deceleration
		{
			int dec;
			token = strtok( NULL, seps );
			if (token==NULL)
				goto error;
			dec=atoi(token);
			if ((dec>500000)||(dec<=0))
			   goto error;
			stepstat.accel=stepstat.decel=dec;
			printf ("set deceleration %i\r\n",dec);
		}
//				else if (0==strncmp(token,"bl",2))
//				{
//					int v;
//					token = strtok( NULL, seps );
//					if (token==NULL)
//						goto error;
//					v=atoi(token);
//					if ((v>1000)||(v<0))
//					   goto error;
//					stepstat.backlash=v;
//					printf ("set backlash %i\r\n",v);
//				}
		else if (0==strncmp(token,"vm",2))	//set vMax
		{
			int v;
			token = strtok( NULL, seps );
			if (token==NULL) goto error;
			v=atoi(token);
			if ((v>5000000)||(v<0))
			   goto error;
			stepstat.v_max=v;
			printf ("set vmax %i\r\n",v);
		}
		else if (0==strncmp(token,"ver",3)) //show version
		{
			printf("version %s %s\r\n",SWVERSION,SWDATE ); 
		}
		else if (0==strncmp(token,"sh",2))
		{
			printf ("position      %li\r\n",stepstat.targetpos );
			printf ("real position %li\r\n",stepstat.currentpos ); 
			printf ("backlash      %i\r\n",stepstat.backlash);
			printf ("vmax          %i\r\n",stepstat.v_max);
			printf ("acceleration  %li\r\n",stepstat.accel);
			printf ("deceleration  %li\r\n",stepstat.decel);
			printf ("state         %i\r\n",stepstat.run_state);
			printf ("runstate      %i\r\n",stepstat.running);
			printf ("decel_val     %li\r\n",stepstat.decel_val);
			printf ("decel_start   %li\r\n",stepstat.decel_start);
			printf ("step_count   %li\r\n",stepstat.step_count);
			printf ("accel_count  %li\r\n",stepstat.accel_count);				

			printf ("=====================\r\n");
		}
		else if (0==strncmp(token,"p?",2))	//show current position
		{
			printf ("real position %li\n",stepstat.currentpos ); 
		}
		else
			goto error;

		goto noerrors;
		
		error:
		printf("command error\n");
		noerrors:
		;
	}
	else
		printf("ready\r\n");
	
}
/***************************************************
* Reads the limit switch 
****************************************************/
static byte GetHomeSwitch(void)
{
	//FIO0MASK1=~HOME_0_MASK;
	//return FIO0PIN1 & HOME_0_MASK;
	return LPC_GPIO3->MASKED_ACCESS [HOME_0_MASK]==HOME_0_MASK;
}



void FindHomePosition(void)
{
	stepstat.homingdone=FALSE;
	//achteruit tot switch schakelt
	
	stepstat.currentpos=0;
	stepstat.accel=DEFAULT_ACC; 
	stepstat.decel=DEFAULT_DEC; 
	stepstat.v_max=systemconfig.homing_speed; 
 	stepstat.currentpos=0;
	
	//homing limit and direction
	stepstat.targetpos=systemconfig.home_detection_steplimit;

	//draaien
	DoMotorRun();
	
	//tot de homeswitch schakelt
	while(GetHomeSwitch());
	
	StopMotor();

	stepstat.currentpos=0;
	stepstat.targetpos=0;

	printf("Detected SWITCH ON\n\r");

	//dan traag terugkeren tot de switch ontschakelt
	do 
	{
		stepstat.targetpos+=1;
		DoMotorRun();
		while(stepstat.running);
	}
	while(GetHomeSwitch()==0);

	//als het schakelmoment niet op de 0 staat, dan hier de juiste waarde invullen
	stepstat.currentpos=systemconfig.homing_offset;//positie van homing switch tov nullpunt
	stepstat.homingdone=TRUE;

	printf("Homing done\n\r");
	//vooruit to switch ontschakeld
}



void Set_MotorPower(enum POWERSTATE powermode)
{
	static enum POWERSTATE prevpowermode;

	if (prevpowermode==powermode)
		return;

	prevpowermode=powermode;
 	
	switch(powermode)
	{
		case POWEROFF:
			//set io
			break;
		case HOLDMODE:
			//set io
			break;
		case FULLPOWER:
			//set io

			//wait
			timer1done=0;
			timer1count=FULLPOWERDELAY;
			while(!timer1done);
			break;
		default:
			break;
	}
}

__inline void Set_Motor_Direction(int dir)
{
	if (dir==CCW)
	{
		//ccw OUTPUT HIGH	 (p0.22)
		//FIO0MASK2=0xbf;
		//FIO0PIN2=0x40;
		LPC_GPIO3->MASKED_ACCESS[0x08]= 0x08;
	}
	else
	{
		//ccw OUTPUT low	(p0.22)
		//FIO0MASK2=0xbf;
		//FIO0PIN2=0x00;
		LPC_GPIO3->MASKED_ACCESS[0x08]= 0x00;
	}
}


 __inline  void Step_Motor(void)
{
  	//step pulse word door MAT02 pin gegenereerd
	//indien geen MAT hw pin gebruikt word dan de code hieronder gebruiken
	//Step output high. (p0.23)
	//FIO0MASK2=0x7f;
	//FIO0PIN2=0x80;

	//Step output low. (p0.23)
	//FIO0MASK2=0x7f;
	//FIO0PIN2=0x00;

	// huidige positie bijhouden
	if(stepstat.dir==CW)
		stepstat.currentpos++;
	else
		stepstat.currentpos--;

	stepstat.step_count++;
}

void StopMotor(void)
{
	stepstat.stopreq=TRUE;	
 	while (stepstat.running);

//	stepstat.stopreq=FALSE;	

	//save power
	Set_MotorPower(HOLDMODE);
}

void DoContineousRun(enum DIRECTION dir)
{
	if (dir==CCW)
		stepstat.targetpos=0x7fffffffUL;
	else
		stepstat.targetpos=0x80000000UL;
		
	DoMotorRun();
}

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  step  Number of steps to move (pos - CW, neg - CCW).
 *  accel  Accelration to use, in 0.01*rad/sec^2.
 *  decel  Decelration to use, in 0.01*rad/sec^2.
 *  speed  Max speed, in 0.01*rad/sec.
 */
void DoMotorRun(void)
{
	long relativesteps;

	long accel=stepstat.accel;
	long decel=stepstat.decel;
	unsigned long vmax=stepstat.v_max;
	  
	//! Number of steps before we must start deceleration (if accel does not hit max speed).
	unsigned long accel_lim;

	if (stepstat.running)
	{
		return;
	}

	stepstat.newtargetpos=stepstat.targetpos;
 
	relativesteps=stepstat.targetpos-stepstat.currentpos;

	// Set direction from sign on step value.
	if(relativesteps < 0)
	{
		stepstat.dir = CCW;
		Set_Motor_Direction(CCW);
	 	relativesteps = -relativesteps; // positief getal van maken.

//		if (stepstat.backlash)
//		{
//			relativesteps+=stepstat.backlash;	// bij achteruit draaien ,iets doordraaien en dan terugkeren
//		}
	}
	else if(relativesteps > 0)
	{
		stepstat.dir = CW;
		Set_Motor_Direction(CW);
	}
	else //niets te doen bij relativesteps is 0
		return;

	Set_MotorPower(FULLPOWER);
	
	//berekenen en onthouden
	stepstat.start_step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)    )/100;
	stepstat.step_delay=stepstat.start_step_delay;
	
	// Reset counter.
	stepstat.accel_count = 0;
	stepstat.running = TRUE;

	LPC_TMR32B1->TCR=TCR_RESET;	//reset
	LPC_TMR32B1->EMR=EMR_EMC0_CLR ;
 
	// If moving only 1 step.
  	if(relativesteps == 1)
  	{
		// Move one step...
		stepstat.accel_count = -1;
		// ...in DECEL state.
		stepstat.run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		LPC_TMR32B1->MR0=stepstat.start_step_delay;
	}
  	else// if(step != 0)	  Only move if number of steps to move is not zero.
  	{
		LPC_TMR32B1->MR0=10;// moet minimum 2 zijn !! anders komt de match pin niet hoog ! niet tegenstaande er toch een irq is.	 geen idee van de reden.

		// Refer to documentation for detailed information about these calculations.
		
		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt)/ w
		//stepstat.min_delay = T1_FREQ / vmax; //maw de max snelheid in usteps/s
		
		//orig
		stepstat.min_delay = A_T_x100 / vmax; //maw de max snelheid in usteps/s

		//	printf("stepstat.min_delay=%i\n\r",stepstat.min_delay);
		
		// Set accelration by calc the first (c0) step delay .
	
		//	printf("stepstat.initial_delay=%i\n\r",stepstat.step_delay);

		// Find out after how many STEPS does the speed hit the max speed limit.
		// max_s_lim = speed^2 / (2*alpha*accel)
		stepstat.max_s_lim = (long)vmax*vmax/(long)(((long)A_x20000*accel)/100);
		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move atleast 1 step to get any speed at all.
		if(stepstat.max_s_lim == 0)
		{
      	stepstat.max_s_lim = 1;
    	}
		//printf("max_s_lim=%i\n\r",max_s_lim);
		
		// Find out after how many steps we must start deceleration.
		// n1 = (n1+n2)decel / (accel + decel)
		//kans op overflow !!!!
		//accel_lim = ((long)step*decel) / (accel+decel);	//	step * 180000 / 185000

		//voorkomt overflow door gebruik 64 bit long long
		accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )

		// We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0)
		{
			accel_lim = 1;
		}
		//	printf("accel_lim=%i\n\r",accel_lim);

	   // Use the limit we hit first to calc decel.
		if(accel_lim <= stepstat.max_s_lim)
		{
		   stepstat.decel_val = accel_lim - relativesteps;	 //driehoek profiel
			//printf("stepstat.decel_val=%i\n\r",stepstat.decel_val);
 	   }
	   else
		{
			stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);	//trapezium profiel
			//printf("STEPstat.decel_val=%i\n\r",stepstat.decel_val);
	   }

	   // We must decelrate at least 1 step to stop.
	   if(stepstat.decel_val == 0)
		{
	   	stepstat.decel_val = -1;
	   }

	   // Find step to start decleration.	decel_cal is altijd negatief
	   stepstat.decel_start = relativesteps + stepstat.decel_val;

		//printf("stepstat.decel_start=%i\n\r",stepstat.decel_start);
	
	    // If the maximum speed is so low that we dont need to go via accelration state.
		if(stepstat.step_delay <= stepstat.min_delay)
		{
	      stepstat.step_delay = stepstat.min_delay;
	      stepstat.run_state = RUN;
	  	}
	  	else
		{
			stepstat.run_state = ACCEL;
		}
   }

	/*	{
		long rest=0;

	 	lut[0]=stepstat.start_step_delay;
		stepstat.accel_count=0;
		for (vmax=1;vmax<LUTSIZE;vmax++)
		{
			 	stepstat.accel_count++;
				lut[vmax]= stepstat.step_delay - ((long)((2 * stepstat.step_delay) + rest)/(4 * stepstat.accel_count + 1));
				rest = (long)((2 * stepstat.step_delay)+rest)%(4 * stepstat.accel_count + 1);
				stepstat.step_delay=lut[vmax];
		} *
 		stepstat.step_delay=stepstat.start_step_delay;
		stepstat.accel_count=0;
	}	 */



	LPC_TMR32B1->TCR=TCR_RUN;		//run
}


void DoMotorRun2(void)
{
	long relativesteps=stepstat.targetpos-stepstat.currentpos;

	long accel=stepstat.accel;
	long decel=stepstat.decel;
	unsigned int vmax=stepstat.v_max;
	  
	//! Number of steps before we hit max speed.
	//unsigned long max_s_lim;
	//! Number of steps before we must start deceleration (if accel does not hit max speed).
	unsigned long accel_lim;

	LPC_TMR32B1->TCR=TCR_RESET;	//reset 
 
	// Set direction from sign on step value.
	if(relativesteps < 0)
	{
		stepstat.dir = CCW;
		Set_Motor_Direction(CCW);
	 	relativesteps = -relativesteps; // positief getal van maken.

		if (stepstat.backlash)
		{
			relativesteps+=stepstat.backlash;	// bij achteruit draaien ,iets doordraaien en dan terugkeren
		}
	}
	else if(relativesteps > 0)
	{
		stepstat.dir = CW;
		Set_Motor_Direction(CW);
	}
	else
	{  
		//zorgtvoor 1 extra int.
		LPC_TMR32B1->MR0=stepstat.step_delay; 
		// go for it :Start Timer
		LPC_TMR32B1->TCR=TCR_RUN;		//run
		//niets te doen bij relativesteps is 0
		return;
	}
	// Reset counter.
	stepstat.accel_count = 0;

	//startsnelheid berekenen
	stepstat.step_delay=stepstat.start_step_delay;

	// If moving only 1 step.
  	if(relativesteps == 1)
  	{
		// Move one step...
		stepstat.accel_count = -1;
		// ...in DECEL state.
		stepstat.run_state = DECEL;
	}
  	// Only move if number of steps to move is not zero.
  	else// if(step != 0)
  	{
		accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )

		// We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0)
			accel_lim = 1;

	   // Use the limit we hit first to calc decel.
		if(accel_lim <= stepstat.max_s_lim)
		{
		   stepstat.decel_val = accel_lim - relativesteps;
 	   }
	   else
		{
			stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);
	   }

	   // We must decelrate at least 1 step to stop.
	   if(stepstat.decel_val == 0)
	   	stepstat.decel_val = -1;

	   // Find step to start decleration.	decel_cal is altijd negatief
	   stepstat.decel_start = relativesteps + stepstat.decel_val;
  	
	    // If the maximum speed is so low that we dont need to go via accelration state.
		if(stepstat.step_delay <= stepstat.min_delay)
		{
	      stepstat.step_delay = stepstat.min_delay;
	      stepstat.run_state = RUN;
	  	}
	  	else
		{
			stepstat.run_state = ACCEL;
		}
   }

	//bijna direct irq als we de timer starten
	LPC_TMR32B1->MR0=10; 

	// go for it :Start Timer
 	LPC_TMR32B1->TCR=TCR_RUN;		//run
}


/*! \brief Timer/Counter1 Output Compare A Match Interrupt.
 *
 *  Timer/Counter1 Output Compare A Match Interrupt.
 *  Increments/decrements the position of the stepper motor
 *  exept after last position, when it stops.
 *  The \ref step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */


long tval1;
long tval2;
long tval3;

void TIMER32_1_IRQHandler( void )//	__irq
{
	volatile int i;//voor dummy delay loop
//	int index;
	
	// Holds next delay period.
	long new_step_delay=0;
	// Remember the last step delay used when accelrating.
	static unsigned long last_accel_delay;
	
	// Keep track of remainder from new_step-delay calculation to increase accurancy
	static long rest = 0;

  	//reset the int flag
	if ((LPC_TMR32B1->IR & 1)==1 )
		LPC_TMR32B1->IR=1; 

	if(stepstat.step_delay>stepstat.start_step_delay)
		stepstat.step_delay=stepstat.start_step_delay;
	
	//the next delay is setup in advance in the matct register
	LPC_TMR32B1->MR0= stepstat.step_delay;

eval:

	switch(stepstat.run_state)
	{
//		case DOBACKLASH:
//			if(step_count==stepstat.backlash)
//			{
//				stepstat.run_state = STOP;
//			}
//			else
//			{
//			    Step_Motor();
//			}
//			new_step_delay=stepstat.step_delay ;
//	     	break;
		case RUN:
			Step_Motor();
			new_step_delay = stepstat.min_delay;

			//herbereken decel_start
			if(stepstat.newposrequest)
			{
				stepstat.newposrequest=FALSE;
							
				//herberekenen van  decel_val bij dezlfde richting
				if(stepstat.dir==CW )
				{
					//hebben we nog genoeg ruimte over om af te remmen ?
					if ((stepstat.currentpos-stepstat.newtargetpos) <  stepstat.decel_val)
					{
						//decel_start punt verder leggen
						//stepstat.decel_start= stepstat.decel_start+(stepstat.newtargetpos-stepstat.targetpos);
						long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						unsigned long accel_lim;

						long relativesteps=stepstat.newtargetpos-stepstat.currentpos;
		 
		 				accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )

					   // Use the limit we hit first to calc decel.
						if(accel_lim <= stepstat.max_s_lim)
						{
						   stepstat.decel_val = accel_lim - relativesteps;
				 	   }
					   else
						{
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);
					   }
				
					   // We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;
				
					   // Find step to start decleration.	decel_cal is altijd negatief
					   stepstat.decel_start = relativesteps + stepstat.decel_val;
  						stepstat.step_count = 0;
					}
					else
					{
						//nieuwe target ligt in huidig decel gebied dus (of in andere richting) direct beginnen vertragen met de 
						stepstat.accel_count = stepstat.decel_val;
						// Start decelration with same delay as accel ended with.
						new_step_delay = last_accel_delay;
						stepstat.run_state = DECEL;
						stepstat.recalc_after_stop=TRUE;
					}
				}
				else	// CCW
				{
					if ((stepstat.newtargetpos-stepstat.currentpos) <  stepstat.decel_val)
					{
						//decel_start verder leggen
						//stepstat.decel_start=stepstat.decel_start-(stepstat.newtargetpos-stepstat.targetpos);
						long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						unsigned long accel_lim;

						long relativesteps=stepstat.currentpos-stepstat.newtargetpos;
		 
		 				accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )

					   // Use the limit we hit first to calc decel.
						if(accel_lim <= stepstat.max_s_lim)
						{
						   stepstat.decel_val = accel_lim - relativesteps;
				 	   }
					   else
						{
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);
					   }
				
					   // We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;
				
					   // Find step to start decleration.	decel_cal is altijd negatief
					   stepstat.decel_start = relativesteps + stepstat.decel_val;
  						stepstat.step_count = 0;
					}
					else
					{
						//we moeten omkeren want we zijn te ver om nog af te remmen
						stepstat.accel_count = stepstat.decel_val;
						// Start decelration with same delay as accel ended with.
						new_step_delay = last_accel_delay;
						stepstat.run_state = DECEL;
						stepstat.recalc_after_stop=TRUE;
					}
				}
				stepstat.targetpos=stepstat.newtargetpos;
				break;
			}
			// Check if we should start decelration.
			if(stepstat.step_count >= stepstat.decel_start || stepstat.stopreq)
			{
				stepstat.stopreq=FALSE;
				stepstat.accel_count = stepstat.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				stepstat.run_state = DECEL;
			}
			break;
  		case STOP:
//			if (stepstat.dir == CCW && stepstat.backlash >0)
//			{
//				long step=stepstat.backlash;
//   
//   			// identiek stuk zoals de initiele berekening.
//				unsigned long accel_lim;
//				
//				stepstat.step_count=0;
//				rest = 0;
//				T0TCR=TCR_STOP;
//				
//				stepstat.dir = CW;
//				Set_Motor_Direction(stepstat.dir);
//				
//				stepstat.step_delay=stepstat.start_step_delay;
//
//				accel_lim = ((long)step*stepstat.decel) / (stepstat.accel+stepstat.decel);
//				if(accel_lim == 0)
//					accel_lim = 1;
//				
//				if(accel_lim <= stepstat.max_s_lim)
//				{
//				   stepstat.decel_val = accel_lim - step;
//				}
//				else
//				{
//					stepstat.decel_val = - ((stepstat.max_s_lim*stepstat.accel)/stepstat.decel);
//				}
//				
//				if(stepstat.decel_val == 0)
//					stepstat.decel_val = -1;
//				
//				stepstat.decel_start = step + stepstat.decel_val;
//				
//				if(stepstat.step_delay <= stepstat.min_delay)
//				{
//				   stepstat.step_delay = stepstat.min_delay;
//				   stepstat.run_state = RUN;
//				}
//				else
//				{
//				   stepstat.run_state = ACCEL;
//				}
//				
//				// Reset counter.
//				stepstat.accel_count = 0;
//				
//				//stepstat.running = TRUE;
//				T0MR0=0; //korte tijd om timer te starten
//				// Start Timer
//				T0TCR=TCR_RESET;	//reset 
//				T0TCR=TCR_RUN;		//run
//				VICVectAddr = 0;  // Acknowledge Interrupt
//				//stepstat.cmdbusy=FALSE;
//				return ;
//				//goto ok;
//			}
//			else //gewoon zonder backlash
			{
				if(stepstat.recalc_after_stop)
			 	{
			 		stepstat.recalc_after_stop=FALSE;
					stepstat.step_count = 0;
					rest = 0;
					DoMotorRun2();
					new_step_delay=stepstat.step_delay;
					break;
				}
				else
				{
					if(stepstat.newposrequest)
					{
						stepstat.targetpos=stepstat.newtargetpos;
						stepstat.newposrequest=0;
						stepstat.step_count = 0;
						rest = 0;
		 				DoMotorRun2();
						new_step_delay=stepstat.step_delay;
						break;
					}	
					else
					{
						// Stop Timer
						LPC_TMR32B1->TCR=TCR_RESET; //impliceert TCR_STOP
						stepstat.step_count = 0;
						rest = 0;
						stepstat.running = FALSE;
						//clear pin en Disable Match Pin
						LPC_TMR32B1->EMR=0;
					}
				}
			}
			break;
	
		case ACCEL:
			Step_Motor();
 
			stepstat.accel_count++;
			new_step_delay = stepstat.step_delay - ((long)((2 * stepstat.step_delay) + rest)/(4 * stepstat.accel_count + 1));
			rest = (long)((2 * stepstat.step_delay)+rest)%(4 * stepstat.accel_count + 1);
			//stepstat.step_delay=new_step_delay;

//			index=stepstat.accel_count;
//			if (index<LUTSIZE)
//			{
//				new_step_delay=lut[index];
//				if (index>0)
//					stepstat.step_delay=lut[index-1];
//			}
			
			if(stepstat.stopreq)
			{
				stepstat.stopreq=FALSE;
				stepstat.accel_count =-stepstat.step_count;
				stepstat.run_state = DECEL;
				//last_accel_delay = stepstat.step_delay;
				new_step_delay = last_accel_delay;
				break;
			}

			if(stepstat.newposrequest)
			{
				stepstat.newposrequest=FALSE;
				//herberekenen van  decel_val bij dezlfde richting
				if(stepstat.dir==CW )
				{
					//ligt de nieuwe positie verder ?
					if (stepstat.currentpos<stepstat.newtargetpos)
					{
						//opnieuw berekenen decel punt en reking houden met reeds afgelegde weg
						long relativesteps;
						long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						  
						//! Number of steps before we must start deceleration (if accel does not hit max speed).
						unsigned long accel_lim;
					 						
						relativesteps=stepstat.newtargetpos-stepstat.currentpos;
					
						accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )
				
						// We must accelrate at least 1 step before we can start deceleration.
						if(accel_lim == 0)
							accel_lim = 1;
				
					   // Use the limit we hit first to calc decel.
						if(accel_lim <= stepstat.max_s_lim)
						   stepstat.decel_val = accel_lim - relativesteps;
					   else
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);
				
					   // We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;
				
					   // Find step to start decleration.	decel_cal is altijd negatief
					   stepstat.decel_start = relativesteps + stepstat.decel_val;
						stepstat.decel_start+= stepstat.accel_count;

					   // If the maximum speed is so low that we dont need to go via accelration state.
						if(stepstat.step_delay <= stepstat.min_delay)
						{
					      stepstat.step_delay = stepstat.min_delay;
					      stepstat.run_state = RUN;
					  	}
					}
					else //OK	vertragen en omkeren
					{
						//nieuwe target ligt in huidig decel gebied dus (of in andere richting) direct beginnen vertragen,bij het stoppen worden de nieuwe punten berekend.
						stepstat.accel_count =-stepstat.step_count;
						stepstat.step_count=0;
						stepstat.run_state = DECEL;
						stepstat.recalc_after_stop=TRUE;
						//break;
					}
				}
				else	// CCW
				{
					if (stepstat.newtargetpos < stepstat.currentpos)
					{
						//dopnieuw berekenen decel punt en reking houden met reeds afgelegde weg
						long relativesteps;
			  			long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						  
						//! Number of steps before we must start deceleration (if accel does not hit max speed).
						unsigned long accel_lim;
					 						
						relativesteps=stepstat.currentpos-stepstat.newtargetpos;	 //omgekeerd als bij CW	 rest is hetzelfde
					
						accel_lim = ((long long)relativesteps*decel) / (accel+decel);	//	(4000 * 180000) / (5000 + 180000 )
				
			  				
						// We must accelrate at least 1 step before we can start deceleration.
						if(accel_lim == 0)
							accel_lim = 1;
				
					   // Use the limit we hit first to calc decel.
						if(accel_lim <= stepstat.max_s_lim)
						   stepstat.decel_val = accel_lim - relativesteps;
					   else
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);
				
					   // We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;
				
					   // Find step to start decleration.	decel_cal is altijd negatief
					   stepstat.decel_start = relativesteps + stepstat.decel_val;
						stepstat.decel_start+= stepstat.accel_count;
					
					    // If the maximum speed is so low that we dont need to go via accelration state.
						if(stepstat.step_delay <= stepstat.min_delay)
						{
					      stepstat.step_delay = stepstat.min_delay;
					      stepstat.run_state = RUN;
					  	}
					}
					else	//OK	vertragen en omkeren
					{
						//nieuwe target ligt in huidig decel gebied dus (of in andere richting) direct beginnen vertragen 
						stepstat.accel_count =-stepstat.step_count;
						stepstat.step_count=0;
						
						stepstat.recalc_after_stop=TRUE;
						//break;
					}
				}
				stepstat.targetpos=stepstat.newtargetpos;
			}
			else	//normale gang van zaken
			{
				// Check if we should start decelration.
				if(stepstat.step_count >= stepstat.decel_start )
				{
					stepstat.accel_count = stepstat.decel_val;
					stepstat.run_state = DECEL;
					stepstat.step_count=0;	//ccccccccccccccccccccccccccccccccccccccccccccccccc
				}
				// Check if we hitted max speed.
				else if(new_step_delay <= stepstat.min_delay)
				{
					last_accel_delay = new_step_delay;
					new_step_delay = stepstat.min_delay;//max speed
					stepstat.run_state = RUN;
				}
			}
			break;

		case DECEL:
			Step_Motor();
  		   stepstat.accel_count++;			//negatief getal -> -> 0
			if(stepstat.accel_count == 0)//laatste stop voor het einde
			{
				new_step_delay = stepstat.start_step_delay ;
				rest=0;
			}
			else
			{
				new_step_delay = stepstat.step_delay - ((long)((2 * stepstat.step_delay) + rest)/(4 * stepstat.accel_count + 1));
				rest = (long)((2 * stepstat.step_delay)+rest)%(4 * stepstat.accel_count + 1);
			}

 #define USEV2
			if(stepstat.newposrequest)
			{
				stepstat.newposrequest=FALSE;
								
				//herberekenen van  decel_val bij dezlfde richting
				if(stepstat.dir==CW )
				{
					//ligt het nieuwe eindpunt verder dan het huidige ?
					if (stepstat.newtargetpos>(1+(stepstat.currentpos - stepstat.accel_count)))	 //accel count is negatief !
					{
						//opnieuw berekenen decel punt en reking houden met reeds afgelegde weg
						long relativesteps=stepstat.newtargetpos-stepstat.currentpos;
		 				long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						unsigned long accel_lim;
						long relativesteps2;
						  
						stepstat.accel_count=-stepstat.accel_count;
						
#ifdef USEV1		 		  		
						relativesteps2= relativesteps - stepstat.accel_count + ((stepstat.accel_count * (accel + decel)) / decel);
						accel_lim = ((long long)relativesteps2*decel) / (accel+decel);
#else
						//accel_lim= ((long long)relativesteps*decel) / (accel+decel);
						//accel_lim= relativesteps - accel_lim;
						//of
						//accel_lim= ((long long)relativesteps*accel) / (accel+decel);
						relativesteps2= relativesteps + 	((stepstat.accel_count * decel) /accel);
						accel_lim = ((long long)relativesteps2*decel) / (accel+decel);
#endif						
						// We must accelrate at least 1 step before we can start deceleration.
						if(accel_lim == 0)
							accel_lim = 1;

					   // Use the limit we hit first to calc decel.
						if(accel_lim <= (stepstat.max_s_lim))
						{
							//driehoek
#ifdef USEV1
		 					stepstat.decel_val =	-accel_lim;	  //v1
#else
							stepstat.decel_val = accel_lim - relativesteps2;//v2
#endif
						}
					   else
						{
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);  //trapezium
						}

					   // We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;
		  						
					   // Find step to start decleration.	decel_cal is altijd negatief
	   				stepstat.decel_start = relativesteps + stepstat.decel_val;
						stepstat.step_count=0;

						stepstat.step_delay=new_step_delay;
						new_step_delay = stepstat.step_delay - ((long)((2 * stepstat.step_delay) + rest)/(4 * stepstat.accel_count + 1));
						rest = (long)((2 * stepstat.step_delay)+rest)%(4 * stepstat.accel_count + 1);

						stepstat.run_state = ACCEL;
					}
					else if (stepstat.newtargetpos<(stepstat.currentpos - stepstat.accel_count))
					// if (stepstat.newtargetpos<=(stepstat.currentpos - stepstat.accel_count))	//	OK blijven vertragen en bij het stoppen heberekenen en omkeren
					{
						stepstat.recalc_after_stop=TRUE;
					}
//					else
//					{
//						 tval1=stepstat.run_state;
//						 tval2=stepstat.accel_count;
//						 if(tval2!=0)
//						 	tval3=12;
//						 stepstat.run_state = STOP;
//					}
				}
				else	// CCW
				{
					if ((stepstat.newtargetpos)<(stepstat.currentpos + stepstat.accel_count-1))
					{
						long relativesteps=stepstat.currentpos-stepstat.newtargetpos;
						//opnieuw berekenen decel punt en reking houden met reeds afgelegde weg
						long accel=stepstat.accel;
						long decel=stepstat.decel;
						unsigned int vmax=stepstat.v_max;
						unsigned long accel_lim;
						long relativesteps2;
						  
						stepstat.accel_count=-stepstat.accel_count;

#ifdef USEV1
					   relativesteps2= relativesteps - stepstat.accel_count + ((stepstat.accel_count * (accel + decel)) / decel);
						accel_lim = ((long long)relativesteps2*decel) / (accel+decel);
#else
						relativesteps2= relativesteps + 	((stepstat.accel_count * (long)decel) /accel);
						accel_lim = ((long long)relativesteps2*decel) / (accel+decel);
#endif
		  				// We must accelrate at least 1 step before we can start deceleration.
						if(accel_lim == 0)
							accel_lim = 1;

						//tval1=accel_lim;
						//tval2=relativesteps2;
						//tval3=relativesteps;

						//tval2= relativesteps + 	((stepstat.accel_count * decel) /accel);
						//tval2= relativesteps + 	((stepstat.accel_count * (long)decel) /accel);
						//tval2= relativesteps + 	((long)(stepstat.accel_count * decel) /accel);
						//tval2= relativesteps + 	((long)(stepstat.accel_count * (long)decel) /(long)accel);	//ok
						//tval2= relativesteps + 	((stepstat.accel_count * decel) /(long)accel);


											
				      // Use the limit we hit first to calc decel.
						if(accel_lim <= (stepstat.max_s_lim))
						{
						  	//driehoek
#ifdef USEV1
		 					stepstat.decel_val =	-accel_lim;	 //v1
#else
							stepstat.decel_val = accel_lim - relativesteps2;	  //v2
#endif
						}
					   else
						{
							stepstat.decel_val = - ((stepstat.max_s_lim*accel)/decel);  //trapezium
						}

					 	// We must decelrate at least 1 step to stop.
					   if(stepstat.decel_val == 0)
					   	stepstat.decel_val = -1;

	  			   	// Find step to start decleration.	decel_cal is altijd negatief
						stepstat.decel_start = relativesteps + stepstat.decel_val;
	  					stepstat.step_count=0;

						stepstat.step_delay=new_step_delay;
						new_step_delay = stepstat.step_delay - ((long)((2 * stepstat.step_delay) + rest)/(4 * stepstat.accel_count + 1));
						rest = (long)((2 * stepstat.step_delay)+rest)%(4 * stepstat.accel_count + 1);
						
						stepstat.run_state = ACCEL;
					}
					else if (stepstat.newtargetpos>(stepstat.currentpos + stepstat.accel_count))//	OK blijven vertragen en omkeren
					{
						stepstat.recalc_after_stop=TRUE;
					}
				}
				stepstat.targetpos=stepstat.newtargetpos;	
			}
			//else	//normale gang van zaken
			// Check if we at last step
		   {
				if(stepstat.accel_count >= 0)
			   {
					tval1=stepstat.run_state;
					tval2=stepstat.accel_count;
					stepstat.run_state = STOP;
					goto eval;
 			   }
			}
		   break;
	}
 
 	stepstat.step_delay = new_step_delay;

	val1=stepstat.step_delay;

	stepstat.toholdpowertimer=TOHOLDPOWERDELAY;  

	if (stepstat.run_state!=STOP)
	 	LPC_TMR32B1->EMR=EMR_EMC0_CLR;		//clear en re-arm	
	else
		LPC_TMR32B1->EMR=0;				 		//clear en disable	

	//VICVectAddr = 0;  			// Acknowledge Interrupt

//	LPC_GPIO2->IC = 0x1; //clear int  ???????????
}


/***************************************************
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 ***************************************************/
static unsigned long sqrt(unsigned long x)
{
	unsigned long xr;  		// result register
	unsigned long q2;  		// scan-bit register
	unsigned char f;   		// flag (one bit)

	xr = 0;                	// clear result
	q2 = 0x40000000L;       // higest possible result bit
	do
	{
		if((xr + q2) <= x)
		{
			x -= xr + q2;
			f = 1;            // set flag
		}
		else
		{
			f = 0;            // clear flag
		}
		xr >>= 1;
		if(f)
		{
			xr += q2;     		// test flag
		}
	}
	while(q2 >>= 2);        // shift twice

	if(xr < x)
  	{
   	return xr +1;        // add for rounding
  	}
  	else
  	{
    	return xr;
  	}
}



