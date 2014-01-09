/*----------------------------------------------------------------------------
 * Name:    CanDemo.c
 * Purpose: CAN example for MCB1000 populated with LPC11C14
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "LPC11xx.h"                            /* LPC11xx definitions        */
#include "CAN.h"                                /* LPC1766 CAN adaption layer */
#include "ADC.h"                                /* ADC function prototypes */

#include "constan.h"
#include "types.h"

#include "motor.h"

#define  LED1OFF		LPC_GPIO0->DATA |= (1UL << 7)
#define  LED1ON		LPC_GPIO0->DATA &= ~(1UL << 7)
#define  LED2OFF		LPC_GPIO3->DATA |= (1UL << 0)
#define  LED2ON		LPC_GPIO3->DATA &= ~(1UL << 0)
#define  LED3OFF		LPC_GPIO1->DATA |= (1UL << 8)
#define  LED3ON		LPC_GPIO1->DATA &= ~(1UL << 8)


extern void process_serialdata(char * serialdata);
extern volatile long timer1count;
extern volatile long timer1done;


#define CANLISTEN_ID1	0x21
#define CANLISTEN_ID2	0x31
#define CANTARGET_ID	   0x21

//LPC_GPIO0->DATA & 0x80 //PIO0_7
//LPC_GPIO1->DATA & 0x10 //PIO1_4

void LED_out(unsigned char value);

volatile int ti_delay;
volatile int ti_done;

volatile byte serial_data_ready;

/* Import external functions from Serial.c file                               */
extern void SER_init (void);

extern volatile status_t stepstat ;

uint8_t val_Tx = 0, val_Rx = 0;                 /* Globals used for display */

uint8_t hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7','8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};


byte rs_232_buffer[SERIALBUFSIZE];
byte serial_buffer[SERIALBUFSIZE];
byte ch_teller;


void init_motorvars(void);


/*----------------------------------------------------------------------------
  insert a delay time.
 *----------------------------------------------------------------------------*/
void delay(unsigned int nCount)
{
  for(; nCount != 0; nCount--);
}


/*----------------------------------------------------------------------------
  convert one byte to string in hexadecimal notation
 *----------------------------------------------------------------------------*/
void Hex_Str (uint8_t hex, char *str)
{
  *str++ = '0';
  *str++ = 'x';
  *str++ = hex_chars[hex >>  4];
  *str++ = hex_chars[hex & 0xF];
}

/*----------------------------------------------------------------------------
  read a converted value from the Analog/Digital converter
 *----------------------------------------------------------------------------*/
unsigned char adc_Get (void)
{
	unsigned char val;

	ADC_startCnv();                                 /* start A/D conversion */
	val = ((ADC_getCnv() >> 2) & 0xFF);             /* use upper 8 bits of 10 bit AD conversion */
	ADC_stopCnv();                                  /* stop A/D conversion */

	return (val);
}

/*----------------------------------------------------------------------------
  display transmit and receive values
 *----------------------------------------------------------------------------*/
void val_display (void)
{
	static char disp_buf[] = " Tx:    , Rx:    \r";   /* display string */

	Hex_Str(val_Tx, &disp_buf[ 4]);                 /* prepare display string */
	Hex_Str(val_Rx, &disp_buf[13]);
	printf(disp_buf);                               /* print string to serial port */
	LED_out (val_Rx);                               /* display RX val on LEDs */
}


/*----------------------------------------------------------------------------
  initialize CAN interface
 *----------------------------------------------------------------------------*/
void CAN_init (void)
{
  CAN_setup ();                                   /* setup CAN Controller   */
  CAN_wrFilter (CANLISTEN_ID1, STANDARD_FORMAT);           /* Enable reception of messages */
  CAN_wrFilter (CANLISTEN_ID2, STANDARD_FORMAT);           /* Enable reception of messages */

 /* start CAN Controller    */
  LPC_CAN->CNTL &= ~_CNTL_INIT;                     /* Enter normal operating mode */

  /* wait til tx mbx is empty */
  CAN_TxRdy = 1;
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
 
volatile int ii;

//#define SENDER

void setup_motorIO(void);
 
int main (void)
{
	int i;
//	int oldval;

	ADC_init ();                                    /* initialize A/D converter */
	
	//uit led init
	LPC_SYSCON->SYSAHBCLKCTRL |= (1UL <<  6);     	/* enable clock for GPIO      */
	
	NVIC_SetPriority (SysTick_IRQn,0x80);
	
	SysTick_Config(SystemCoreClock/100);            /* Generate IRQ each ~10 ms   */

	SER_init ();                                    /* initialize serial port */
	CAN_init ();                                    /* initialize CAN interface */
	
	
	serial_data_ready=0;

	printf("    MCB1000 Stepmotor Demo    \n\r");
	printf("        CAN         \n\r");
	printf("    www.keil.com    \n\r");
	printf("\r\n");
	printf(" CAN at 500kbit/s   \n\r\n\r");

	CAN_TxMsg.id     =CANTARGET_ID;                        /* initialize message to send */
	for (i = 0; i < 8; i++)
		CAN_TxMsg.data[i] = i; //0;
	CAN_TxMsg.len    = 8;
	CAN_TxMsg.format = STANDARD_FORMAT;
	CAN_TxMsg.type   = DATA_FRAME;

	//led D1 IO Function
	LPC_IOCON->PIO1_10=0xc0;
	//led D2 IO Function
	LPC_IOCON->PIO1_10=0xc0;

	LPC_GPIO1->DIR = 0x00000c00;
	LPC_GPIO1->DATA= 0x00000000;
	
	for (ii=0;ii<1000;ii++)
		delay (10000);  
		
	LED1OFF;
	for (ii=0;ii<1000;ii++)
		delay (1000);  
		
	LED2OFF;
	for (ii=0;ii<1000;ii++)
		delay (1000);  
	LED1ON;
	for (ii=0;ii<1000;ii++)
		delay (1000);  
		
	LED2ON;
	for (ii=0;ii<1000;ii++)
		delay (1000);  
	LED3ON;
		
	setup_motorIO();
	
	init_motorvars();
	
  while (1)
  {
#ifdef SENDER  
    val_Tx = adc_Get ();                          /* TX value changes in any case */
	 
	 i=0;
	 if (LPC_GPIO0->DATA & 0x80)
		i|=0x1;
			
	 if (LPC_GPIO1->DATA & 0x10) //PIO1_4
		i|=0x2;
 
	if (oldval!=i)
	{
 		oldval=i;
		if (CAN_TxRdy)
		{                              /* tx message on CAN Controller */
			CAN_TxRdy = 0;

			//CAN_TxMsg.data[0] = val_Tx;                 /* data[0] field = ADC value */
			CAN_TxMsg.data[0] = i; 
			CAN_wrMsg (&CAN_TxMsg);                     /* transmit message */
		 }
	}

    //delay (10000);                                /* Wait a while to receive the message */
	#endif
   if (CAN_RxRdy)
	 {                              /* rc message on CAN Controller */
      CAN_RxRdy = 0;

      val_Rx = CAN_RxMsg.data[0];
		
		 val_display ();                               /* display  RX values */
    }

 

	//indien een rs232 commando ontvangen
	if (serial_data_ready)
	{
		serial_data_ready=0;
		//printf ("\r\n");
		process_serialdata((char *)serial_buffer);
	}
	

  }
}




void LED_out(unsigned char value)
{
	if (value & 0x01)
		LED1ON;
	else
		LED1OFF;
		
	if (value & 0x02)
		LED2ON;
	else
		LED2OFF;
		
	if (value & 0x04)
		LED3ON;
	else
		LED3OFF;
}



/*----------------------------------------------------------------------------
  SysTick IRQ: Executed periodically (10ms) 
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void)
{
	if (ti_delay)
	{
		ti_delay--;
		if (ti_delay==0)
			ti_done=1;
			
	}
	
	if (timer1count)
	{
		timer1count--;
		if(timer1count==0)
		{
			timer1done=1;
		}
	}
	
	if (stepstat.toholdpowertimer)
	{
		stepstat.toholdpowertimer--;
		if(stepstat.toholdpowertimer==0)
		{
			Set_MotorPower(HOLDMODE);
		}
	}
	
	
}


/**************************************
* Configure timer T32B1 for capture on RFDet input
***************************************/
/*
void SetupT32B1_RFDet_Capture(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 10);    // Enable clock to T32B1 block  

	LPC_IOCON->R_PIO1_0=0xe3;		//PIN function CT32B1_CAP0
	
	//capture on both edges and enable interrupt
	LPC_TMR32B1->CCR=0x07;	
	
	//Prescaler gives  1ùs resolution
	LPC_TMR32B1->PR=48;	
	
	//CT32B1_CAP0
	//TMR32B1CCR
	
	//RUN
	LPC_TMR32B1->TCR=1;
	
	//enable IRQ in NVIC
	NVIC_EnableIRQ(TIMER_32_1_IRQn);
}
*/

void UART_IRQHandler(void)	 
{
	byte received_byte=LPC_UART->RBR;

	if (received_byte == '*')
	{
		ch_teller = 0;           // pointer terug bij begin van string plaatsen
	}
	// einde frame OF PDA, ontvangen
	else if (received_byte == 0xd) //return
	{  
		if((ch_teller < sizeof(rs_232_buffer))  && (serial_data_ready==0))
		{
			// data kopieren
			rs_232_buffer[ch_teller]=0;	 //add null terminator
			strcpy((char *) serial_buffer,(char *)rs_232_buffer); //copieren van de rs232 data          
			serial_data_ready=1;
		}
		ch_teller = 0;
	}
	else if (ch_teller< (sizeof(rs_232_buffer)-1))  //just store       
	{
		rs_232_buffer[ch_teller]=received_byte ;
		ch_teller++;
	}

	//echo to remote
	while ((LPC_UART->LSR & 0x20)==0);   // wacht tot THR leeg is
	LPC_UART->THR=received_byte;
}

