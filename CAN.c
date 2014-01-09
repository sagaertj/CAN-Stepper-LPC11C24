/*----------------------------------------------------------------------------
 * Name:    Can.c
 * Purpose: CAN interface for for MCB100 populated with LPC11C14
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

#include "LPC11xx.h"                             /* LPC11xx definitions        */
#include "CAN.h"                                 /* LPC11xx CAN adaption layer */

CAN_msg       CAN_TxMsg;                         /* CAN message for sending */
CAN_msg       CAN_RxMsg;                         /* CAN message for receiving */                                
CAN_msg       CAN_RtMsg;                         /* CAN message for sending */

unsigned int  CAN_TxRdy = 0;                     /* CAN HW ready to transmit a message */
unsigned int  CAN_RxRdy = 0;                     /* CAN HW received a message */


/*----------------------------------------------------------------------------
  setup CAN interface.
 *----------------------------------------------------------------------------*/
void CAN_setup (void)
{
	uint32_t i;

	/* CAN_RX, CAN_TX are dedicated Pins so no GPIO configuration is necessary */ 

	/* configure CAN */
	LPC_SYSCON->PRESETCTRL    |=  (1UL <<  3);    /* de-asserts the reset signal */
	LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 17);    /* enable power to CAN  block */

	LPC_CAN->CLKDIV = (SystemCoreClock / 8000000UL) - 1; /* set CAN clk to 8MHz (48000000/8000000)*/

	LPC_CAN->CNTL  =  _CNTL_INIT;                 /* set initialization mode by default */
	LPC_CAN->CNTL |=  _CNTL_CCE;                 /* Start configuring bit timing */

	/* Set bit timing */
	LPC_CAN->BT   = 0x2301;        					/* 500kBit/s @ 8MHz CAN clk */
	LPC_CAN->BRPE = 0x0000;
	
	LPC_CAN->CNTL &= ~_CNTL_CCE;                 /* Stop configuring bit timing */

	NVIC_EnableIRQ(CAN_IRQn);                     /* enable CAN interrupt */

	/* pre-initialize CAN message objects */ 
	for ( i = 0; i < _MSG_OBJ_MAX; i++ )
	{
		LPC_CAN->IF1_CMDMSK = _CMDMASK_WR | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL | _CMDMASK_DATAA | _CMDMASK_DATAB;
		LPC_CAN->IF1_MCTRL  = 0;

		LPC_CAN->IF1_MSK1   = 0;
		LPC_CAN->IF1_MSK2   = 0;

		LPC_CAN->IF1_ARB1   = 0;
		LPC_CAN->IF1_ARB2   = 0;

		LPC_CAN->IF1_DA1    = 0;
		LPC_CAN->IF1_DA2    = 0;
		LPC_CAN->IF1_DB1    = 0;
		LPC_CAN->IF1_DB2    = 0;

		LPC_CAN->IF1_CMDREQ = i+1;                 /* Transfer message object data to message RAM */
		while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
	}

	LPC_CAN->STAT = 0;                            /* reset CAN status register */
	LPC_CAN->CNTL |= (/*_CNTL_DAR| */                  /* disable automatic retransmision */
                    _CNTL_IE  |                 /* enable CAN module interrupts */
                    _CNTL_EIE |                 /* enable CAN error interrupts */
                    _CNTL_SIE );                /* enable CAN status change interrupts */

}


/*----------------------------------------------------------------------------
  set a message to CAN peripheral which is send upon a remote transmission request.
 *----------------------------------------------------------------------------*/
void CAN_stMsg (CAN_msg *msg)
{
	int32_t i;
	uint32_t can_msgObj;
	uint32_t can_msgv;

	can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

	for (i = _MSG_OBJ_MAX - 1; i > -1; i--)
	{
		if ((can_msgv & (1 << i)) == 0)
			break;
	}
	if (i > -1)
		can_msgObj = i + 1;			           /*   valid message object number */
	else
		can_msgObj = 0;			               /* invalid message object number */


	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL | _CMDMASK_DATAA | _CMDMASK_DATAB;
	LPC_CAN->IF1_MCTRL  = _MCTRL_UMASK  /*| _MCTRL_TXRQST */ | _MCTRL_RMTEN | _MCTRL_TXIE  | _MCTRL_EOB    | (msg->len & _MCTRL_DLC);

	if (msg->format == STANDARD_FORMAT)
	{  
		/* handle standard format */
		LPC_CAN->IF1_MSK1  = 0;
		LPC_CAN->IF1_MSK2  = ((msg->id & _STD_FORMAT) << 2) | _MSK2_MDIR;

		LPC_CAN->IF1_ARB1  = 0;
		LPC_CAN->IF1_ARB2  = ((msg->id & _STD_FORMAT) << 2) | _ARB2_DIR | _ARB2_MSGVAL;
	}
	else
	{  
		/* handle extended format */
		LPC_CAN->IF1_MSK1 = ( msg->id        & _MSK1_MSK);
		LPC_CAN->IF1_MSK2 = ((msg->id >> 16) & _MSK2_MSK) | _MSK2_MXTD;

		LPC_CAN->IF1_ARB1 = ( msg->id        & _ARB1_ID);
		LPC_CAN->IF1_ARB2 = ((msg->id >> 16) & _ARB2_ID)  | _ARB2_XTD | _ARB2_DIR | _ARB2_MSGVAL;
	}

	//niet nodig
	//LPC_CAN->IF1_DA1 = *(uint16_t *)&msg->data[0];
	//LPC_CAN->IF1_DA2 = *(uint16_t *)&msg->data[2];
	//LPC_CAN->IF1_DB1 = *(uint16_t *)&msg->data[4];
	//LPC_CAN->IF1_DB2 = *(uint16_t *)&msg->data[6];

	LPC_CAN->IF1_CMDREQ = can_msgObj;               /* Transfer message object data to message RAM */
	while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
}

/*----------------------------------------------------------------------------
  write a message to CAN peripheral and transmit it.
 *----------------------------------------------------------------------------*/
void CAN_wrMsg (CAN_msg *msg)
{
	int32_t i;
	uint32_t can_msgObj;
	uint32_t can_msgv;

	can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

	for (i = _MSG_OBJ_MAX - 1; i > -1; i--)
	{
		if ((can_msgv & (1 << i)) == 0)
			break;
	}
	if (i > -1)
		can_msgObj = i + 1;			           /*   valid message object number */
	else
		can_msgObj = 0;			               /* invalid message object number */


	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR | _CMDMASK_ARB | _CMDMASK_CTRL | _CMDMASK_TREQ | _CMDMASK_DATAA | _CMDMASK_DATAB;
	LPC_CAN->IF1_MCTRL  = _MCTRL_TXRQST  | _MCTRL_TXIE  | _MCTRL_EOB    | (msg->len & _MCTRL_DLC);

	if (msg->format == STANDARD_FORMAT)
	{   /* handle standard format */
		LPC_CAN->IF1_ARB1  = 0;
		LPC_CAN->IF1_ARB2  = ((msg->id & _STD_FORMAT) << 2)           | _ARB2_DIR | _ARB2_MSGVAL;
	}
	else
	{                                  /* handle extended format */
		LPC_CAN->IF1_ARB1 = ( msg->id        & _ARB1_ID);
		LPC_CAN->IF1_ARB2 = ((msg->id >> 16) & _ARB2_ID)  | _ARB2_XTD | _ARB2_DIR | _ARB2_MSGVAL;
	}

	LPC_CAN->IF1_DA1 = *(uint16_t *)&msg->data[0];
	LPC_CAN->IF1_DA2 = *(uint16_t *)&msg->data[2];
	LPC_CAN->IF1_DB1 = *(uint16_t *)&msg->data[4];
	LPC_CAN->IF1_DB2 = *(uint16_t *)&msg->data[6];

	if (msg->type == REMOTE_FRAME)
	{
		LPC_CAN->IF1_CMDMSK &= ~(_CMDMASK_DATAA | _CMDMASK_DATAB);
		LPC_CAN->IF1_ARB2   &= ~(_ARB2_DIR);                 /* trasnmit a Remote Frame Request */
	}

	LPC_CAN->IF1_CMDREQ = can_msgObj;               /* Transfer message object data to message RAM */
	while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);

}

/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it.
 *----------------------------------------------------------------------------*/
void CAN_rdMsg (uint32_t can_msgObj, CAN_msg *msg)
{

  while (LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY);
  LPC_CAN->IF2_CMDMSK = _CMDMASK_RD     | _CMDMASK_MASK | _CMDMASK_ARB   | _CMDMASK_CTRL |  _CMDMASK_INTPND | _CMDMASK_TREQ | _CMDMASK_DATAA | _CMDMASK_DATAB;	
  LPC_CAN->IF2_CMDREQ = can_msgObj;        /* Transfer message object data from message RAM */

  while (LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY);

	/* check the message object whether it is receive or transmit*/
	if (LPC_CAN->IF2_MCTRL & _MCTRL_TXIE)
	{
		CAN_TxRdy = 1;                          /*  set transmit flag   */

		/* release message obect */
		LPC_CAN->IF2_CMDMSK = _CMDMASK_WR | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL | _CMDMASK_DATAA | _CMDMASK_DATAB;
		LPC_CAN->IF2_MCTRL  = 0x0000;

		LPC_CAN->IF2_MSK1   = 0x0000;
		LPC_CAN->IF2_MSK2   = 0x0000;

		LPC_CAN->IF2_ARB1   = 0x0000;
		LPC_CAN->IF2_ARB2   = 0x0000;

		LPC_CAN->IF2_DA1    = 0x0000;
		LPC_CAN->IF2_DA2    = 0x0000;
		LPC_CAN->IF2_DB1    = 0x0000;
		LPC_CAN->IF2_DB2    = 0x0000;

		LPC_CAN->IF2_CMDREQ = can_msgObj;	   	/* Transfer message object data to message RAM */
		while( LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY );
	}
	else
	{
		if (LPC_CAN->IF2_ARB2 & (1UL << 14))
		{  /* check XTD bit (extended format) */
			msg->id = ((LPC_CAN->IF2_ARB2 & _ARB2_ID) << 16) | LPC_CAN->IF2_ARB1;
			msg->format = EXTENDED_FORMAT;
		}
		else
		{
			msg->id = (LPC_CAN->IF2_ARB2 >> 2) & _STD_FORMAT; /* bit 28-18 is 11-bit standard frame */
			msg->format = STANDARD_FORMAT;
		}

		/* add code to check for remote / data frame */

		msg->len = LPC_CAN->IF2_MCTRL & _MCTRL_DLC;	         /* get message object data length */
		*(uint16_t *)&msg->data[0] = LPC_CAN->IF2_DA1;
		*(uint16_t *)&msg->data[2] = LPC_CAN->IF2_DA2;
		*(uint16_t *)&msg->data[4] = LPC_CAN->IF2_DB1;
		*(uint16_t *)&msg->data[6] = LPC_CAN->IF2_DB2;

		CAN_RxRdy = 1;                          /*  set receive flag   */
	}
}


/*----------------------------------------------------------------------------
  setup acceptance filter.
 *----------------------------------------------------------------------------*/
void CAN_wrFilter (uint32_t id, uint8_t format)
{
	int32_t i;
	uint32_t can_msgObj;
	uint32_t can_msgv;

	can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

	for (i = 0; i < _MSG_OBJ_MAX; i++)
	{
		if ((can_msgv & (1 << i)) == 0)
			break;
	}
	
	if (i < _MSG_OBJ_MAX)
		can_msgObj = i + 1;			           /*   valid message object number */
	else
		can_msgObj = 0;			               /* invalid message object number */

	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR  | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL;
	LPC_CAN->IF1_MCTRL  = _MCTRL_UMASK | _MCTRL_RXIE   | _MCTRL_EOB   | _MCTRL_DLC;

	if (format == STANDARD_FORMAT)
	{
		/* handle standard format */
      id = id & _STD_FORMAT;               

      LPC_CAN->IF1_MSK1  =  0;
      LPC_CAN->IF1_MSK2  = (id << 2);

      LPC_CAN->IF1_ARB1  =  0;
      LPC_CAN->IF1_ARB2  = (id << 2)            | _ARB2_MSGVAL;      /* id is stored left-aligned */
   }
   else
	{
		/* handle extended format */
		id = id & _EXT_FORMAT;               

		LPC_CAN->IF1_MSK1 = (id        & _MSK1_MSK);
		LPC_CAN->IF1_MSK2 = (id >> 16) | _MSK2_MXTD;

	   LPC_CAN->IF1_ARB1 = (id        & _ARB1_ID);
	   LPC_CAN->IF1_ARB2 = (id >> 16)             | _ARB2_XTD | _ARB2_MSGVAL;
   }

	LPC_CAN->IF1_CMDREQ = can_msgObj;         /* Transfer message object data to message RAM */
	while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
}


/*----------------------------------------------------------------------------
  CAN interrupt handler
 *----------------------------------------------------------------------------*/
void CAN_IRQHandler (void)
{
  volatile uint32_t can_int, can_stat;
  uint32_t can_msgObj;

  can_int = LPC_CAN->INT;                          /* read interrupt status */
	switch (can_int)
	{
		case 0x0000:
			/* no interrupt */
			break;
		case 0x8000:
			/* status interrupt */
			can_stat = LPC_CAN->STAT;
			if (can_stat & _STAT_TXOK)
			{                   /* TXOK       */
				LPC_CAN->STAT &= ~_STAT_TXOK;                /* reset TXOK */
			}	  
			if (can_stat & _STAT_RXOK)
			{                   /* RXOK       */
				LPC_CAN->STAT &= ~_STAT_RXOK;                /* reset RXOK */
			}	  
			break;
		default:
			/* message object interrupt */
			can_msgObj = can_int & 0x7FFF;
			if ((can_msgObj >=  1) &&  (can_msgObj <= 32))
			{  
				/*   valid msgObj number */
				CAN_rdMsg (can_msgObj, &CAN_RxMsg);      /*  read the message  */
			}
			break;
	}
}
