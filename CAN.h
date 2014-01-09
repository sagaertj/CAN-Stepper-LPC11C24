/*----------------------------------------------------------------------------
 * Name:    CAN.h
 * Purpose: CAN interface for LPC11C1x
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

#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>                   /* Include standard types               */

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

typedef struct  {
  uint32_t  id;                       /* 29 bit identifier                    */
  uint8_t   data[8];                  /* Data field                           */
  uint8_t   len;                      /* Length of data field in bytes        */
  uint8_t   format;                   /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  uint8_t   type;                     /* 0 - DATA FRAME, 1 - REMOTE FRAME     */
} CAN_msg;

/* Functions defined in module CAN.c */
void CAN_setup         (void);
void CAN_init          (void);
void CAN_wrMsg         (CAN_msg *msg);
void CAN_stMsg         (CAN_msg *msg);
void CAN_rdMsg         (uint32_t can_msgObj, CAN_msg *msg);
void CAN_wrFilter      (uint32_t id, uint8_t filter_type);


extern CAN_msg       CAN_TxMsg;       /* CAN messge for sending               */
extern CAN_msg       CAN_RxMsg;       /* CAN message for receiving            */                                
extern CAN_msg       CAN_RtMsg;       /* CAN message for RTR                  */                                
extern unsigned int  CAN_TxRdy;       /* CAN HW ready to transmit a message   */
extern unsigned int  CAN_RxRdy;       /* CAN HW received a message            */

/*------- LPC11Cxx  CAN Register Definitions ---------------------------------*/

/*- CAN CMDREQ register ------------------------------------------------------*/
#define _CMDREQ_BUSY       (1UL << 15)

/*- CAN CTRL register --------------------------------------------------------*/
#define _CNTL_INIT         (1UL <<  0)
#define _CNTL_IE           (1UL <<  1) 
#define _CNTL_SIE          (1UL <<  2)
#define _CNTL_EIE          (1UL <<  3) 
#define _CNTL_DAR          (1UL <<  5)
#define _CNTL_CCE          (1UL <<  6) 
#define _CNTL_TEST         (1UL <<  7)
	
/*- CAN TEST register --------------------------------------------------------*/
#define _TEST_BASIC        (1UL <<  2)
#define _TEST_SILENT       (1UL <<  3)
#define _TEST_LBACK        (1UL <<  4)

/*- CAN STAT register --------------------------------------------------------*/
#define _STAT_LEC          (7UL <<  0)
#define _STAT_TXOK         (1UL <<  3) 
#define _STAT_RXOK         (1UL <<  4)
#define _STAT_EPASS        (1UL <<  5) 
#define _STAT_EWARN        (1UL <<  6)
#define _STAT_BOFF         (1UL <<  7)

/*- CAN CMDMASK register -----------------------------------------------------*/
#define	_CMDMASK_DATAB     (1UL <<  0)
#define	_CMDMASK_DATAA     (1UL <<  1)
#define	_CMDMASK_TREQ      (1UL <<  2)
#define	_CMDMASK_INTPND    (1UL <<  3)
#define	_CMDMASK_CTRL      (1UL <<  4)
#define	_CMDMASK_ARB       (1UL <<  5)
#define	_CMDMASK_MASK      (1UL <<  6)
#define	_CMDMASK_WR        (1UL <<  7)
#define  _CMDMASK_RD        (0UL <<  7)

/*- CAN MSK1 register --------------------------------------------------------*/
#define _MSK1_MSK          (0xFFFF)

/*- CAN MSK2 register --------------------------------------------------------*/
#define  _MSK2_MSK          (0x1FFF)
#define	_MSK2_MXTD         (1UL << 15)
#define	_MSK2_MDIR         (1UL << 14)

/*- CAN ARB1 register --------------------------------------------------------*/
#define  _ARB1_ID           (0xFFFF)

/*- CAN ARB2 register --------------------------------------------------------*/
#define  _ARB2_ID           (0x1FFF)
#define	_ARB2_DIR          (1UL << 13)
#define	_ARB2_XTD          (1UL << 14)
#define	_ARB2_MSGVAL       (1UL << 15)

/*- CAN MCTRL register -------------------------------------------------------*/
#define	_MCTRL_DLC         (0x0F)   
#define	_MCTRL_EOB         (1UL <<  7)
#define  _MCTRL_TXRQST      (1UL <<  8)
#define	_MCTRL_RMTEN       (1UL <<  9)
#define	_MCTRL_RXIE        (1UL << 10)
#define	_MCTRL_TXIE        (1UL << 11)
#define  _MCTRL_UMASK       (1UL << 12)
#define	_MCTRL_INTPND      (1UL << 13)
#define	_MCTRL_MSGLST      (1UL << 14)
#define	_MCTRL_NEWDAT      (1UL << 15)


#define _MSG_OBJ_MAX        0x0020         /* 1..32 */

#define _STD_FORMAT         0x000007FF     /* 11 bit standard format */	
#define _EXT_FORMAT         0x1FFFFFFF     /* 29 bit extended format */
/* -- end LPC11Cxx  CAN Register Definitions -------------------------------- */

#endif // _CAN_H_


