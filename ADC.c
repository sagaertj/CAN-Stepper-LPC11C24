/*----------------------------------------------------------------------------
 * Name:    ADC.c
 * Purpose: MCB1000 low level ADC functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "LPC11xx.h"                            /* LPC11xx definitions        */
#include "ADC.h"

/*----------------------------------------------------------------------------
  initialize ADC Pins
 *----------------------------------------------------------------------------*/
void ADC_init (void)
{

  /* configure PIN GPIO0.11 for AD0 */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1UL <<  6) |   /* enable clock for GPIO      */
                                (1UL << 16) );  /* enable clock for IOCON     */

  LPC_IOCON->R_PIO0_11  =  (2UL <<  0);         /* P0.11 is AD0               */

  LPC_GPIO0->DIR &= ~(1UL << 11);               /* configure GPIO as input    */

  /* configure ADC */
  LPC_SYSCON->PDRUNCFG      &= ~(1UL <<  4);    /* Enable power to ADC block  */
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 13);    /* Enable clock to ADC block  */

  LPC_ADC->CR          =  ( 1UL <<  0) |        /* select AD0 pin             */
                          (23UL <<  8) |        /* ADC clock is 24MHz/24      */
                          ( 1UL << 21);         /* enable ADC                 */ 

}


/*----------------------------------------------------------------------------
  start ADC Conversion
 *----------------------------------------------------------------------------*/
void ADC_startCnv (void)
{
  LPC_ADC->CR &= ~(7UL << 24);                  /* stop conversion            */
  LPC_ADC->CR |=  (1UL << 24);                  /* Start A/D Conversion       */
}


/*----------------------------------------------------------------------------
  stop ADC Conversion
 *----------------------------------------------------------------------------*/
void ADC_stopCnv (void)
{
  LPC_ADC->CR &= ~(7UL << 24);                  /* stop conversion            */
}


/*----------------------------------------------------------------------------
  get converted ADC value
 *----------------------------------------------------------------------------*/
uint32_t ADC_getCnv (void)
{
  uint32_t adGdr;

  while (!(LPC_ADC->GDR & (1UL << 31)));        /* Wait for Conversion end    */
  adGdr = LPC_ADC->GDR;
  return((adGdr >> 6) & ADC_VALUE_MAX);         /* read converted value       */
}
