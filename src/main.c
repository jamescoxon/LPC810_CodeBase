/**************************************************************************/
/*!
 @file     main.c
 
 @section LICENSE
 
 Software License Agreement (BSD License)
 
 Copyright (c) 2013, K. Townsend (microBuilder.eu)
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/
#include <stdio.h>
#include <string.h>
#include "LPC8xx.h"
#include "gpio.h"
#include "mrt.h"
#include "uart.h"
#include "gps.h"

#if defined(__CODE_RED)
#include <cr_section_macros.h>
#include <NXP/crp.h>
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

#define RFM69_SPI_WRITE_MASK 0x80
#define SS_LOCATION    (2)

void configurePins()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);
    
    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0203UL;
    
    /* Pin Assign 1 bit Configuration */
    LPC_SWM->PINENABLE0 = 0xffffffffUL;
}

int main(void)
{
    int navmode = 9;
    
    /* Initialise the GPIO block */
    gpioInit();
    
    /* Initialise the UART0 block for printf output */
    uart0Init(9600);
    
    /* Configure the multi-rate timer for 1ms ticks */
    mrtInit(__SYSTEM_CLOCK/1000);
    
    /* Configure the switch matrix (setup pins for UART0 and GPIO) */
    configurePins();
    
    /* Set the LED pin to output (1 = output, 0 = input) */
    LPC_GPIO_PORT->DIR0 |= (1 << SS_LOCATION);
    
    /* Send some text (printf is redirected to UART0) */
    printf("Hello, LPC810!\n\r");
    
    setupGPS();
    
    while(1)
    {
        mrtDelay(5000);
        navmode = gps_check_nav();
        mrtDelay(500);
        gps_get_position();
        mrtDelay(500);
        gps_check_lock();
        mrtDelay(500);
        
        printf("Data: %d,%d,%d,%d,%d,%d\n\r", lat,lon,alt,navmode,lock,sats);
    }
}