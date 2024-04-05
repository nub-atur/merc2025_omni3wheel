/**

 * original author: Husamuldeen <https://github.com/hussamaldean>

   ----------------------------------------------------------------------
   	Copyright (C) husamuldeen, 2020

    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */

#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_it.h"

volatile uint64_t ms,rms;
//volatile uint64_t millis_count = 0;
void systick_init_ms(uint32_t freq) /*Frequency in MHz*/
	{
	__disable_irq();
		SysTick->LOAD=(freq/1000)-1;
		SysTick->VAL=0;
		SysTick->CTRL=7; //0b00000111;
		NVIC_SetPriority(SysTick_IRQn,7);
		__enable_irq();
}

uint64_t millis(void)
	{
	__disable_irq();
	rms=ms; //store current ms in rms
	__enable_irq();
	return rms;

	}

void reset_tick(void){
	rms=0;
	ms=0;
}

void delay(uint32_t delay)
	{
	
		uint64_t start=millis();
	do
		{}while(millis()-start!=delay);
	}
