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


#ifndef __delay__H__
#define __delay__H__


#include <stdint.h>

extern volatile uint64_t ms,rms;
//extern volatile uint64_t millis_count;
extern uint64_t millis(void);
void systick_init_ms(uint32_t freq);
void delay(uint32_t delay);
void reset_tick(void);
#endif
