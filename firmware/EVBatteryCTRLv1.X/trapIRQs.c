/*  Electric Vehicle Battery Monitoring System.>
    Copyright (C) <2020>  <Jarrett R. Cigainero>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>*/

#ifndef TRAPIRQS_C
#define TRAPIRQS_C

#include "IRQs.h"
#include "common.h"
#include "sysChecks.h"
#include "eeprom.h"

/*****************/
/* IRQs go here. */
/*****************/

/* CPU TRAPS, log erros here and shut down if needed.*/
void __attribute__((interrupt, no_auto_psv)) _FLTAInterrupt (void){
    CPUact = 1;
    io_off();
    STINGbits.fault_shutdown = 1;
    fault_log(0x0C);        //PWM fault. External.
    save_vars();
    IFS2bits.FLTAIF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail (void){
    //Check the PLL lock bit as fast as possible before anything else because it doesn't stay set for long.
    if(OSCCONbits.LOCK == 0){
        if(!STINGbits.osc_fail_event){
            fault_log(0x12);
        }
    }
    CPUact = 1;
    io_off();
    STINGbits.fault_shutdown = 1;
    if(STINGbits.osc_fail_event){
        fault_log(0x0D);
    }
    STINGbits.osc_fail_event = 1;         //Only log it once per reset.
    save_vars();
    INTCON1bits.OSCFAIL = 0;
}
void __attribute__((interrupt, no_auto_psv)) _AddressError (void){
    CPUact = 1;
    io_off();
    STINGbits.fault_shutdown = 1;
    //send_string(0x03, "Address Error Event.");
    fault_log(0x0E);
    save_vars();
    INTCON1bits.ADDRERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _StackError (void){
    CPUact = 1;
    SPLIM = stackFaultDefault;
    io_off();
    STINGbits.fault_shutdown = 1;
    //send_string(0x03, "Stack Error Event.");
    fault_log(0x0F);
    save_vars();
    INTCON1bits.STKERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _MathError (void){
    CPUact = 1;
    io_off();
    STINGbits.fault_shutdown = 1;
    //send_string(0x03, "Math Error Event.");
    fault_log(0x10);
    save_vars();
    INTCON1bits.MATHERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _ReservedTrap7 (void){
    CPUact = 1;
    io_off();
    STINGbits.fault_shutdown = 1;
    fault_log(0x11);
    save_vars();
    //INTCON1bits.DMACERR = 0;
}
/****************/
/* END IRQs     */
/****************/   

#endif