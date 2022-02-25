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

#ifndef DATAIRQS_C
#define DATAIRQS_C

#include <p30f3011.h>
#include "IRQs.h"
#include "common.h"
#include "DataIO.h"
#include "Init.h"
#include "display.h"
#include "eeprom.h"
#include "checksum.h"

/*****************/
/* IRQs go here. */
/*****************/

/* Wheel Rotate and low priority IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt (void){
    CPUact = 1;
    wheelTime = TMR3;
    TMR3 = 0;
    if (CONDbits.wheelSpin)
        dsky.speed = 3600 * (sets.travel_dist / (wheelTime / 65535)); //This gives us KM per hour.
    CONDbits.wheelSpin = 1;
    IFS1bits.INT1IF = 0;
}
/* Wheel Rotate Timer 3 IRQ */
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void){
    CPUact = 1;
    dsky.speed = 0;
    CONDbits.wheelSpin = 0;
    //End IRQ
    IFS0bits.T3IF = 0;
}

/* Non-critical systems. Timer 4 IRQ */
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt (void){
    CPUact = 1;
    //Check settings ram in background. (lowest priority IRQ))
    if(check_ramSets()){
        //If failed, shutdown and attempt to recover.
        get_settings();
        //Make no more than 5 attempts to recover before going into debug mode.
        if(ram_err_count >= 5) death_loop();
        else ram_err_count++;
    }
    //Runtime program memory check. Checks every half hour.
    if(check_timer == 0x0708){
        check_prog();
        check_timer = 0;
    }
    else check_timer++;
    //End IRQ
    IFS1bits.T4IF = 0;
}

//Heavy process IRQ
//For low priority CPU intensive processes and checks.
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt (void){
    IFS1bits.T5IF = 0;
}

/* Data and Command input and processing IRQ for Port 1 */
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void){
    CPUact = 1;
    Command_Interp(PORT1);
/****************************************/
    /* End the IRQ. */
    IFS0bits.U1RXIF = 0;
}

/* Data and Command input and processing IRQ for Port 2. */
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt (void){
    CPUact = 1;
    Command_Interp(PORT2);
/****************************************/
    /* End the IRQ. */
    IFS1bits.U2RXIF = 0;
}

/* Output IRQ for Port 1 */
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt (void){
    CPUact = 1;
    //Dispatch the buffer to the little 4 word Serial Port buffer as it empties.
    while(U1STAbits.UTXBF == 0 && (Buff_index[PORT1] < Buff_count[PORT1])){
        U1TXREG = Buffer[PORT1][Buff_index[PORT1]];
        Buff_index[PORT1]++;
    }
    //Reset the buffer index and count when done sending.
    Buffrst(PORT1);
    /****************************************/
    /* End the IRQ. */
    IFS0bits.U1TXIF = 0;
}

/* Output IRQ for Port 2 */
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt (void){
    CPUact = 1;
    //Dispatch the buffer to the little 4 word Serial Port buffer as it empties.
    while(U2STAbits.UTXBF == 0 && (Buff_index[PORT2] < Buff_count[PORT2])){
        U2TXREG = Buffer[PORT2][Buff_index[PORT2]];
        Buff_index[PORT2]++;
    }
    //Reset the buffer index and count when done sending.
    Buffrst(PORT2);
    /****************************************/
    /* End the IRQ. */
    IFS1bits.U2TXIF = 0;
}

/****************/
/* END IRQs     */
/****************/   

#endif