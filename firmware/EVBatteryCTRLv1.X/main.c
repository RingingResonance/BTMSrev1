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

#include <p30f3011.h>
#include <libpic30.h>
#include <xc.h>

/* Include the Source AND Headers HERE or else mplab gets pissy for some reason.
 * Don't bother trying to include the source and header files into the project,
 * it doesn't work. Either I'm doing it wrong or it's broken, but what I have here WORKS SO DON'T TOUCH IT.
 */

#include "chipconfig.h"
#include "IRQs.h"
#include "IRQs.c"
#include "subs.h"
#include "subs.c"
#include "DataIO.h"
#include "DataIO.c"
#include "Init.h"
#include "Init.c"
#include "display.c"

/* Program Start */
/***********************************************************
***********************************************************/
int main(void){
    //Calculate space required for eeprom storage.
    cfg_space = cfig_space();
    vr_space = var_space();
    //Set Initial Defaults if EEPROM is blank. Otherwise load Settings from EEPROM
    if (eeprom_read(0x00) != 0x4567){
        default_sets();
    }
    else{
        read_save(0x00);
    }
    //Check to see if we have saved vars once before. Load them if we have.
    if (eeprom_read((cfg_space / 2) + 1) == 0x7654){
        read_romvars((cfg_space / 2) + 1);
    }
    first_check();              //Do an initial reset and warm start check.
    EnableChIRQ = 0;            //Disable Charge Detect IRQ on power up.
    Init();                     //First initialize.
    analog_smpl_time = 1 / (((IPS * 1000000) / (ADCON3upper8 + ADCON3lower8)) / 45);        //Calculate analog sample time.
    //send_string(NLtxtNL, "System Start.", PORT1);
    /*config_save();
    config_load();
    float_send(testguy2);*/

/*****************************/
    // Main Loop.
/*****************************/
    /* Everything is run using IRQs from here on out.
     * Look in the other source files that MPLAB doesn't open automatically.
     */
    for (;;)        //loop forever. or not, idc, we have IRQs for stuff.
    {
        //Deep sleep check.
        if(deep_sleep == 1){
            io_off();               //Turn off all IO before sleeping.
            PORTBbits.RB6 = 0;      //Turn CPU ACT light off.
            power_off();        //Cuts main power to self.
            /* If the keep alive pin isn't used or if the power control hardware
             * is faulty then this does nothing and the micro will 
             * sleep on the next instruction. Even in Sleep it still draws 
             * nearly a quarter of a watt! WTF Microchip? Why did I choose 
             * this microcontroller again?
             * It's what I had on hand when I first started developing this.
             * Oh well, we have ways of getting around it so it works for now.
             */
            Idle();
            deep_sleep = 0;
        }
        else{
            PORTBbits.RB6 = 0;      //Turn CPU ACT light off.
            Idle();                 //Idle Loop, saves power.
        }
    }
    return 0;
}