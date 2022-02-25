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

#ifndef SUBSYS_C
#define SUBSYS_C

#include <p30f3011.h>

#include "common.h"
#include "Init.h"
/* Fun fact, you can comment out these includes and it still compiles even though they are needed!
 * Probably because they are included in main.c IDK mplab is weird.
 */

//System power off for power saving.
void power_off(void){
    vars.voltage_percentage_old = voltage_percentage;    //Save a copy of voltage percentage before we shut down.
    // Enough time should have passed by now that the open circuit voltage should be stabilized enough to get an accurate reading.
    save_vars();      //Save variables before power off.
    keepAlive = 0; //Disable Keep Alive signal.
}

//Get absolute value of a variable
float absFloat(float number){
    if(number < 0){
        return number * -1;
    }
    else return number;
}

//Battery Percentage Calculation. This does NOT calculate the % total charge of battery, only the total voltage percentage.
void volt_percent(void){
    if (dsky.battery_voltage <= (sets.dischrg_voltage - 2)){
        voltage_percentage = 0;
        CONDbits.got_open_voltage = 0;
        open_voltage = 0;
    }
    else if (absFloat(dsky.battery_current) < 0.05){
        open_voltage = dsky.battery_voltage;
        voltage_percentage = 100 * ((open_voltage - sets.dischrg_voltage) / (sets.battery_rated_voltage - sets.dischrg_voltage));
        CONDbits.got_open_voltage = 1;
    }
}

//Find current compensation value.
//This code dances around like spaghetti
void current_cal(void){
    float signswpd_avg_cnt = 0;
    signswpd_avg_cnt = dsky.battery_crnt_average * -1;
    //do the current cal.
    if(curnt_cal_stage == 4){
        current_compensate = (signswpd_avg_cnt - sets.circuit_draw);
        curnt_cal_stage = 5;        //Current Cal Complete
        if(vars.heat_cal_stage != 5){
            vars.heat_cal_stage = 1;     //Do a heater cal after we have done current cal unless it is disabled.
        }
        CONDbits.soft_power = 0;
        //Done with current cal.
    }
    //Initialize current cal.
    if(curnt_cal_stage == 1){
        current_compensate = 0;
        io_off();    //Turn off all inputs and outputs.
        curnt_cal_stage = 4;
        CONDbits.soft_power = 1;         //Turn soft power on to run 0.125s IRQ.
    }
}

#endif