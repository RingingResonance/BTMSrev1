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

#ifndef REGULATE_C
#define REGULATE_C

#include <p30f3011.h>

#include "regulate.h"
#include "common.h"
#include "Init.h"

void temperatureCalc(void){
    //Calculate max discharge current based off battery remaining and battery temp.
    temp_dischrg_rate = 1;
    if (dsky.battery_temp > sets.dischrg_reduce_high_temp || dsky.battery_temp < sets.dischrg_reduce_low_temp){
        temp_dischrg_rate = 0.8;     //Decrease the current by 20% if we are too hot or too cold.
    }
    if (dsky.battery_temp < sets.dischrg_min_temp){
        temp_dischrg_rate = 0.5;     //Decrease the current by 50% if we are too cold.
    }
    dischrg_current = (sets.dischrg_C_rating * vars.battery_remaining) * temp_dischrg_rate;
    
    //Don't set our max discharge current below the limp current setpoint.
    if(dischrg_current < sets.limp_current || dsky.battery_temp < sets.dischrg_min_temp){
        dischrg_current = sets.limp_current;
    }
    //Calculate max charge current based off battery temp.
    temp_chrg_rate = 1;
    if (dsky.battery_temp > sets.chrg_reduce_high_temp || dsky.battery_temp < sets.chrg_reduce_low_temp){
        temp_chrg_rate = 0.8;     //Decrease the current by 20% if we are too hot or too cold.
    }
}

void outputReg(void){
    ////Check for key power or command power signal, but not soft power signal.
    if((keySwitch || CONDbits.cmd_power)){
        if(!STINGbits.fault_shutdown){
            ctRelay = 1;          //contactor relay on, but only if fault shutdown is 0
            AUXrelay = 1;         //Turn on AUX 
            if(contact_rly_timer == 3)
                contact_rly_timer = 2;         //wait two 0.125ms cycles before allowing charge regulation to start.
        }
        //Run heater if needed, but don't run this sub a second time if we are getting charge power while key is on.
        //If we are getting charge power then we need to use it to warm the battery to a higher temp if needed.
        //So check charge input first.
        if(!chrgSwitch){
            heat_control(sets.dischrg_target_temp);
        }
        if(contact_rly_timer == 0){
            //Integral Mode
            //High temp limiting.
            voltage_output += analog_smpl_time * ((dsky.battery_temp - sets.dischrg_max_temp) * temp_proportion);
            if(voltage_output > sets.battery_rated_voltage){
                voltage_output = sets.battery_rated_voltage;
            }
            else if(voltage_output < sets.dischrg_voltage){
                voltage_output = sets.dischrg_voltage;
            }
            
            //Low voltage limiting.
            current_output += analog_smpl_time * ((dsky.battery_voltage - voltage_output) * vltg_proportion);
            if(current_output > dischrg_current){
                current_output = dischrg_current;
            }
            else if(current_output < 0){
                current_output = 0;
            }
            
            //Current regulation.
            crnt_error = (current_output - absFloat(dsky.battery_current)) * crnt_proportion;
            if(crnt_error > 10000){
                crnt_error = 10000;
            }
            else if(crnt_error < -10000){
                crnt_error = -10000;
            }
            crnt_integral += (analog_smpl_time * crnt_error);
            if(crnt_integral > 95){
                crnt_integral = 95;
            }
            else if(crnt_integral < 0){
                crnt_integral = 0;
            }
            output_power = crnt_integral;
        }
    }
    else{
        output_power = 0;
        outPWM = 0;             //set output control to 0 before turning off the relay
        LATE = 0;               //Insure all PORTE outputs are off.
        PWMCON1bits.PEN3L = 0;  //Set PWM3 Low side to standard output so that it can be set to 0
        crnt_integral = 0;
        contact_rly_timer = 3; //Reset contactor relay timer
        AUXrelay = 0; //Turn off AUX 
        ctRelay = 0; //Turn off contactor relay
    }
}

void chargeReg(void){
    //Charge current read and target calculation.
    chrg_current = (sets.chrg_C_rating * sets.amp_hour_rating) * temp_chrg_rate;
    //Inhibit charging battery if temperature or voltage is out of range.
    if(dsky.battery_temp > sets.chrg_max_temp || dsky.battery_temp < sets.chrg_min_temp || 
    dsky.battery_voltage > dsky.chrg_voltage + 0.05) chrg_current = 0.01;

    //// Check for Charger.
    if(chrgSwitch){
        chrgLight = 1;          //charger light on.
        if(!STINGbits.fault_shutdown){
            chrgRelay = 1;          //charger relay on, but only if fault shutdown is 0
            if(chrg_rly_timer == 3)chrg_rly_timer = 2;         //wait two 0.125ms cycles before allowing charge regulation to start.
        }
        //Charger timeout check. If charger is plugged in but we aren't getting current then we 
        //need to shutdown and log an error code so we don't run down the battery.
        //Half a volt less than charge voltage.
        if(chrg_check < 20000 && dsky.battery_current < 0 && !keySwitch && 
        dsky.battery_voltage < (dsky.chrg_voltage - 0.5) && !chrg_rly_timer && 
        !heat_power && chrg_current > 0.01) chrg_check++;
        else if(chrg_check >= 20000 && !keySwitch){
            fault_log(0x1B);            //Log insufficient current from charger.
            STINGbits.fault_shutdown = 1;
            chrg_check = 0;
            chrgRelay = 0;          //charger relay off. Don't waste power on a relay that is doing nothing.
            //This usually happens when we detect a charge voltage but the charge regulator isn't passing enough current.
        }
        else if(chrg_check > 0) chrg_check--;
        
        //Run heater if needed, but don't turn it up more than what the charger can handle.
        //This way we don't discharge the battery from trying to run the heater while the charger is plugged in, but
        //not supplying enough current to do both.
        if(dsky.battery_current >= 0) heat_control(sets.chrg_target_temp);
        else if(dsky.battery_current < 0.01 && heat_power > 0) heat_power--;

        //Regulate the charger input.
        if(vars.heat_cal_stage != 2 && !chrg_rly_timer){
            // Charge regulation routine. Clean this up, it needs to use integral math for regulation!!!
            if(((charge_power > 0) && (dsky.battery_voltage >= (sets.battery_rated_voltage - 0.01))) || 
            (dsky.battery_current > (chrg_current + 0.02)))charge_power--;
            else if(((charge_power < 101) && (dsky.battery_voltage < sets.battery_rated_voltage - 0.07)) || 
            (dsky.battery_current < chrg_current))charge_power++;
        }
        else charge_power = 0;       //Inhibit charging if we are in the middle of heater calibration.
    }
    else{
        chrgLight = 0;          //charger light off.
        charge_power = 0;       //charger set to 0
        chrgPWM = 0;            //Set charger output to 0 before turning off relay
        chrgRelay = 0;          //charger relay off.
        chrg_rly_timer = 3;     //reset charge relay timer.
        chrg_check = 0;         //Reset charger check timer
    }
}

//Heater regulation.
void heat_control(float target_temp){
    /* Heater regulation. Ramp the heater up or down. If the battery temp is out
     * of range then the target charge or discharge current will be set to 0 and the charge 
     * regulation routine will power the heater without charging the battery.
     */
    if(vars.heat_cal_stage == 3){
        if(dsky.battery_temp < (target_temp - 0.5) && heat_power < heat_set){
            if(heat_rly_timer == 0)heat_power++;
            heatRelay = 1;     //Heat Relay On
            if(heat_rly_timer == 3)heat_rly_timer = 2; //wait two 0.125ms cycles before allowing heat regulation to start.
        }
        else if(dsky.battery_temp > (target_temp + 0.5)){
            if(heat_power > 0)heat_power--;
            if(heat_power <= 0){
                heatRelay = 0;     //Heat Relay Off
                heat_rly_timer = 3;     //Reset heat relay timer
            }
        }
    }
}

#endif