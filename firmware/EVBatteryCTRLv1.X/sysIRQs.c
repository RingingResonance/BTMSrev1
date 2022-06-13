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

#ifndef sysIRQS_C
#define sysIRQS_C

#include "IRQs.h"
#include "common.h"
#include "DataIO.h"
#include "Init.h"
#include "display.h"
#include "eeprom.h"
#include "checksum.h"
#include "sysChecks.h"
#include "regulate.h"


/* Charge Detect IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt (void){
    CPUact = 1;
    //Don't do anything here. Charger plug IRQ is only to wake the micro from deep sleep if needed.
    IFS0bits.INT0IF = 0;
}

/* Analog Input IRQ */
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void){
    CPUact = on;
    
    /* I have left commented out code in this section for showing the process
     * of converting analog inputs into voltages, currents, and temps.
     */
    //Get 8 samples for averaging.
    if (analog_avg_cnt >= 8){
        //Battery current.
        avgCurnt /= 8;      //Sample average.
        avgCurnt -= 32768;    //Set zero point.
        avgCurnt /= 32768;    //Convert to signed fractional. -1 to 1
        avgCurnt *= 2.5;      //Convert to +-2.5'volts'. It's still a 0 - 5 volt signal on the analog input. The zero point is at 2.5v
        dsky.battery_current = (avgCurnt / 0.04) + current_compensate; //Offset for ACS780LLRTR-050B-T Current Sensor.
        currentCheck();     //Check for over current condition.
        avgCurnt = 0;       //Clear average.

        //Battery voltage.
        //avgVolt /= x;      //Sample average.
        //avgVolt /= 65535;  //Convert to unsigned fractional
        //avgVolt *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        avgVolt /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        dsky.battery_voltage = (avgVolt / vltg_dvid) + sets.bt_vlt_adjst;    //Use resistor divider values to covert to actual voltage.
        avgVolt = 0;       //Clear average.

        //Battery temperature.
        //avgBTemp /= x;      //Sample average.
        //avgBTemp /= 65535;  //Convert to unsigned fractional
        //avgBTemp *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        avgBTemp /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        avgBTemp -= 0.48;   //Offset for LM62 temp sensor.
        dsky.battery_temp = avgBTemp / 0.0156;    //Convert to Degrees C
        avgBTemp = 0;       //Clear average.

        //motor controller temperature.
        //avgMTemp /= x;      //Sample average.
        //avgMTemp /= 65535;  //Convert to unsigned fractional
        //avgMTemp *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        avgMTemp /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        avgMTemp -= 0.48;   //Offset for LM62 temp sensor.
        dsky.motor_ctrl_temp = avgMTemp / 0.0156;    //Convert to Degrees C
        avgMTemp = 0;       //Clear average.

        //Snowman's temperature.
        //avgSTemp /= x;      //Sample average.
        //avgSTemp /= 65535;  //Convert to unsigned fractional
        //avgSTemp *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        avgSTemp /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        avgSTemp -= 0.48;   //Offset for LM62 temp sensor.
        dsky.my_temp = avgSTemp / 0.0156;    //Convert to Degrees C
        avgSTemp = 0;       //Clear average.
        
        //Copy board temp to battery temp and controller temp for now.
        dsky.motor_ctrl_temp = dsky.my_temp;
        
        //Reset analog average count.
        analog_avg_cnt = 0;

        //Do a battery check after each valid sample.
        //Check to make sure the battery and other systems are within safe operating conditions.
        //Shutdown and log the reason why if they aren't safe.
        if(STINGbits.adc_sample_burn && !STINGbits.fault_shutdown) explody_preventy_check();
        
        //Check to see if the system is ready to run.
        //If there is a fault, keep it from running.
        if(!STINGbits.deep_sleep && !STINGbits.fault_shutdown){
            //do heater calibration 
            if(vars.heat_cal_stage != disabled)heater_calibration();
            //Do power regulation and heater control.
            if((vars.heat_cal_stage > calibrating || !vars.heat_cal_stage) && CONDbits.main_power && first_cal == fCalReady){
                outputReg();    //Output regulation routine
                chargeReg();    //Charge input regulation routine
                //Check for fault shutdown.
                //If there was a fault, shut everything down as fast as possible.
                //This seems redundant, but it isn't. 
                //Shuts down a detected fault just after the regulation routine.
                if(STINGbits.fault_shutdown)io_off();
                else {
                    //Set the PWM output to what the variables are during normal operation.
                    PWMCON1bits.PEN3L = 1;  //Set PWM3 Low side to PWM output.
                    heatPWM = heat_power;               //set heater control
                    chrgPWM = charge_power;             //set charge control
                    outPWM = output_power;             //set output control
                }
            }
            else if (!CONDbits.main_power)io_off();
        }
        else io_off();
        
        //ADC sample burn check. Only burn once when main power is on.
        if (!CONDbits.main_power && STINGbits.adc_sample_burn){
            ADCON1bits.ADON = 0;    // turn ADC off to save power.
            STINGbits.adc_sample_burn = no;      //Burn the first ADC sample on every power up of ADC.
        }
        else STINGbits.adc_sample_burn = yes;      //We have burned the first set.
    }
    else {
        //Force use of all 0's if we haven't burned the first ADC sample after a startup.
        //The ADC takes a moment to get correct values but it still sends the IRQs anyways.
        if (STINGbits.adc_sample_burn){
            avgVolt += adcVoltage; //Voltage
            avgCurnt += adcCurrent; //Current
            avgBTemp += adcBTemp; //Batt Temp
            avgMTemp += adcMTemp; //Motor Temp
            avgSTemp += adcSTemp; //Self Temp
            //Check for sane values.
            analog_sanity();
        }
        else{
            //Burn it to the ground.
            avgVolt = 0;
            avgCurnt = 0;
            avgBTemp = 0;
            avgMTemp = 0;
            avgSTemp = 0;
        }
        analog_avg_cnt++;
    } 
    //End IRQ
    IFS0bits.ADIF = 0;
}

/* Heartbeat IRQ, Once every Second. Lots of stuff goes on here. */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void){
    CPUact = on;
    //Check for receive buffer overflow.
    if(U1STAbits.OERR){
        fault_log(0x2D);
        char garbage;
        //Flush the buffer.
        while(U1STAbits.URXDA)garbage = U1RXREG;
        U1STAbits.OERR = 0; //Clear the fault bit so that receiving can continue.
    }
    if(U2STAbits.OERR){
        fault_log(0x2E);
        char garbage;
        //Flush the buffer.
        while(U2STAbits.URXDA)garbage = U2RXREG;
        U2STAbits.OERR = 0; //Clear the fault bit so that receiving can continue.
    }
    ADCON1bits.ADON = on;    // turn ADC on to get a sample.
    //Get open circuit voltage percentage if current is less than 0.1 amps
    volt_percent();
    //Calculate limit currents based on temperature.
    temperatureCalc();
    //Initial startup sequence and calibration.
    initialCal();
    //Main power check
    main_power_check();
    //Charger plug-in check.
    chargeDetect();
    /* Power off TIMER stuff. Do this to save power.
     * This is so that this system doesn't drain your 1000wh battery over the 
     * course of a couple weeks while being unplugged from a charger. 
     * The dsPIC30F3011 is a power hog even in Idle and Sleep modes.
     * For future people, KEEP USING IRQs FOR STUFF!!! Don't make the CPU wait
     * for anything! The dsPIC30F3011 is an impatient hog and will consume all
     * your electrons and burn your lunch and house down from the heat that it generates! */
    if(CONDbits.main_power){
        PowerOffTimer = sets.PowerOffAfter;      //reset the timer when main_power is on.
        PowerOffTimerSec = 59;                   //60 seconds.
    }
    else{
        //When main_power is off count down in minutes.
        if(PowerOffTimerSec <= 0){
            PowerOffTimerSec = 59;
            if(PowerOffTimer <= 0) STINGbits.deep_sleep = 1; //set deep_sleep to 1. Power off the system to save power after all IRQ's are finished.
            else PowerOffTimer--;
        }
        else PowerOffTimerSec--;
    }
    /***************************************************************************/
    //Over current shutdown timer stuff.
    if(oc_shutdown_timer > 0 && dischr_current < sets.over_current_shutdown) oc_shutdown_timer--;
    // Check for charger disconnect.
    if(!chrgSwitch){
        chrgLight = off;  //charger light off.
        charge_power = 0;
        chrg_check = 0;
    }
    
    //Fan control
    if(CONDbits.main_power && (dsky.battery_temp > sets.batt_fan_start 
    || dsky.my_temp > sets.ctrlr_fan_start || dsky.motor_ctrl_temp > sets.ctrlr_fan_start))fanRelay = 1;
    else if(!CONDbits.main_power || (dsky.battery_temp < (sets.batt_fan_start - 5) 
    && dsky.my_temp < (sets.ctrlr_fan_start - 5) && dsky.motor_ctrl_temp < (sets.ctrlr_fan_start - 5)))fanRelay = 0;

    //Clear fault_shutdown if all power modes are turned off.
    if(!CONDbits.pwr_detect){
        shutdown_timer = 1;     //Acts like a resettable circuit breaker.
        STINGbits.fault_shutdown = no;
    }
    
    //Blink Check Light if any faults are logged and any power modes are on regardless of what fault_shutdown says.
    if(CONDbits.pwr_detect){
        if(CONDbits.error_blink){
            CONDbits.error_blink = off;
            errLight = off;
        }
        else{
            CONDbits.error_blink = on;    //Used for blinking stuff on displays.
            if(vars.fault_count) errLight = on;
        }
    }
    else errLight = off;

    //Get the absolute value of battery_usage and store it in absolute_battery_usage.
    vars.absolute_battery_usage = absFloat(vars.battery_usage);
    //Calculate the max capacity of the battery once the battery has been fully charged and fully discharged.
    if(voltage_percentage > 99 && power_session != 4){
        //battery_capacity = absolute_battery_usage;
        vars.battery_remaining = vars.battery_capacity;
        power_session = 4;
        vars.battery_usage = 0;  //reset battery usage session.
    }
    //At about 82% voltage the battery is at about 50% actual capacity for lithium ion.
    else if(voltage_percentage < 83 && voltage_percentage > 81 && (power_session == 4 || power_session == 0)){
        vars.battery_capacity = vars.absolute_battery_usage;  //Calculate the max capacity of the battery after a half discharge.
        vars.battery_capacity *= 2;
        power_session = 2;
    }
    else if(voltage_percentage < 1 && power_session != 0){
        vars.battery_capacity = vars.absolute_battery_usage;  //Calculate the max capacity of the battery after a full discharge.
        vars.battery_remaining = 0;  // Set ah remaining to 0 when less than 2% voltage.
        power_session = 0;
        vars.battery_usage = 0;  //reset battery usage session.
    }
    //Don't let battery_remaining go below 0;
    //This should never happen in normal conditions. This is just a catch.
    if(vars.battery_remaining < 0) vars.battery_remaining = 0;
    //*******************************************
    //Don't let battery_remaining go above battery capacity.
    //Go slightly above. Just slightly.
    if(vars.battery_remaining > vars.battery_capacity + 0.005) vars.battery_remaining = vars.battery_capacity + 0.005;
    //Don't let 'battery_remaining' go above the partial charge percentage when partial charging.
    //To Do: This isn't exactly right because, for example, ~90% Voltage != ~90% total capacity!!! 
    //It's only a few % off so for now it's okay. Will implement real voltage curve calculation later.
    if(dsky.battery_voltage < (sets.partial_charge * sets.battery_rated_voltage) 
    && p_charge && vars.battery_remaining > (vars.battery_capacity * sets.partial_charge))
        vars.battery_remaining = (vars.battery_capacity * sets.partial_charge);
    //**************************************************
    //Circuit draw compensation.
    //Heart beat draws power even when the system is off. This logs that current draw.
    if(!CONDbits.main_power) vars.battery_remaining -= (sets.circuit_draw * 0.0002777);
    //Calculate battery %
    dsky.chrg_percent = ((vars.battery_remaining / vars.battery_capacity) * 100);
/****************************************/
    /* End the IRQ. */
	IFS0bits.T1IF = 0;
}

/* 0.125 second IRQ */
//Used for some critical math timing operations. Cycles through every 1/8 sec.
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void){
    CPUact = on;
    dsky.watts = dsky.battery_crnt_average * dsky.battery_vltg_average;
    //Relay On Timers. Wait a little bit after turning on the relays before trying to regulate.
    if(chrg_rly_timer > 0 && chrg_rly_timer != 3)
        chrg_rly_timer--;
    if(contact_rly_timer > 0 && contact_rly_timer != 3)
        contact_rly_timer--;
    if(heat_rly_timer > 0 && heat_rly_timer != 3)
        heat_rly_timer--;
    
    //Get average voltage and current.
    if(avg_cnt >= 8){
        bt_crnt_avg_temp /= 8;
        dsky.battery_crnt_average = bt_crnt_avg_temp;
        bt_vltg_avg_temp /= 8;
        dsky.battery_vltg_average = bt_vltg_avg_temp;
        bt_vltg_avg_temp = 0;
        bt_crnt_avg_temp = 0;
        avg_cnt = 0;
        current_cal();
    }
    else{
        bt_crnt_avg_temp += dsky.battery_current;
        bt_vltg_avg_temp += dsky.battery_voltage;
        avg_cnt++;
    }
    //*************************
    //Get peak power output.
    float power = absFloat(dsky.battery_crnt_average * dsky.battery_vltg_average);
    if (power > dsky.peak_power){
        dsky.peak_power = power;
        dsky.peak_pwr_vlts = dsky.battery_vltg_average;
        dsky.peak_pwr_crnt = dsky.battery_crnt_average;
    }
    //*****************************
    //Calculate and Log AH usage for battery remaining and total usage.
    //Do not track current usage if below +-0.002 amps due to error in current sensing. We need a lower pass filter then what we already have.
    if(absFloat(dsky.battery_current) > 0.002){
        //Calculate how much current has gone in/out of the battery.
        vars.battery_usage += (calc_125 * dsky.battery_current);
        vars.battery_remaining += (calc_125 * dsky.battery_current);
    }
    //*************************************************************
    /****************************************/
    /* End the IRQ. */
	IFS0bits.T2IF = 0;   
}
/****************/
/* END IRQs     */
/****************/   

#endif