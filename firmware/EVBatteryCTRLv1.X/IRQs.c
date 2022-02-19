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

#ifndef IRQS_C
#define IRQS_C

#include <p30f3011.h>
#include "IRQs.h"
#include "subs.h"
#include "DataIO.h"
#include "Init.h"
#include "display.h"
#include "eeprom.h"
#include "checksum.h"

/*****************/
/* IRQs go here. */
/*****************/

/* Charge Detect IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt (void){
    PORTBbits.RB6 = 1;
    //reset peak power when we plug in a charger.
    peak_power = 0;
    //Check for partial charge status to see if we need to do a full charge to ballance the cells.
    if(sets.partial_charge > 1){
        TMR4 = 0;   //Reset timer 4 to prevent a check between writes.
        sets.partial_charge = 1;
        ram_chksum_update();     //Update the checksum after a write.
        fault_log(0x1C);            // Log Partial charge was set higher than 100% event.
    }
    //If parial_charge is set to 0% then we disable and charge the battery up to full every time.
    if(sets.partial_charge == 0){
        chrg_voltage = sets.battery_rated_voltage;
        p_charge = 0;
    }
    //Do a full charge every 10 cycles so that we can ballance the cells.
    if(power_plugged != 1 && vars.partial_chrg_cnt < 10){
        vars.partial_chrg_cnt++;
        p_charge = 1;
        chrg_voltage = ((sets.battery_rated_voltage - sets.dischrg_voltage) * sets.partial_charge) + sets.dischrg_voltage;
    }
    else if(power_plugged != 1 && vars.partial_chrg_cnt >= 10){
        vars.partial_chrg_cnt = 0;
        p_charge = 0;
        chrg_voltage = sets.battery_rated_voltage;
    }
    /* If heat_cal is not running, is not disabled, and key switch/cmd_power is off
     * run heater calibration check when charger is plugged in.
     */
    if(heat_cal_stage != 2 && PORTFbits.RF1 == 1 && cmd_power == 0 && power_plugged == 0 && fault_shutdown == 0){
        curnt_cal_stage = 1;
        //Check for disabled heater setting.
        if(heat_cal_stage != 5)
            heat_cal_stage = 1;
        //soft_power = 1;
        power_plugged = 1;
    }
    
    //Reset battery usage session when charger is plugged in and power is turned off.
    if(PORTFbits.RF1 == 1 && cmd_power == 0){
        vars.battery_usage = 0;      //Reset battery usage session.
        vars.battery_usage_smll = 0;
    }
    charger_detected = 1;
    IFS0bits.INT0IF = 0;
}

/* Wheel Rotate and low priority IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt (void){
    PORTBbits.RB6 = 1;
    wheelTime = TMR3;
    TMR3 = 0;
    if (wheelSpin)
        speed = 3600 * (sets.travel_dist / (wheelTime / 65535)); //This gives us KM per hour.
    wheelSpin = 1;
    IFS1bits.INT1IF = 0;
}
/* Wheel Rotate Timer 3 IRQ */
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void){
    PORTBbits.RB6 = 1;
    speed = 0;
    wheelSpin = 0;
    //End IRQ
    IFS0bits.T3IF = 0;
}

/* Non-critical systems. Timer 4 IRQ */
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt (void){
    PORTBbits.RB6 = 1;
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

/* Analog Input IRQ */
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void){
    PORTBbits.RB6 = 1;
    
//BUFFER 0 is Battery Voltage
//BUFFER 1 is Battery Current
//BUFFER 2 is Battery Temperature
//BUFFER 3 is Motor Controller Temperature
//BUFFER 4 is My Temperature
    /* I have left commented out code in this section for showing the process
     * of converting analog inputs into voltages, currents, and temps.
     */
    //Get 8 samples for averaging.
    if (analog_avg_cnt >= 8){
        //Battery current.
        adctemp1 /= 8;      //Sample average.
        adctemp1 -= 32768;    //Set zero point.
        adctemp1 /= 32768;    //Convert to signed fractional. -1 to 1
        adctemp1 *= 2.5;      //Convert to +-2.5'volts'. It's still a 0 - 5 volt signal on the analog input. The zero point is at 2.5v
        battery_current = (adctemp1 / 0.04) + current_compensate; //Offset for ACS780LLRTR-050B-T Current Sensor.
        currentCheck();     //Check for over current condition.
        adctemp1 = 0;       //Clear average.

        //Battery voltage.
        //adctemp0 /= x;      //Sample average.
        //adctemp0 /= 65535;  //Convert to unsigned fractional
        //adctemp0 *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        adctemp0 /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        battery_voltage = (adctemp0 / vltg_dvid) + sets.bt_vlt_adjst;    //Use resistor divider values to covert to actual voltage.
        adctemp0 = 0;       //Clear average.

        //Battery temperature.
        //adctemp2 /= x;      //Sample average.
        //adctemp2 /= 65535;  //Convert to unsigned fractional
        //adctemp2 *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        adctemp2 /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        adctemp2 -= 0.48;   //Offset for LM62 temp sensor.
        battery_temp = adctemp2 / 0.0156;    //Convert to Degrees C
        adctemp2 = 0;       //Clear average.

        //motor controller temperature.
        //adctemp3 /= x;      //Sample average.
        //adctemp3 /= 65535;  //Convert to unsigned fractional
        //adctemp3 *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        adctemp3 /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        adctemp3 -= 0.48;   //Offset for LM62 temp sensor.
        motor_ctrl_temp = adctemp3 / 0.0156;    //Convert to Degrees C
        adctemp3 = 0;       //Clear average.

        //Snowman's temperature.
        //adctemp4 /= x;      //Sample average.
        //adctemp4 /= 65535;  //Convert to unsigned fractional
        //adctemp4 *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        adctemp4 /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        adctemp4 -= 0.48;   //Offset for LM62 temp sensor.
        my_temp = adctemp4 / 0.0156;    //Convert to Degrees C
        adctemp4 = 0;       //Clear average.
        
        //Copy board temp to battery temp and controller temp for now.
        motor_ctrl_temp = my_temp;
//        battery_temp = my_temp;
        
        //Reset analog average count.
        analog_avg_cnt = 0;

        //Do a battery check after each valid sample.
        //Check to make sure the battery and other systems are within safe operating conditions.
        //Shutdown and log the reason why if they aren't safe.
        if((b_safe != 0x55FF) && (adc_sample_burn == 1) && (fault_shutdown == 0)){
            explody_preventy_check();
        }
        
        //Check to see if the system is ready to run.
        if(deep_sleep == 0 && fault_shutdown == 0){
            //Check for heater calibration event.
            heater_calibration();
            //Do power regulation and heater control.
            if((heat_cal_stage >= 3 || heat_cal_stage == 0) && main_power == 1 && curnt_cal_stage == 5)
                regulate();
            else if (main_power == 0){
                io_off();
            }
        }
        else{
            io_off();
        }
        
        //ADC sample burn check. Only burn once when main power is on.
        if (main_power == 0 && adc_sample_burn == 1){
            ADCON1bits.ADON = 0;    // turn ADC off to save power.
            adc_sample_burn = 0;      //Burn the first ADC sample on every power up of ADC.
        }
        else {
            adc_sample_burn = 1;      //We have burned the first set.
        }
    }
    else {
        //Force use of all 0's if we haven't burned the first ADC sample after a startup.
        //The ADC takes a moment to get correct values but it still sends the IRQs anyways.
        if (adc_sample_burn == 1){
            adctemp0 += ADCBUF0;
            adctemp1 += ADCBUF1;
            adctemp2 += ADCBUF2;
            adctemp3 += ADCBUF3;
            adctemp4 += ADCBUF4;
            //Check for sane values.
            if(b_safe != 0x55FF){
                analog_sanity();
            }
        }
        else{
            //Burn it to the ground.
            adctemp0 = 0;
            adctemp1 = 0;
            adctemp2 = 0;
            adctemp3 = 0;
            adctemp4 = 0;
        }
        analog_avg_cnt++;
    } 
    //End IRQ
    IFS0bits.ADIF = 0;
}

/* Data and Command input and processing IRQ for Port 1 */
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void){
    PORTBbits.RB6 = 1;
    Command_Interp(PORT1);
/****************************************/
    /* End the IRQ. */
    IFS0bits.U1RXIF = 0;
}

/* Data and Command input and processing IRQ for Port 2. */
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt (void){
    PORTBbits.RB6 = 1;
    Command_Interp(PORT2);
/****************************************/
    /* End the IRQ. */
    IFS1bits.U2RXIF = 0;
}

/* Output IRQ for Port 1 */
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt (void){
    PORTBbits.RB6 = 1;
    //Dispatch the buffer to the little 4 word Serial Port buffer as it empties.
    while(U1STAbits.UTXBF == 0 && (Buff_index[PORT1] < Buff_count[PORT1])){
        U1TXREG = Buffer[PORT1][Buff_index[PORT1]];
        Buff_index[PORT1]++;
    }
    //Reset the buffer index and count when done sending.
    if (Buff_index[PORT1] >= Buff_count[PORT1]){
        Buff_index[PORT1] = 0;
        Buff_count[PORT1] = 0;
        portBSY[PORT1] = 0;//Inhibits writing to buffer while the serial port is transmitting buffer.
    }
    /****************************************/
    /* End the IRQ. */
    IFS0bits.U1TXIF = 0;
}

/* Output IRQ for Port 2 */
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt (void){
    PORTBbits.RB6 = 1;
    //Dispatch the buffer to the little 4 word Serial Port buffer as it empties.
    while(U2STAbits.UTXBF == 0 && (Buff_index[PORT2] < Buff_count[PORT2])){
        U2TXREG = Buffer[PORT2][Buff_index[PORT2]];
        Buff_index[PORT2]++;
    }
    //Reset the buffer index and count when done sending.
    if (Buff_index[PORT2] >= Buff_count[PORT2]){
        Buff_index[PORT2] = 0;
        Buff_count[PORT2] = 0;
        portBSY[PORT2] = 0;//Inhibits writing to buffer while the serial port is transmitting buffer.
    }
    /****************************************/
    /* End the IRQ. */
    IFS1bits.U2TXIF = 0;
}

/* Heartbeat IRQ, Once every Second. Lots of stuff goes on here. */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void){
    PORTBbits.RB6 = 1;
    ADCON1bits.ADON = 1;    // turn ADC on to get a sample.
    
    /* Power off TIMER stuff. Do this to save power.
     * This is so that this system doesn't drain your 1000wh battery over the 
     * course of a couple weeks while being unplugged from a charger. 
     * The dsPIC30F3011 is a power hog even in Idle and Sleep modes.
     * For future people, KEEP USING IRQs FOR STUFF!!! Don't make the CPU wait
     * for anything! The dsPIC30F3011 is an impatient hog and will consume all
     * your electrons and burn your lunch and house down from the heat that it generates!
     */

    if(main_power){
        PowerOffTimer = sets.PowerOffAfter;      //reset the timer when main_power is on.
        PowerOffTimerSec = 59;              //60 seconds.
        PORTDbits.RD2 = 1; //Enable Keep Alive signal. System keeps itself on while main_power is enabled.
    }
    else{
        //When main_power is off count down in minutes.
        if(PowerOffTimerSec <= 0){
            PowerOffTimerSec = 59;
            if(PowerOffTimer <= 0){
                deep_sleep = 1; //set deep_sleep to 1. Power off the system to save power after all IRQ's are finished.
            }
            else{
                PowerOffTimer--;
            }
        }
        else{
            PowerOffTimerSec--;
        }
    }
    /***************************************************************************/
    //Over current shutdown timer stuff.
    if(oc_shutdown_timer > 0 && dischr_current < sets.over_current_shutdown){
        oc_shutdown_timer--;
    }
    // Check for charger disconnect.
    if(PORTEbits.RE8 == 0){
        PORTBbits.RB4 = 0;          //charger light off.
        charge_power = 0;
        chrg_check = 0;
    }

    //Fan control
    if(main_power && (battery_temp > sets.batt_fan_start || my_temp > sets.ctrlr_fan_start || motor_ctrl_temp > sets.ctrlr_fan_start)){
        PORTFbits.RF0 = 1;
    }
    else if(main_power == 0 || (battery_temp < (sets.batt_fan_start - 5) && my_temp < (sets.ctrlr_fan_start - 5) && motor_ctrl_temp < (sets.ctrlr_fan_start - 5))){
        PORTFbits.RF0 = 0;
    }
    
    //Main power check
    main_power_check();

    //Clear fault_shutdown if all power modes are turned off.
    if(PORTEbits.RE8 == 0 && PORTFbits.RF1 == 1 && soft_power == 0 && cmd_power == 0 && shutdown_timer == 0){
        shutdown_timer = 1;     //Acts like a resettable circuit breaker.
    }
    
    //Blink Check Light if any faults are logged and any power modes are on regardless of what fault_shutdown says.
    if(pwr_detect){
        if(error_blink == 1){
            error_blink = 0;
            PORTBbits.RB5 = 0;
        }
        else{
            error_blink = 1;    //Used for blinking stuff on displays.
            if(vars.fault_count != 0){
                PORTBbits.RB5 = 1;
            }
        }
    }
    else{
        PORTBbits.RB5 = 0;
    }
    
    
    //Initial Power up Sequence. VERY IMPORTANT
    //Calibrate the current sense and calculate remaining capacity on first power up based on voltage percentage and rated capacity of battery.
    if(first_cal == 0 && curnt_cal_stage == 0){
        curnt_cal_stage = 1;
        soft_power = 1;
        first_cal = 1;
    }
    else if(first_cal < 8){
        first_cal++;            //delay, wait about 8 seconds for other services to complete.
    }
    else if (first_cal == 8 && got_open_voltage == 1){
        //Check to see if we have valid data in EEPROM that we can start with.
        if (eeprom_read((cfg_space) + 1) == 0x7654){
            //Calculate how much power was used while the power was off. This is not exact, but should be close enough.
            int power_diff = (vars.battery_capacity * ((vars.voltage_percentage_old - voltage_percentage) / 100));
            vars.battery_remaining -= power_diff;        //Subtract the power difference
            vars.absolute_battery_usage -= power_diff;
            vars.battery_usage -= power_diff;
        }
        else{
            vars.battery_capacity = sets.amp_hour_rating; //Just use the amp hour rating on first start.
            vars.battery_remaining = sets.amp_hour_rating * (voltage_percentage / 100);   //Rough estimation of how much power is left.
        }
        first_cal = 9;      //Signal that we are done with power up.
        soft_power = 0;
        //We have our starting variables, now check for a charger manually this one time. 
        //Do this here because plugging in a charger on a power off state might cause the charger IRQ to do things
        //while the rest of the system isn't ready if it even triggers it at all.
        //Enable Charger IRQ.
        IFS0bits.INT0IF = 0;    //Make sure we don't trigger an IRQ just yet.
        __asm__ volatile ("DISI #0x3FFF");
        IEC0bits.INT0IE = 1;    //Charge Detect IRQ Enabled.
        //Check for charger input on power up.
        if(PORTEbits.RE8 == 1){
            //Check to see if we have already ran the IRQ. If we have, no need to manually trigger it again.
            if (charger_detected == 0){
                IFS0bits.INT0IF = 1;    //Force a charger plugged IRQ.
            }
        }
    }
    //********************************************************************//
    //Get open circuit voltage percentage if current is less than 0.1
    volt_percent();
    
    //Get the absolute value of battery_usage and store it in absolute_battery_usage.
    if(vars.battery_usage < 0){
        vars.absolute_battery_usage = vars.battery_usage * -1;
    }
    else{
        vars.absolute_battery_usage = vars.battery_usage;
    }
    
    //Calculate the max capacity of the battery once the battery has been fully charged and fully discharged.
    if(voltage_percentage > 99 && power_session != 4){
        //battery_capacity = absolute_battery_usage;
        vars.battery_remaining = vars.battery_capacity;
        //if(power_session != 4){
            power_session = 4;
            vars.battery_usage = 0;  //reset battery usage session.
            vars.battery_usage_smll = 0;
        //}
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
        vars.battery_remaining_smll = 0;
        power_session = 0;
        vars.battery_usage = 0;  //reset battery usage session.
        vars.battery_usage_smll = 0;
    }

    //Don't let battery_remaining go below 0;
    //This should never happen in normal conditions. This is just a catch.
    if(vars.battery_remaining < 0){
        vars.battery_remaining = 0;
        vars.battery_remaining_smll = 0;
    }
    //*******************************************
    //Don't let battery_remaining go above battery capacity, ever.
    if(vars.battery_remaining > vars.battery_capacity + 0.005){
        vars.battery_remaining = vars.battery_capacity + 0.005;        //Go slightly above. Just slightly.
    }
    //Don't let battery remaining go above the partial charge percentage when partial charging.
    //To Do: This isn't exactly right because 90% Voltage != 90% total capacity!!! It's only a few % off so for now it's okay.
    if(battery_voltage < (sets.partial_charge * sets.battery_rated_voltage) && p_charge && vars.battery_remaining > (vars.battery_capacity * sets.partial_charge)){
        vars.battery_remaining = (vars.battery_capacity * sets.partial_charge);
    }
    //**************************************************
    //Circuit draw compensation.
    //Heart beat draws power even when the system is off. This logs that current draw.
    if(main_power == 0){
        vars.battery_remaining -= (sets.circuit_draw * 0.0002777);
    }
//**************************************************
//battery_usage = 0;      //Reset battery usage session.
/****************************************/
    /* End the IRQ. */
	IFS0bits.T1IF = 0;
}

/* 0.125 second IRQ */
//Used for some critical math timing operations. Cycles through every 1/8 sec.
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void){
    PORTBbits.RB6 = 1;
    float power = 0;
    
    //Relay On Timers. Wait a little bit after turning on the relays before trying to regulate.
    if(chrg_rly_timer > 0 && chrg_rly_timer != 3)
        chrg_rly_timer--;
    if(contact_rly_timer > 0 && contact_rly_timer != 3)
        contact_rly_timer--;
    if(heat_rly_timer > 0 && heat_rly_timer != 3)
        heat_rly_timer--;
    
    //Get average current.
    if(crnt_avg_cnt >= 8){
        bt_crnt_avg_temp /= 8;
        battery_crnt_average = bt_crnt_avg_temp;
        bt_crnt_avg_temp = 0;
        crnt_avg_cnt = 0;
        current_cal();
    }
    else{
        bt_crnt_avg_temp += battery_current;
        crnt_avg_cnt++;
    }
    //************************
    ////Get average voltage.
    if(vltg_avg_cnt >= 8){
        bt_vltg_avg_temp /= 8;
        battery_vltg_average = bt_vltg_avg_temp;
        bt_vltg_avg_temp = 0;
        vltg_avg_cnt = 0;
    }
    else{
        bt_vltg_avg_temp += battery_voltage;
        vltg_avg_cnt++;
    }
    //*************************
    //Get peak power output.
    power = battery_crnt_average * battery_vltg_average;
    if(power < 0){
        power *= -1;
    }
    if (power > peak_power){
        peak_power = power;
        peak_pwr_vlts = battery_vltg_average;
        peak_pwr_crnt = battery_crnt_average;
    }
    //*****************************
    //Calculate and Log AH usage for battery remaining and total usage.
    //Do not track current usage if below +-0.002 amps due to error in current sensing. We need a lower pass filter then what we already have.
    if(battery_current > 0.002 || battery_current < -0.002){
        //Calculate how much current has gone in/out of the battery.
        vars.battery_usage_smll = vars.battery_usage_smll + (calc_125 * battery_current); 
        vars.battery_remaining_smll = vars.battery_remaining_smll + (calc_125 * battery_current);
        if(vars.battery_usage_smll > 0.001){
            vars.battery_usage_smll -= 0.001;
            vars.battery_usage += 0.001;
        }
        else if(vars.battery_usage_smll < -0.001){
            vars.battery_usage_smll += 0.001;
            vars.battery_usage -= 0.001;
        }
        if(vars.battery_remaining_smll > 0.001){
            vars.battery_remaining_smll -= 0.001;
            vars.battery_remaining += 0.001;
        }
        else if(vars.battery_remaining_smll < -0.001){
            vars.battery_remaining_smll += 0.001;
            vars.battery_remaining -= 0.001;
        }
    }
    //*************************************************************
/****************************************/
    /* End the IRQ. */
	IFS0bits.T2IF = 0;   
}

/* CPU TRAPS, log erros here and shut down if needed.*/
void __attribute__((interrupt, no_auto_psv)) _FLTAInterrupt (void){
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    fault_log(0x0C);        //PWM fault. External.
    IFS2bits.FLTAIF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail (void){
    //Check the PLL lock bit as fast as possible before anything else because it doesn't stay set for long.
    if(OSCCONbits.LOCK == 0){
        if(osc_fail_event == 0){
            fault_log(0x12);
        }
    }
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    if(osc_fail_event == 0){
        fault_log(0x0D);
    }
    osc_fail_event = 1;         //Only log it once per reset.
    INTCON1bits.OSCFAIL = 0;
}
void __attribute__((interrupt, no_auto_psv)) _AddressError (void){
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    //send_string(0x03, "Address Error Event.");
    fault_log(0x0E);
    INTCON1bits.ADDRERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _StackError (void){
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    //send_string(0x03, "Stack Error Event.");
    fault_log(0x0F);
    INTCON1bits.STKERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _MathError (void){
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    //send_string(0x03, "Math Error Event.");
    fault_log(0x10);
    INTCON1bits.MATHERR = 0;
}
void __attribute__((interrupt, no_auto_psv)) _ReservedTrap7 (void){
    PORTBbits.RB6 = 1;
    io_off();
    fault_shutdown = 1;
    fault_log(0x11);
    //INTCON1bits.DMACERR = 0;
}
/****************/
/* END IRQs     */
/****************/   

#endif