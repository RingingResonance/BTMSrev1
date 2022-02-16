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

#include "subs.h"
#include "Init.h"
/* Fun fact, you can comment out these includes and it still compiles even though they are needed!
 * Probably because they are included in main.c IDK mplab is weird.
 * TODO: This file needs to be split up into it's seperate systems.
 */

//Save settings to EEPROM
int set_save(int offset){
    int x = 0;
    while (x <= (cfg_space / 2)){
        eeprom_erase(x + offset);
        while (NVMCONbits.WR){}
        eeprom_write(x + offset, config_start[x]);
        while (NVMCONbits.WR){}
        x++;
    }
    return x;
}
//Read settings from EEPROM
void read_save(int offset){
    int x = 0;
    while (x <= (cfg_space / 2)){
        config_start[x] = eeprom_read(x + offset);
        x++;
    }
}
//Save vars to EEPROM
int var_save(int offset){
    int x = 0;
    while (x <= (vr_space / 2)){
        eeprom_erase(x + offset);
        while (NVMCONbits.WR){}
        if (x == 0){
            eeprom_write(x + offset,0x7654);
        }
        else{
            eeprom_write(x + offset,var_start[x]);
        }
        while (NVMCONbits.WR){}
        x++;
    }
    return x;
}
//Read vars from EEPROM
void read_romvars(int offset){
    int x = 0;
    while (x <= (vr_space / 2)){
        var_start[x] = eeprom_read(x + offset);
        x++;
    }
}

//Calculate Space required for saving to eeprom.
//warning: assignment makes integer from pointer without a cast ya ya I know, but it works like this so don't touch it.
int cfig_space(void){
    int data_space1 = 0;
    int data_space2 = 0;
    data_space1 = &config_start[0];
    data_space2 = &config_end;
    return data_space2 - data_space1;
}
int var_space(void){
    int data_space1 = 0;
    int data_space2 = 0;
    data_space1 = &var_start[0];
    data_space2 = &var_end;
    return data_space2 - data_space1;
}

//System power off for power saving.
void power_off(void){
    voltage_percentage_old = voltage_percentage;    //Save a copy of voltage percentage before we shut down.
    // Enough time should have passed by now that the open circuit voltage should be stabilized enough to get an accurate reading.
    var_save((cfg_space / 2) + 1);      //Save variables before power off.
    PORTDbits.RD2 = 0; //Disable Keep Alive signal.
}

//Warm start and reset check.
void first_check(void){
    static int reset_chk;              //Do not initialize this var. Needs to stay the same on a reset.
    while(start_timer < 0xFFFE){
        start_timer++;          //Wait here for the power supply to stabilize, POR isn't long enough sometimes.
    }
    //Do not check why we reset on initial power up. No reason to. We don't want a reset error on first power up.
    if(reset_chk == 0xAA55){
        reset_check();              //Check for reset events.
    }
    reset_chk = 0xAA55;         //Warm start.
}
//Main Power Check.
void main_power_check(void){
        /* Check for charger, key, or software power up. */
        if((PORTEbits.RE8 == 1 || PORTFbits.RF1 == 0 || soft_power == 1 || cmd_power == 1 || power_plugged == 1)){
            pwr_detect = 1;         //Used for blinking error light when in a fault shutdown.
            
            if(diag_state){
                diag_count += 5;
                diag_state = 0;
            }
            
            if(shutdown_timer == 1){
                fault_shutdown = 0;
                osc_fail_event = 0;
                shutdown_timer = 0;
            }
            
            if(fault_shutdown == 0){
                main_power = 1;     //Main power is ON.
                //Reinit if we haven't already.
                if (init_done == 0){
                    Init();
                }
            }
            else{
                io_off();
                main_power = 0;
                heat_power = 0;               //set heat power
                charge_power = 0;             //set charge control
                output_power = 0;             //set output control
                //Deinit if we haven't already.
                if (lw_pwr == 0){
                    low_power_mode();   //Go into idle mode with heart beat running.
                }
            }
        }
        else{
            diag_state = 1;
            io_off();
            pwr_detect = 0;     //Used for blinking error light when in a fault shutdown.
            main_power = 0;     //Main power is OFF.
            heat_power = 0;               //set heat power
            charge_power = 0;             //set charge control
            output_power = 0;             //set output control
            
            //Deinit if we haven't already.
            if (lw_pwr == 0){
                low_power_mode();   //Go into idle mode with heart beat running.
            }
        }
}

void analog_sanity(void){
    //Battery Voltage
    if(ADCBUF0 >= 0xFFFE){
        fault_log(0x1D);
        general_shutdown();     //If we can't get battery voltage then what's the point of living?
    }
    if(ADCBUF0 <= 0x0001){
        fault_log(0x1E);
        general_shutdown();     //If we can't get battery voltage then what's the point of living?
    }
    //Battery Current
    if(ADCBUF1 >= 0xFFFE){
        fault_log(0x1F);
        general_shutdown();
    }
    if(ADCBUF1 <= 0x0001){
        fault_log(0x20);
        general_shutdown();
    }
    //Battery temperature.
    if(ADCBUF2 >= 0xFFFE){
        fault_log(0x21);
        heat_cal_stage = 5;     //Disable Heater if we can't get battery temperature.
    }
    if(ADCBUF2 <= 0x0001){
        fault_log(0x22);
        heat_cal_stage = 5;     //Disable Heater if we can't get battery temperature.
    }
    //motor controller temperature.
    //We can live with these temps in error so don't bother disabling or shutting anything down here. Just log the error.
    if(ADCBUF3 >= 0xFFFE){
        fault_log(0x23);
    }
    if(ADCBUF3 <= 0x0001){
        fault_log(0x24);
    }
    //Snowman's temperature.
    if(ADCBUF4 >= 0xFFFE){
        fault_log(0x25);
    }
    if(ADCBUF4 <= 0x0001){
        fault_log(0x26);
    }

}

//Check reset conditions and log them.
void reset_check(void){
    if(RCONbits.BOR){
        RCONbits.BOR = 0;
        fault_log(0x13);        //Brown Out Event.
    }
    if(RCONbits.WDTO){
        RCONbits.WDTO = 0;
        fault_log(0x14);        //WDT Reset Event.
    }
    if(RCONbits.TRAPR){
        RCONbits.TRAPR = 0;
        fault_log(0x15);        //TRAP Conflict Event.
    }
    if(RCONbits.IOPUWR){
        RCONbits.IOPUWR = 0;
        fault_log(0x16);        //Illegal opcode or uninitialized W register access Event.
    }
    if(RCONbits.EXTR){
        RCONbits.EXTR = 0;
        fault_log(0x17);        //External Reset Event.
    }
    if(RCONbits.SWR){
        RCONbits.SWR = 0;
        fault_log(0x18);        //Reset Instruction Event.
    }
    //if(reset_chk == 0xAA55){
        fault_log(0x19);        //General Reset Check. Did we start from cold? If not then log that we had a reset.
    //}
}

//Battery Percentage Calculation. This does NOT calculate the % total charge of battery, only the total voltage percentage.
void volt_percent(void){
    if (battery_voltage <= (dischrg_voltage - 2)){
        voltage_percentage = 0;
    }
    else if (battery_current < 0.05 && battery_current > -0.05){
        open_voltage = battery_voltage;
        voltage_percentage = 100 * ((open_voltage - dischrg_voltage) / (battery_rated_voltage - dischrg_voltage));
        got_open_voltage = 1;
    }
}

//Find current compensation value.
//This code dances around like spaghetti
void current_cal(void){
    float signswpd_avg_cnt = 0;
    signswpd_avg_cnt = battery_crnt_average * -1;
    //do the current cal.
    if(curnt_cal_stage == 4){
        current_compensate = (signswpd_avg_cnt - circuit_draw);
        curnt_cal_stage = 5;        //Current Cal Complete
        //send_string(NLtxtNL, "Current Cal.", PORT1);
        if(heat_cal_stage != 5){
            heat_cal_stage = 1;     //Do a heater cal after we have done current cal unless it is disabled.
        }
        soft_power = 0;
        //Done with current cal.
    }
    //Initialize current cal.
    if(curnt_cal_stage == 1){
        current_compensate = 0;
        io_off();    //Turn off all inputs and outputs.
        curnt_cal_stage = 4;
        soft_power = 1;         //Turn soft power on to run 0.125s IRQ.
    }
}

//Charge, discharge, and heater regulation.
void regulate(void){
    float dischrg_current = 0;
    float chrg_current = 0;
    double dischrg_read = 0;
    float chrg_current_read = 0;
    //Discharge current read and target calculation.
    //Copy battery values so that we don't disturb them.
    dischrg_read = battery_current;
    chrg_current_read = battery_current;
    
    //Calculate max discharge current based off battery remaining and battery temp.
    temp_dischrg_rate = 1;
    if (battery_temp > dischrg_reduce_high_temp || battery_temp < dischrg_reduce_low_temp){
        temp_dischrg_rate = 0.8;     //Decrease the current by 20% if we are too hot or too cold.
    }
    if (battery_temp < dischrg_min_temp){
        temp_dischrg_rate = 0.5;     //Decrease the current by 50% if we are too cold.
    }
    dischrg_current = (dischrg_C_rating * battery_remaining) * temp_dischrg_rate;
    
    //Don't set our max discharge current below the limp current setpoint.
    if(dischrg_current < limp_current || battery_temp < dischrg_min_temp){
        dischrg_current = limp_current;
    }
    
    //Get absolute value for battery current.
    if (dischrg_read < 0){
        dischrg_read *= -1;
    }
    else{
        dischrg_read = 0;
    }
    //Calculate max charge current based off battery temp.
    temp_dischrg_rate = 1;
    if (battery_temp > chrg_reduce_high_temp || battery_temp < chrg_reduce_low_temp){
        temp_dischrg_rate = 0.8;     //Decrease the current by 20% if we are too hot or too cold.
    }
    
    //Charge current read and target calculation.
    chrg_current = (chrg_C_rating * amp_hour_rating) * temp_dischrg_rate;
    if(battery_temp > chrg_max_temp || battery_temp < chrg_min_temp){
        chrg_current = 0;   //Inhibit charging battery if temperature is out of range.
    }
    
    //Charge current should be positive, if it is negative then set it to 0.
    if (chrg_current_read < 0){
        chrg_current_read = 0;
    }
    
    //Delay is over, start checking for charger again.
    power_plugged = 0;
    
    //RF1 is key switch and active low, RE8 is Charger Port and active high.
//// Check for Charger.
    if(PORTEbits.RE8 == 1 && first_cal == 9){
        PORTBbits.RB4 = 1;          //charger light on.
        if(fault_shutdown != 1){
            PORTFbits.RF6 = 1;          //charger relay on, but only if fault shutdown is 0
            if(chrg_rly_timer == 3)
                chrg_rly_timer = 2;         //wait two 0.125ms cycles before allowing charge regulation to start.
        }
        //Charger timeout check. If charger is plugged in but we aren't getting current then we 
        //need to shutdown and log an error code so we don't run down the battery.
        float chk_voltage = 0;
        chk_voltage = chrg_voltage - 0.5;       //Half a volt less than charge voltage.
        if(chrg_check < 20000 && chrg_current_read == 0 && PORTFbits.RF1 == 1 && battery_voltage < chk_voltage && chrg_rly_timer == 0){
            chrg_check++;
        }
        else if(chrg_check >= 20000 && PORTFbits.RF1 == 1){
            fault_log(0x1B);            //Log insufficient current from charger.
            fault_shutdown = 1;
            chrg_check = 0;
            PORTFbits.RF6 = 0;          //charger relay off. Don't waste power on a relay that is doing nothing.
            //This usually happens when we detect a charge voltage but the charge regulator isn't passing enough current.
        }
        else if(chrg_check > 0){
            chrg_check--;
        }
        
        //Run heater if needed, but don't turn it up more than what the charger can handle.
        if(chrg_current_read > 0.01 || (battery_voltage >= chrg_voltage - 0.15)){
            heat_control(chrg_target_temp);
        }
        else if(chrg_current_read == 0 && heat_power > 0){
            heat_power--;
        }
        
        //Set current to 0 if battery voltage is over the charge set voltage.
        //Do this so that we can still run the heater without charging or discharging the battery when doing a partial charge.
        float   input_current = 0;
        if((battery_voltage > chrg_voltage + 0.05)){
            zero_current = 0;
        }
        else if((battery_voltage < chrg_voltage - 0.05)){
            zero_current = 1;
        }
        //We do it like this so that float_current can be updated even when neither of the previous if statements are true.
        if(zero_current){
            input_current = chrg_current;
        }
        else{
            input_current = 0;
        }
        if(heat_cal_stage != 2 && chrg_rly_timer == 0){
            // Charge regulation routine. Clean this up, it needs to use integral math for regulation!!!
            if(((charge_power > 0) && (battery_voltage >= (battery_rated_voltage - 0.01))) || (chrg_current_read > (input_current + 0.01)))
                charge_power--;
            else if(((charge_power < 101) && (battery_voltage < battery_rated_voltage - 0.07)) || (chrg_current_read < input_current))
                charge_power++;
        }
        else
            charge_power = 0;       //Inhibit charging if we are in the middle of heater calibration.
    }
    else{
        PORTBbits.RB4 = 0;          //charger light off.
        charge_power = 0;
        PDC2 = 0;                   //Set charger output to 0 before turning off relay
        PORTFbits.RF6 = 0;          //charger relay off.
        chrg_rly_timer = 3;         //reset charge relay timer.
        chrg_check = 0;
    }

////Check for key power or command power signal, but not soft power signal.
    if((PORTFbits.RF1 == 0 || cmd_power == 1) && (first_cal == 9 || b_safe == 0x55FF)){
        if(fault_shutdown != 1){
            PORTDbits.RD1 = 1;          //contactor relay on, but only if fault shutdown is 0
            PORTBbits.RB8 = 1; //Turn on AUX 
            if(contact_rly_timer == 3)
                contact_rly_timer = 2;         //wait two 0.125ms cycles before allowing charge regulation to start.
        }
        //Run heater if needed, but don't run this sub a second time if we are getting charge power while key is on.
        //If we are getting charge power then we need to use it to warm the battery to a higher temp if needed.
        //So check charge input first.
        if(PORTEbits.RE8 == 0){
            heat_control(dischrg_target_temp);
        }
        //Discharge regulation.
        if(b_safe == 0x55FF){
            output_power = v_test;
            /*if(output_power < 95){
                output_power++; //RAMP full output power when b_safe is off. For Debugging.
            }*/
        }
        else if(contact_rly_timer == 0){
            //Integral Mode
            //High temp limiting.
            float temp_error = 0;
            temp_error = battery_temp - dischrg_max_temp;
            voltage_output += analog_smpl_time * (temp_error * temp_proportion);
            if(voltage_output > battery_rated_voltage){
                voltage_output = battery_rated_voltage;
            }
            else if(voltage_output < dischrg_voltage){
                voltage_output = dischrg_voltage;
            }
            
            //Low voltage limiting.
            float vltg_error = 0;
            vltg_error = battery_voltage - voltage_output;
            current_output += analog_smpl_time * (vltg_error * vltg_proportion);
            if(current_output > dischrg_current){
                current_output = dischrg_current;
            }
            else if(current_output < 0){
                current_output = 0;
            }
            
            //Current regulation.
            if(current_output > absolute_max_current){
                current_output = absolute_max_current;
            }
            crnt_error = (current_output - dischrg_read) * crnt_proportion;
            if(crnt_error > 10000){
                crnt_error = 10000;
            }
            else if(crnt_error < -10000){
                crnt_error = -10000;
            }
            crnt_integral = crnt_integral + (analog_smpl_time * crnt_error);
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
        PDC3 = 0; //set output control to 0 before turning off the relay
        LATE = 0;               //Insure all PORTE outputs are off.
        PWMCON1bits.PEN3L = 0;  //Set PWM3 Low side to standard output so that it can be set to 0
        crnt_integral = 0;
        contact_rly_timer = 3; //Reset contactor relay timer
        PORTBbits.RB8 = 0; //Turn off AUX 
        PORTDbits.RD1 = 0; //Turn off contactor relay
    }
    
    if(main_power == 0){
        heat_power = 0;
        PDC1 = 0;               //Heater output to 0 before we turn off the relay.
        PORTCbits.RC15 = 0;     //Heat Relay Off
        heat_rly_timer = 3;     //Reset heat relay timer
    }
    
    //Check for fault shutdown.
    if(fault_shutdown == 1){
        io_off();               //There was a fault, shut everything down as fast as possible.
    }
    else {
        //Set the PWM output to what the variables are during normal operation.
        PWMCON1bits.PEN3L = 1;  //Set PWM3 Low side to PWM output.
        PDC1 = heat_power;               //set heater control
        PDC2 = charge_power;             //set charge control
        PDC3 = output_power;             //set output control
    }
}


//Heater regulation.
void heat_control(float target_temp){
    /* Heater regulation. Ramp the heater up or down. If the battery temp is out
         * of range then the target charge or discharge current will be set to 0 and the charge 
         * regulation routine will power the heater without charging the battery.
         */
        if(heat_cal_stage == 3){
            if(battery_temp < (target_temp - 0.5) && heat_power < heat_set){
                if(heat_rly_timer == 0)
                    heat_power++;
                PORTCbits.RC15 = 1;     //Heat Relay On
                if(heat_rly_timer == 3)
                    heat_rly_timer = 2; //wait two 0.125ms cycles before allowing heat regulation to start.
            }
            else if(battery_temp > (target_temp + 0.5) && heat_power > 0){
                heat_power--;
                if(heat_power <= 0)
                    PORTCbits.RC15 = 0;     //Heat Relay Off
                    heat_rly_timer = 3;     //Reset heat relay timer
            }
        }
}


//Used to log fault codes. Simple eh? Just call it with the code you want to log.
void fault_log(int f_code){
    if (fault_count < 10){
        fault_codes[fault_count] = f_code;
        fault_count++;
    }
    else{
        fault_count = 11;       //Fault log full.
    }
}

//Check and calibrate heater to the wattage chosen by the user.
void heater_calibration(void){
        if (heat_cal_stage == 2 && main_power == 1){
            float watts = 0;
            watts = (battery_voltage * battery_current) * -1;
            if (watts < max_heat){
            PORTCbits.RC15 = 1;     //Heat Relay On
                if(heat_rly_timer == 0){
                    heat_set++;
                    PDC1 = heat_set;
                    if (heat_set > 95){
                        fault_log(0x0001);      //Log fault, heater is too small for the watts you want.
                        heat_cal_stage = 4;
                        PORTCbits.RC15 = 0;     //Heat Relay Off
                        heat_rly_timer = 3;     //Reset heat relay timer
                        heat_set = 0;
                    }
                    if (heat_set > 50 && watts < 2){
                        fault_log(0x0002);      //Log fault, no heater detected.
                        heat_cal_stage = 4;
                        PORTCbits.RC15 = 0;     //Heat Relay Off
                        heat_rly_timer = 3;     //Reset heat relay timer
                        heat_set = 0;
                    }
                    if (heat_set < 5 && watts > 10){
                        fault_log(0x0003);      //Log fault, short circuit on heater.
                        heat_cal_stage = 4;
                        PORTCbits.RC15 = 0;     //Heat Relay Off
                        heat_rly_timer = 3;     //Reset heat relay timer
                        heat_set = 0;
                    }
                }
                if(heat_rly_timer == 3)
                    heat_rly_timer = 2; //wait two 0.125ms cycles before allowing heat regulation to start.
            }
            else{
                heat_cal_stage = 3; // Heater calibration completed.
                PDC1 = 0000;        //Heater PWM output off.
                soft_power = 0; //Go back to normal operation.
                PORTCbits.RC15 = 0;     //Heat Relay Off
                heat_rly_timer = 3;     //Reset heat relay timer
            }
    }
    
    if (heat_cal_stage == 1 && curnt_cal_stage == 5){
        PDC1 = 0000;    //Heater PWM output off.
        Init();         //Re-init.
        io_off();    //Turn off all inputs and outputs.
        soft_power = 1; //Force device to run in soft power mode.
        heat_set = 0;
        heat_power = 0;
        heat_cal_stage = 2; //If heat_cal_stage is 2 then a calibration is in progress.
        heat_rly_timer = 3;     //Reset heat relay timer
    }
    

    //Heater calibration is in error.
    if(heat_cal_stage == 4){
        PDC1 = 0000;
        heat_set = 0;
        heat_power = 0;
        soft_power = 0; //Go back to normal operation.
    }
}

//Check battery status for faults and dangerous conditions.
void explody_preventy_check(void){
    //Battery over voltage check
    if(battery_voltage >= max_battery_voltage){
        fault_log(0x07);    //Log a high battery voltage shutdown event.
        general_shutdown();
    }
    //Battery under voltage check.
    if(battery_voltage < low_voltage_shutdown && PORTEbits.RE8 == 0){
        fault_log(0x04);    //Log a low battery shutdown event.
        low_battery_shutdown();
    }
    //Battery temp shutdown check
    if(battery_temp > battery_shutdown_temp){
        fault_log(0x08);    //Log a battery over temp shutdown event.
        general_shutdown();
    }
    //Controller temp shutdown check
    if(motor_ctrl_temp > ctrlr_shutdown_temp){
        fault_log(0x09);    //Log a motor controller over temp shutdown event.
        general_shutdown();
    }
    //My temp shutdown check
    if(my_temp > ctrlr_shutdown_temp){
        fault_log(0x0A);    //Log a My Temp over temp shutdown event.
        general_shutdown();
    }
}

void currentCheck(void){
        //Battery over current check.
    if(battery_current < 0){
        dischr_current = battery_current * -1;
    }
    else {
        dischr_current = 0;
    }
    if(dischr_current > over_current_shutdown){
        if(oc_shutdown_timer > 5){
            general_shutdown();
            oc_shutdown_timer = 0;
            fault_log(0x05);    //Log a discharge over current shutdown event.
        }
        oc_shutdown_timer++;
    }
    //Battery charge over current check.
    if(battery_current > max_chrg_current){
        general_shutdown();
        fault_log(0x06);    //Log a charge over current shutdown event.
    }
}

//Turns off all outputs and logs a general shutdown event.
void general_shutdown(void){
    io_off();               //Shutdown all IO except Serial Comms.
    fault_shutdown = 1;       //Tells other stuff that we had a general shutdown.
    cmd_power = 0;
    soft_power = 0;
    power_plugged = 0;
    fault_log(0x0B);            //Log a general Shutdown Event.
}

//Turns off all outputs.
void io_off(void){
//    PORTDbits.RD3 = 0;      //output shutdown. Not Used In current Schematic.
    PDC3 = 0000;            //set output control
    LATE = 0;               //Insure all PORTE outputs are off.
    PWMCON1bits.PEN3L = 0;  //Set PWM3 Low side to standard output so that it can be set to 0
    PDC1 = 0000;            //set heater control
    PDC2 = 0000;            //set charge control
    PORTDbits.RD1 = 0;      //Main Contactor Relay Off
    contact_rly_timer = 3;  //reset contactor relay timer
    PORTFbits.RF0 = 0;      //Fan Relay Off
    PORTFbits.RF6 = 0;      //Charger Relay Off
    chrg_rly_timer = 3;     //reset charge relay timer
    PORTCbits.RC15 = 0;     //Heat Relay Off
    heat_rly_timer = 3;     //reset heat relay timer
    PORTBbits.RB8 = 0;      //Aux Power Relay Off
    heat_power = 0;         //set heater control to 0
    charge_power = 0;       //set charge control to 0
    output_power = 0;       //set output control to 0
    current_output = 0;
    crnt_integral = 0;
}

#endif