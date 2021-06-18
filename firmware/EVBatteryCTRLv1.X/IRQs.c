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

/*****************/
/* IRQs go here. */
/*****************/

/* Charge Detect IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt (void){
    PORTBbits.RB6 = 1;
    //reset peak power when we plug in a charger.
    peak_power = 0;
    //Check for partial charge status to see if we need to do a full charge to ballance the cells.
    if(partial_charge > 1){
        partial_charge = 1;
        fault_log(0x1C);            // Log Partial charge was set higher than 100% event.
    }
    //If parial_charge is set to 0% then we disable and charge the battery up to full every time.
    if(partial_charge == 0){
        chrg_voltage = battery_rated_voltage;
        p_charge = 0;
        send_string(NLtxtNL,"Regular Charge.", PORT1);
    }
    //Do a full charge every 10 cycles so that we can ballance the cells.
    if(power_plugged != 1 && partial_chrg_cnt < 10){
        partial_chrg_cnt++;
        p_charge = 1;
        chrg_voltage = ((battery_rated_voltage - dischrg_voltage) * partial_charge) + dischrg_voltage;
        send_string(NLtxtNL,"Partial Charge.", PORT1);
    }
    else if(power_plugged != 1 && partial_chrg_cnt >= 10){
        partial_chrg_cnt = 0;
        p_charge = 0;
        chrg_voltage = battery_rated_voltage;
        send_string(NLtxtNL,"Full Charge.", PORT1);
    }
    /* If heat_cal is not running, is not disabled, and key switch/cmd_power is off
     * run heater calibration check when charger is plugged in.
     */
    if(heat_cal_stage != 2 && PORTFbits.RF1 == 1 && cmd_power == 0 && power_plugged == 0 && fault_shutdown == 0){
        curnt_cal_stage = 1;
        //Check for disabled heater setting.
        if(heat_cal_stage != 5){
            heat_cal_stage = 1;
        }
        //soft_power = 1;
        power_plugged = 1;
    }
    
    //Reset battery usage session when charger is plugged in and power is turned off.
    if(PORTFbits.RF1 == 1 && cmd_power == 0){
        battery_usage = 0;      //Reset battery usage session.
        battery_usage_smll = 0;
    }
    charger_detected = 1;
    IFS0bits.INT0IF = 0;
}

/* Wheel Rotate IRQ. */
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt (void){
    PORTBbits.RB6 = 1;

    IFS1bits.INT1IF = 0;
}

/* Brake Switch IRQ */
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt (void){
    PORTBbits.RB6 = 1;

    IFS1bits.INT2IF = 0;
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
        //Battery voltage.
        //adctemp0 /= x;      //Sample average.
        //adctemp0 /= 65535;  //Convert to unsigned fractional
        //adctemp0 *= 5;      //Converted to 0 - 5V voltage.
        //Do it all at once to save time.
        adctemp0 /= 104856;   //(13107 * 8)Average and Convert to unsigned fractional, 0v - 5v
        battery_voltage = (adctemp0 / vltg_dvid) + bt_vlt_adjst;    //Use resistor divider values to covert to actual voltage.
        adctemp0 = 0;       //Clear average.

        //Battery current.
        adctemp1 /= 8;      //Sample average.
        adctemp1 -= 32768;    //Set zero point.
        adctemp1 /= 32768;    //Convert to signed fractional. -1 to 1
        adctemp1 *= 2.5;      //Convert to +-2.5'volts'. It's still a 0 - 5 volt signal on the analog input. The zero point is at 2.5v
        battery_current = (adctemp1 / 0.04) + current_compensate; //Offset for ACS780LLRTR-050B-T Current Sensor.
        adctemp1 = 0;       //Clear average.

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
        
        //Copy board temp to battery temp and controler temp for now.
        motor_ctrl_temp = my_temp;
//        battery_temp = my_temp;
        
        //Reset analog average count.
        analog_avg_cnt = 0;

        //Do a battery check after each valid sample.
        //Check to make sure the battery and other systems are within safe operating conditions.
        //Shutdown and log the reason why if they aren't safe.
        if(b_safe != 0x55FF && adc_sample_burn == 1 && fault_shutdown == 0){
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

/* This needs to be cleaned up and combined into a sub that
 * works with Port 1 and Port 2 using IRQs to save memory and CPU time. The current system will
 * work for now but it's lazy and takes up too much memory. */

/* Data and Command input and processing IRQ for Port 1 */
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void){
    PORTBbits.RB6 = 1;
    while (U1STAbits.URXDA && cmdRDY == 0){       //Data ready?
        cmd[cmdPoint] = U1RXREG;
        if (Lecho){
            U1TXREG = cmd[cmdPoint];
        }
        if (cmd[cmdPoint] == 0x0D){     //Check for a RETURN
            if (Lecho){
                U1TXREG = 0x0A;             //Send NewLine
            }
            bufsize = cmdPoint;
            cmdPoint = 0;
            cmdRDY = 1;                 //Tell our command handler to process.
        }
        else if (cmdPoint > cdmd){      //Check for buffer overflow.
            cmdPoint = 0;
            cmdOVFL = 1;
        }
        else {
            cmdPoint++;
        }
    }
    //Command Handler.
    if (cmdRDY == 1){
        int tempPoint = 0;
        U1TXREG = 0x61;                 //Send command receive "a"
        //Check for faults.
        if(fault_count > 0){
            U1TXREG = 0x21;             //Send fault alert "!"
        }
        
        while (tempPoint < bufsize){
            if (cmd[tempPoint] == 0x0D){    //Enter Key.
                if (Lecho){
                    U1TXREG = 0x0A;             //Send NewLine
                }
                break;                      //Used as a test, do nothing.
            }
            else if (cmd[tempPoint] == 0x23){   // "#" Reset.
                cmdPoint = 0;
                cmdRDY = 0;
                asm("reset");
            }
            else if (cmd[tempPoint] == 'H'){   //Enable and Run heater calibration. "H"
                tempPoint++;
                heat_cal_stage = 1;
            }
            else if (cmd[tempPoint] == 'E'){   //Local Echo. "E"
                tempPoint++;
                if (cmd[tempPoint] == 0x79){
                    Lecho = 1;
                }
                else if (cmd[tempPoint] == 0x6E){
                    Lecho = 0;
                }
                else {
                    Terr = -1;
                }
            }
            else if (cmd[tempPoint] == 'R'){   //Read Vars "R"
                tempPoint++;
                if (read_vars == 0){
                    read_vars = 100;    //Read 100 vars before stopping.
                }
                else {
                    read_vars = 0;
                }
            }
            else if (cmd[tempPoint] == 'S'){    //Low Power Shutdown Test. "S"
                tempPoint++;
                deep_sleep = 1;
            }
            else if (cmd[tempPoint] == 'r'){    //Read Var Once. "r"
                tempPoint++;
                if (read_vars == 0){
                    read_vars = 1;    //Read vars once before stopping.
                }
                else {
                    read_vars = 0;
                }
            }
            else if (cmd[tempPoint] == 'F'){    //Fault Code Read. "F"
                tempPoint++;
                fault_read(full, PORT1);          //Read full fault codes.
            }
            else if (cmd[tempPoint] == 'P'){    //Toggle main power. "P"
                tempPoint++;
                if(cmd_power == 0){
                    send_string(NLtxtNL, "Power On", PORT1);
                    cmd_power = 1;
                    
                }
                else{
                    send_string(NLtxtNL, "Power Off", PORT1);
                    cmd_power = 0;
                    
                }
            }
            else if (cmd[tempPoint] == 'M'){    //Toggle Battery Safety Monitor. "M"
                tempPoint++;
                if(b_safe == 0x55FF){
                    b_safe = 0;
                    send_string(NLtxtNL, "Battery Monitor On", PORT1);
                }
                else{
                    b_safe = 0x55FF;
                    send_string(NLtxtNL, "Battery Monitor Off", PORT1);
                }
            }
            else if (cmd[tempPoint] == 'C'){    //Clear Codes and reset fault_shutdown. "C"
                tempPoint++;
                fault_count = 0;
                fault_shutdown = 0;
                heat_cal_stage = 0;
                osc_fail_event = 0;
            }
            else if (cmd[tempPoint] == 'Z'){    //Set to fully charge the battery. "Z"
                tempPoint++;
                p_charge = 0;
                chrg_voltage = battery_rated_voltage;
                if(PORTEbits.RE8 == 1){
                    partial_chrg_cnt = 0;
                }
                else{
                    partial_chrg_cnt = 10;
                }
            }
            else if (cmd[tempPoint] == 'h'){    //Disable auto heater calibration. "h"
                tempPoint++;
                heat_cal_stage = 5;
            }
            else if (cmd[tempPoint] == 'V'){    //Send Voltage Percentage. "V"
                tempPoint++;
                send_string(NLtxt, "Battery charge voltage is at ", PORT1);
                float_send(voltage_percentage, PORT1);
                send_string(txtNL, "%. ", PORT1);
            }
            else if (cmd[tempPoint] == 'B'){    //Read battery calculated ratings. "B"
                tempPoint++;
                send_string(NLtxt, "Rated Battery capacity ", PORT1);
                float_send(amp_hour_rating, PORT1);
                send_string(txtNL, "Ah. ", PORT1);
                send_string(NLtxt, "Calculated Battery capacity ", PORT1);
                float_send(battery_capacity, PORT1);
                send_string(txtNL, "Ah. ", PORT1);
                send_string(NLtxt, "Remaining Battery capacity ", PORT1);
                float_send(battery_remaining, PORT1);
                send_string(txtNL, "Ah. ", PORT1);
            }
            else if (cmd[tempPoint] == 'O'){           // Use POS display for HUD.
                tempPoint++;
                use_POS_display = 1;
            }
            else if (cmd[tempPoint] == 'o'){           // Disable POS display for HUD.
                tempPoint++;
                use_POS_display = 0;
            }
            else if (cmd[tempPoint] == 'u'){           // increase v_test
                tempPoint++;
                if(v_test < 99){
                    v_test++;
                }
            }
            else if (cmd[tempPoint] == 0x20){           // Space.
                tempPoint++;
            }
            else {
                send_string(NLtxt, "Unknown Command.", PORT1);     //Print "S" for Syntax Error.
                Terr = -1;
                break;
            }
        }
        cmdPoint = 0;
        cmdRDY = 0;
        if (Terr == -1){
            Terr = 0;
            cmdPoint = 0;
            tempPoint = 0;
            U1TXREG = 0x45;     //Print "E" for Error.
        }
    }
/****************************************/
    /* End the IRQ. */
    IFS0bits.U1RXIF = 0;
}

/* Data and Command input and processing IRQ for Port 2. */
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt (void){
    PORTBbits.RB6 = 1;
    /*
    while (U2STAbits.URXDA && cmdRDY == 0){       //Data ready?
        cmd2[cmdPoint2] = U2RXREG;
        if (Lecho2){
            U2TXREG = cmd2[cmdPoint2];
        }
        if (cmd2[cmdPoint2] == 0x0D){     //Check for a RETURN
            if (Lecho2){
                U2TXREG = 0x0A;             //Send NewLine
            }
            bufsize2 = cmdPoint2;
            cmdPoint2 = 0;
            cmdRDY2 = 1;                 //Tell our command handler to process.
        }
        else if (cmdPoint2 > cdmd2){      //Check for buffer overflow.
            cmdPoint2 = 0;
            cmdOVFL2 = 1;
        }
        else {
            cmdPoint2++;
        }
    }
    //Command Handler.
    if (cmdRDY2 == 1){
        int tempPoint2 = 0;
        U2TXREG = 0x61;                 //Send command receive "a"
        //Check for faults.
        if(fault_count > 0){
            U2TXREG = 0x21;             //Send fault alert "!"
        }
        
        while (tempPoint2 < bufsize2){
            if (cmd2[tempPoint2] == 0x0D){    //Enter Key.
                if (Lecho2){
                    U2TXREG = 0x0A;             //Send NewLine
                }
                break;                      //Used as a test, do nothing.
            }
            else if (cmd2[tempPoint2] == 0x23){   // "#" Reset.
                cmdPoint2 = 0;
                cmdRDY2 = 0;
                asm("reset");
            }
            else if (cmd2[tempPoint2] == 'H'){   //Enable and Run heater calibration. "H"
                tempPoint2++;
                heat_cal_stage = 1;
            }
            else if (cmd2[tempPoint2] == 'E'){   //Local Echo. "E"
                tempPoint2++;
                if (cmd2[tempPoint2] == 0x79){
                    Lecho2 = 1;
                }
                else if (cmd2[tempPoint2] == 0x6E){
                    Lecho2 = 0;
                }
                else {
                    Terr2 = -1;
                }
            }
            else if (cmd2[tempPoint2] == 'S'){    //Low Power Shutdown Test. "S" Might also be useful for security shutdown/lockout.
                tempPoint2++;
                deep_sleep = 1;
            }
            else if (cmd2[tempPoint2] == 'F'){    //Fault Code Read. "F"
                tempPoint2++;
                fault_read(simple, PORT2);          //Read simple fault codes.
            }
            else if (cmd2[tempPoint2] == 'P'){    //Toggle main power. "P"
                tempPoint2++;
                if(cmd_power == 0){
                    send_string(NLtxtNL, "P On", PORT2);
                    cmd_power = 1;
                    
                }
                else{
                    send_string(NLtxtNL, "P Off", PORT2);
                    cmd_power = 0;
                    
                }
            }
            else if (cmd2[tempPoint2] == 'M'){    //Toggle Battery Safety Monitor. "M"
                tempPoint2++;
                if(b_safe == 0x55FF){
                    b_safe = 0;
                    send_string(NLtxtNL, "M On", PORT2);
                }
                else{
                    b_safe = 0x55FF;
                    send_string(NLtxtNL, "M Off", PORT2);
                }
            }
            else if (cmd2[tempPoint2] == 'C'){    //Clear Codes and reset fault_shutdown. "C"
                tempPoint2++;
                fault_count = 0;
                fault_shutdown = 0;
                heat_cal_stage = 0;
                osc_fail_event = 0;
            }
            else if (cmd[tempPoint2] == 'Z'){    //Set to fully charge the battery. "Z"
                tempPoint2++;
                p_charge = 0;
                chrg_voltage = battery_rated_voltage;
                if(PORTEbits.RE8 == 1){
                    partial_chrg_cnt = 0;
                }
                else{
                    partial_chrg_cnt = 10;
                }
            }
            else if (cmd2[tempPoint2] == 'h'){    //Disable auto heater calibration. "h"
                tempPoint2++;
                heat_cal_stage = 5;
            }
            else if (cmd2[tempPoint2] == 'V'){    //Send Voltage Percentage. "V"
                tempPoint2++;
                send_string(NLtxt, "BV is ", PORT2);
                float_send(voltage_percentage, PORT2);
                send_string(txtNL, "%. ", PORT2);
            }
            else if (cmd2[tempPoint2] == 'B'){    //Read battery calculated ratings. "B"
                tempPoint2++;
                //send_hud_vars(PORT2);             //Send simple battery stats.
            }
            else if (cmd2[tempPoint2] == 'O'){           // Use POS display for HUD.
                tempPoint2++;
                use_POS_display = 1;
            }
            else if (cmd2[tempPoint2] == 'o'){           // Disable POS display for HUD.
                tempPoint2++;
                use_POS_display = 0;
            }
            else if (cmd2[tempPoint2] == 0x20){           // Space.
                tempPoint2++;
            }
            else {
                send_string(NLtxt, "UC.", PORT2);     //Print "S" for Syntax Error.
                Terr = -1;
                break;
            }
        }
        cmdPoint2 = 0;
        cmdRDY2 = 0;
        if (Terr2 == -1){
            Terr2 = 0;
            cmdPoint2 = 0;
            tempPoint2 = 0;
            U2TXREG = 0x45;     //Print "E" for Error.
        }
    }
     */
/****************************************/
    /* End the IRQ. */
    IFS1bits.U2RXIF = 0;
}

/* Display output IRQ for Port 2 */
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt (void){
    PORTBbits.RB6 = 1;
    //Dispatch the big buffer to the little 4 word Serial Port buffer as it empties.
    while(U2STAbits.UTXBF == 0 && CBuff_index <= CBuff_max_data){
        U2TXREG = Port2_Buffer[CBuff_index];
        CBuff_index++;
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
     * your electrons and burn your lunch from the heat that it generates!
     */

    if(main_power){
        PowerOffTimer = PowerOffAfter;      //reset the timer when main_power is on.
        PowerOffTimerSec = 59;              //60 seconds.
        PORTDbits.RD2 = 1; //Enable Keep Alive signal. System keeps itself on while main_power is enabled.
    }
    else{
        //When main_power is off count down in minutes.
        if(PowerOffTimerSec <= 0){
            PowerOffTimerSec = 59;
            if(PowerOffTimer <= 0){
                power_off(); //power off the system to save power.
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
    if(oc_shutdown_timer > 0 && dischr_current < over_current_shutdown){
        oc_shutdown_timer--;
    }
    // Check for charger disconnect.
    if(PORTEbits.RE8 == 0){
        PORTBbits.RB4 = 0;          //charger light off.
        charge_power = 0;
        chrg_check = 0;
    }
    //***************************************
    //What display are we using?
    if(diag_count >= 9){
        diagmode = 1;
        diag_count = 0;
    }
    else if(diag_count > 0){
        diag_count--;
    }
    else if(main_power == 0){
        diagmode = 0;
    }
    //Initialize display if we are going to use it.
    if(use_POS_display && main_power){
        POS_disp_init();
        if(diagmode){
            POS_diag_display();      //Update Diag display.
        }
        else{
            POS_dash_display();      //Update Dash display.
        }
    }
    else{
        dispinit = 0;
    }
    //Fan control
    if(main_power && (battery_temp > batt_fan_start || my_temp > ctrlr_fan_start || motor_ctrl_temp > ctrlr_fan_start)){
        PORTFbits.RF0 = 1;
    }
    else if(main_power == 0 || (battery_temp < (batt_fan_start - 5) && my_temp < (ctrlr_fan_start - 5) && motor_ctrl_temp < (ctrlr_fan_start - 5))){
        PORTFbits.RF0 = 0;
    }
    
    //Main power check
    main_power_check();
    
    //Print vars in plain text once every second if the user requests.
    //This needs to not exist. Need to implement another IRQ driven request system like the one used for the HUD.
    //Uses too much CPU time while waiting for the serial port to send data. System cannot regulate power while this is running!!!
    if (read_vars >= 1){
        read_vars--;
        print_vars(PORT1);      //Only allow printing these vars on port 1
    }
    
    //Clear fault_shutdown if all power modes are turned off.
    if(PORTEbits.RE8 == 0 && PORTFbits.RF1 == 1 && soft_power == 0 && cmd_power == 0 && shutdown_timer == 0){
        shutdown_timer = 1;     //Acts like a resettable circuit breaker.
    }
    
    //Blink Check Engine Light if any faults are logged and any power modes are on regardless of what fault_shutdown says.
    if(pwr_detect){
        if(error_blink == 1){
            error_blink = 0;
            PORTBbits.RB5 = 0;
        }
        else{
            error_blink = 1;
            if(fault_count != 0){
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
        if (eeprom_read((cfg_space / 2) + 1) == 0x7654){
            //Calculate how much power was used while the power was off. This is not exact, but should be close enough.
            int power_diff = (battery_capacity * ((voltage_percentage_old - voltage_percentage) / 100));
            battery_remaining -= power_diff;        //Subtract the power difference
            absolute_battery_usage -= power_diff;
            battery_usage -= power_diff;
        }
        else{
            battery_capacity = amp_hour_rating; //Just use the amp hour rating on first start.
            battery_remaining = amp_hour_rating * (voltage_percentage / 100);   //Rough estimation of how much power is left.
        }
        first_cal = 9;      //Signal that we are done with power up.
        soft_power = 0;
        //We have our starting variables, now check for a charger manually this one time. 
        //Do this here because plugging in a charger on a power off state might cause the charger IRQ to do things
        //while the rest of the system isn't ready if it even trigger it at all.
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
    if(battery_usage < 0){
        absolute_battery_usage = battery_usage * -1;
    }
    else{
        absolute_battery_usage = battery_usage;
    }
    
    //Calculate the max capacity of the battery once the battery has been fully charged and fully discharged.
    if(voltage_percentage > 99 && power_session != 4){
        //battery_capacity = absolute_battery_usage;
        battery_remaining = battery_capacity;
        //if(power_session != 4){
            power_session = 4;
            battery_usage = 0;  //reset battery usage session.
            battery_usage_smll = 0;
        //}
    }
    //At about 82% voltage the battery is at about 50% actual capacity for lithium ion.
    else if(voltage_percentage < 83 && voltage_percentage > 81 && (power_session == 4 || power_session == 0)){
        battery_capacity = absolute_battery_usage;  //Calculate the max capacity of the battery after a half discharge.
        battery_capacity *= 2;
        power_session = 2;
    }
    else if(voltage_percentage < 1 && power_session != 0){
        battery_capacity = absolute_battery_usage;  //Calculate the max capacity of the battery after a full discharge.
        battery_remaining = 0;  // Set ah remaining to 0 when less than 2% voltage.
        battery_remaining_smll = 0;
        power_session = 0;
        battery_usage = 0;  //reset battery usage session.
        battery_usage_smll = 0;
    }

    //Don't let battery_remaining go below 0;
    //This should never happen in normal conditions. This is just a catch.
    if(battery_remaining < 0){
        battery_remaining = 0;
        battery_remaining_smll = 0;
    }
    //*******************************************
    //Don't let battery_remaining go above battery capacity, ever.
    if(battery_remaining > battery_capacity + 0.005){
        battery_remaining = battery_capacity + 0.005;        //Go slightly above. Just slightly.
    }
    //Don't let battery remaining go above the partial charge percentage when partial charging.
    //To Do: This isn't exactly right because 90% Voltage != 90% total capacity!!! It's only a few % off so for now it's okay.
    if(battery_voltage < (partial_charge * battery_rated_voltage) && p_charge && battery_remaining > (battery_capacity * partial_charge)){
        battery_remaining = (battery_capacity * partial_charge);
    }
    //**************************************************
    //Circuit draw compensation.
    //Heart beat draws power even when the system is off. This logs that current draw.
    if(main_power == 0){
        battery_remaining -= (circuit_draw * 0.0002777);
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
    float x = 8;        //Why is this not an int? Because weird things happen when you divide a float by an int. Oh Well.
    float power = 0;
    
    
    //Get average current.
    if(crnt_avg_cnt >= x){
        bt_crnt_avg_temp /= x;
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
    if(vltg_avg_cnt >= x){
        bt_vltg_avg_temp /= x;
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
        battery_usage_smll = battery_usage_smll + (calc_125 * battery_current); 
        battery_remaining_smll = battery_remaining_smll + (calc_125 * battery_current);
        if(battery_usage_smll > 0.001){
            battery_usage_smll -= 0.001;
            battery_usage += 0.001;
        }
        else if(battery_usage_smll < -0.001){
            battery_usage_smll += 0.001;
            battery_usage -= 0.001;
        }
        if(battery_remaining_smll > 0.001){
            battery_remaining_smll -= 0.001;
            battery_remaining += 0.001;
        }
        else if(battery_remaining_smll < -0.001){
            battery_remaining_smll += 0.001;
            battery_remaining -= 0.001;
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
    fault_log(0x0C);        //PWM fault??? What fault is this?
    //INTCON1bits.COVTE = 0;
}
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail (void){
    //Check the PLL lock bit as fast as possible before anything else because it doesn't stay set for long.
    if(OSCCONbits.LOCK == 0){
        if(osc_fail_event == 0){
            fault_log(0x12);
        }
    }
    PORTBbits.RB6 = 1;
    //io_off();
    //fault_shutdown = 1;
    //send_string(0x03, "Oscillator Fail Event.");
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
    //send_string(0x03, "Reserved Trap 7 Event.");
    fault_log(0x11);
    //INTCON1bits.DMACERR = 0;
}
/****************/
/* END IRQs     */
/****************/   

#endif