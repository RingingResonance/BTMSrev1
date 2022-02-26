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

#ifndef DKSY_C
#define DKSY_C

#include <p30f3011.h>
#include "common.h"
#include "display.h"
#include "DataIO.h"
#include "errorCodes.h"
#include "eeprom.h"
#include "checksum.h"

//Loads one of the serial buffers with user defined data and then dispatches it.
void pageOut(int pageNum, int serial_port){
    int pageVar = 0;
    int varNum = 0;
    for(pageVar=0;pageVar<6;pageVar++){
        varNum = sets.page[pageNum][pageVar];
        if(varNum < 8)dynaSpace[serial_port] = 1;       //Do not reserve a space for a sign char if we don't have to.
        if(varNum == 8)config_space[serial_port] = 1;   //Reserve spaces for 0's when showing watts.
        //Skip page if first Var is listed as '0'
        if((varNum + pageVar) == 0)break;
        //Check if loading custom data or not.
        if(varNum > 0xF9){
            load_string(sets.custom_data[varNum - 0xFA], serial_port);
        }
        else{
            load_float(dsky.diskarrayFloat[varNum], serial_port);   //Load the number.
            Buff_index[serial_port] -= 2;                           //Subtract 2 from buffer index.
            load_string(Vlookup[varNum], serial_port);              //Load the number's text.
            load_string(" ", serial_port);                          //Load a space afterwards.
        }
    }
    dispatch_Serial(serial_port);
}

//Check if serial port is busy.
//If used, ensure that it is used by an IRQ priority that is lower than TX IRQs.
void portBusyIdle(int serial_port){
    while(portBSY[serial_port]){
        CPUact = 0;      //Turn CPU ACT light off.
        Idle();                 //Idle Loop, saves power.
    }
    CPUact = 1;      //Turn CPU ACT light on.
}

/* Read fault codes to serial port.
 * This takes up over 5% of program space.*/
void fault_read(int serial_port){
    load_string("Reading Faults.\r\n", serial_port);
    dispatch_Serial(serial_port);
    flt_index[serial_port] = 0;
    portBusyIdle(serial_port);  //Check to see if port is ready.
    if(vars.fault_count > 10){
        load_string("Fault Log Full.\r\n", serial_port);
        dispatch_Serial(serial_port);
    }
    if(vars.fault_count == 0){
        load_string("No fault codes.\r\n", serial_port);
        dispatch_Serial(serial_port);
    }
    else{
        while(flt_index[serial_port] < vars.fault_count){
            portBusyIdle(serial_port);  //Check to see if port is ready.
            load_string("Code: ", serial_port);
            load_hex(vars.fault_codes[flt_index[serial_port]],serial_port);
            load_string(" -> ", serial_port);
            int ecode = vars.fault_codes[flt_index[serial_port]];
            if(!ecode || ecode < numOfCodes)load_string(errArray[ecode-1], serial_port);
            else load_string(codeDefault, serial_port);
            load_string("\r\n", serial_port);
            dispatch_Serial(serial_port);
            flt_index[serial_port]++;
        }
    }
}

void Command_Interp(int serial_port){
    //Get data. Get allll the data.
    while (((serial_port == PORT1 && U1STAbits.URXDA) || (serial_port == PORT2 && U2STAbits.URXDA)) && !cmdRDY[serial_port]){
        //Data input
        if(serial_port)CMD_buff[serial_port][CMD_Point[serial_port]] = U2RXREG;
        else CMD_buff[serial_port][CMD_Point[serial_port]] = U1RXREG;
        //Data echo
        if (Lecho[serial_port] && ((CMD_buff[serial_port][CMD_Point[serial_port]] != 0x0D) || 
        (CMD_buff[serial_port][CMD_Point[serial_port]] != 0x0A))){
            if(serial_port)U2TXREG = CMD_buff[serial_port][CMD_Point[serial_port]];
            else U1TXREG = CMD_buff[serial_port][CMD_Point[serial_port]];
        }
        //Check for a RETURN
        char cmdinput = CMD_buff[serial_port][CMD_Point[serial_port]];
        if (cmdinput == 0x0D || cmdinput == 0x0A){
            if (Lecho[serial_port])load_string("\r\n", serial_port);
            bufsize[serial_port] = CMD_Point[serial_port];
            CMD_Point[serial_port] = 0;
            cmdRDY[serial_port] = 1; //Tell our command handler to process.
        }
        //No RETURN detected? Store the data.
        else if(CMD_Point[serial_port] < Clength)CMD_Point[serial_port]++;
    }
    //Command Handler.
    if (cmdRDY[serial_port]){
        //Send command receive "@"
        load_string("@", serial_port);
        //Check for faults.
        if(vars.fault_count > 0){
            //Send fault alert "!"
            load_string("!", serial_port);
        }
        switch(CMD_buff[serial_port][tempPoint[serial_port]]){
            case 0x0D:
            break;
            case '#':
                CMD_Point[serial_port] = 0;
                cmdRDY[serial_port] = 0;
                asm("reset");
            break;
            case '$':       
                //Generate flash and chip config checksum and compare it to the old one.
                load_string("\r\nPRG:\r\nStored:", serial_port);
                load_hex(sets.flash_chksum_old, serial_port);
                load_string("\r\nCalc:  ", serial_port);
                check_prog();
                load_hex(flash_chksum, serial_port);
                load_string("\r\n", serial_port);
            break;
            case '&':
                //Generate EEPROM checksum and compare it to the old one.
                load_string("\r\nNVM:\r\nStored:", serial_port);
                load_hex(eeprom_read(0x01FF), serial_port);
                load_string("\r\nCalc:  ", serial_port);
                check_nvmem();
                load_hex(rom_chksum, serial_port);
                load_string("\r\n", serial_port);
            break;
            case '%':
                load_string("\r\nSettings and Vars Saved.\r\n", serial_port);
                save_sets();
                save_vars();  //Save settings to NV-memory
            break;
            case '~':
                default_sets(); //Load defaults.
                eeprom_erase(0x0000);   //Erase address 0x0000 to 0xFFFF
                nvm_chksum_update();    //Update EEPROM checksum.
            break;
            case 't':
                pageOut(0, serial_port);    //Test page.
            break;
            case 'T':
                load_string("\n\rRated:", serial_port);
                load_float(sets.amp_hour_rating, serial_port);
                load_string("\n\rRemaining:", serial_port);
                load_float(vars.battery_remaining, serial_port);
                load_string("\n\rCapacity:", serial_port);
                load_float(vars.battery_capacity, serial_port);
                load_string("\n\r", serial_port);
            break;
            case 'V':
                load_string("\n\rGotVolt? ", serial_port);
                load_float(CONDbits.got_open_voltage, serial_port);
                load_string("\n\rOpenVlt: ", serial_port);
                load_float(voltage_percentage, serial_port);
                load_string("\n\r", serial_port);
            break;
            case 'H':
                vars.heat_cal_stage = 1;
            break;
            case 'h':
                vars.heat_cal_stage = 5;
            break;
            case 'E':
                Lecho[serial_port] = 1;
            break;
            case 'e':
                Lecho[serial_port] = 0;
            break;
            case 'S':
                STINGbits.deep_sleep = 1;
            break;
            case 'F':
                fault_read(serial_port);          //Read all fault codes.
            break;
            case 'P':
                load_string("P On \r\n", serial_port);
                CONDbits.cmd_power = 1;
            break;
            case 'p':
                load_string("P Off\r\n", serial_port);
                v_test = 0;
                CONDbits.cmd_power = 0;
            break;
            case 'C':
                vars.fault_count = 0;
                STINGbits.fault_shutdown = 0;
                if(vars.heat_cal_stage != 5)
                    vars.heat_cal_stage = 0;
                STINGbits.osc_fail_event = 0;
                save_vars();
                load_string("Faults Cleared.\r\n", serial_port);
            break;
            case 'Z':
                p_charge = 0;
                dsky.chrg_voltage = sets.battery_rated_voltage;
                if(PORTEbits.RE8){
                    vars.partial_chrg_cnt = 0;
                }
                else{
                    vars.partial_chrg_cnt = 10;
                }
            break;
            default:
                load_string("Unknown Command.\r\n", serial_port);
            break;
        }
        CMD_Point[serial_port] = 0;
        cmdRDY[serial_port] = 0;
        tempPoint[serial_port] = 0;
    }
    dispatch_Serial(serial_port);
}

#endif