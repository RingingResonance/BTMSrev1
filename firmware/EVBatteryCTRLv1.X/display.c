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

#include "common.h"
#include "display.h"
#include "DataIO.h"
#include "errorCodes.h"
#include "eeprom.h"
#include "checksum.h"

void displayOut(int serial_port){
    if(sets.PxVenable[serial_port] && !portBSY[serial_port] && CONDbits.main_power){
        if(PxVtimer[serial_port] == 0){
            pageOut(PxPage[serial_port], serial_port);
            PxVtimer[serial_port] = sets.pageDelay[serial_port][PxPage[serial_port]] - 1;
            if (PxVtimer[serial_port] < 0) PxVtimer[serial_port] = 0;
            if(PxPage[serial_port] < 4 && sets.page[serial_port][PxPage[serial_port]][0]) PxPage[serial_port]++; //If we get a NULL for first variable then skip go back to page 0
            else PxPage[serial_port] = 0;
        }
        else PxVtimer[serial_port]--;
    }
}

//Loads one of the serial buffers with user defined data and then dispatches it.
void pageOut(int pageNum, int serial_port){
    unsigned int pageVar = 0;
    unsigned int varNum = 0;
    unsigned int dodispatch = 0;
    for(pageVar=0;pageVar<6;pageVar++){
        varNum = sets.page[serial_port][pageNum][pageVar];
        varNum &= 0xFF;
        if(varNum < 8)dynaSpace[serial_port] = 1;       //Do not reserve a space for a sign char if we don't have to.
        if(varNum == 8)config_space[serial_port] = 1;   //Reserve spaces for 0's when showing watts so that text isn't jumping around.
        //Skip page if first Var is listed as NULL, or if we get a NULL later then we are done.
        if(varNum == 0)break;
        //Check if loading custom data or not.
        if(varNum > 0x00FB){
            switch(varNum){
                case 0x00FC:
                    load_string(sets.custom_data1, serial_port);
                break;
                case 0x00FD:
                    load_string(sets.custom_data2, serial_port);
                break;
                case 0x00FE:
                    load_string(sets.custom_data3, serial_port);
                break;
                case 0x00FF:
                    load_string(sets.custom_data4, serial_port);
                break;
            }
            dodispatch = 1; //if we have at least one variable to display then dispatch.
        }
        else if(varNum <= varLimit){
            load_float(dsky.dskyarrayFloat[varNum], serial_port);   //Load the number.
            Buff_index[serial_port] -= 2;                           //Subtract 2 from buffer index.
            load_string(Vlookup[varNum], serial_port);              //Load the number's text.
            load_string(" ", serial_port);                          //Load a space afterwards.
            dodispatch = 1; //if we have at least one variable to display then dispatch.
        }
    }
    if (dodispatch) dispatch_Serial(serial_port);
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
        if(Lecho[serial_port])load_string("@", serial_port);
        //Check for faults.
        if(vars.fault_count > 0){
            //Send fault alert "!"
            if(Lecho[serial_port])load_string("!", serial_port);
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
                if(Lecho[serial_port])load_string("\r\nSettings and Vars Saved.\r\n", serial_port);
                save_sets();
                save_vars();  //Save settings to NV-memory
            break;
            case '~':
                default_sets(); //Load defaults.
                eeprom_erase(0x0000);   //Erase address 0x0000 to 0xFFFF
                nvm_chksum_update();    //Update EEPROM checksum.
            break;
            case 'H':
                vars.heat_cal_stage = initialize;
            break;
            case 'h':
                vars.heat_cal_stage = disabled;
                save_vars();
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
                if(Lecho[serial_port])load_string("P On \r\n", serial_port);
                CONDbits.cmd_power = 1;
            break;
            case 'p':
                if(Lecho[serial_port])load_string("P Off\r\n", serial_port);
                v_test = 0;
                CONDbits.cmd_power = 0;
            break;
            case 'O':
                if(Lecho[serial_port])load_string("HUD On \r\n", serial_port);
                sets.PxVenable[serial_port] = 1;
                ram_chksum_update();        //Generate new checksum.
            break;
            case 'o':
                if(Lecho[serial_port])load_string("HUD Off\r\n", serial_port);
                sets.PxVenable[serial_port] = 0;
                ram_chksum_update();        //Generate new checksum.
            break;
            case 'C':
                vars.fault_count = 0;
                STINGbits.fault_shutdown = 0;
                if(vars.heat_cal_stage != disabled)
                    vars.heat_cal_stage = notrun;
                STINGbits.osc_fail_event = 0;
                save_vars();
                if(Lecho[serial_port])load_string("Faults Cleared.\r\n", serial_port);
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
            case '*':   //Print firmware version.
                load_string(version, serial_port);
            break;
            case '!':
                
            break;
            default:
                if(Lecho[serial_port])load_string("Unknown Command.\r\n", serial_port);
            break;
        }
        CMD_Point[serial_port] = 0;
        cmdRDY[serial_port] = 0;
        tempPoint[serial_port] = 0;
    }
    dispatch_Serial(serial_port);
}

#endif