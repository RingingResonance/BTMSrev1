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

#include "display.h"
#include "DataIO.h"
#include "errorCodes.h"

//Check if serial port is busy.
//If used, ensure that it is used by an IRQ priority that is lower than TX IRQs.
void portBusyIdle(int serial_port){
    while(portBSY[serial_port]){
        PORTBbits.RB6 = 0;      //Turn CPU ACT light off.
        Idle();                 //Idle Loop, saves power.
    }
    PORTBbits.RB6 = 1;      //Turn CPU ACT light on.
}

/* Read fault codes to serial port.
 * This takes up over 5% of program space.*/
void fault_read(int serial_port){
    load_string("Reading Faults.\r\n", serial_port);
    dispatch_Serial(serial_port);
    flt_index[serial_port] = 0;
    const char *textpointer;
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
                switch(vars.fault_codes[flt_index[serial_port]]){
                    case 0x01:
                        textpointer = code01;
                    break;
                    case 0x02:
                        textpointer = code02;
                    break;
                    case 0x03:
                        textpointer = code03;
                    break;
                    case 0x04:
                        textpointer = code04;
                    break;
                    case 0x05:
                        textpointer = code05;
                    break;
                    case 0x06:
                        textpointer = code06;
                    break;
                    case 0x07:
                        textpointer = code07;
                    break;
                    case 0x08:
                        textpointer = code08;
                    break;
                    case 0x09:
                        textpointer = code09;
                    break;
                    case 0x0A:
                        textpointer = code0A;
                    break;
                    case 0x0B:
                        textpointer = code0B;
                    break;
                    case 0x0C:
                        textpointer = code0C;
                    break;
                    case 0x0D:
                        textpointer = code0D;
                    break;
                    case 0x0E:
                        textpointer = code0E;
                    break;
                    case 0x0F:
                        textpointer = code0F;
                    break;
                    case 0x10:
                        textpointer = code10;
                    break;
                    case 0x11:
                        textpointer = code11;
                    break;
                    case 0x12:
                        textpointer = code12;
                    break;
                    case 0x13:
                        textpointer = code13;
                    break;
                    case 0x14:
                        textpointer = code14;
                    break;
                    case 0x15:
                        textpointer = code15;
                    break;
                    case 0x16:
                        textpointer = code16;
                    break;
                    case 0x17:
                        textpointer = code17;
                    break;
                    case 0x18:
                        textpointer = code18;
                    break;
                    case 0x19:
                        textpointer = code19;
                    break;
                    case 0x1A:
                        textpointer = code1A;
                    break;
                    case 0x1B:
                        textpointer = code1B;
                    break;
                    case 0x1C:
                        textpointer = code1C;
                    break;
                    case 0x1D:
                        textpointer = code1D;
                    break;
                    case 0x1E:
                        textpointer = code1E;
                    break;
                    case 0x1F:
                        textpointer = code1F;
                    break;
                    case 0x20:
                        textpointer = code20;
                    break;
                    case 0x21:
                        textpointer = code21;
                    break;
                    case 0x22:
                        textpointer = code22;
                    break;
                    case 0x23:
                        textpointer = code23;
                    break;
                    case 0x24:
                        textpointer = code24;
                    break;
                    case 0x25:
                        textpointer = code25;
                    break;
                    case 0x26:
                        textpointer = code26;
                    break;
                    case 0x27:
                        textpointer = code27;
                    break;
                    case 0x28:
                        textpointer = code28;
                    break;
                    case 0x29:
                        textpointer = code29;
                    break;
                    case 0x2A:
                        textpointer = code2A;
                    break;
                    default:
                        textpointer = codeDefault;
                    break;
                }
            portBusyIdle(serial_port);  //Check to see if port is ready.
            load_string("Code: ", serial_port);
            load_hex(vars.fault_codes[flt_index[serial_port]],serial_port);
            load_string(" -> ", serial_port);
            load_string(textpointer, serial_port);
            load_string("\r\n", serial_port);
            dispatch_Serial(serial_port);
            flt_index[serial_port]++;
        }
    }
}

void cGetData(int serial_port){
    //Data input
    if(serial_port)
        CMD_buff[serial_port][CMD_Point[serial_port]] = U2RXREG;
    else
        CMD_buff[serial_port][CMD_Point[serial_port]] = U1RXREG;
    //Data echo
    if (Lecho[serial_port] && ((CMD_buff[serial_port][CMD_Point[serial_port]] != 0x0D) || (CMD_buff[serial_port][CMD_Point[serial_port]] != 0x0A))){
        if(serial_port)
            U2TXREG = CMD_buff[serial_port][CMD_Point[serial_port]];
        else
            U1TXREG = CMD_buff[serial_port][CMD_Point[serial_port]];
    }
    //Check for a RETURN
    if (CMD_buff[serial_port][CMD_Point[serial_port]] == 0x0D){
        if (Lecho[serial_port]){
            load_string("\r\n", serial_port);
        }
        bufsize[serial_port] = CMD_Point[serial_port];
        CMD_Point[serial_port] = 0;
        cmdRDY[serial_port] = 1; //Tell our command handler to process.
    }
    //No RETURN detected? Store the data.
    else if(CMD_Point[serial_port] < Clength) {
        CMD_Point[serial_port]++;
    }
}

void Command_Interp(int serial_port){
    if(serial_port){
        while (U2STAbits.URXDA && cmdRDY[serial_port] == 0){
            cGetData(serial_port);
        }
    }
    else{
        while (U1STAbits.URXDA && cmdRDY[serial_port] == 0){
            cGetData(serial_port);
        }
    }
    //Command Handler.
    if (cmdRDY[serial_port] == 1){
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
                load_hex(vars.flash_chksum_old, serial_port);
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
                save_sets(0x00);
                var_save((cfg_space / 2) + 1);  //Save settings to NV-memory
            break;
            case '1':
                load_hex(eeprom_read(0x00),serial_port);
            break;
            case 'H':
                heat_cal_stage = 1;
            break;
            case 'h':
                heat_cal_stage = 5;
            break;
            case 'E':
                Lecho[serial_port] = 1;
            break;
            case 'e':
                Lecho[serial_port] = 0;
            break;
            case 'S':
                deep_sleep = 1;
            break;
            case 'F':
                fault_read(serial_port);          //Read all fault codes.
            break;
            case 'P':
                load_string("P On \r\n", serial_port);
                cmd_power = 1;
            break;
            case 'p':
                load_string("P Off\r\n", serial_port);
                v_test = 0;
                cmd_power = 0;
            break;
            case 'M':
                b_safe = 0;
                v_test = 0;
                load_string("BMon On :D\r\n", serial_port);
            break;
            case 'm':
                b_safe = 0x55FF;
                load_string("!!BMon Off!!\r\n", serial_port);
            break;
            case 'C':
                vars.fault_count = 0;
                fault_shutdown = 0;
                if(heat_cal_stage != 5)
                    heat_cal_stage = 0;
                osc_fail_event = 0;
                load_string("Faults Cleared.\r\n", serial_port);
            break;
            case 'Z':
                p_charge = 0;
                chrg_voltage = sets.battery_rated_voltage;
                if(PORTEbits.RE8 == 1){
                    vars.partial_chrg_cnt = 0;
                }
                else{
                    vars.partial_chrg_cnt = 10;
                }
            break;
            case 'U':
                if(v_test < 99){
                    v_test++;
                }
            break;
            case 'u':
                if(v_test > 0){
                    v_test--;
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