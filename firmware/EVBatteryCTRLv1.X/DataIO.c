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

#ifndef DATAIO_C
#define DATAIO_C

#include <p30f3011.h>
#include "DataIO.h"

//EEPROM functions. Yes, it is faster to write 16 words at a time rather than one at a time. I'm too lazy to implement that.
int eeprom_erase(int addrs){
    int adrfinal = 0;
    if (addrs > 0x1FF){
        return 1;       //error clearing, address out of range.
    }
    else{
        adrfinal = addrs * 2;
        adrfinal += 0xFC00;
        //Erase location.
        NVMADRU = 0x7F;
        NVMADR = adrfinal;
        NVMCON = 0x4044;
        NVMKEY = 0x55;
        NVMKEY = 0xAA;
        __asm__ ("NOP");
        __asm__ ("NOP");
        NVMCONbits.WR = 1;
        return 0;
    }
}
/** Write a Byte to EEPROM **/
int eeprom_write(int addrs, int data){
    int adrfinal = 0;
    if (addrs > 0x1FF){
        return 1;       //error writing, address out of range.
    }
    else{
        adrfinal = addrs * 2;
        adrfinal += 0xFC00;
        //Write to location.
        //DO NOT TOUCH THIS, EVERYTHING MUST BE DONE IN THIS EXACT ORDER OR IT WILL NOT WORK!!!
        WREG1 = 0x7F;     //ADDRESS UPPER
        __asm__ ("MOV W1,TBLPAG");
        WREG2 = data; //data pattern.
        WREG0 = adrfinal;   //ADDRESS LOWER
        //ESPECIALLY DON'T TOUCH THE ORDER THAT THE WREGs GET WRITTEN TO.
        __asm__ ("TBLWTL w2,[w0]");
        NVMCON = 0x4004;
        NVMKEY = 0x55;
        NVMKEY = 0xAA;
        __asm__ ("NOP");
        __asm__ ("NOP");
        NVMCONbits.WR = 1;
        return 0;
    }
}
int eeprom_read(int addrs){
    int adrfinal = 0;
    if (addrs > 0x1FF){
        return 0;       //error reading, address out of range. Return what we hope to be a safe value.
    }
    else{
        adrfinal = addrs * 2;
        adrfinal += 0xFC00;
        TBLPAG = 0x7F;
        WREG0 = adrfinal; //address to read from
        __asm__ ("TBLRDL [w0],w4");
        __asm__ ("NOP");
        __asm__ ("NOP");
        return WREG4; //WREG4 will contain data read from eeprom
    }
}
//*************************************************************************************************


//Converts four bit hex numbers to ASCII
char four_bit_hex_cnvt(int numb){
    char asci_hex = 0;
    int temp = 0;
    temp = 0x0F & numb;

    if(temp < 10){
        asci_hex = temp + 48;
    }
    else{
        asci_hex = temp + 55;
    }
    return asci_hex;
}

//Send a string of text to a buffer that can then be dispatched to a serial port.
void send_string(const char *string_point, int serial_port){
    int x = 0;
    while (string_point[x] != 0){
        if (Buff_index[serial_port] < 45){
            Buff_index[serial_port]++;
        }
        Buffer[serial_port][Buff_index[serial_port]] = string_point[x];
        x++;

        if (serial_port == 0x00){
            fault_log(0x1A);        //Log invalid port error.
            x = 0;
            break;
        }
    }
}

/* Sends a float through to buffer. */
void float_send(float f_data, int serial_port){

    int x = 0;
    float tx_float = 0;
    int tx_temp = 0;
    //f_data = 0.0010;

    float_out[serial_port][0] = ' ';        //Put a SPACE in "0"
    if (f_data < 0){
        f_data *= -1;
        float_out[serial_port][0] = '-';    //Put a - in "0"
    }
    if (f_data > 9999.999){
        f_data = 9999.999;
        float_out[serial_port][0] = '?';    //Put a ? in "0"
    }
    tx_float = f_data / 1000;
    x++;
    while (x <= 8){
        if (x == 5){
            float_out[serial_port][5] = '.';
            x++;
        }
        tx_temp = tx_float;
        float_out[serial_port][x] = tx_temp + 48;
        x++;
        tx_float = (tx_float - tx_temp) * 10;
    }

/* Write to buffer */
    x = 0;
    int ifzero = 1;
    while (x <= 8){
        if(float_out[serial_port][x] == 0x30 && ifzero){
            if(config_space){
                if (Buff_index[serial_port] < 45){
                    Buff_index[serial_port]++;
                }
                Buffer[serial_port][Buff_index[serial_port]] = 0x20;
                x++;
            }
            else{
                x++;
            }
        }
        else{
            if (Buff_index[serial_port] < 45){
                Buff_index[serial_port]++;
            }
            Buffer[serial_port][Buff_index[serial_port]] = float_out[serial_port][x];
            x++;
            if (serial_port != 0x00 || serial_port != 0x01){
                fault_log(0x1A);        //Log invalid port error.
                x = 0;
                break;
            }
        }
        if(float_out[serial_port][x] != 0x30 && float_out[serial_port][x] != 0x2D && float_out[serial_port][x] != 0x20){
            ifzero = 0;
        }
    }
    config_space = 0;
}

unsigned int BaudCalc(double BD, double mlt){
    /* Calculate baud rate. */
    double INS;
    double OutPut;
    unsigned int Oputs;
    INS = mlt * 1000000;
    OutPut = ((INS/BD)/16)-1;
    Oputs = OutPut;
    return Oputs;             //Weird things happen when you try to calculate
                              //a float directly into an int. Don't do this.
    /************************/
}

#endif
