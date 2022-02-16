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

//Start the data transfer from one of the buffers to the selected serial port
//Dispatch the data in the buffers to the display by creating a TX IRQ
void dispatch_Serial(int serial_port){
    portBSY[serial_port] = 1;
    Buff_index[serial_port] = 0;  //Start Index at 0.
    if(serial_port){
        IEC1bits.U2TXIE = 1; //Enable interrupts for UART2 Tx.
        IFS1bits.U2TXIF = 1;        //Start transmitting by manually send an IRQ.
    }
    else{
        IEC0bits.U1TXIE = 1; //Enable interrupts for UART1 Tx.
        IFS0bits.U1TXIF = 1;        //Start transmitting by manually send an IRQ.
    }
}

//Send a string of text to a buffer that can then be dispatched to a serial port.
void load_string(const char *string_point, int serial_port){
    //Check if valid port has been selected.
    if (serial_port > 0x01){
        fault_log(0x1A);        //Log invalid port error.
        FtempIndex[serial_port] = 0;
        return;
    }
    //check if port is busy sending data.
    if(portBSY[serial_port]){
        if(serial_port)
            fault_log(0x28);       //Log Port2 Busy Error.
        else
            fault_log(0x27);       //Log Port1 Busy Error.
        return;
    }
    //check if another process is currently writing to buffer.
    if(writingbuff[serial_port]){
        return;
    }
    writingbuff[serial_port] = 1;
    StempIndex[serial_port] = 0;
    while (string_point[StempIndex[serial_port]] != 0){
        Buffer[serial_port][Buff_index[serial_port]] = string_point[StempIndex[serial_port]];
        if (Buff_index[serial_port] < 49)
            Buff_index[serial_port]++;
        StempIndex[serial_port]++;
    }
    Buff_count[serial_port] = Buff_index[serial_port];
    writingbuff[serial_port] = 0;
}

/* Sends a float to buffer. */
void load_float(float f_data, int serial_port){
    if (serial_port > 0x01){
        fault_log(0x1A);        //Log invalid port error.
        FtempIndex[serial_port] = 0;
        return;
    }
    FtempIndex[serial_port] = 0;
    tx_float[serial_port] = 0;
    tx_temp[serial_port] = 0;

    float_out[serial_port][0] = ' ';        //Put a SPACE in first char
    if (f_data < 0){
        f_data *= -1;                       //Convert to absolute value.
        float_out[serial_port][0] = '-';    //Put a - in first char
    }
    if (f_data > 9999.999){
        f_data = 9999.999;      //truncate it
        float_out[serial_port][0] = '?';    //Put a ? in first char if truncated
    }
    tx_float[serial_port] = f_data / 1000;
    FtempIndex[serial_port]++;
    while (FtempIndex[serial_port] <= 8){
        if (FtempIndex[serial_port] == 5){
            float_out[serial_port][5] = '.';
            FtempIndex[serial_port]++;
        }
        tx_temp[serial_port] = tx_float[serial_port];
        float_out[serial_port][FtempIndex[serial_port]] = tx_temp[serial_port] + 48;
        FtempIndex[serial_port]++;
        tx_float[serial_port] = (tx_float[serial_port] - tx_temp[serial_port]) * 10;
    }

/* Write to buffer */
    if(writingbuff[serial_port]){
        return;
    }
    writingbuff[serial_port] = 1;
    ifzero[serial_port] = 1;
    while (FtempIndex[serial_port] <= 8){
        //If config_space == 1, then load buffer with only spaces.
        if(float_out[serial_port][FtempIndex[serial_port]] == 0x30 && ifzero[serial_port]){
            if(config_space[serial_port]){
                Buffer[serial_port][Buff_index[serial_port]] = 0x20;
                if (Buff_index[serial_port] < 49)
                    Buff_index[serial_port]++;
                FtempIndex[serial_port]++;
            }
            else{
                FtempIndex[serial_port]++;
            }
        }
        else{
            Buffer[serial_port][Buff_index[serial_port]] = float_out[serial_port][FtempIndex[serial_port]];
            //Do not overrun the buffer.
            if (Buff_index[serial_port] < 49)
                Buff_index[serial_port]++;
            FtempIndex[serial_port]++;
        }
        if(float_out[serial_port][FtempIndex[serial_port]] != 0x30 && float_out[serial_port][FtempIndex[serial_port]] != 0x2D && float_out[serial_port][FtempIndex[serial_port]] != 0x20){
            ifzero[serial_port] = 0;
        }
    }
    FtempIndex[serial_port] = 0;
    config_space[serial_port] = 0;
    Buff_count[serial_port] = Buff_index[serial_port];
    writingbuff[serial_port] = 0;
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
