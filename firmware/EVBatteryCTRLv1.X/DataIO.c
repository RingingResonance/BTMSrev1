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

void Buffrst(int serial_port){
    if (Buff_index[serial_port] >= Buff_count[serial_port]){
            Buff_index[serial_port] = 0;
            Buff_count[serial_port] = 0;
            portBSY[serial_port] = 0;//Inhibits writing to buffer while the serial port is transmitting buffer.
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

void port_check(int serial_port){
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
}

//Loads ascii HEX into buffer.
void load_hex(int numb, int serial_port){
    port_check(serial_port);
    nibble[serial_port][0] = (numb & 0xF000)/4096;
    nibble[serial_port][1] = (numb & 0x0F00)/256;
    nibble[serial_port][2] = (numb & 0x00F0)/16;
    nibble[serial_port][3] = (numb & 0x000F);
    writingbuff[serial_port] = 1;
    Buffer[serial_port][Buff_index[serial_port]] = '0';
    Buff_index[serial_port]++;
    Buffer[serial_port][Buff_index[serial_port]] = 'x';
    Buff_index[serial_port]++;
    int x;
    for(x=0;x<4;x++){
        Buffer[serial_port][Buff_index[serial_port]] = four_bit_hex_cnvt(nibble[serial_port][x]);
        if (Buff_index[serial_port] < bffize-1)
            Buff_index[serial_port]++;
    }
    Buff_count[serial_port] = Buff_index[serial_port];
    writingbuff[serial_port] = 0;
}
//Start the data transfer from one of the buffers to the selected serial port
//Dispatch the data in the buffers to the display by creating a TX IRQ
void dispatch_Serial(int serial_port){
    if(portBSY[serial_port])return;             //If port it busy, don't dispatch a second time.
    portBSY[serial_port] = 1;
    Buff_index[serial_port] = 0;                //Start Index at 0.
    if(serial_port) IFS1bits.U2TXIF = 1;        //Start transmitting by manually sending an IRQ.
    else IFS0bits.U1TXIF = 1;                   //Start transmitting by manually sending an IRQ.
}

//Send a string of text to a buffer that can then be dispatched to a serial port.
void load_string(const char *string_point, int serial_port){
    port_check(serial_port);
    writingbuff[serial_port] = 1;
    StempIndex[serial_port] = 0;
    while (string_point[StempIndex[serial_port]] != 0){
        Buffer[serial_port][Buff_index[serial_port]] = string_point[StempIndex[serial_port]];
        if (Buff_index[serial_port] < bffize-1)
            Buff_index[serial_port]++;
        StempIndex[serial_port]++;
    }
    Buff_count[serial_port] = Buff_index[serial_port];
    writingbuff[serial_port] = 0;
}
//Copy float data to buffer.
void cpyFLT(int serial_port){
    Buffer[serial_port][Buff_index[serial_port]] = float_out[serial_port][FtempIndex[serial_port]];
    //Do not overrun the buffer.
    if (Buff_index[serial_port] < bffize-1)Buff_index[serial_port]++;
}
/* Sends a float to buffer. */
void load_float(float f_data, int serial_port){
    FtempIndex[serial_port] = 0;
    tx_float[serial_port] = 0;
    tx_temp[serial_port] = 0;
    int extSpc = 0;
    
    float_out[serial_port][0] = ' ';
    if (f_data < 0){
        f_data *= -1;                       //Convert to absolute value.
        float_out[serial_port][0] = '-';    //Put a - in first char
        extSpc = 1;
    }
    if (f_data > 9999.999){
        f_data = 9999.999;      //truncate it if it's too big of a number.
        float_out[serial_port][0] = '?';    //Put a ? in first char if truncated.
        extSpc = 1;
    }

    tx_float[serial_port] = f_data / 1000;
    FtempIndex[serial_port]++;
    while (FtempIndex[serial_port] <= 8){
        if (FtempIndex[serial_port] == 5){
            float_out[serial_port][5] = '.';    //Put a decimal at the 5th spot.
            FtempIndex[serial_port]++;          //Got to 6th spot after.
        }
        tx_temp[serial_port] = tx_float[serial_port];   //Truncate anything on the right side of the decimal.
        float_out[serial_port][FtempIndex[serial_port]] = tx_temp[serial_port] + 48; //Convert it to ASCII.
        FtempIndex[serial_port]++;                                                   //Go to next ASCII spot.
        tx_float[serial_port] = (tx_float[serial_port] - tx_temp[serial_port]) * 10; //Shift the float left by 1
    }

/* Write to buffer */
    FtempIndex[serial_port] = 0;
    port_check(serial_port);        //Check serial port to see if it's busy before writing to buffer.
    writingbuff[serial_port] = 1;   //Tell other processes we are busy with the buffer.
    while (FtempIndex[serial_port] < 9){
        //If 0, do not copy unless it's 00.000 or a '-' && config_space is 1
        while(float_out[serial_port][FtempIndex[serial_port]] == '0' && 
        !config_space[serial_port] && FtempIndex[serial_port] < 3){
            FtempIndex[serial_port]++;
        }
        //Copy data. Check for dynamic spacing for sign char.
        if(dynaSpace[serial_port]){ 
            switch(float_out[serial_port][FtempIndex[serial_port]]){
                case '-':
                    cpyFLT(serial_port);
                case ' ':
                    FtempIndex[serial_port]++;
                break;
            }
            dynaSpace[serial_port] = 0;
        }
        else{
            cpyFLT(serial_port);
            FtempIndex[serial_port]++;
        }
        //Clear 'ifzero' if anything but '0', '-', or ' ' found.
        if(float_out[serial_port][FtempIndex[serial_port]] != '0' && 
        float_out[serial_port][FtempIndex[serial_port]] != '-' && 
        float_out[serial_port][FtempIndex[serial_port]] != ' '){
            config_space[serial_port] = 0;
        }
    }
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
