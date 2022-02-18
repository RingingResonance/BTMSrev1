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
        __asm__ volatile ("DISI #0x3FFF");   //Disable all non-critical IRQs.
        IPC2bits.ADIP = 0;  //Disable analog IRQs.
        adrfinal = addrs * 2;
        adrfinal += 0xFC00;
        //Erase location.
        NVMADRU = 0x7F;
        NVMADR = adrfinal;
        __asm__ ("MOV #0x4044,w1");
        __asm__ ("MOV w1,NVMCON");
        __asm__ ("MOV #0x55,w1");
        __asm__ ("MOV w1,NVMKEY");
        __asm__ ("MOV #0xAA,w1");
        __asm__ ("MOV w1,NVMKEY");
        __asm__ ("BSET NVMCON,#0x0F");
        __asm__ ("NOP");
        __asm__ ("NOP");
        while(NVMCONbits.WR) //Wait here until write is finished.
        DISICNT = 0;    //ReEnable all non-critical IRQs.
        IPC2bits.ADIP = 7;  //ReEnable analog IRQs.
        return 0;
    }
}
/** Write a Byte to EEPROM **/
int eeprom_write(int addrs, int data){
    eeprom_erase(addrs);
    if (addrs > 0x1FF){
        return 1;       //error writing, address out of range.
    }
    else{
        __asm__ volatile ("DISI #0x3FFF");   //Disable all non-critical IRQs.
        IPC2bits.ADIP = 0;  //Disable analog IRQs.
        addrs *=  2;
        addrs += 0xFC00;
        //Write to location.
        //DO NOT TOUCH THIS, EVERYTHING MUST BE DONE IN THIS EXACT ORDER OR IT WILL NOT WORK!!!
        TBLPAG = 0x7F;     //ADDRESS UPPER
        //__asm__ ("MOV W1,TBLPAG");
        WREG2 = data; //DATA
        WREG0 = addrs;   //ADDRESS LOWER
        //ESPECIALLY DON'T TOUCH THE ORDER THAT THE WREGs GET WRITTEN TO.
        __asm__ ("TBLWTL w2,[w0]");
        __asm__ ("MOV #0x4004,w1");
        __asm__ ("MOV w1,NVMCON");
        __asm__ ("MOV #0x55,w1");
        __asm__ ("MOV w1,NVMKEY");
        __asm__ ("MOV #0xAA,w1");
        __asm__ ("MOV w1,NVMKEY");
        __asm__ ("BSET NVMCON,#0x0F");
        __asm__ ("NOP");
        __asm__ ("NOP");
        while(NVMCONbits.WR) //Wait here until write is finished.
        DISICNT = 0;        //ReEnable all non-critical IRQs.
        IPC2bits.ADIP = 7;  //ReEnable analog IRQs.
        return 0;
    }
}

int memread(int offset, int address){
    __asm__ volatile ("DISI #0x3FFF");   //Disable all non-critical IRQs.
    IPC2bits.ADIP = 0;  //Disable analog IRQs.
    TBLPAG = offset;    //Offset in memory
    WREG0 = address;    //Address to read from
    __asm__ ("TBLRDH [w0],w4");
    __asm__ ("NOP");
    __asm__ ("NOP");
    upperMem = WREG4;
    TBLPAG = offset;    //Offset in memory
    WREG0 = address;    //Address to read from
    __asm__ ("TBLRDL [w0],w4");
    __asm__ ("NOP");
    __asm__ ("NOP");
    int x = WREG4;
    DISICNT = 0;        //ReEnable all non-critical IRQs.
    IPC2bits.ADIP = 7;  //ReEnable analog IRQs.
    return x; //WREG4 will contain data read from NVmem
}

int eeprom_read(int addrs){
    if (addrs > 0x1FF){
        return 0;       //error reading, address out of range. Return what we hope to be a safe value.
    }
    else{
        return memread(0x7F, (0xFC00 + (addrs * 2)));
    }
}

//Generates checksum of flash and chip config memory.
void flash_checksum(void){
    flash_chksum = 0;
    int xaddr = 0;
    int yaddr = 0;
    //Read program memory space.
    for(xaddr=0;xaddr<0x1FFF;xaddr++){
        flash_chksum += ((0x00FF & memread(0x00,xaddr*2)) + ((0xFF00 & memread(0x00,xaddr*2)) / 256)); 
        flash_chksum += upperMem;
    }
    //Read chip config memory space.
    for(yaddr=0xF8;yaddr<0xFF;yaddr++)
        for(xaddr=0;xaddr<0x7FFF;xaddr++){flash_chksum += ((0x00FF & memread(yaddr,xaddr*2)) + ((0xFF00 & memread(yaddr,xaddr*2)) / 256));}
    flash_chksum += (210 * 2); //No matter what I do, the checksum is always a difference of 420 from what MPLAB says it should be.
}

//Generates checksum of flash and chip config memory.
void eeprom_checksum(void){
    rom_chksum = 0;
    int xaddr = 0;
    //Read program memory space except last byte because that's where the EEPROM's checksum is stored.
    for(xaddr=0;xaddr<0x01FE;xaddr++){
        rom_chksum += ((0x00FF & eeprom_read(xaddr)) + ((0xFF00 & eeprom_read(xaddr)) / 256));
    }
}
void check_prog(void){
    flash_checksum();
    if(vars.flash_chksum_old != flash_chksum){
        fault_log(0x29);    //Log an error if it doesn't match.
    }
}
void check_nvmem(void){
    eeprom_checksum();
    if(eeprom_read(0x01FF) != rom_chksum){
        fault_log(0x2A);    //Log an error if it doesn't match.
    }
}
// just some garbage, ignore.
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
        if (Buff_index[serial_port] < 49)
            Buff_index[serial_port]++;
    }
    Buff_count[serial_port] = Buff_index[serial_port];
    writingbuff[serial_port] = 0;
}
//Start the data transfer from one of the buffers to the selected serial port
//Dispatch the data in the buffers to the display by creating a TX IRQ
void dispatch_Serial(int serial_port){
    portBSY[serial_port] = 1;
    Buff_index[serial_port] = 0;  //Start Index at 0.
    if(serial_port){
        IFS1bits.U2TXIF = 1;        //Start transmitting by manually sending an IRQ.
    }
    else{
        IFS0bits.U1TXIF = 1;        //Start transmitting by manually sending an IRQ.
    }
}

//Send a string of text to a buffer that can then be dispatched to a serial port.
void load_string(const char *string_point, int serial_port){
    port_check(serial_port);
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
    port_check(serial_port);
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
