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

#ifndef eeprom_C
#define eeprom_C

#include <p30f3011.h>
#include "eeprom.h"
#include "checksum.h"
#include "common.h"

void get_variables(void){
    int var_exist = eeprom_read(cfg_space + 1);
    if(var_exist == 0x7654) {
        read_vars();
    }
}

void get_settings(void){
    int var_exist = eeprom_read(0x0000);
    if(var_exist == 0x4567){
        int x = 0;
        for(;;){
            if(!read_sets()){
                //Attempt to copy data no more than 5 times.
                if(x >= 5)death_loop();//If settings are read back but the EEPROM shows corruption, we cannot trust the settings, nor can we trust defaults.
                x++;
            }
            else break;
        }
    }
    else if(var_exist == 0xFFFF){
        //If no settings were previously stored in EEPROM then it should be safe to load defaults.
        prgm_chksum_update();//If a first read is detected, update the prog_checksum.
        default_sets();
        save_sets();//Save sets including program memory checksum to eeprom.
    }
}
//Save settings to EEPROM
void save_sets(void){
    int x;
    for(x=0;x<(cfg_space);x++){
        eeprom_write(x, sets.settingsArray[x]);
    }
    //Write this in memory so we know we have written data at least once before.
    eeprom_write(0x0000, 0x4567);
    nvm_chksum_update();
}
//Read settings from EEPROM
//Returns 1 on success.
int read_sets(void){
    if(check_nvmem())return 0;          //First check to make sure EEPROM checksum hasn't changed since last write.
    int x;
    for(x=0;x<(cfg_space);x++){
        sets.settingsArray[x] = eeprom_read(x);
    }
    //Now generate and compare checksums of settings ram and settings EEPROM to ensure a correct copy of data.
    if(check_nvmSets())return 0;
    //Update the settings ram checksum.
    ram_chksum_update();
    return 1;
}
//Save vars to EEPROM
void save_vars(void){
    int x;
    int caddr = cfg_space + 1;
    for(x=0;x<(vr_space);x++){
        eeprom_write(x + caddr, vars.variablesArray[x]);
    }
    //Write this in memory so we know we have written data at least once before.
    eeprom_write(caddr,0x7654);
    nvm_chksum_update();
    CONDbits.failSave = 0;
}
//Read vars from EEPROM
//Returns 1 on success
int read_vars(void){
    if(check_nvmem())return 0;
    int x;
    int caddr = cfg_space + 1;
    for(x=0;x<(vr_space);x++){
        vars.variablesArray[x] = eeprom_read(x + caddr);
    }
    return 1;
}

/**********************/
/* EEPROM Interaction */
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

#endif
