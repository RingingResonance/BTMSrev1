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

#ifndef DATAIO_H
#define	DATAIO_H

#define PORT1 0
#define PORT2 1
#define simple 1
#define full 0

extern unsigned int BaudCalc(double, double);
extern void load_float(float, int);
extern void load_string(const char*, int);
extern void dispatch_Serial(int);
extern char four_bit_hex_cnvt(int);
extern int eeprom_erase(int);   //address. 0x00 through 0x1FF.      Returns 0 on success.
extern int eeprom_write(int, int);  //address, data.    Returns 0 on success.
extern int eeprom_read(int);    //address. Returns data or 0 on failure.
void load_hex(int, int);
void flash_checksum(void);
void eeprom_checksum(void);
void check_prog(void);
void check_nvmem(void);


// Tm = 32767 * (1 / (((clkSpeedmhz * PLL) / 4) / tiksPerIRQ))
//#define IPS 29.48;   //million instructions per second.
float IPS = 14.74;   //million instructions per second.
//Don't define baud rate as static because we might want to add the option of changing it later during runtime.
float BAUD1 = 9600;     //BAUD rate for PORT 1
float BAUD2 = 9600;     //BAUD rate for PORT 2

//Memory stuff
int upperMem = 0;
int flash_chksum = 0;
int rom_chksum = 0;

//Serial port stuff
static int Clength = 10;   //Command Buff Length
char CMD_buff[2][10];              //Command Buffer
unsigned int CMD_Point[2] = {0,0};  //Command Pointer
int cmdRDY[2] = {0,0};
int cmdOVFL[2] = {0,0};
int Lecho[2] = {1,1};              //Local echo is on by default
int Terr[2] = {0,0};
int bufsize[2]= {0,0};

int nibble[2][4];
char float_out[2][7];
int  Buff_count[2] = {0,0};
int  Buff_index[2] = {0,0};
char Buffer[2][50];
int  StempIndex[2] = {0,0};
int  FtempIndex[2] = {0,0};
int config_space[2] = {0,0};
int ifzero[2] = {1,1};
float tx_float[2] = {0,0};
int tx_temp[2] = {0,0};
int portBSY[2] = {0,0};
int writingbuff[2] = {0,0};

#endif	/* DATAIO_H */

