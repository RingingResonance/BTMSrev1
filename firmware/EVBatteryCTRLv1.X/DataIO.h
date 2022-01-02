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

#define PORT1 1
#define PORT2 2
#define BigBuffer 3
#define simple 1
#define full 0
#define TXT 0
#define NLtxt 1
#define txtNL 2
#define NLtxtNL 3

extern unsigned int BaudCalc(double, double);
extern void float_send(float, int);
extern void nl_send(int);
extern void return_send(int);
extern void send_string(int, const char*, int);
extern void data_send(char, int);
extern char four_bit_hex_cnvt(int);
extern int eeprom_erase(int);   //address. 0x00 through 0x1FF.      Returns 0 on success.
extern int eeprom_write(int, int);  //address, data.    Returns 0 on success.
extern int eeprom_read(int);    //address. Returns data or 0 on failure.


// Tm = 32767 * (1 / (((clkSpeedmhz * PLL) / 4) / tiksPerIRQ))
//#define IPS 29.48;   //million instructions per second.
float IPS = 14.74;   //million instructions per second.
//Don't define baud rate because we might want to add the option of changing it later during runtime.
float BAUD1 = 9600;     //BAUD rate for PORT 1
float BAUD2 = 9600;     //BAUD rate for PORT 2
//int PORT1 = 0x01;
//int PORT2 = 0x01;

/* Command vars and stuff. */
//Port 1 stuff
static int cdmd = 10;   //Command Buff Length
char cmd[10];              //Command Buffer
unsigned int cmdPoint = 0;  //Command Pointer
int cmdRDY = 0;
int cmdOVFL = 0;
int Lecho = 1;              //Local echo is on by default on PORT 1
int Terr = 0;
int bufsize = 0;
//Port 2 stuff
/*
static int cdmd2 = 10;   //Command Buff Length
char cmd2[10];              //Command Buffer
unsigned int cmdPoint2 = 0;  //Command Pointer
int cmdRDY2 = 0;
int cmdOVFL2 = 0;
int Lecho2 = 0;              //Local echo is off by default on PORT 2
int Terr2 = 0;
int bufsize2 = 0;
*/
int config_space = 0;

char float_out[7];

int  CBuff_max_data = 0;
int  CBuff_index = 0;
char Port2_Buffer[50];

#endif	/* DATAIO_H */

