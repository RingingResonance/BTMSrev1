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
#define bffize 60

extern unsigned int BaudCalc(double, double);
extern void load_float(float, int);
extern void load_string(const char*, int);
extern void dispatch_Serial(int);
extern char four_bit_hex_cnvt(int);
extern void Buffrst(int);
void load_hex(int, int);

//Serial port stuff
#define Clength 4   //Command Buff Length - 1
char CMD_buff[2][5];              //Command Buffer
unsigned int CMD_Point[2] = {0,0};  //Command Pointer
char cmdRDY[2] = {0,0};
char cmdOVFL[2] = {0,0};
char Lecho[2] = {1,1};              //Local echo is on by default
char Terr[2] = {0,0};
char bufsize[2]= {0,0};

char nibble[2][4];
char float_out[2][7];
char  Buff_count[2] = {0,0};
int  Buff_index[2] = {0,0};
char Buffer[2][bffize];
int  StempIndex[2] = {0,0};
int  FtempIndex[2] = {0,0};
char config_space[2] = {0,0};
float tx_float[2] = {0,0};
char tx_temp[2] = {0,0};
char portBSY[2] = {0,0};
char writingbuff[2] = {0,0};
char dynaSpace[2] = {0,0};

#endif	/* DATAIO_H */

