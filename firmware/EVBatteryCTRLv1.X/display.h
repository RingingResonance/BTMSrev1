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

#ifndef DSKY_H
#define	DSKY_H

extern void Command_Interp(int);
extern void fault_read(int);
extern void displayOut(int);

unsigned int CMD_Point[2];  //Command Pointer
unsigned int bufsize[2];
char CMD_buff[2][6];              //Command Buffer
char cmdRDY[2] = {0,0};
char cmdOVFL[2] = {0,0};
char Lecho[2] = {1,1};              //Local echo is on by default
char Terr[2] = {0,0};

unsigned int tempPoint[2];
unsigned int flt_index[2];
unsigned int PxPage[2];
unsigned int pageVar[2];
unsigned int varNum[2];
unsigned int dodispatch[2];
char PxVtimer[2];

#endif	/* DATAIO_H */

