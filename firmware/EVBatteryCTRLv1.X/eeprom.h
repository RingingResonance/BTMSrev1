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

#ifndef eeprom_H
#define	eeprom_H

extern int eeprom_erase(int);   //address. 0x00 through 0x1FF.      Returns 0 on success.
extern int eeprom_write(int, int);  //address, data.    Returns 0 on success.
extern int eeprom_read(int);    //address. Returns data or 0 on failure.
extern void save_sets(void);
extern int read_sets(void);
extern void save_vars(void);
extern int read_vars(void);
extern void get_variables(void);
void get_settings(void);

int upperMem = 0;
int printSets = 0;
int eRead = 0;
char offset = 0;
int address = 0;

#endif	/* DATAIO_H */

