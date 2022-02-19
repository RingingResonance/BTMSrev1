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

#ifndef chksum_H
#define	chksum_H

void flash_checksum(void);
void eeprom_checksum(void);
int check_prog(void);
int check_nvmem(void);
void nvmSets_checksum(void);
void ramSets_checksum(void);
int check_ramSets(void);
int check_nvmSets(void);
void ram_chksum_update(void);
void prgm_chksum_update(void);
void nvm_chksum_update(void);

int flash_chksum = 0;
int rom_chksum = 0;
int nvSets_chksum = 0;
int ramSets_chksum = 0;
int ramSets_chksum_old = 0;
int check_timer = 0;
int ram_err_count = 0;

#endif	/* DATAIO_H */

