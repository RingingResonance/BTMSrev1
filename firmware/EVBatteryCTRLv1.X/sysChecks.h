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

#ifndef SYSCHKS_H
#define	SYSCHKS_H

#define fCalReady 3
#define fCalTimer fCalReady - 1

extern void explody_preventy_check(void);
extern void currentCheck(void);
extern void heater_calibration(void);
extern void fault_log(int);
extern void general_shutdown(void);
extern void io_off(void);
extern void reset_check(void);
extern void hud_vars(int);
extern void main_power_check(void);
extern void first_check(void);
extern void analog_sanity(void);
extern void death_loop(void);
extern void initialCal(void);
extern void chargeDetect(void);

/*****************************/
/* Init vars and stuff. */
/* Temperatures are in C */
/******************************/
int heat_set;               //Calculated heater output for wattage chosen by user.
int oc_shutdown_timer = 0;
int shutdown_timer = 0;
#endif	/* SUBS_H */

