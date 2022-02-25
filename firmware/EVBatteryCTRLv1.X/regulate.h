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

#ifndef REGULATE_H
#define	REGULATE_H

extern void temperatureCalc(void);
extern void outputReg(void);
extern void chargeReg(void);
extern void heat_control(float);

float dischrg_current = 0;
float chrg_current = 0;
float dischrg_read = 0;
float analog_smpl_time = 0;
float crnt_proportion = 70000;
float crnt_error = 0;
float crnt_integral = 0;
float vltg_proportion = 70000;
float current_output = 0;
float temp_proportion = 70000;
float voltage_output = 0;

#endif	/* SUBS_H */

