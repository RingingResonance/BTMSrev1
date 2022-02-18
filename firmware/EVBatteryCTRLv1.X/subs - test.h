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

#ifndef SUBSYS_H
#define	SUBSYS_H


extern void explody_preventy_check(void);
extern void currentCheck(void);
extern void heater_calibration(void);
extern void fault_log(int);
extern void general_shutdown(void);
extern void io_off(void);
extern void regulate(void);
extern void config_save(void);
extern void config_load(void);
extern void calc_save(void);
extern void calc_load(void);
extern void current_cal(void);
extern void volt_percent(void);
extern void print_vars(int);
extern void reset_check(void);
extern void hud_vars(int);
extern void heat_control(float);
extern void POS_disp_init(void);
extern void POS_diag_display(void);
extern void POS_dash_display(void);
extern void main_power_check(void);
extern void first_check(void);
extern void power_off(void);
extern void analog_sanity(void);
extern int cfig_space(void);
extern int var_space(void);
int set_save(int);
void read_sets(int);
int var_save(int);
void read_romvars(int);

/*****************************/
/* Init vars and stuff. */
/* Temperatures are in C */
/*****************************/
//Analog input constants
#define   R1_resistance (*((volatile float *) 0x0800))            //R1 resistance in Kohms
#define   R2_resistance (*((volatile float *) 0x0804))             //R2 resistance in Kohms
#define   bt_vlt_adjst (*((volatile float *) 0x0808))               //battery voltage input compensation in volts.
/*****************************/
//Battery Ratings and setpoints
#define   partial_charge (*((volatile float *) 0x080C))            //Percentage of voltage to charge the battery up to. Set to 0 to disable.
#define   max_battery_voltage (*((volatile float *) 0x0810))    //Max battery voltage before shutdown.     
#define   battery_rated_voltage (*((volatile float *) 0x0814))           //Target max charge voltage
#define   dischrg_voltage (*((volatile float *) 0x0818))        //Minimum battery voltage
#define   low_voltage_shutdown (*((volatile float *) 0x081C))    //Battery Low Total Shutdown Voltage
#define   dischrg_C_rating (*((volatile float *) 0x0820))           //Discharge C rating
#define   limp_current (*((volatile float *) 0x0824))              //Limp mode current in amps
#define   chrg_C_rating (*((volatile float *) 0x0828))          //Charge C rating.
#define   amp_hour_rating (*((volatile float *) 0x082C))         //Battery amp hour rating.
#define   over_current_shutdown (*((volatile float *) 0x0830))        //Shutdown current. Sometimes the regulator isn't fast enough and this happens.
#define   absolute_max_current (*((volatile float *) 0x0834))      //Max regulating current.
//Charge temps.
#define   chrg_min_temp (*((volatile float *) 0x0838))          //Battery minimum charge temperature. Stop Charging at this temp.
#define   chrg_reduce_low_temp (*((volatile float *) 0x083C))      //Reduce charge current when lower than this temp.
#define   chrg_max_temp (*((volatile float *) 0x0840))          //Battery max charge temp. Stop charging at this temp.
#define   chrg_reduce_high_temp (*((volatile float *) 0x0844)) //Reduce charge current when higher than this temp.
#define   chrg_target_temp (*((volatile float *) 0x0848))      //Battery heater charge target temp. Keeps us nice and warm in the winter time.
//Discharge temps.
#define   dischrg_min_temp (*((volatile float *) 0x084C))       //Battery minimum discharge temperature.
#define   dischrg_reduce_low_temp (*((volatile float *) 0x0850))    //Reduced current discharge low temperature.
#define   dischrg_max_temp (*((volatile float *) 0x0854))       //Battery max discharge temperature.
#define   dischrg_reduce_high_temp (*((volatile float *) 0x0858))       //Battery reduced discharge high temperature.
#define   dischrg_target_temp (*((volatile float *) 0x085C))      //Battery heater discharge target temp. Keeps us nice and warm in the winter time.
//Shutdown temps.
#define   battery_shutdown_temp (*((volatile float *) 0x0860))      //Max battery temp before shutting down everything.
#define   ctrlr_shutdown_temp (*((volatile float *) 0x0864))        //Max motor or motor controller temp shutdown.
//Fan ctrl temps.
#define   ctrlr_fan_start (*((volatile float *) 0x0868))               //Turns on cooling fan.
#define   batt_fan_start (*((volatile float *) 0x086C))
//Some other stuff.
#define   max_heat (*((volatile float *) 0x0870))              //Heater watts that you want to use.
#define   travel_dist (*((volatile float *) 0x0874))         //Travel Distance in CM per tire rotation.
#define   circuit_draw (*((volatile float *) 0x0878))        //Amount of current that Yeti himself draws. Used for current calibration.
#define   PowerOffAfter (*((volatile int *) 0x087C))      //Power off the system after this many minutes of not being plugged in or keyed on. 120 minutes is 2 hours.
//***************************************************************************************
/*****************************/
/*****************************/
/*****************************/
/*****************************/
#define  flash_chksum_old  (*((volatile int *) 0x087E))           //System Flash Checksum as stored in NV-mem
// Calculated Battery Ratings
#define  battery_capacity (*((volatile double *) 0x0880))           //Calculated total battery capacity in ah
#define  absolute_battery_usage (*((volatile float *) 0x0888))     //Max total power used from battery.
#define  partial_chrg_cnt (*((volatile int *) 0x088C))           //How many times have we plugged in the charger since the last full charge?
#define  voltage_percentage_old (*((volatile float *) 0x088E))     //Voltage percentage from the last time we where on.
#define  battery_usage (*((volatile double *) 0x0892))              //Calculated Ah usage in/out of battery
#define  battery_usage_smll (*((volatile double *) 0x089A))
#define  battery_remaining (*((volatile double *) 0x08A2))          //Calculated remaining capacity in battery.
#define  battery_remaining_smll (*((volatile double *) 0x08AA))
/*****************************/
/* End of SAVE vars. */
/*****************************/
/*****************************/
// Fault Codes.
int fault_codes[10];
int fault_count = 0;
// Calculated battery values. These don't need to be saved on shutdown.
float   chrge_rate = 0;             //calculated charge rate based off temperature
float   vltg_dvid = 0;              //Value for calculating the ratio of the input voltage divider.
float   calc_125 = 0.00003472;          //Value for calculating the total current in and out of battery.
int     ADCON3upper8 = 0;
int     ADCON3lower8 = 0;
double analog_smpl_time = 0;
float crnt_proportion = 70000;
double crnt_error = 0;
double crnt_integral = 0;
float vltg_proportion = 70000;
double current_output = 0;
float temp_proportion = 70000;
double voltage_output = 0;
float   watts;                  //watts in or out of battery.
float   temp_dischrg_rate;      //calculated discharge rate based off temp and capacity remaining. Is in %
float   max_chrg_current;       //calculated charge rate based off temp and capacity remaining. Is in %
float   chrg_voltage;
float   chrg_percent;           //Percentage of battery charge
int     heat_set;               //Calculated heater output for wattage chosen by user.
float   current_compensate;     //Current compensation.
int     heat_cal_stage = 0;     //0 - 4, stage 0 = not run, set 1 to start, stage 2 = in progress, stage 3 = completed, 4 is Error. 5 is disable heater.
int     zero_current = 0;
float   peak_power = 0;
float   peak_pwr_vlts = 0;
float   peak_pwr_crnt = 0;
/*****************************/
//Battery Sensor Input
float   battery_temp = 0;           //Battery Temperature
float   my_temp = 0;                //Controller board Temperature
float   motor_ctrl_temp = 0;         //Motor or Motor controller Temperature
float   battery_voltage = 0;        //Battery voltage
float   open_voltage = 0;           //Battery Open Circuit Voltage
float   voltage_percentage = 0;     //Battery Voltage Percentage.
float   battery_current = 0;        //Battery charge/discharge current
/*****************************/
/* General Vars */
int got_open_voltage = 0;
int oc_shutdown_timer = 0;
float dischr_current = 0;
int p_charge = 0;
float speed = 0;         //How fast are we going?
float wheelTime = 0;       //Time it takes for a single rotation of the wheel.
int wheelSpin = 0;      //Is the wheel spinning?
int soft_power = 0;
int main_power = 0;
int pwr_detect = 0;
int init_done = 0;
int lw_pwr = 0;
int deep_sleep = 0;
int analog_avg_cnt = 0;
double adctemp0 = 0;
double adctemp1 = 0;
double adctemp2 = 0;
double adctemp3 = 0;
double adctemp4 = 0;
int crnt_avg_cnt = 0;
int vltg_avg_cnt = 0;
float battery_crnt_average = 0;        //Battery charge/discharge current
float battery_vltg_average = 0;
double bt_crnt_avg_temp = 0;
double bt_vltg_avg_temp = 0;
int adc_sample_burn = 0;            //Burn it. Don't touch this var it will burn you if you do.
int read_vars = 0;
int heat_rly_timer = 3;     //3 is resting, setting to 2 starts the countdown, 0 = relay is ready
int contact_rly_timer = 3;
int chrg_rly_timer = 3;
int curnt_cal_stage = 0;     
/* 0 - 4, stage 0 = not run, set 1 to start, stage 2 = in progress, stage 3 = completed, 4 is Error. 
 */
int fault_shutdown = 0;       //General shutdown event.
int cmd_power = 0;
int b_safe = 1; 
/* Always start up with this != 0x55FF. It should only ever be set to 0x55FF 
 * when in a controlled debugging environment because this disables 
 * explody-preventy-check and other shutdown systems.
 */
int error_blink = 0;
int power_session = 1;
int power_plugged = 0;
int shutdown_timer = 0;
int first_cal = 0;
int chrg_check = 0;
int start_timer = 0;
int osc_fail_event = 0;
int hunter = 0;
int dispinit = 0;
int err_scroll = 0;
int diagmode = 0;
int diag_count = 0;
int diag_state = 0;
int charger_detected = 0;       //Used for confirming the charger is plugged in.
int EnableChIRQ = 1;            //By default, enable charge detect IRQ on init.
int PowerOffTimer = 0;
int PowerOffTimerSec = 59;      //default state.
int cfg_space = 0;
int vr_space = 0;
int v_test = 0;
/*****************************/
//Control Output
int     output_power = 0;          //output power
int     charge_power = 0;           //charge rate
int     heat_power = 0;             //heater power
/*****************************/


#endif	/* SUBS_H */

