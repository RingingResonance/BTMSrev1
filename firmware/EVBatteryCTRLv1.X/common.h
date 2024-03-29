/*Copyright (c) <2024> <Jarrett Cigainero>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef COMMON_H
#define	COMMON_H

extern void current_cal(void);
extern void volt_percent(void);
extern void reset_check(void);
extern void power_off(void);
extern float absFloat(float);
extern void sysReady(void);
extern void calcAnalog(void);

/* NOTE: Try to keep memory usage below about 75% for the dsPIC30F3011 as the stack can use as much as 15% */
/*****************************/
/* Init vars and stuff. */
/* Temperatures are in C */
/*********;********************/
#pragma pack(1)
struct Settings{
    //Analog input constants
    int     settingsArray[1];
    float   R1_resistance;              //R1 resistance in Kohms
    float   R2_resistance;              //R2 resistance in Kohms
    float   bt_vlt_adjst;               //battery voltage input compensation in volts.
    /*****************************/
    //Battery Ratings and setpoints
    float   partial_charge;             //Percentage of voltage to charge the battery up to. Set to 0 to disable.
    float   max_battery_voltage;        //Max battery voltage before shutdown.
    float   battery_rated_voltage;      //Target max charge voltage
    float   dischrg_voltage;            //Minimum battery voltage
    float   low_voltage_shutdown;       //Battery Low Total Shutdown Voltage
    float   dischrg_C_rating;           //Discharge C rating
    float   limp_current;               //Limp mode current in amps
    float   chrg_C_rating;              //Charge C rating.
    float   amp_hour_rating;            //Battery amp hour rating.
    float   over_current_shutdown;      //Shutdown current. Sometimes the regulator isn't fast enough and this happens.
    float   absolute_max_current;       //Max regulating current.
    //Charge temps.
    float   chrg_min_temp;              //Battery minimum charge temperature. Stop Charging at this temp.
    float   chrg_reduce_low_temp;       //Reduce charge current when lower than this temp.
    float   chrg_max_temp;              //Battery max charge temp. Stop charging at this temp.
    float   chrg_reduce_high_temp;      //Reduce charge current when higher than this temp.
    float   chrg_target_temp;           //Battery heater charge target temp. Keeps us nice and warm in the winter time.
    //Discharge temps.
    float   dischrg_min_temp;           //Battery minimum discharge temperature.
    float   dischrg_reduce_low_temp;    //Reduced current discharge low temperature.
    float   dischrg_max_temp;           //Battery max discharge temperature.
    float   dischrg_reduce_high_temp;   //Battery reduced discharge high temperature.
    float   dischrg_target_temp;        //Battery heater discharge target temp. Keeps us nice and warm in the winter time.
    //Shutdown temps.
    float   battery_shutdown_temp;      //Max battery temp before shutting down everything.
    float   ctrlr_shutdown_temp;        //Max motor or motor controller temp shutdown.
    //Fan ctrl temps.
    float   ctrlr_fan_start;            //Turns on cooling fan.
    float   batt_fan_start;
    //Some other stuff.
    float   max_heat;           //Heater watts that you want to use.
    float   travel_dist;        //Travel Distance in CM per tire rotation.
    float   circuit_draw;       //Amount of current that Yeti himself draws. Used for current calibration.
    unsigned int     PowerOffAfter;      //Power off the system after this many minutes of not being plugged in or keyed on. 120 minutes is 2 hours.
    unsigned int     flash_chksum_old;   //System Flash Checksum as stored in NV-mem
    char    PxVenable[2];
    char    custom_data1[6];    //4 blocks of 6 chars of custom user text or data, can be terminated by a NULL char. (0xFC - 0xFF)
    char    custom_data2[6];
    char    custom_data3[6];
    char    custom_data4[6];
    char    page[2][4][6];      //Display page holder. (port)(Page#)(Variable to Display: A '0' at the start = Skip Page)
    char    pageDelay[2][4];    //Page delay. (Page#Delay in 1/8 seconds) 0 = full speed.
    int     testBYTE;
}sets;

struct Variables{
    int     variablesArray[1];
// Calculated Battery Ratings
    float   battery_capacity;           //Calculated total battery capacity in ah
    float   absolute_battery_usage;     //Max total power used from battery.
    float   voltage_percentage_old;     //Voltage percentage from the last time we where on.
    float   battery_usage;              //Calculated Ah usage in/out of battery
    float   battery_remaining;          //Calculated remaining capacity in battery.
    // Fault Codes.
    unsigned int     fault_codes[10];
    unsigned int     fault_count;
    // Other stuff.
    unsigned int     partial_chrg_cnt;           //How many times have we plugged in the charger since the last full charge?
    char     heat_cal_stage;             //0 - 4, stage 0 = not run, set 1 to start, stage 2 = in progress, stage 3 = completed, 4 is Error. 5 is disable heater.
    char     testBYTE;
}vars;

const char VSPC[] = " ";
const char V01[] = "MPH";
const char V02[] = "%";
const char V03[] = "V";
const char V04[] = "TV";
const char V05[] = "PW";
const char V06[] = "PWV";
const char V07[] = "AV";
const char V08[] = "W";
const char V09[] = "A";
const char V0A[] = "C";
const char V0D[] = "PWA";
const char V0E[] = "AA";
const char V0F[] = "OCV";
const char V10[] = "\n\r";  //Newline + Return

const char * const Vlookup[] = {VSPC,V01,V02,V03,V04,V05,V06,V07,V08,V09,V0A,V0A,V0A,V0D,V0E,V0F,V10};

//Variables that can be sent out the serial port.
struct dskyvars{
    int     dskyarray[1];           //For sending raw data
    float   dskyarrayFloat[1];      //For sending float data to display
    float   speed;                  //1: 8char 00.0xxx How fast are we going?
    float   chrg_percent;           //2: 6char 00.0% Percentage of battery charge
    float   battery_voltage;        //3: 6char 00.0V Battery voltage
    float   chrg_voltage;           //4: 7char 00.0TV Charge Target Voltage
    float   peak_power;             //5: 7char 00.0PW Peak output power
    float   peak_pwr_vlts;          //6: 8char 00.0PWV Voltage at peak output power
    float   battery_vltg_average;   //7: 7char 00.0AV Battery average voltage
    float   watts;                  //8: 7char -0000W watts in or out of battery.
    float   battery_current;        //9: 7char -00.0A Battery charge/discharge current
    float   battery_temp;           //A: 7char -00.0C Battery Temperature
    float   my_temp;                //B: 7char -00.0C Controller board Temperature
    float   motor_ctrl_temp;        //C: 7char -00.0C Motor or Motor controller Temperature
    float   peak_pwr_crnt;          //D: 9char -00.0PWA Current at peak output power
    float   battery_crnt_average;   //E: 8char -00.0AA Battery charge/discharge average current
    float   open_voltage;           //F: 9char -00.0OCV Battery Open Circuit Voltage
}dsky;
#define varLimit 0x0010

// Calculated battery values. These don't need to be saved on shutdown.
float   chrge_rate = 0;             //calculated charge rate based off temperature
float   vltg_dvid = 0;              //Value for calculating the ratio of the input voltage divider.
int     ADCON3upper8 = 0;
int     ADCON3lower8 = 0;
/*****************************/
/* General Vars */
float voltage_percentage;     //Battery Open Circuit Voltage Percentage.
float current_compensate;     //Current compensation.
float dischr_current = 0;
float wheelTime = 0;       //Time it takes for a single rotation of the wheel.
float avgVolt = 0;     //averaged voltage input
float avgCurnt = 0;    //averaged current input
float avgBTemp = 0;    //averaged battery temperature
float avgMTemp = 0;    //averaged motor temperature
float avgSTemp = 0;    //averaged self temperature
float bt_crnt_avg_temp = 0;
float bt_vltg_avg_temp = 0;
char avg_cnt = 0;
char analog_avg_cnt = 0;
char heat_rly_timer = 3;     //3 is resting, setting to 2 starts the countdown, 0 = relay is ready
char contact_rly_timer = 3;
char chrg_rly_timer = 3;
char curnt_cal_stage = 0;
/* 0 - 4, stage 0 = not run, set 1 to start, stage 2 = in progress, stage 3 = completed, 4 is Error.
 */
char power_session = 1;
char start_timer = 0;
char diag_count = 0;
char PowerOffTimer = 0;
char PowerOffTimerSec = 59;      //default state.
char cfg_space = 0;
char vr_space = 0;
char dsky_space = 0;
char v_test = 0;
char first_cal = 0;
/*****************************/
//Control Output
unsigned int     output_power = 0; //output power
unsigned int     charge_power = 0; //charge rate
unsigned int     heat_power = 0;   //heater power
/*****************************/

/* Boolean Variables */
//Conditions.
#define COND COND
volatile unsigned int COND;
typedef struct tagCONDBITS {
  unsigned error_blink:1;
  unsigned wheelSpin:1; //Is the wheel spinning?
  unsigned soft_power:1;
  unsigned main_power:1;
  unsigned pwr_detect:1;
  unsigned cmd_power:1;
  unsigned charger_detected:1; //Used for confirming the charger is plugged in.
  unsigned diagmode:1;
  unsigned diag_state:1;
  unsigned EnableChIRQ:1; // = 1  //By default, enable charge detect IRQ on init.
  unsigned got_open_voltage:1;
  unsigned failSave:1;
  unsigned chkInProgress:1;
  unsigned gotBatteryAmps:1;
  unsigned P1display:1;
  unsigned P2display:1;
} CONDBITS;
volatile CONDBITS CONDbits;

#define STING STING
volatile unsigned int STING;
typedef struct tagSTINGBITS {
  unsigned lw_pwr:1;
  unsigned deep_sleep:1;
  unsigned zero_current:1;
  unsigned adc_sample_burn:1; //Burn it. Don't touch this var it will burn you if you do.
  unsigned init_done:1;
  unsigned fault_shutdown:1; //General shutdown event.
  unsigned osc_fail_event:1;
  unsigned p_charge:1;
  unsigned sw_off:1;
} STINGBITS;
volatile STINGBITS STINGbits;

#endif	/* SUBS_H */

