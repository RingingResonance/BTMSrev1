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

#ifndef INIT_C
#define INIT_C


#include <p30f3011.h>
#include "Init.h"

void default_sets(void){
    config_start[0] = 0x4567;
    R1_resistance = 200;            //R1 resistance in Kohms
    R2_resistance = 16;             //R2 resistance in Kohms
    bt_vlt_adjst = -0.231;               //battery voltage input compensation in volts.
    /*****************************/
    //Battery Ratings and setpoints
    partial_charge = 0.90;            //Percentage of voltage to charge the battery up to. Set to 0 to disable.
    max_battery_voltage = 59.08;    //Max battery voltage before shutdown.
    battery_rated_voltage = 58.8;           //Target max charge voltage
    dischrg_voltage = 39.2;        //Minimum battery voltage
    low_voltage_shutdown = 35;    //Battery Low Total Shutdown Voltage
    dischrg_C_rating = 2;           //Discharge C rating
    limp_current = 5;              //Limp mode current in amps
    chrg_C_rating = 0.5;          //Charge C rating.
    amp_hour_rating = 22;         //Battery amp hour rating.
    over_current_shutdown = 30;        //Shutdown current. Sometimes the regulator isn't fast enough and this happens.
    absolute_max_current = 25;      //Max regulating current.
    //Charge temps.
    chrg_min_temp = 10;          //Battery minimum charge temperature. Stop Charging at this temp.
    chrg_reduce_low_temp = 15;      //Reduce charge current when lower than this temp.
    chrg_max_temp = 40;          //Battery max charge temp. Stop charging at this temp.
    chrg_reduce_high_temp = 35; //Reduce charge current when higher than this temp.
    chrg_target_temp = 25;      //Battery heater charge target temp. Keeps us nice and warm in the winter time.
    //Discharge temps.
    dischrg_min_temp = 0;       //Battery minimum discharge temperature.
    dischrg_reduce_low_temp = 10;    //Reduced current discharge low temperature.
    dischrg_max_temp = 55;       //Battery max discharge temperature.
    dischrg_reduce_high_temp = 50;       //Battery reduced discharge high temperature.
    dischrg_target_temp = 15;      //Battery heater discharge target temp. Keeps us nice and warm in the winter time.
    //Shutdown temps.
    battery_shutdown_temp = 60;      //Max battery temp before shutting down everything.
    ctrlr_shutdown_temp = 80;        //Max motor or motor controller temp shutdown.
    //Fan ctrl temps.
    ctrlr_fan_start = 50;               //Turns on cooling fan.
    batt_fan_start = 30;
    //Some other stuff.
    max_heat = 50;              //Heater watts that you want to use.
    travel_dist = 0.012;         //Travel Distance in KM per tire rotation or between TAC ticks.
    circuit_draw = 0.05;        //Amount of current that Yeti himself draws. Used for current calibration.
    PowerOffAfter = 120;    //Power off the system after this many minutes of not being plugged in or keyed on. 120 minutes is 2 hours.
}

void Init(void){
/*******************************
 * initialization values setup.
*******************************/
    /* Osc Config*/
    OSCCONbits.NOSC = 1;
    OSCCONbits.OSWEN = 1;
    OSCCONbits.LPOSCEN = 0;
    /**************************/
    /* General IO. */
    TRISD = 0xFFF1; //set portd to all inputs except for RD2(KEEPALIVE), RD3(UNUSED), and RD1(mainContactor)
    LATD = 0;
    PORTDbits.RD2 = 1; //Enable Keep Alive signal. System keeps itself on while main_power is enabled.
    /**************************/
    /* Analog inputs and general IO */
    TRISB = 0x008F;              //set portb to mix analog inputs and digital outputs.
    LATB = 0;               //clear portb
    /**************************/
    /* General IO */
    TRISC = 0x0000;
    LATC = 0;
    /**************************/
    /* PWM outputs and charge detect input. */
    TRISE = 0xFFEF; //set porte to all inputs except RE4
    LATE = 0;
    /**************************/
    /* General IO */
    TRISF = 0xFFBE; //set portf to all inputs and two output.
    LATF = 0;

/***************
**************/
/* Configure PWM */
/*****************************/
    //FBORPOR = 0x82B2;
    PTCON = 0x0006;     //Set the PWM module and set to up/down mode for center aligned PWM. 6 = 37khz (4 Tcy per bit))
    PTMR = 0;
    PTPER = 49;         //set period. 0% - 99%
    SEVTCMP = 0;
    PWMCON1 = 0x00FB;       //Set PWM output for complementary mode.
    PWMCON2 = 0x0000;
    DTCON1 = 0x8181;        //Dead time is 4Tcy (1 bit)
    FLTACON = 0;
    OVDCON = 0xFF00;
    PDC1 = 0000;            //set output to 0
    PDC2 = 0000;            //set output to 0
    PDC3 = 0000;            //set output to 0

/*****************************/
/* Configure UARTs */
/*****************************/
    //PORT 1 setup
    U1STA = 0;
    U1MODE = 0;
    U1BRG = BaudCalc(BAUD1, IPS);     //calculate the baud rate.
    U1MODEbits.ALTIO = 1;           //Use alternate IO for UART1.
    //Default power up of UART should be 8n1

    //PORT 2 setup
    U2STA = 0;
    U2STAbits.UTXISEL = 1;
    U2MODE = 0;
    U2BRG = BaudCalc(BAUD2, IPS);     //calculate the baud rate.
    //Default power up of UART should be 8n1
/*****************************/
/* Configure Timer 1 */
/* Scan IO about every second when KeySwitch is off. */
/*****************************/
/* Timer one CTRL. */
    PR1 = 0xFFFF;
    TMR1 = 0x0000;
    T1CON = 0x0000;
    T1CONbits.TCKPS = 3;        //1:256 prescale

/*****************************/
/* Configure Timer 2 */
/*****************************/
/* For 0.125 second timing operations. */
    //PR2 = 0xE4E2;   //58,594
    PR2 = 0x7075;     //28,789
    TMR2 = 0x0000;
    T2CON = 0x0000;
    T2CONbits.TCKPS = 2;        //1:64 prescale

/*****************************/
/* Configure Timer 3 */
/*For speed calculation. */
/*****************************/
/* For exactly 1 second timing operations. */
    PR3 = 0xE0EA;   //57,578
    //PR3 = 0x7271;     //29,297
    TMR3 = 0x0000;
    T3CON = 0x0000;
    T3CONbits.TCKPS = 3;        //1:256 prescale

/*****************************/
/* Configure and Enable analog inputs */
/*****************************/
    ADCON3upper8 = 0x0F;
    ADCON3lower8 = 0x0F;
    ADCON1 = 0x02E4;
    ADCON2 = 0x0410;
    ADCON3 = 0x0F0F;    //0x0F0F;
    ADCHS = 0x0000;
    ADPCFG = 0xFF70;
    ADCSSL = 0x008F;

/*****************************/
/* Configure IRQs. */
/*****************************/
    //Configure Priorities
    IPC2bits.ADIP = 7;      //Analog inputs and regulation routines, Important
    IPC1bits.T2IP = 6;      //0.125 second IRQ for some math timing
    IPC4bits.INT1IP = 5;    //Wheel rotate IRQ, timing is important.
    IPC0bits.T1IP = 4;      //Heartbeat IRQ, eh...
    IPC0bits.INT0IP = 3;    //Charger detect IRQ, need to know basis
    IPC1bits.T3IP = 2;      //Timer 3 IRQ. Not critical.
    IPC2bits.U1RXIP = 1;    //RX 1 IRQ, Text can wait
    IPC2bits.U1TXIP = 1;    //TX 1 IRQ, Text can wait
    IPC6bits.U2RXIP = 1;    //RX 2 IRQ, Text can wait
    IPC6bits.U2TXIP = 1;    //TX 2 IRQ, Text can wait
    IPC5bits.INT2IP = 1;    //Not yet used.
    
    //Ensure interrupt nesting is enabled.
    INTCON1bits.NSTDIS = 0; //This should be the default state on chip reset.
    // Clear all interrupts flags
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;

	// enable interrupts
	__asm__ volatile ("DISI #0x3FFF");
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
	IEC0bits.T1IE = 1;	// Enable interrupts for timer 1
    IEC0bits.U1RXIE = 1; //Enable interrupts for UART1 Rx.
    IEC1bits.U2RXIE = 1; //Enable interrupts for UART2 Rx.
    IEC1bits.U2TXIE = 1; //Enable interrupts for UART2 Tx.
    if(EnableChIRQ == 1){
        IEC0bits.INT0IE = 1;    //Charge Detect IRQ
    }
    EnableChIRQ = 1;    //By default, enable charge detect IRQ on init.
    IEC1bits.INT1IE = 1;    //Wheel rotate IRQ
    IEC1bits.INT2IE = 0;  //Disable irq for INT2, not used.
    IEC0bits.T2IE = 1;	// Enable interrupts for timer 2
    IEC0bits.T3IE = 1;	// Enable interrupts for timer 3
    IEC0bits.ADIE = 1;  // Enable ADC IRQs.
    INTCON2bits.INT0EP = 0;
    INTCON2bits.INT2EP = 0;
DISICNT = 0;
/*****************************/
/* Enable our devices. */
/*****************************/
    ADCON1bits.ADON = 1;    // turn ADC ON
    PTCONbits.PTEN = 1;     // Enable PWM
    T2CONbits.TON = 1;      // Start Timer 2
    T1CONbits.TON = 1;      // Start Timer 1
    T3CONbits.TON = 1;      // Start Timer 3
    U1MODEbits.UARTEN = 1;  //enable UART1
    U1STAbits.UTXEN = 1;    //enable UART1 TX
    U2MODEbits.UARTEN = 1;  //enable UART2
    U2STAbits.UTXEN = 1;    //enable UART2 TX
/* End Of Initial Config stuff. */
/* Now do some pre-calculations. */

    //Calculate our voltage divider values.
    vltg_dvid = R2_resistance / (R1_resistance + R2_resistance);
    //Calculate our charge/discharge rate.
    calc_125 = 0.125 / 3600;
    //We've done Init.
    init_done = 1;
    //We aren't in low power mode
    lw_pwr = 0;
    //Calculate max charge current.
    max_chrg_current = chrg_C_rating * amp_hour_rating;
    //Send Init message.
    //send_string(NLtxtNL, "Initialized.", PORT1);

}

//Go in to low power mode when not in use.
void low_power_mode(void){
    io_off();
    ADCON1bits.ADON = 0;    // turn ADC off
    T2CONbits.TON = 0;      // Stop Timer 2
    T3CONbits.TON = 0;      // Stop Timer 3
    	// disable interrupts
	__asm__ volatile ("DISI #0x3FFF");
    IEC1bits.INT1IE = 0;    //disable Wheel rotate IRQ
    IEC0bits.T2IE = 0;	// disable interrupts for timer 2
    INTCON2bits.INT1EP = 0;
    INTCON2bits.INT2EP = 0;
    DISICNT = 0;
    //Need to reinit on restart
    init_done = 0;
    //Tell everyone we are in low power mode.
    lw_pwr = 1;
    //Turn off Outputs, etc
    PORTBbits.RB6 = 0;
}

/* Turn everything off so we don't waste any more power.
 * Only plugging in the charge will restart the CPU, or yaknow, just restart the CPU... */
void low_battery_shutdown(void){
    cmd_power = 0;
    soft_power = 0;
    PTCONbits.PTEN = 0;     // off PWM
    T1CONbits.TON = 0;      // Stop Timer 1
    // Clear all interrupts flags
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    	// disable interrupts
	__asm__ volatile ("DISI #0x3FFF");
	IEC0bits.T1IE = 0;	// disable interrupts for timer 1
    IEC0bits.ADIE = 0;  //disable ADC IRQs.
    INTCON2bits.INT1EP = 0;
    INTCON2bits.INT2EP = 0;
    low_power_mode();
    deep_sleep = 1;     //Tell the system to enter a deep sleep after we have completed all tasks one last time.
}

#endif
