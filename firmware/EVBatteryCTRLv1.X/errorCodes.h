/* 
 * File:   errorCodes.h
 * Author: User
 *
 * Created on January 2, 2022, 2:46 PM
 */

#ifndef ERRORCODES_H
#define	ERRORCODES_H

const char code01[] = "Heater load too small.";
const char code02[] = "No heater detected.";
const char code03[] = "Heater load too large.";
const char code04[] = "Low battery shutdown.";
const char code05[] = "Discharge over current.";
const char code06[] = "Charge over current.";
const char code07[] = "High battery voltage.";
const char code08[] = "Battery over temp.";
const char code09[] = "Motor controller over temp.";
const char code0A[] = "Abominable Snow Monster over temp.";
const char code0B[] = "Shutdown Event.";
const char code0C[] = "TRAP: PWM Fault Event.";
const char code0D[] = "TRAP: Oscillator Fail Event.";
const char code0E[] = "TRAP: Address Error Event.";
const char code0F[] = "TRAP: Stack Error Event.";
const char code10[] = "TRAP: Math Error Event.";
const char code11[] = "TRAP: Reserved Trap 7 Event.";
const char code12[] = "TRAP: PLL out of lock.";
const char code13[] = "RESET: Brown Out Event.";
const char code14[] = "RESET: WDT Reset Event.";
const char code15[] = "RESET: TRAP Conflict Event.";
const char code16[] = "RESET: Illegal opcode or uninitialized W register access Event.";
const char code17[] = "RESET: External Reset Event.";
const char code18[] = "RESET: Instruction Reset Event.";
const char code19[] = "CPU: Reset.";
const char code1A[] = "Invalid serial port.";
const char code1B[] = "No current from charger detected.";
const char code1C[] = "Partial charge set higher than 100%. Clamping.";
const char code1D[] = "VHigh on Battery voltage input.";
const char code1E[] = "VLow on Battery voltage input.";
const char code1F[] = "VHigh on Battery current input.";
const char code20[] = "VLow on Battery current input.";
const char code21[] = "VHigh on Battery temp input.";
const char code22[] = "VLow on Battery temp input.";
const char code23[] = "VHigh on motor control temp input.";
const char code24[] = "VLow on motor control temp input.";
const char code25[] = "VHigh on snowman's temp input.";
const char code26[] = "VLow on snowman's temp input.";
const char codeDefault[] = "Unknown Fault code.";


#endif	/* ERRORCODES_H */

