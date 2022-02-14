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

#ifndef IRQS_H
#define	IRQS_H
extern void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _FLTAInterrupt (void);
extern void __attribute__((interrupt, no_auto_psv)) _OscillatorFail (void);
extern void __attribute__((interrupt, no_auto_psv)) _AddressError (void);
extern void __attribute__((interrupt, no_auto_psv)) _StackError (void);
extern void __attribute__((interrupt, no_auto_psv)) _MathError (void);
extern void __attribute__((interrupt, no_auto_psv)) _ReservedTrap7 (void);


#endif	/* IRQS_H */

