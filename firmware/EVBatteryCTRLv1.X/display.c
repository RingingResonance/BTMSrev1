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

#ifndef DKSY_C
#define DKSY_C

#include <p30f3011.h>

#include "DataIO.h"
#include "errorCodes.h"

/* Read fault codes to serial port.
 * This takes up over 5% of program space.
 */
void fault_read(int smpl, int serial_port){
    send_string("Reading Codes...", serial_port);
    int x = 0;
    const char *textpointer;
    if(fault_count > 10){
        send_string("Fault Log Is Full. Please clear faults.", serial_port);
    }
    if(fault_count == 0){
        send_string("No fault codes to report.", serial_port);
    }
    else{
        while(x < fault_count){
                switch(fault_codes[x]){
                    case 0x01:
                        textpointer = code01;
                    break;
                    case 0x02:
                        textpointer = code02;
                    break;
                    case 0x03:
                        textpointer = code03;
                    break;
                    case 0x04:
                        textpointer = code04;
                    break;
                    case 0x05:
                        textpointer = code05;
                    break;
                    case 0x06:
                        textpointer = code06;
                    break;
                    case 0x07:
                        textpointer = code07;
                    break;
                    case 0x08:
                        textpointer = code08;
                    break;
                    case 0x09:
                        textpointer = code09;
                    break;
                    case 0x0A:
                        textpointer = code0A;
                    break;
                    case 0x0B:
                        textpointer = code0B;
                    break;
                    case 0x0C:
                        textpointer = code0C;
                    break;
                    case 0x0D:
                        textpointer = code0D;
                    break;
                    case 0x0E:
                        textpointer = code0E;
                    break;
                    case 0x0F:
                        textpointer = code0F;
                    break;
                    case 0x10:
                        textpointer = code10;
                    break;
                    case 0x11:
                        textpointer = code11;
                    break;
                    case 0x12:
                        textpointer = code12;
                    break;
                    case 0x13:
                        textpointer = code13;
                    break;
                    case 0x14:
                        textpointer = code14;
                    break;
                    case 0x15:
                        textpointer = code15;
                    break;
                    case 0x16:
                        textpointer = code16;
                    break;
                    case 0x17:
                        textpointer = code17;
                    break;
                    case 0x18:
                        textpointer = code18;
                    break;
                    case 0x19:
                        textpointer = code19;
                    break;
                    case 0x1A:
                        textpointer = code1A;
                    break;
                    case 0x1B:
                        textpointer = code1B;
                    break;
                    case 0x1C:
                        textpointer = code1C;
                    break;
                    case 0x1D:
                        textpointer = code1D;
                    break;
                    case 0x1E:
                        textpointer = code1E;
                    break;
                    case 0x1F:
                        textpointer = code1F;
                    break;
                    case 0x20:
                        textpointer = code20;
                    break;
                    case 0x21:
                        textpointer = code21;
                    break;
                    case 0x22:
                        textpointer = code22;
                    break;
                    case 0x23:
                        textpointer = code23;
                    break;
                    case 0x24:
                        textpointer = code24;
                    break;
                    case 0x25:
                        textpointer = code25;
                    break;
                    case 0x26:
                        textpointer = code26;
                    break;
                    default:
                        textpointer = codeDefault;
                    break;
                }
            send_string(textpointer, serial_port);
            x++;
        }
    }
}


#endif