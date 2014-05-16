/*
* N3310LCD. A program to interface mbed with the nuelectronics
* Nokia 3310 LCD shield from www.nuelectronics.com. Ported from
* the nuelectronics Arduino code.
*
* Copyright (C) <2009> Petras Saduikis <petras@petras.co.uk>
*
* This file is part of N3310LCD.
*
* N3310LCD is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* N3310LCD is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with N3310LCD.  If not, see <http://www.gnu.org/licenses/>.
*/

/*------------------------------------------------------------------------------
; mbed bitmap - size 48x24 pixels, black/white image
------------------------------------------------------------------------------*/ 

unsigned char mbed_bmp[]=
{

0x00,0x80,0x80,0x80,0x00,0x80,0x80,0x80,
0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,
0xF8,0xF8,0xF8,0x00,0x80,0x80,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,
0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
0x80,0x80,0x00,0xF8,0xF8,0xF8,0x00,0x00,
0x00,0xFF,0xFF,0xFF,0x01,0x01,0xFF,0xFF,
0xFF,0x01,0x01,0xFF,0xFF,0xFF,0x00,0x00,
0xFF,0xFF,0xFF,0x83,0x01,0x83,0xFF,0xFF,
0xFE,0x00,0x00,0x7C,0xFF,0xFF,0x19,0x19,
0xDF,0xDF,0xDC,0x00,0x00,0xFE,0xFF,0xFF,
0x83,0x01,0x83,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0x03,0x03,0x03,0x00,0x00,0x03,0x03,
0x03,0x00,0x00,0x03,0x03,0x03,0x00,0x00,
0x03,0x03,0x03,0x01,0x03,0x03,0x03,0x01,
0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x03,
0x03,0x01,0x00,0x00,0x00,0x00,0x01,0x03,
0x03,0x03,0x01,0x03,0x03,0x03,0x00,0x00
};