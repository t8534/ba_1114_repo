/* 
 * libmbed-graphics 2D and wireframe 3D graphics library for the MBED
 * microcontroller platform
 * Copyright (C) <2009> Michael Sheldon <mike@mikeasoft.com>
 * Optimized and adapted for AbstractLCD interface
 * Copyright (C) <2010> Igor Skochinsky <skochinsky@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "st7565.h"
#include "graphics.h"

// swap two values
#define SWAP(a, b) (((a) == (b)) || (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))))


void GRAPH_line(int x0, int y0, int x1, int y1, int colour) {
    // Bresenham
    //printf("line(%d, %d, %d, %d, %d)\n", x0, y0, x1, y1, colour);
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    int temp, deltax, deltay, error, ystep, y, x;
    if (steep) {
        temp = y0;
        y0 = x0;
        x0 = temp;
        temp = y1;
        y1 = x1;
        x1 = temp;
    }
    if (x0 > x1) {
        temp = x1;
        x1 = x0;
        x0 = temp;
        temp = y1;
        y1 = y0;
        y0 = temp;
    }
    deltax = x1 - x0;
    deltay = abs(y1 - y0);
    error = deltax / 2;
    y = y0;
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }
    for (x=x0; x<=x1; x++) {
        if (steep) {
            ST7565_pixel(y, x, colour);
        } else {
        	ST7565_pixel(x, y, colour);
        }
        error = error - deltay;
        if (error < 0) {
            y = y + ystep;
            error = error + deltax;
        }
    }
}

void GRAPH_line3d(int x0, int y0, int z0, int x1, int y1, int z1, int colour) {

	int _cx3d, _cy3d, _cz3d; // 3D focal point


    _cx3d = ST7565_getWidth() / 2;
    _cy3d = ST7565_getHeight() / 2;
    _cz3d = 150;

	if (z0 + _cz3d <= 0 || z1 + _cz3d <= 0) {
        // Behind the camera
        return;
    }
    
    int u0 = _cx3d + x0 * _cz3d / (z0 + _cz3d);
    int v0 = _cy3d + y0 * _cz3d / (z0 + _cz3d);
    int u1 = _cx3d + x1 * _cz3d / (z1 + _cz3d);
    int v1 = _cy3d + y1 * _cz3d / (z1 + _cz3d);
    line(u0, v0, u1, v1, colour);
}

void GRAPH_circle(int cx, int cy, int radius, int colour) {
    int x = 0;
    int y = radius;
    int d = 3 - (2 * radius);

    while (x <= y) {
    	ST7565_pixel(cx + x, cy + y, colour);
    	ST7565_pixel(cx + y, cy + x, colour);
    	ST7565_pixel(cx - x, cy + y, colour);
    	ST7565_pixel(cx + y, cy - x, colour);
    	ST7565_pixel(cx - x, cy - y, colour);
    	ST7565_pixel(cx - y, cy - x, colour);
    	ST7565_pixel(cx + x, cy - y, colour);
    	ST7565_pixel(cx - y, cy + x, colour);

        if (d<0)
            d += (4 * x) + 6;
        else
        {
            d += (4 * (x - y)) + 10;
            y--;
        }
        x++;
    }
}
