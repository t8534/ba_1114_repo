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

#ifndef GRAPHICS_H
#define GRAPHICS_H




/* Function: line
 * Draw a coloured line between two points.
 *
 * Parameters:
 *  x0 - X co-ordinate of the start of the line.
 *  y0 - Y co-ordinate of the start of the line.
 *  x1 - X co-ordinate of the end of the line.
 *  y1 - Y co-ordinate of the end of the line.
 *  colour - The colour of the line.
 */
void GRAPH_line(int x0, int y0, int x1, int y1, int colour);
        
/* Function: line3d
 * Draws a coloured line in 3D space. The 3D origin point is in
 * the centre of the screen.
 *
 * Parameters:
 * x0 - X co-ordinate of the start of the line.
 * y0 - Y co-ordinate of the start of the line.
 * z0 - Z (depth) co-ordinate of the start of the line.
 * x1 - X co-ordinate of the end of the line.
 * y1 - Y co-ordinate of the end of the line.
 * z1 - Z co-ordinate of the end of the line.
 * colour - The colour of the line.
 */
void GRAPH_line3d(int x0, int y0, int z0, int x1, int y1, int z0, int colour);
        
/* Function: circle
 * Draw a coloured circle.
 *
 * Parameters:
 * cx - X co-ordinate of the centre of the circle.
 * cy - Y co-ordinate of the centre of the circle.
 * radius - The radius of the circle.
 * colour - The colour of the circle.
 */
void circle(int cx, int cy, int radius, int colour);
        

#endif
