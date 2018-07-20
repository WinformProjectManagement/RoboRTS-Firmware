/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file kalman_filter.c
 *  @version 1.0
 *  @date Apr 2018
 *
 *  @brief kalman filter realization
 *
 *  @copyright 2018 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "iir_filter.h"

double NUM[7] = 
{
  0.000938328292536,
 -0.005375225952906,
  0.01306656496967,
 -0.01725922344534,
  0.01306656496967,
 -0.005375225952906,
  0.000938328292536
};
double DEN[7] = 
{
   1,
  -5.733703351811,
  13.70376013927,
 -17.47505866491,
  12.53981348666,
  -4.800968865471,
   0.7661573674342
};

iir_filter_t yaw_msg_t, pit_msg_t;

double iir_filter(iir_filter_t *F)
{
  int i;
  for(i = 6; i > 0; i--)
  {
    F->xbuf[i] = F->xbuf[i-1];
    F->ybuf[i] = F->ybuf[i-1];
  }
  
  F->xbuf[0] = F->raw_value;
  F->ybuf[0] = NUM[0] * F->xbuf[0];
  
  for(i = 1; i < 7; i++)
  {
    F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
  }
  
  F->filtered_value = F->ybuf[0];
  
  return F->filtered_value;
}

