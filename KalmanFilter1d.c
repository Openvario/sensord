/*  
	sensord - Sensor Interface for XCSoar Glide Computer - http://www.openvario.org/
    Copyright (C) 2014  The openvario project
    A detailed list of copyright holders can be found in the file "AUTHORS" 

    This program is free software; you can redistribute it and/or 
    modify it under the terms of the GNU General Public License 
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, see <http://www.gnu.org/licenses/>.	
*/

#include <math.h>
#include "KalmanFilter1d.h"

void KalmanFiler1d_update(t_kalmanfilter1d* filter, float z_abs, float var_z_abs, float dt)
{
	float F1=1.0;
	float dt2, dt3, dt4;
	float y;
	float s_inv;
	float k_abs;
	float k_vel;
	
	// check if dt is positive
	
	//Predict step
	//update state estimate
	filter->x_abs_ += filter->x_vel_ * dt;
	
	// update state covariance
	dt2 = dt * dt;
	dt3 = dt * dt2;
	dt4 = dt2 * dt2;

	filter->p_abs_abs_ += 2*(dt*filter->p_abs_vel_) + dt2 * filter->p_vel_vel_ + 0.25*(filter->var_x_accel_ * dt4);
  filter->p_abs_vel_ += dt * filter->p_vel_vel_ + (filter->var_x_accel_ * dt3)/2;
  filter->p_vel_vel_ += filter->var_x_accel_ * dt2;
	
	// Update step
	y = z_abs - filter->x_abs_;		// Innovation
	s_inv = F1 / (filter->p_abs_abs_ + var_z_abs);		// Innovation precision 
	k_abs = filter->p_abs_abs_*s_inv; // Kalman gain
	k_vel = filter->p_abs_vel_*s_inv;
	
	// Update state estimate.
	filter->x_abs_ += k_abs * y;
	filter->x_vel_ += k_vel * y;
  
  // Update state covariance.
  filter->p_vel_vel_ -= filter->p_abs_vel_ * k_vel;
  filter->p_abs_vel_ -= filter->p_abs_vel_ * k_abs;
  filter->p_abs_abs_ -= filter->p_abs_abs_ * k_abs;
	
}

void KalmanFilter1d_reset(t_kalmanfilter1d* filter)
{
	filter->x_abs_ = 0.0;
	filter->x_vel_ = 0.0;

	filter->p_abs_abs_ = 0.0;
	filter->p_abs_vel_ = 0.0;
	filter->p_vel_vel_ = 0.0;
	
	filter->var_x_accel_ = 0.0;
	
}
