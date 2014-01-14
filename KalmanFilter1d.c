#include <math.h>
#include "KalmanFilter1d.h"

void KalmanFiler1d_update(t_kalmanfilter1d* filter, float z_abs, float var_z_abs, long dt)
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
	dt2 = sqrt(dt);
	dt3 = dt * dt2;
	dt4 = sqrt(dt2);

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
