
typedef struct {
	float x_abs_;		// the absolute quantity x
	float x_vel_;		// the rate of change of x, in x units per second squared.
	
	// Covariance matrix for the state
	float p_abs_abs_;
	float p_abs_vel_;
	float p_vel_vel_;
	
	// The variance of the acceleration noise input to the system model
	float var_x_accel_;
	} t_kalmanfilter1d;

void KalmanFiler1d_update(t_kalmanfilter1d* , float , float , long);
void KalmanFilter1d_reset(t_kalmanfilter1d*);
	