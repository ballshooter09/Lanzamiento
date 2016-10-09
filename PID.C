float PID_Controller(float INPUT, float SET, float dt, float last_error, float Integral) {

	float OUTPUT, error, Derivative;

	error = SET - INPUT;
	Integral = Integral + error;
	Derivative = error - last_error;

	OUTPUT = Kp*error + Ki*Integral*dt + Kd*Derivative/dt;

	if (OUTPUT > 255)
		OUTPUT = 255;
	else if (OUTPUT < 0)
		OUTPUT = 0;

	return OUTPUT;
}


float Kp, Ki, Kd, last_error, Integral;