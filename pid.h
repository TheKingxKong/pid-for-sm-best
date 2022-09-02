/*pid.h 
Generic C code for a PID function*/

#define PID_KP 1.0
#define PID_KI 0.0
#define PID_KD 0.0

#define PID_MIN_OUTPUT -255
#define PID_MAX_OUTPUT 255
#define PID_MIN_INPUT 0
#define PID_MAX_INPUT 255

void pid_init(pid_t *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_sum = 0;
    pid->last_error = 0;
    pid->last_output = 0;
}

void pid_set_limits(pid_t *pid, double min_output, double max_output, double min_input, double max_input) {
    pid->min_output = min_output;
    pid->max_output = max_output;
    pid->min_input = min_input;
    pid->max_input = max_input;
}

void pid_update(pid_t *pid, double input, double setpoint) {
    double error = setpoint - input;
    pid->error_sum += error;
    double d_error = error - pid->last_error;
    pid->last_error = error;
    double output = pid->kp * error + pid->ki * pid->error_sum + pid->kd * d_error;
    output = constrain(output, pid->min_output, pid->max_output);
    pid->last_output = output;
}

double pid_get_output(pid_t *pid) {
    return pid->last_output;
}

double pid_get_error(pid_t *pid) {
    return pid->last_error;
}

double pid_get_setpoint(pid_t *pid) {
    return pid->last_setpoint;
}

void pid_set_setpoint(pid_t *pid, double setpoint) {
    pid->last_setpoint = setpoint;
}

void pid_reset_sum(pid_t *pid) {
    pid->error_sum = 0;
}
