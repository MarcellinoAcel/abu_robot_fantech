#include <pid.h>
PID::PID(float MIN_VAL, float MAX_VAL, float kp_, float ki_, float kd_) : min_val_(MIN_VAL),
                                                                          max_val_(MAX_VAL),
                                                                          KP(kp_),
                                                                          KI(ki_),
                                                                          KD(kd_)
{
}
void PID::parameter(float kp_, float ki_, float kd_)
{

    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::parameterT(float kp_, float ki_, float kd_)
{
    kpT = kp_;
    kiT = ki_;
    kdT = kd_;
}

float PID::control_angle(float target, float enc, float pwm, float deltaT)
{
    deg2target = target / 360 * 3840;

    err.proportional = deg2target - enc;

    err.integral += err.proportional * deltaT;

    err.derivative = (err.proportional - err.previous);
    err.previous = err.proportional;

    err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;

    return fmax(-1 * pwm, fmin(err.u, pwm));
}

float PID::control_angle_speed(float target_angle, float target_speed, float enc, float deltaT)
{
    deg2target = target_angle / 360 * 3840;

    err.proportional = deg2target - enc;

    err.integral += err.proportional * deltaT;

    err.derivative = (err.proportional - err.previous);
    err.previous = err.proportional;

    err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;
    return control_speed(target_speed, enc, deltaT);
}

float PID::control_base(float error, float speed, int condition, float deltaT)
{
    if (condition)
    {
        eProportional = error;
        if (eProportional > 180)
        {
            eProportional -= 360;
        }
        else if (eProportional < -180)
        {
            eProportional += 360;
        }
    }
    else
    {
        eProportional = error;
    }

    eIntegral += eProportional * deltaT;

    float eDerivative = (eProportional - prevError) / deltaT;
    prevError = eProportional;

    float u = kp * eProportional + ki * eIntegral + kd * eDerivative;
    float uT = kpT * eProportional + kiT * eIntegral + kdT * eDerivative;

    return condition ? uT : u;
}
float PID::control_speed(float target, float enc, float deltaT)
{
    radian = (enc - encPrev) / deltaT;
    encPrev = enc;
    angular_vel = radian / (total_gear_ratio * enc_ppr);

    angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
    angular_vel_Prev = angular_vel;

    err.proportional = target - angular_vel_Filt;

    err.integral += err.proportional * deltaT;
    fmax(-125, fmin(err.integral, 125));
    err.derivative = (err.proportional - err.previous) / deltaT;
    err.previous = err.proportional;

    err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;
    return fmax(min_val_, fmin(err.u, max_val_));
}
float PID::control_default(float target, float curr, float deltaT)
{
    float error = target - curr;

    error_integral += error * deltaT;

    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;

    float u = KP * error + KI * error_integral + KD * error_derivative;
    return fmax(min_val_, fmin(u, max_val_));
}
float PID::get_filt_vel()
{
    return angular_vel_Filt;
}