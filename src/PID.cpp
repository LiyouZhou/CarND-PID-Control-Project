#include "PID.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

using namespace std;
#define INTEGRAL_LIMIT 0.5
/*
* TODO: Complete the PID class.
*/

PID::PID() :
initialized(false), Kp(0), Ki(0), Kd(0),
p_error(0), i_error(0), d_error(0), counter(0),
accumulative_error(0)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0;
    i_error = 0;
    d_error = 0;
    counter = 0;
    accumulative_error = 0;
    initialized = true;
}

void PID::UpdateError(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError()
{
    counter++;
    double i_term = i_error * Ki;

    // avoid integral wind up by limiting the integral term
    if (i_term > INTEGRAL_LIMIT) i_term = INTEGRAL_LIMIT;
    if (i_term < -INTEGRAL_LIMIT) i_term = -INTEGRAL_LIMIT;

    double err = -(p_error * Kp + i_term + d_error * Kd);

    // printf("-(%f * %f + %f * %f + %f * %f) = %f", p_error, Kp, i_error, Ki, d_error, Kd, err);
    accumulative_error += fabs(err);
    return err;
}
