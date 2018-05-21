#include "PID.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

using namespace std;
#define INTEGRAL_LIMIT 1
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

    // avoid integral wind up by limiting the integral term
    if (i_error > INTEGRAL_LIMIT) i_error = INTEGRAL_LIMIT;
    if (i_error < -INTEGRAL_LIMIT) i_error = -INTEGRAL_LIMIT;
}

double PID::TotalError()
{
    counter++;
    double err = -(p_error * Kp + i_error * Ki + d_error * Kd);
    // printf("-(%f * %f + %f * %f + %f * %f) = %f", p_error, Kp, i_error, Ki, d_error, Kd, err);
    accumulative_error += fabs(err);
    return err;
}
