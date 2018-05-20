#include <uWS/uWS.h>
#include "PID.h"

#ifndef TWIDDLE_H
#define TWIDDLE_H

void twiddle_init(const double init_coeffs[3],
                  const double init_d_coeffs[3],
                  const float init_twiddle_decend_ratio);

void twiddle_state_machine(uWS::WebSocket<uWS::SERVER> ws, PID &pid);
#endif /* TWIDDLE_H */
