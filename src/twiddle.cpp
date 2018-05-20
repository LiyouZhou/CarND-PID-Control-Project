#include <uWS/uWS.h>
#include <iostream>
#include <stdio.h>
#include "PID.h"
#include <math.h>
#include "twiddle.h"

static double d_coeffs[] = {0.0,0.003,0.1};
static double coeffs[] = {0.0084,0.000,0.8};
static float twiddle_decend_ratio = 0.6;

// set hyperparameters of the twiddle algorithm
void twiddle_init(const double init_coeffs[3],
                  const double init_d_coeffs[3],
                  const float init_twiddle_decend_ratio)
{
  for (int i = 0; i < 3; i++)
  {
    coeffs[i] = init_coeffs[i];
    d_coeffs[i] = init_d_coeffs[i];
  }
  twiddle_decend_ratio = init_twiddle_decend_ratio;
}

void twiddle_state_machine(uWS::WebSocket<uWS::SERVER> ws, PID &pid)
{
  static double last_accumulative_error = std::numeric_limits<double>::max();
  static int current_tuning_index = 0;
  const char* reset_msg = "42[\"reset\"]";

  static enum {
    TRY_POSITIVE_TWIDDLE,
    POSITIVE_TWIDDLE_RESULT,
    TRY_NEGATIVE_TWIDDLE,
    NEGATIVE_TWIDDLE_RESULT,
    TWIDDLE_FINISH
  } state = TRY_POSITIVE_TWIDDLE;

  switch(state) {
    case TRY_POSITIVE_TWIDDLE:
      printf("TRY_POSITIVE_TWIDDLE\r\n");

      // twiddle coefficient in the positive direction
      coeffs[current_tuning_index] += d_coeffs[current_tuning_index];

      // Initialise PID with the widdled coefficients
      printf("coeffs %.02e %.02e %.02e d_coeffs %.02e %.02e %.02e\r\n", coeffs[0], coeffs[1], coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2]);
      pid.Init(coeffs[0], coeffs[1], coeffs[2]);

      // when the result return we expect to go into the POSITIVE_TWIDDLE_RESULT state
      state = POSITIVE_TWIDDLE_RESULT;

      // reset the simulation
      ws.send(reset_msg, strlen(reset_msg), uWS::OpCode::TEXT);

      break;

    case POSITIVE_TWIDDLE_RESULT:
      printf("POSITIVE_TWIDDLE_RESULT\r\n");

      // if the positive twiddle resulted in an improvement
      if (pid.accumulative_error < last_accumulative_error) {
        printf("POSITIVE_TWIDDLE success %f < %f\r\n", pid.accumulative_error, last_accumulative_error);
        last_accumulative_error = pid.accumulative_error;
        // increase the twiddle amount
        d_coeffs[current_tuning_index] /= twiddle_decend_ratio;

        // move onto next coefficient
        state = TWIDDLE_FINISH;
      } else { // positive twiddle made result worse
        printf("POSITIVE_TWIDDLE fail %f > %f\r\n", pid.accumulative_error, last_accumulative_error);
        // put the coefficient back to before twiddle
        coeffs[current_tuning_index] -= d_coeffs[current_tuning_index];

        // Try twiddle it in the opposite direction
        state = TRY_NEGATIVE_TWIDDLE;
      }

      // drive state machine to next state
      twiddle_state_machine(ws, pid);
      break;

    case TRY_NEGATIVE_TWIDDLE:
      printf("TRY_NEGATIVE_TWIDDLE\r\n");

      // twiddle coefficient in the negative direction
      coeffs[current_tuning_index] -= d_coeffs[current_tuning_index];
      printf("coeffs %.02e %.02e %.02e d_coeffs %.02e %.02e %.02e\r\n", coeffs[0], coeffs[1], coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2]);

      // when the result return we expect to go into the NEGATIVE_TWIDDLE_RESULT state
      state = NEGATIVE_TWIDDLE_RESULT;

      // Fail immediately if ceoffs negative
      if (coeffs[current_tuning_index] < 0) {
        printf("coeff below zero\r\n");
        pid.accumulative_error = last_accumulative_error;

        // go directly to next state
        twiddle_state_machine(ws, pid);
      } else {
        // Initialise PID with the widdled coefficients
        pid.Init(coeffs[0], coeffs[1], coeffs[2]);

        // reset the simulation
        ws.send(reset_msg, strlen(reset_msg), uWS::OpCode::TEXT);
      }

      break;

    case NEGATIVE_TWIDDLE_RESULT:
      printf("TRY_NEGATIVE_TWIDDLE\r\n");

      // if the negative twiddle resulted in an improvement
      if (pid.accumulative_error < last_accumulative_error) {
        printf("NEGATIVE_TWIDDLE success %f < %f\r\n", pid.accumulative_error, last_accumulative_error);
        last_accumulative_error = pid.accumulative_error;
        // increase twiddle amount
        d_coeffs[current_tuning_index] /= twiddle_decend_ratio;
      } else {
        printf("NEGATIVE_TWIDDLE fail %f > %f\r\n", pid.accumulative_error, last_accumulative_error);
        // put the coefficient back to before twiddle
        coeffs[current_tuning_index] += d_coeffs[current_tuning_index];
        // decrease twiddle amount
        d_coeffs[current_tuning_index] *= twiddle_decend_ratio;
      }

      // try positive twiddle next
      state = TWIDDLE_FINISH;
      twiddle_state_machine(ws, pid);

      break;

    case TWIDDLE_FINISH:
      printf("TWIDDLE_FINISH\r\n");

      printf("coeffs %.02e %.02e %.02e d_coeffs %.02e %.02e %.02e\r\n", coeffs[0], coeffs[1], coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2]);
      printf("=======================================\r\n");

      if ((d_coeffs[0] + d_coeffs[1] + d_coeffs[2] > 0.02) ||
          (current_tuning_index % 3)) {
        // move on to next coefficient
        current_tuning_index++;
        current_tuning_index %= 3;
        printf("Twiddle coefficient no. %u\r\n", current_tuning_index);
        state = TRY_POSITIVE_TWIDDLE;
        twiddle_state_machine(ws, pid);
      } else {
        printf("Twiddle Done, final coefficients: \r\n");
        printf("coeffs %.02e %.02e %.02e d_coeffs %.02e %.02e %.02e\r\n", coeffs[0], coeffs[1], coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2]);
      }

      break;
  }
}