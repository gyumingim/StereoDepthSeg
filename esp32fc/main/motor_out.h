/* motor_out.h — motor output abstraction: standard PWM (default, LEDC), OneShot125 (LEDC) or DShot600/300 (RMT).
 * Values are 0..1 throttle. "Disarmed" = PWM 1000 us / OneShot 125 us / DShot value 0 (ESC: motor stop). */
#ifndef MOTOR_OUT_H
#define MOTOR_OUT_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t motor_out_init(void);           /* outputs start in the disarmed state */
void motor_out_write(const float *thr);   /* FC_N_MOTORS values 0..1; ignored while the e-stop latch is set */
void motor_out_disarm(void);              /* zero / stop frame on all motors (control task) */
/* callable from ANY task (supervisor on the other core): latches, sends stop frames; if the control task is stuck
 * inside a write it disables the RMT channels and drives the pins low (ESC signal loss -> motor stop) */
void motor_out_emergency_stop(void);
void motor_out_clear_estop(void);         /* only when DISARMED, by the control task */
bool motor_out_estopped(void);

#endif
