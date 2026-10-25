/* board.h — battery ADC, kill switch, LED, hover-record persistence. */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include "fc_safety.h"

void board_init(void);
float board_battery_v(void);       /* ~20-50 us ADC read (call at <= 100 Hz, not from the control loop) */
bool board_kill_switch(void);      /* raw level, active = true (fast GPIO read, ok in the control loop) */
void board_led(bool on);

/* hover record of the takeoff (fc_safety) : RTC no-init RAM survives a watchdog / panic reset, NVS survives power-off */
void hover_store_load(fs_hover_rec_t *prev);   /* at boot: RTC copy if valid, else NVS, else invalid */
void hover_store_save_rtc(const fs_hover_rec_t *r); /* fast (memcpy), from the control task */
void hover_store_save_nvs(const fs_hover_rec_t *r); /* slow (flash write, ms): link task, only when DISARMED */
void hover_store_flush_nvs(void);                    /* writes the RTC copy to NVS if valid and changed */

#endif
