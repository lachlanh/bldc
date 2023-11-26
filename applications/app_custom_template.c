/*
        Copyright 2019 Benjamin Vedder	benjamin@vedder.se

        This file is part of the VESC firmware.

        The VESC firmware is free software: you can redistribute it and/or
   modify it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "comm_can.h"
#include "commands.h"
#include "encoder/encoder.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// pas config
// Pedal Assist constants
const int CADENCE_MAGNETS = 12;
const int CADENCE_MIN = 15; // minimum cadence for motor to run
const int CADENCE_MAX = 45; // cadence value that will result in full throttle

const float THROTTLE_OFF = 0.0;
const float THROTTLE_DUTY_MIN = 0.05;
const float THROTTLE_DUTY_MID = 0.5;
const float THROTTLE_DUTY_MAX = 1.0;

const int PAS_TIMEOUT = 400; // ms to stop power after pedal stopped
#define FILTER_SAMPLES 5

// Private functions
static float calculateCadence(long edgeInterval, int cadenceMagnets);
static float calculateThrottleDuty(float targetDuty, float cadence);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {

  commands_printf("CUSTOM APP START!!!!");

  stop_now = false;
  chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread,
                    NULL);

  // Configure the servo input
  // palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
  // adc 1 ?
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_PULLUP);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {

  stop_now = true;
  while (is_running) {
    chThdSleepMilliseconds(1);
  }
}

void app_custom_configure(app_configuration *conf) { (void)conf; }

static THD_FUNCTION(my_thread, arg) {
  (void)arg;

  chRegSetThreadName("App Custom PAS");

  is_running = true;

  bool pas_input = true;
  bool pas_previous = true;
  static systime_t last_time = 0;
  static systime_t elapsed_time = 0;
  static systime_t report_time = 0;
  float cadence = 0.0;
  float throttleDuty = 0.0;
  bool switchOn1 = false;
  bool switchOn3 = false;
  float targetDuty = THROTTLE_OFF;

  for (;;) {
    // Check if it is time to stop.
    if (stop_now) {
      is_running = false;
      return;
    }

    targetDuty = THROTTLE_DUTY_MAX;

    // PAS bits
    // pas_input = palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
    pas_input = palReadPad(GPIOA, 5);
    if (pas_input != pas_previous) {
      if (pas_input == 1) {
        elapsed_time = ST2MS(chVTTimeElapsedSinceX(last_time));
        last_time = chVTGetSystemTimeX();

        cadence = calculateCadence(elapsed_time, 12);
        throttleDuty = calculateThrottleDuty(targetDuty, cadence);

        commands_printf("PAS CHANGED pas_input := %d elapsed time: %d cadence: "
                        "%f throttleDuty : %f",
                        pas_input, elapsed_time, cadence, throttleDuty);
      }
    };
    pas_previous = pas_input;

    // from adc filter the sample
    //  Optionally apply a mean value filter
    // if (config.use_filter{
    static float filter_buffer[FILTER_SAMPLES];
    static int filter_ptr = 0;
    static float throttleFilter = 0.0;

    filter_buffer[filter_ptr++] = throttleDuty;
    if (filter_ptr >= FILTER_SAMPLES) {
      filter_ptr = 0;
    }

    throttleFilter = 0.0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
      throttleFilter += filter_buffer[i];
    }
    throttleFilter /= FILTER_SAMPLES;

    if (ST2MS(chVTTimeElapsedSinceX(report_time)) > 500) {
      report_time = chVTGetSystemTimeX();
      commands_printf("cadence: %f,  throttleDuty: %f , throttleFilter: %f ---",
                      cadence, throttleDuty, throttleFilter);
    }

    if (throttleDuty == THROTTLE_OFF) {
      mc_interface_set_current(THROTTLE_OFF);
    } else {
      mc_interface_set_duty(throttleDuty);
    }
    if (last_time != 0 && ST2MS(chVTTimeElapsedSinceX(last_time)) > 400) {
      // check if the motor is running and set to off
      if (mc_interface_get_duty_cycle_now() > 0.0) {
        mc_interface_set_current(THROTTLE_OFF);
        last_time = 0;
        throttleDuty = 0;
        commands_printf("PAS TIMEOUT");
      }
    }

    timeout_reset(); // Reset timeout if everything is OK.

    // Run your logic here. A lot of functionality is available in
    // mc_interface.h.

    chThdSleepMilliseconds(10);
  }
}

static float calculateThrottleDuty(float targetDuty, float cadence) {

  float throttleDuty = THROTTLE_OFF;
  float throttleStep = 0.0;

  throttleStep = (targetDuty - THROTTLE_DUTY_MIN) / (CADENCE_MAX - CADENCE_MIN);
  if (targetDuty == THROTTLE_OFF) {
    throttleDuty = THROTTLE_OFF;
  } else if (cadence > CADENCE_MAX) {
    throttleDuty = targetDuty;
  } else if (cadence < CADENCE_MIN) {
    throttleDuty = THROTTLE_OFF;
  } else {
    // need to start collecting these up
    throttleDuty = ((cadence - CADENCE_MIN) * throttleStep) + THROTTLE_DUTY_MIN;
  }
  return throttleDuty;
}

static float calculateCadence(long edgeInterval, int cadenceMagnets) {
  float cadence = 0.0;
  if (edgeInterval > 0) {
    // this calculation always gives multiples of 5.. think it is faulty
    // cadence = (1000 / edgeInterval) * (60 / 12); //should give rpm this is
    // getting truncated somehow..

    cadence = 60000 / (edgeInterval * cadenceMagnets);
  } else {
    cadence = 0.0;
  }
  return cadence;
}
