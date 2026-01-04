// /* adg5436.h - GPIO-controlled ADG5436 helper
//  *
//  * ADG5436 is a dual SPDT analog switch.
//  * In TSSOP package, there is no EN pin.
//  * In LFCSP package, there is EN pin (active high).
//  *
//  * We treat IN1 and IN2 as tied together by hardware and driven by one GPIO.
//  */

// #pragma once

// #include <stdbool.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// int adg5436_init(void);

// /* phase=false/true maps to GPIO low/high (respecting DT GPIO flags). */
// int adg5436_set_phase(bool phase);

// /* Optional enable control (only if DT alias adg5436_en exists). */
// int adg5436_enable(bool enable);

// #ifdef __cplusplus
// }
// #endif
