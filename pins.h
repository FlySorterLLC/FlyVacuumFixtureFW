///////////////////////////////////////
//  Copyright 2018, FlySorter LLC    //
//  Subject to terms and conditions  //
//  as defined in LICENSE file that  //
//  accompanies this distribution.   //
///////////////////////////////////////

#ifndef __PINS_H__
#define __PINS_H__

// INPUTS
#define PRESSURE_SENSOR_ADC 7      // Analog pin PIN_F7
#define PHOTOGATE           PIN_B4

// SWITCH INPUTS
#define EJECT_SW_OPEN       PIN_D4
#define EJECT_SW_CLOSED     PIN_D1
#define GATE_SW_INPUT       PIN_D3
#define GATE_SW_CLOSED      PIN_D2
#define GATE_SW_OUTPUT      PIN_D0
#define NEEDLE_SW           PIN_E1

// OUTPUTS

// LEDs

#define ILLUM_EN1           PIN_B6
#define ILLUM_EN2           PIN_B7
#define LURE_EN             PIN_B5

// SOLENOIDS

#define PUSH_EN             PIN_A2
#define EJECT_EN            PIN_A1
#define NEEDLE_NEG_EN       PIN_A3
#define SPARE_EN            PIN_A4

// MOTORS

#define NEEDLE_PWM          PIN_C6
#define NEEDLE_FWD          PIN_C1
#define NEEDLE_REV          PIN_C0

#define GATE_PWM            PIN_C4
#define GATE_FWD            PIN_E0
#define GATE_REV            PIN_D7

#define EJECT_PWM           PIN_C5
#define EJECT_FWD           PIN_D6
#define EJECT_REV           PIN_D5


#endif

