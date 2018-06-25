///////////////////////////////////////
//  Copyright 2017, FlySorter LLC    //
//  Subject to terms and conditions  //
//  as defined in LICENSE file that  //
//  accompanies this distribution.   //
///////////////////////////////////////

#ifndef __PINS_H__
#define __PINS_H__

#define PRESSURE_SENSOR_ADC 7 // Analog pin PIN_F7
#define PEN_TIP_PHOTOGATE PIN_E4

// These come from the datasheet for the processor (AT90USB1286)
// E.g. Pin PE5 is also labeled INT5, pin PE4 is INT4, and so on

#define PEN_TIP_PHOTOGATE_INT 4

#define LSWITCH_INLET    PIN_A5
#define LSWITCH_OUTLET   PIN_A6
#define LSWITCH_MID      PIN_C7

#define BUTTON_IN         PIN_A7

#define VACUUM_ENABLE     PIN_F2
#define CAPTURE_ENABLE    PIN_E6
#define DISPENSE_ENABLE   PIN_F0
#define EJECT_ENABLE      PIN_F1

// MOTOR1 on PCB
#define GEAR_MOTOR_PWM    PIN_C5
#define GEAR_MOTOR_IN1    PIN_F5
#define GEAR_MOTOR_IN2    PIN_F6

// MOTOR2 on PCB
#define TAPPER_MOTOR_PWM  PIN_C6
#define TAPPER_MOTOR_IN1  PIN_F3
#define TAPPER_MOTOR_IN2  PIN_F4

#define SPEAKER           PIN_C4

#define LED1              PIN_A3
#define LED2              PIN_A4


#define BUTTONA           PIN_D2
#define BUTTONB           PIN_D3

#define FLASH_ENABLE      PIN_C0

// These come from the datasheet for the processor (AT90USB1286)
// E.g. Pin PD2 is also labeled INT2
#define BUTTONA_INT       2
#define BUTTONB_INT       3


#endif

