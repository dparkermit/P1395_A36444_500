#ifndef __A36444_500_H
#define __A36444_500_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>

#include "P1395_CAN_SLAVE.h"
#include "ETM_ANALOG.h"
#include "A36444_500_SETTINGS.h"
#include "P1395_MODULE_CONFIG.h"


/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module

  Timer5 - Used for 10msTicToc

  ADC Module - See Below For Specifics

*/




// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 - (unused) Fiber Trigger IN
  RA13 - (unused) Fiber Energy Select
  RA14 - (unused) Auto Inhibit Detect
  RA15 - (unused) Lambda EOC

  RD8  - (unused) Lambda Not Powered
  RD9  - PIN INPUT TEMPERATURE OK (formerly) Lambda Over Temp FLT
  RD10 - PIC INPUT CROWBAR UP     (formerly) Lamdba Interlock FLT
  RD12 - PIB INPUT HEATER OV OK   (formerly) Lambda Load FLT
  RD13 - (unused) Lambda Sum FLT
  RD14 - (unused) Lambda Phase Loss FLT
  RD15 - (unused)Lambda HV ON Readback
  
  RG14 - Reset Detect

  Analog Input Pins


  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RA9  ADC VREF-
  RA10 ADC VREF+

  RB0 PROGRAM
  RB1 PROGRAM
  RB3  - Analog Input - unused
  RB4  - Analog Input - unused
  RB5  - Analog Input - PIC ADC MAGNET IMON
  RB6  - Analog Input - PIC ADC HEATER IMON
  RB12 - Analog Input - unused 
  RB13 - Analog Input - unused
  RB14 - Analog Input - unused
  RB15 - Analog Input - unused

  RF0 CAN 1
  RF1 CAN 1
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RC1  (DAC LDAC)
  RG15 (DAC CS/LD)
  

*/

//   ------------------  Digital Output Pins ---------------
/*
  
  RD11 - Lamdba Voltage Select
  RD0 - (unused) Lambda Inhibit (This is also Output Compare 1 - If we want to use that module to generate Inhibit signal)
  RD1 - HEATER MAGNET DISABLE     (formerly) Lambda Enable


  RA7 - LED Operational
  RB8 - Test Point E
  RB9 - Test Point F
  RF4 - Test Point A
  RF5 - Test Point B
  RG0 - Test Point C
  RG1 - Test Point D
  RG12 - LED A RED
  RG13 - LED B GREEN
  
*/


#define A36444_500_TRISA_VALUE 0b1111011000000000 
#define A36444_500_TRISB_VALUE 0b1111000001111011 
#define A36444_500_TRISC_VALUE 0b0000000000000010 
#define A36444_500_TRISD_VALUE 0b1111011100000000 
#define A36444_500_TRISF_VALUE 0b0000000111000011 
#define A36444_500_TRISG_VALUE 0b1100000000001100



// -------- Digital Input Pins ----------//
#define PIN_PIC_INPUT_TEMPERATURE_OK          _RD9
#define PIN_PIC_INPUT_CROWBAR_UP              _RD10
#define PIN_PIC_INPUT_HEATER_OV_OK            _RD12
#define PIN_FIBER_ENERGY_SELECT               _RA13


#define PIN_RESET_DETECT                      _RG14

#define ILL_HEATER_OV                         1
#define ILL_TEMP_SWITCH_FAULT                 0
#define ILL_RELAY_OPEN                        1
#define ILL_ENERGY_SELECT_WATER_FLOW_OK       1



// ------- Digital Output Pins ---------//

#define PIN_HEATER_MAGNET_DISABLE             _LATD1
#define PIN_SELECT_DAC_C_D                    _LATD11

#define PIN_LED_OPERATIONAL_GREEN             _LATA7
#define PIN_LED_A_RED                         _LATG12
#define PIN_LED_B_GREEN                       _LATG13  // This is is configured by the CAN module to flash on CAN Bus activity

#define PIN_OUT_TP_A                          _LATF4
#define PIN_OUT_TP_B                          _LATF5
#define PIN_OUT_TP_C                          _LATG0
#define PIN_OUT_TP_D                          _LATG1
#define PIN_OUT_TP_E                          _LATB8
#define PIN_OUT_TP_F                          _LATB9

#define OLL_LED_ON                            0
#define OLL_CLOSE_RELAY                       0
#define OLL_SELECT_DAC_C                      1



// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN3 - (unused) Lambda Vmon
   AN4 - (unused) Lambda Heat Sink Temp
   AN5 - Magnet Imon       (formerly) Lambda VPeak
   AN6 - Heater Imon       (formerly) Lambda Imon
   
   AN12 - (unused) ADC Test Input
   AN13 - (unused) 5V Mon
   AN14 - (unused) +15V Mon
   AN15 - (unused) -15V Mon
   
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  Conversion rate of 111KHz (13.888 Khz per Channel), 138 Samples per 10mS interrupt
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN5 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN6 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)

#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)


/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)


typedef struct {
  // all currents are scaled to 1mA per lsb
  // all voltages are scaled to 1mV per lsb

  AnalogInput analog_input_heater_voltage;
  AnalogInput analog_input_heater_current;

  AnalogInput analog_input_electromagnet_voltage;
  AnalogInput analog_input_electromagnet_current;

  AnalogOutput analog_output_heater_current;
  AnalogOutput analog_output_electromagnet_current;

  unsigned int  accumulator_counter;

  unsigned int  adc_ignore_current_sample;

  unsigned int startup_count;
  unsigned int fault_active;
  unsigned int power_up_test_timer;

  unsigned int control_state;

} HeaterMagnetControlData;


extern HeaterMagnetControlData global_data_A36444_500;



#define _STATUS_MAGNET_OFF_READBACK                     _STATUS_0
#define _STATUS_HEATER_OFF_READBACK                     _STATUS_1
#define _STATUS_OUTPUT_RELAY_OPEN                       _STATUS_2
#define _STATUS_PERMA_FAULTED                           _STATUS_3


#define _FAULT_HEATER_OVER_CURRENT_ABSOLUTE             _FAULT_0
#define _FAULT_HEATER_UNDER_CURRENT_ABSOLUTE            _FAULT_1
#define _FAULT_HEATER_OVER_CURRENT_RELATIVE             _FAULT_2
#define _FAULT_HEATER_UNDER_CURRENT_RELATIVE            _FAULT_3
#define _FAULT_HEATER_OVER_VOLTAGE_ABSOLUTE             _FAULT_4
#define _FAULT_HEATER_UNDER_VOLTAGE_RELATIVE            _FAULT_5


#define _FAULT_MAGNET_OVER_CURRENT_ABSOLUTE             _FAULT_6
#define _FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE            _FAULT_7
#define _FAULT_MAGNET_OVER_CURRENT_RELATIVE             _FAULT_8
#define _FAULT_MAGNET_UNDER_CURRENT_RELATIVE            _FAULT_9
#define _FAULT_MAGNET_OVER_VOLTAGE_ABSOLUTE             _FAULT_A
#define _FAULT_MAGNET_UNDER_VOLTAGE_RELATIVE            _FAULT_B

#define _FAULT_HW_HEATER_OVER_VOLTAGE                   _FAULT_C
#define _FAULT_HW_TEMPERATURE_SWITCH                    _FAULT_D
#define _FAULT_COOLANT_FAULT                            _FAULT_E
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_F


#endif
