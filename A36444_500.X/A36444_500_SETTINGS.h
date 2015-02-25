#ifndef __A36444_500_SETTINGS_H
#define __A36444_500_SETTINGS_H



// Configuration Settings for the Electromagnet Supply
#define ELECTROMAGNET_MAX_IPROG                 22000
#define ELECTROMAGNET_MIN_IPROG                 9500

#define ELECTROMAGNET_CURRENT_OVER_TRIP         24000                                 // 24 Amps
#define ELECTROMAGNET_CURRENT_UNDER_TRIP        0                                     // No under trip Point
#define ELECTROMAGNET_CURRENT_RELATIVE_TRIP     MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
#define ELECTROMAGNET_CURRENT_RELATIVE_FLOOR    2000                                  // 2 Amps
#define ELECTROMAGNET_CURRENT_TRIP_TIME         50                                    // This is in 10ms Units 

#define NOMINAL_ELECTROMAGNET_RESISTANCE        1.00                                  // 1 Ohm
#define ELECTROMAGNET_VOLTAGE_OVER_TRIP         24000                                 // 24 Volts
#define ELECTROMAGNET_VOLTAGE_UNDER_TRIP        0                                     // No under trip Point
#define ELECTROMAGNET_VOLTAGE_RELATIVE_TRIP     MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
#define ELECTROMAGNET_VOLTAGE_RELATIVE_FLOOR    2000                                  // 2 Volts
#define ELECTROMAGNET_VOLTAGE_TRIP_TIME         50                                    // This is in 10ms Units 



// Configuration Settings for the Heater Supply
#define HEATER_MAX_IPROG                        12000
#define HEATER_MIN_IPROG                        0

#define HEATER_CURRENT_OVER_TRIP                15000                                 // 15 Amps
#define HEATER_CURRENT_UNDER_TRIP               0                                     // No under trip Point
#define HEATER_CURRENT_RELATIVE_TRIP            MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
#define HEATER_CURRENT_RELATIVE_FLOOR           2000                                  // 2 Amps
#define HEATER_CURRENT_TRIP_TIME                50                                    // This is in 10ms Units 

#define NOMINAL_HEATER_RESISTANCE               1.44                                  // OHM    
#define HEATER_VOLTAGE_OVER_TRIP                24000                                 // 24 Volts
#define HEATER_VOLTAGE_UNDER_TRIP               0                                     // No under trip Point
#define HEATER_VOLTAGE_RELATIVE_TRIP            MACRO_DEC_TO_CAL_FACTOR_2(.9)        // 90%
#define HEATER_VOLTAGE_RELATIVE_FLOOR           2000                                  // 2 Volts
#define HEATER_VOLTAGE_TRIP_TIME                500                                   // This is in 10ms Units 



#endif
