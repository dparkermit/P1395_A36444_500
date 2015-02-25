#include "A36444_500.h"
#include "FIRMWARE_VERSION.h"
#include "LTC265X.h"
#include "ETM_EEPROM.h"



// This is firmware for the Magnet Supply Test Board

_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


LTC265X U14_LTC2654;
HeaterMagnetControlData global_data_A36444_500;

void DisableHeaterMagnetOutputs(void);
void EnableHeaterMagnetOutputs(void);


void InitializeA36444_500(void);
void DoStateMachine(void);
void DoA36444_500(void);


#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_POWER_UP_TEST          0x30
#define STATE_OPERATE                0x40
#define STATE_FAULT                  0x50
#define STATE_FAULT_NO_RECOVERY      0x60

int main(void) {
  global_data_A36444_500.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


#define TIME_POWER_UP_TEST     1000 // 10 seconds
#define MAX_RESET_ATTEMPTS          5

void DoStateMachine(void) {

  switch (global_data_A36444_500.control_state) {
    
  case STATE_STARTUP:
    InitializeA36444_500();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    global_data_A36444_500.startup_count = 0;
    global_data_A36444_500.control_state = STATE_WAITING_FOR_CONFIG;
    break;

    
  case STATE_WAITING_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    DisableHeaterMagnetOutputs();
    while (global_data_A36444_500.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36444_500();
    
      if (_CONTROL_NOT_CONFIGURED == 0) {
	global_data_A36444_500.control_state = STATE_POWER_UP_TEST;
      }
    }
    break;

    
  case STATE_POWER_UP_TEST:
    global_data_A36444_500.startup_count++;
    _CONTROL_NOT_READY = 1;
    global_data_A36444_500.power_up_test_timer = 0;
    EnableHeaterMagnetOutputs();
    while (global_data_A36444_500.control_state == STATE_POWER_UP_TEST) {
      DoA36444_500();
  
      if (global_data_A36444_500.power_up_test_timer >= TIME_POWER_UP_TEST) {
	// We passed the warmup time without a fault, clear the startup counter
	global_data_A36444_500.startup_count = 0;
	global_data_A36444_500.power_up_test_timer = TIME_POWER_UP_TEST;
	// We can moce to the operate sate if there are no latched faults or if the reset is active
	if ((_FAULT_REGISTER == 0) || (_SYNC_CONTROL_RESET_ENABLE)) {
	  global_data_A36444_500.control_state = STATE_OPERATE;
	}
      }
      
      if (global_data_A36444_500.fault_active) {
	if (global_data_A36444_500.startup_count <= MAX_RESET_ATTEMPTS){
	  global_data_A36444_500.control_state = STATE_FAULT;
	} else {
	  global_data_A36444_500.control_state = STATE_FAULT_NO_RECOVERY;
	}
      }
    }
    break;


  case STATE_OPERATE:
    _CONTROL_NOT_READY = 0;
    _FAULT_REGISTER = 0;
    while (global_data_A36444_500.control_state == STATE_OPERATE) {
      DoA36444_500();
      
      if (global_data_A36444_500.fault_active) {
	global_data_A36444_500.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    DisableHeaterMagnetOutputs();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36444_500.control_state == STATE_FAULT) {
      DoA36444_500();

      if (!global_data_A36444_500.fault_active) {
	global_data_A36444_500.control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
    
  case STATE_FAULT_NO_RECOVERY:
    DisableHeaterMagnetOutputs();
    _CONTROL_NOT_READY = 1;
    _STATUS_PERMA_FAULTED = 1;
    while (global_data_A36444_500.control_state == STATE_FAULT_NO_RECOVERY) {
      DoA36444_500();
    }
    
  default:
    global_data_A36444_500.control_state = STATE_FAULT_NO_RECOVERY;
    break;
    
  }
}


void DisableHeaterMagnetOutputs(void) {
  global_data_A36444_500.analog_output_heater_current.enabled = 0;
  global_data_A36444_500.analog_output_electromagnet_current.enabled = 0;
  PIN_HEATER_MAGNET_DISABLE = !OLL_CLOSE_RELAY;
}


void EnableHeaterMagnetOutputs(void) {
  global_data_A36444_500.analog_output_heater_current.enabled = 1;
  global_data_A36444_500.analog_output_electromagnet_current.enabled = 1;
  PIN_HEATER_MAGNET_DISABLE = OLL_CLOSE_RELAY;  
}












void DoA36444_500(void) {
  ETMCanSlaveDoCan();
  
  
  // Check the status of these pins every time through the loop
  if (PIN_PIC_INPUT_HEATER_OV_OK == ILL_HEATER_OV) {
    _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
    global_data_A36444_500.fault_active = 1;
  }
  
  if (PIN_PIC_INPUT_TEMPERATURE_OK == ILL_TEMP_SWITCH_FAULT) {
    _FAULT_HW_TEMPERATURE_SWITCH = 1;
    global_data_A36444_500.fault_active = 1;
  }

  
  if (_T5IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms unless the configuration file is changes
    _T5IF = 0;
    
    local_debug_data.debug_0 = global_data_A36444_500.startup_count;
    local_debug_data.debug_1 = global_data_A36444_500.fault_active;
    local_debug_data.debug_2 = global_data_A36444_500.power_up_test_timer;
    local_debug_data.debug_3 = global_data_A36444_500.control_state;
    local_debug_data.debug_4 = _SYNC_CONTROL_WORD;

    
    if (global_data_A36444_500.control_state == STATE_POWER_UP_TEST) {
      global_data_A36444_500.power_up_test_timer++;
    }

    // Update the error counters that get returned
    local_debug_data.i2c_bus_error_count = 0;  // There are no I2C devices on this board
    local_debug_data.spi_bus_error_count = etm_spi1_error_count + etm_spi2_error_count;
    local_debug_data.scale_error_count = etm_scale_saturation_etmscalefactor2_count + etm_scale_saturation_etmscalefactor16_count;

    // If the system is faulted or inhibited set the red LED
    if (_CONTROL_NOT_READY) {
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }


    // Update the digital input status pins
    if (PIN_PIC_INPUT_CROWBAR_UP == ILL_RELAY_OPEN) {
      _STATUS_OUTPUT_RELAY_OPEN = 1;
    } else {
      _STATUS_OUTPUT_RELAY_OPEN = 0;
    }
    
    
    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444_500.analog_input_electromagnet_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444_500.analog_input_electromagnet_voltage);
   
// -------------------- CHECK FOR FAULTS ------------------- //

    global_data_A36444_500.fault_active = 0;
   
    if (PIN_PIC_INPUT_HEATER_OV_OK == ILL_HEATER_OV) {
      _FAULT_HW_HEATER_OVER_VOLTAGE = 1;
      global_data_A36444_500.fault_active = 1;
    }
    
    if (PIN_PIC_INPUT_TEMPERATURE_OK == ILL_TEMP_SWITCH_FAULT) {
      _FAULT_HW_TEMPERATURE_SWITCH = 1;
      global_data_A36444_500.fault_active = 1;
    } 

    // DPARKER check SYNC message for coolant flow and fault if there is a problem
    if (_SYNC_CONTROL_COOLING_FAULT) {
      _FAULT_COOLANT_FAULT = 1;
      global_data_A36444_500.fault_active = 1;
    } else {
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_FAULT_COOLANT_FAULT = 0;
      }
    }

    if (_CONTROL_CAN_COM_LOSS) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 1;
      global_data_A36444_500.fault_active = 1;
    }

    if ((global_data_A36444_500.control_state == STATE_OPERATE) || (global_data_A36444_500.control_state == STATE_POWER_UP_TEST)) {
      global_data_A36444_500.analog_input_electromagnet_current.target_value = global_data_A36444_500.analog_output_electromagnet_current.set_point;
      global_data_A36444_500.analog_input_heater_current.target_value = global_data_A36444_500.analog_output_heater_current.set_point;
      
      if (ETMAnalogCheckOverAbsolute(&global_data_A36444_500.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36444_500.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36444_500.analog_input_heater_current)) {
	_FAULT_HEATER_OVER_CURRENT_RELATIVE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36444_500.analog_input_heater_current)) {
	_FAULT_HEATER_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36444_500.fault_active = 1;
      }


      
      if (ETMAnalogCheckOverAbsolute(&global_data_A36444_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_ABSOLUTE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36444_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_ABSOLUTE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckOverRelative(&global_data_A36444_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_OVER_CURRENT_RELATIVE = 1;
	global_data_A36444_500.fault_active = 1;
      }
      if (ETMAnalogCheckUnderRelative(&global_data_A36444_500.analog_input_electromagnet_current)) {
	_FAULT_MAGNET_UNDER_CURRENT_RELATIVE = 1;
	global_data_A36444_500.fault_active = 1;
      }
    } else {
      global_data_A36444_500.analog_input_electromagnet_current.target_value = 0;
      global_data_A36444_500.analog_input_heater_current.target_value = 0;
    }
   
    // Set DAC outputs
    if ((global_data_A36444_500.control_state == STATE_OPERATE) || (global_data_A36444_500.control_state == STATE_POWER_UP_TEST)) {
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36444_500.analog_output_heater_current);
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36444_500.analog_output_electromagnet_current);

      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36444_500.analog_output_electromagnet_current.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36444_500.analog_output_heater_current.dac_setting_scaled_and_calibrated);

      
    } else {
      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36444_500.analog_output_electromagnet_current.disabled_dac_set_point,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36444_500.analog_output_heater_current.disabled_dac_set_point);
    }

  }
}












void InitializeA36444_500(void) {
  unsigned int startup_counter;


  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;



  // Initialize the Analog Input * Output Scaling
  ETMAnalogInitializeOutput(&global_data_A36444_500.analog_output_electromagnet_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.6),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_0,
			    ELECTROMAGNET_MAX_IPROG,
			    ELECTROMAGNET_MIN_IPROG,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36444_500.analog_output_heater_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1.6),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_2,
			    HEATER_MAX_IPROG,
			    HEATER_MIN_IPROG,
			    0);
  
  ETMAnalogInitializeInput(&global_data_A36444_500.analog_input_electromagnet_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.6250),
			   OFFSET_ZERO,
			   ANALOG_INPUT_5,
			   ELECTROMAGNET_CURRENT_OVER_TRIP,
			   ELECTROMAGNET_CURRENT_UNDER_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_TRIP,
			   ELECTROMAGNET_CURRENT_RELATIVE_FLOOR,
			   ELECTROMAGNET_CURRENT_TRIP_TIME);
  
  ETMAnalogInitializeInput(&global_data_A36444_500.analog_input_heater_current,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.6250),
			   OFFSET_ZERO,
			   ANALOG_INPUT_6,
			   HEATER_CURRENT_OVER_TRIP,
			   HEATER_CURRENT_UNDER_TRIP,
			   HEATER_CURRENT_RELATIVE_TRIP,
			   HEATER_CURRENT_RELATIVE_FLOOR,
			   HEATER_CURRENT_TRIP_TIME);

  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  

  global_data_A36444_500.analog_output_electromagnet_current.set_point = 0;
  global_data_A36444_500.analog_output_heater_current.set_point = 0;

  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  // Initialize all I/O Registers
  TRISA = A36444_500_TRISA_VALUE;
  TRISB = A36444_500_TRISB_VALUE;
  TRISC = A36444_500_TRISC_VALUE;
  TRISD = A36444_500_TRISD_VALUE;
  TRISF = A36444_500_TRISF_VALUE;
  TRISG = A36444_500_TRISG_VALUE;

  PIN_SELECT_DAC_C_D = OLL_SELECT_DAC_C;
  
  
  // Initialize TMR5
  PR5   = PR5_VALUE_10_MILLISECONDS;
  TMR5  = 0;
  _T5IF = 0;
  T5CON = T5CON_VALUE;


  // Initialize internal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = 0b0000111100000000;//ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  
  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

  
  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize the Can module
  ETMCanSlaveInitialize();


  // Flash LEDs at Startup
  startup_counter = 0;
  while (startup_counter <= 400) {  // 4 Seconds total
    ETMCanSlaveDoCan();
    if (_T5IF) {
      _T5IF =0;
      startup_counter++;
    } 
    switch (((startup_counter >> 4) & 0b11)) {
      
    case 0:
      PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;
      
    case 1:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;
      
    case 2:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 3:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = OLL_LED_ON;
      break;
    }
  }
  
  PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
  
  _CONTROL_SELF_CHECK_ERROR = 0;

}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  if (global_data_A36444_500.adc_ignore_current_sample) {
    // There was a pulse durring the sample sequence.  Throw the data away!!!
    global_data_A36444_500.adc_ignore_current_sample = 0;
  } else {
    // Copy Data From Buffer to RAM
    if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator += ADCBUF0 + ADCBUF4;
      global_data_A36444_500.analog_input_heater_current.adc_accumulator        += ADCBUF1 + ADCBUF5;
      global_data_A36444_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUF2 + ADCBUF6;
      global_data_A36444_500.analog_input_heater_voltage.adc_accumulator        += ADCBUF3 + ADCBUF7;
    } else {
      // read ADCBUF 8-15
      global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator += ADCBUF8 + ADCBUFC;
      global_data_A36444_500.analog_input_heater_current.adc_accumulator        += ADCBUF9 + ADCBUFD;
      global_data_A36444_500.analog_input_electromagnet_voltage.adc_accumulator += ADCBUFA + ADCBUFE;
      global_data_A36444_500.analog_input_heater_voltage.adc_accumulator        += ADCBUFB + ADCBUFF;
    }
    
    global_data_A36444_500.accumulator_counter += 2;
    
    if (global_data_A36444_500.accumulator_counter >= 256) {

      global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36444_500.analog_input_electromagnet_current.filtered_adc_reading = global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator;
      global_data_A36444_500.analog_input_electromagnet_current.adc_accumulator = 0;
      
      global_data_A36444_500.analog_input_electromagnet_voltage.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36444_500.analog_input_electromagnet_voltage.filtered_adc_reading = global_data_A36444_500.analog_input_electromagnet_voltage.adc_accumulator;
      global_data_A36444_500.analog_input_electromagnet_voltage.adc_accumulator = 0;

      global_data_A36444_500.analog_input_heater_current.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36444_500.analog_input_heater_current.filtered_adc_reading = global_data_A36444_500.analog_input_heater_current.adc_accumulator;
      global_data_A36444_500.analog_input_heater_current.adc_accumulator = 0;

      global_data_A36444_500.analog_input_heater_voltage.adc_accumulator >>= 4;  // This is now a 16 bit number average of previous 256 samples 
      global_data_A36444_500.analog_input_heater_voltage.filtered_adc_reading = global_data_A36444_500.analog_input_heater_voltage.adc_accumulator;
      global_data_A36444_500.analog_input_heater_voltage.adc_accumulator = 0;

      global_data_A36444_500.accumulator_counter = 0;
    }
  }
}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}



