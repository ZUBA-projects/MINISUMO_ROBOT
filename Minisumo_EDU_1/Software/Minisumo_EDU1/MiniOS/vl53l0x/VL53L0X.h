#ifndef VL53L0X_h
#define VL53L0X_h


#include "../../hal.h"

//------------------------------------------------------------
// Defines
//------------------------------------------------------------
// I use a 8-bit number for the address, LSB must be 0 so that I can
// OR over the last bit correctly based on reads and writes

#define ADDRESS_DEFAULT 		0b00101001
#define MAX_VL53L0X_ERR_COUNTS		5	//max counted timeouts and errors to decide about deleting sensor

#define VL53l0X_TOUT_CFG			100

// register addresses from API vl53l0x_device.h (ordered as listed there)
enum regAddr {
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

typedef enum {
	VcselPeriodPreRange,
	VcselPeriodFinalRange
}vcselPeriodType;

typedef enum{
	VL53L0X_IS_INITED=0x01,
	VL53L0X_TOUTERR=0x02,
	VL53L0X_DONE=0x04,
}vl53l0xstate_t;

typedef struct{
	//gpio config
	uint8_t _gpio0pin;
	uint8_t _gpio1pin;
	uint8_t _sclpin;
	uint8_t _sdapin;

	//config of displaying place
	uint8_t _dpposx;
	uint8_t _dpposy;

	//state of sensor
	uint8_t _sensorstatebus;
	uint16_t _lastrange;

	//sensor config
	uint8_t g_i2cAddr;
	int16_t _positionangle;

	//private data for sensor working
	uint32_t g_timeoutStartMs;
	uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	uint32_t g_measTimBudUs;
	uint8_t _errcount;

}vl53l0x_t;

typedef struct{
	//all range sensor module config
	uint16_t _timmingbudget;
	uint16_t _timmingpetroid;
	uint32_t _SignalRateLimit;
	uint8_t _VcselPulsePeriodFinal;
	uint8_t _VcselPulsePeriodPre;
}rangesensorconfig_t;




// Additional info for one measurement
typedef struct{
  uint16_t rawDistance; //uncorrected distance  [mm],   uint16_t
  uint16_t signalCnt;   //Signal  Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t ambientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t spadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
  uint8_t  rangeStatus; //Ranging status (0-15)
} statInfo_t;


//------------------------------------------------------------
// API Functions
//------------------------------------------------------------
// configures chip i2c and lib for `new_addr` (8 bit, LSB=0)
void setAddress(uint8_t new_addr);
// Returns the current I²C address.
uint8_t getAddress(void);

// Iniitializes and configures the sensor. 
// If the optional argument io_2v8 is 1, the sensor is configured for 2V8 mode (2.8 V I/O); 
// if 0, the sensor is left in 1V8 mode. Returns 1 if the initialization completed successfully.
uint8_t initVL53L0X(uint8_t io_2v8);

// Sets the return signal rate limit to the given value in units of MCPS (mega counts per second). 
// This is the minimum amplitude of the signal reflected from the target and received by the sensor 
//  necessary for it to report a valid reading. Setting a lower limit increases the potential range 
// of the sensor but also increases the likelihood of getting an inaccurate reading because of 
//  reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS 
//  by default. The return value is a boolean indicating whether the requested limit was valid.
//uint8_t setSignalRateLimit(float limit_Mcps);
uint8_t setSignalRateLimit(uint32_t limit_Mcps);


// Returns the current return signal rate limit in MCPS.
//float getSignalRateLimit(void);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
uint8_t setMeasurementTimingBudget(uint32_t budget_us);

// Returns the current measurement timing budget in microseconds.
uint32_t getMeasurementTimingBudget(void);

// Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type
// (VcselPeriodPreRange or VcselPeriodFinalRange) to the given value (in PCLKs). 
// Longer periods increase the potential range of the sensor. Valid values are (even numbers only):
// Pre: 12 to 18 (initialized to 14 by default)
// Final: 8 to 14 (initialized to 10 by default)
// The return value is a boolean indicating whether the requested period was valid.
uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

// Returns the current VCSEL pulse period for the given period type.
uint8_t getVcselPulsePeriod(vcselPeriodType type);

// Starts continuous ranging measurements. If the argument period_ms is 0, 
// continuous back-to-back mode is used (the sensor takes measurements as often as possible); 
// if it is nonzero, continuous timed mode is used, with the specified inter-measurement period 
// in milliseconds determining how often the sensor takes a measurement.
void startContinuous(uint32_t period_ms);

// Stops continuous mode.
void stopContinuous(void);

// Returns a range reading in millimeters when continuous mode is active.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeContinuousMillimeters( statInfo_t *extraStats );

// Performs a single-shot ranging measurement and returns the reading in millimeters.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeSingleMillimeters( statInfo_t *extraStats );

// Sets a timeout period in milliseconds after which read operations will abort 
// if the sensor is not ready. A value of 0 disables the timeout.
void setTimeout(uint32_t timeout);


//set up current sensor instance

void setvl53l0xinstance(vl53l0x_t * _vl53l0xsensor);

//get sensor status
uint8_t  checkRangeContinousSensor();

//read range sensor value and clear system int

uint16_t readRangeContinousSensor();

void initVL53L0X_to_continous(rangesensorconfig_t *_cfg);

void preinitVL53L0X();

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

#endif
