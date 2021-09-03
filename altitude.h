/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 altitude.h

 Interrupt driven collection of altitude adc values. Inserts
 adc values into a circular buffer and calculates an average. Converts
 Average into an altitude percentage between 0 and 100 and adds to
 altitue queue
----------------------------------------------------------------*/

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

/* Includes -----------------------------------------*/
#include "circBufT.h"
/*--------------------------------------------------------------*/

/* Definitions -------------------------------------------------*/
// Altitude scaling factors
#define ALT_ADC_SWING        800 // Number of adc points between 0% and 100%
#define ALT_ADC_NOISE_MARGIN 100
// Altitude setting constants
#define ALT_TARGET_MAX       100
#define ALT_TARGET_MIN       0
#define ALT_TARGET_STEP      10
// Altitude ADC parameters
#define ALTITUDE_DELAY        7  // ms between averaging
#define ALT_ADC_BUF_LENGTH    10
#define ALT_ADC_SAMPLE_DELAY  2  // ms between samples. 4ms = 250Hz

/*--------------------------------------------------------------*/

/* Globals -------------------------------------------------*/
//external global queue handles
extern QueueHandle_t xMeasuredAltitudeQueue;
extern QueueHandle_t xAltitudeADCQueue;
extern QueueHandle_t xTelemetryQueue;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/
void altitudeInitADC(void);

// ISR to send complete ADC conversions to the queue
void altitudeADCIntHandler(void);

// Task to trigger ADC samples
void altitudeADCSamplingTask(void *pvParameters);

// Task to average the ADC values and send to PID controller
void altitudeAvgTask(void *pvParameters);

// Finds the ADC value at 0% altitude
uint32_t altitudeFindADCValueAt0(circBuf_t* altitudeADCBuffer);

// Calculates the average ADC value from the buffer
uint32_t calcAvgAltADC(circBuf_t* altBuf);

/*--------------------------------------------------------------*/

#endif /* ALTITUDE_H_ */
