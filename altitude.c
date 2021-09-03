/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 altitude.c

 Interrupt driven collection of altitude adc values. Inserts
 adc values into a circular buffer and calculates an average. Converts
 Average into an altitude percentage between 0 and 100 and adds to
 altitue queue
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "altitude.h"
#include "uart.h"
#include "circBufT.h"
/*--------------------------------------------------------------*/

// Altitude ADC sample trigger task.
// This task sets the sampling rate of the ADC
void altitudeADCSamplingTask(void *pvParameters) {
    while(1){
        // Trigger an ADC sample capture
        ADCProcessorTrigger(ADC0_BASE, 3);
        // Task delay
        vTaskDelay(ALT_ADC_SAMPLE_DELAY / portTICK_RATE_MS);
    }
}

// Interrupt handler for ADC conversion completion
void altitudeADCIntHandler (void) {
    // Get value
    uint32_t ADCValue;
    ADCSequenceDataGet(ALTITUDE_ADC_BASE, ALTITUDE_ADC_SEQ_NUM, &ADCValue);

    // Put the value onto the queue
    xQueueSendFromISR(xAltitudeADCQueue, &ADCValue, pdFALSE );

    // Clear Interrupt
    ADCIntClear(ALTITUDE_ADC_BASE, ALTITUDE_ADC_SEQ_NUM);
}

// Main altitude task. Calculates the current altitude and sends
// current position to a queue for the PID controller to consume
void altitudeAvgTask(void *ptr) {
    circBuf_t altitudeADCBuffer;
    uint32_t buffer[ALT_ADC_BUF_LENGTH];
    altitudeADCBuffer.size = ALT_ADC_BUF_LENGTH;
    altitudeADCBuffer.data = buffer;
    altitudeADCBuffer.windex = 0;
    altitudeADCBuffer.rindex = 0;

    // Find ADC value for 0% and 100% height
    const uint32_t altADCValueAt0 = altitudeFindADCValueAt0(&altitudeADCBuffer);
    const uint32_t altADCValueAt100 = altADCValueAt0 - ALT_ADC_SWING;

    while (1) {
        int32_t avgAltitudeADCValue;
        int32_t avgAltitudePcnt;
        uint32_t ADCValue;

        // Get the ADC value from the queue when ready
        while(uxQueueMessagesWaiting(xAltitudeADCQueue) > 0) {
            if(xQueueReceive(xAltitudeADCQueue, (void *) &ADCValue, (TickType_t) 10) != pdPASS) {
                uartSend("altADCRxFail\r\n");
            }
            // Write the ADC values to the circular buffer
            writeCircBuf (&altitudeADCBuffer, ADCValue);
        }

        // Calculate average ADCvalue
        avgAltitudeADCValue = calcAvgAltADC(&altitudeADCBuffer);

        // Calculate altitude percentage and send it to the queue to be processed by the PID controller
        avgAltitudePcnt = 100 * (altADCValueAt0 - avgAltitudeADCValue) / (altADCValueAt0 - altADCValueAt100);
        if (avgAltitudePcnt > 250 || avgAltitudePcnt < 5) {avgAltitudePcnt = 0;}

        xQueueSend(xMeasuredAltitudeQueue, (void *) &avgAltitudePcnt, (TickType_t) 10);
        // Task delay
        vTaskDelay(ALTITUDE_DELAY/ portTICK_RATE_MS);
    }

}

// Initalise the altitude adc buffer.
// Fill for first time and return initial average value
uint32_t altitudeFindADCValueAt0(circBuf_t* altitudeADCBuffer) {
    int i;
    uint32_t ADCValue;
    uint32_t maxInitialValue;
    for(i = 0; i <= 10*ALT_ADC_BUF_LENGTH; i++) {
        // Trigger the sample
        ADCProcessorTrigger(ADC0_BASE, ALTITUDE_ADC_SEQ_NUM);
        // Wait for the sample to be sent to the queue
        while(uxQueueMessagesWaiting(xAltitudeADCQueue) == 0) {}
        // Get the sample
        if(xQueueReceive(xAltitudeADCQueue, (void *) &ADCValue, (TickType_t) 10) != pdPASS) {
            uartSend("altADCRxFail\r\n");
        }
        if (ADCValue > maxInitialValue) {
            maxInitialValue = ADCValue;
        }
        writeCircBuf(altitudeADCBuffer, ADCValue);
   }
   return maxInitialValue;
}

// Calculate the average value of the buffer
uint32_t calcAvgAltADC(circBuf_t* altBuf) {
    uint32_t sum = 0;
    // find the sum of the circular buffer
    int i;
    for (i = 0; i < ALT_ADC_BUF_LENGTH; i++) {
        sum += readCircBuf(altBuf);
    }
    // Calculate the rounded mean of the buffer contents
    return  (2 * sum + ALT_ADC_BUF_LENGTH) / 2 / ALT_ADC_BUF_LENGTH;
}

// Altitude ADC initialization function
void altitudeInitADC (void) {
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(ALTITUDE_ADC_PERIPH);

    // Wait for peripheral to be ready
    while(!SysCtlPeripheralReady(ALTITUDE_ADC_PERIPH));

    // Configure altitude ADC sample sequence using
    // processor trigger. This converts a single sample each time
    // the processor triggers the ADC with priority 0.
    ADCSequenceConfigure(ALTITUDE_ADC_BASE,
                         ALTITUDE_ADC_SEQ_NUM,
                         ALTITUDE_ADC_TRIGGER,
                         ALTITUDE_ADC_PRIORITY);

    // Configure altitude ADC step on sequence: ALTITUDE_ADC_SEQ_NUM.
    // ADC_CTL_IE configures the ADC to trigger an interrupt
    // when each sample is acquired.
    ADCSequenceStepConfigure(ALTITUDE_ADC_BASE,
                             ALTITUDE_ADC_SEQ_NUM,
                             ALTITUDE_ADC_STEP,
                             ALTITUDE_ADC_CHANNEL | ADC_CTL_IE | ADC_CTL_END);

    // Register the interrupt handler for the sequence step configured above
    ADCIntRegister (ALTITUDE_ADC_BASE, ALTITUDE_ADC_SEQ_NUM, altitudeADCIntHandler);

    // Enable altitude ADC sample sequence
    ADCSequenceEnable(ALTITUDE_ADC_BASE, ALTITUDE_ADC_SEQ_NUM);

    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ALTITUDE_ADC_BASE, ALTITUDE_ADC_SEQ_NUM);
}
