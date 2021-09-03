/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 yaw.c
 Module handles:
 - Yaw GPIO initialisation
 - Yaw position sensing
 - conversion to degrees
 - Yaw target as response to user input
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "yaw.h"
#include "pwm.h"
#include "uart.h"
/*--------------------------------------------------------------*/

/* Globals -----------------------------------------------------*/
const int32_t lookup[4][4]= {     // Lookup table for change in
        { 0,  1, -1,  1},         // yaw direction based on the
        {-1,  0,  1, -1},         // previous and current encoder
        { 1, -1,  0,  1},         // states.
        {-1,  1, -1,  0}};
/*--------------------------------------------------------------*/

/* Function definitions ----------------------------------------*/

//Task: Recieves yaw quadrature state from the yaw ISR queue and increments/decrements
//the task's yaw position count depending on the yaw direction. Calculates the
//angle from the yaw position an sends it to the measured angle queue.
void yawCalculateTask(void *pvParameters) {
    encoderHandle_t yawState = {0}; // Encoder state variable
    int32_t yawPosition = 0;        // Total quadrature positions traveled
    int32_t yawAngle = 0;           // Current angle

    // Wait for ISR to give semaphore
    xSemaphoreTake(ctrlYawRefSmph, (TickType_t) 0xffff);
    xQueueReset(xYawEncoderQueue);
    xQueueReset(xMeasuredAltitudeQueue);
    xQueueReset(xMeasuredYawQueue);
    xSemaphoreGive(ctrlYawRefSmph);

    while (1) {
        while(uxQueueMessagesWaiting(xYawEncoderQueue) > 0) {
            if(xQueueReceive(xYawEncoderQueue, (void *) &yawState, (TickType_t) 10) != pdPASS) {
                uartSend("yawIntRXfail\r\n");
            }
            // Calculate the new position and angle
            yawPosition += yawChange(&yawState);
            yawAngle = yawCalcDeg(yawPosition);
        }

        // Send the yaw position to the PID controller task
        xQueueSend(xMeasuredYawQueue, (void *) &yawAngle, (TickType_t) 10);

        vTaskDelay(YAW_TASK_RATE / portTICK_RATE_MS);
    }
}

//Returns the yaw direction change depending on the given
//quadrature encoder state
int8_t yawChange(encoderHandle_t* h) {
    // update current state
    h->prevState = h->currentState;

    bool stateA = h->pins & YAW_QUAD_PIN_A;
    bool stateB = h->pins & YAW_QUAD_PIN_B;

    if (!stateA) {
        if (stateB) { h->currentState = 1; }
        else        { h->currentState = 0; }
    } else {
        if (stateB) { h->currentState = 2; }
        else        { h->currentState = 3; }
    }

    // Update yaw position based on current and prev states
    return lookup[h->prevState][h->currentState];
}


//Returns the current angle of the helicopter depending on
//the given encodeer counts
int32_t yawCalcDeg (int32_t yawPostion) {
    // Calculates current yaw position in degrees (-179 -> 180)
    // from total yaw quadrature encodes traveled
    int32_t yawAngle;
    yawAngle = (yawPostion * YAW_SCALE_NUM / YAW_SCALE_DEN) % 360;

    if (abs(yawAngle) > 180) {
        if (yawAngle > 0) {yawAngle = yawAngle - 360;}
        else              {yawAngle = yawAngle + 360;}
    }
    return yawAngle;
}

//Initialises the peripherals required to determine the yaw
//reference point and the yaw position
void yawInit (void) {
    // GPIO setup for yaw reference pin
    // GPIO setup for quadrature encoding channels
    SysCtlPeripheralEnable(YAW_QUAD_PERIPH);
    SysCtlPeripheralEnable(YAW_REF_PERIPH);
    GPIOPinTypeGPIOInput(YAW_QUAD_BASE, YAW_QUAD_PIN_A | YAW_QUAD_PIN_B);
    GPIOPinTypeGPIOInput(YAW_REF_BASE, YAW_REF_PIN);
    IntRegister(YAW_QUAD_INT_GROUP, yawIntHandler);
    IntRegister(YAW_REF_INT_GROUP, yawRefIntHandler);
    GPIOIntTypeSet(YAW_QUAD_BASE, YAW_QUAD_PIN_A | YAW_QUAD_PIN_B, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(YAW_REF_BASE, YAW_REF_PIN, GPIO_RISING_EDGE);
    GPIOIntEnable(YAW_QUAD_BASE, YAW_QUAD_PIN_A | YAW_QUAD_PIN_B);
    GPIOIntEnable(YAW_REF_BASE, YAW_REF_PIN);
    IntEnable(YAW_QUAD_INT_GROUP);
    IntEnable(YAW_REF_INT_GROUP);
}

//Interupt handler for quadrature encoder. Places encoder state
//on a queue
void yawIntHandler(void) {
    // Clear interrupt
    GPIOIntClear(YAW_QUAD_BASE, YAW_QUAD_PIN_A | YAW_QUAD_PIN_B);

    // Get pin states
    uint8_t a = GPIOPinRead(YAW_QUAD_BASE, YAW_QUAD_PIN_A);
    uint8_t b = GPIOPinRead(YAW_QUAD_BASE, YAW_QUAD_PIN_B);

    // Package both bits into one uint8_t
    uint8_t input = a | b;

    // Send position change inputs to the calculate yaw position task
    xQueueSendFromISR( xYawEncoderQueue, &input, pdFALSE );
}

//Interupt handler for yaw reference pin. Gives
//semaphore for unblocking tasks when found.
void yawRefIntHandler(void) {
    GPIOIntClear(YAW_REF_BASE, YAW_REF_PIN);
    xSemaphoreGiveFromISR(ctrlYawRefSmph, NULL);
}
/*--------------------------------------------------------------*/
