/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 pid.c

 Module handles PID calculations for both main and tail
 rotors. Calculates resulting PWM values for main and tail
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "altitude.h"
#include "userInput.h"
#include "pid.h"
#include "pwm.h"
#include "yaw.h"
#include "uart.h"
#include "control.h"
/*--------------------------------------------------------------*/

/* Function definitions ----------------------------------------*/

// Task: defines pid handles for main and tail rotors. Receieves current
// and target values for each rotor. Sends calculated pwm values to the
// pwm queue
void pidTask(void *pvParameters) {
    pidHandle_t altitude = {0};
    altitude.kp = MAIN_PROP_GAIN;
    altitude.ki = MAIN_INT_GAIN;
    altitude.integralError = -(MAIN_DUTY_OFFSET*1000/2 / MAIN_INT_GAIN);
    altitude.kd = MAIN_DIFF_GAIN;
    altitude.offset = MAIN_DUTY_OFFSET;
    pidHandle_t yaw = {0};
    yaw.kp = TAIL_PROP_GAIN;
    yaw.ki = TAIL_INT_GAIN;
    yaw.kd = TAIL_DIFF_GAIN;
    yaw.offset = TAIL_DUTY_OFFSET;

    int i;
    while (1) {
        i++;
        // Get target
        while(uxQueueMessagesWaiting(xControlTargetQueue) > 0) {
            controlTargetMessage_t recievedTarget;
            if(xQueueReceive(xControlTargetQueue, (void *) &recievedTarget, (TickType_t) 10) != pdPASS) {
                uartSend("targetRxFail\r\n");
            }
            altitude.target = recievedTarget.altitude;
            yaw.target = recievedTarget.yaw;
        }
        // Get current position values
        while(uxQueueMessagesWaiting(xMeasuredAltitudeQueue) > 0) {
            if(xQueueReceive(xMeasuredAltitudeQueue, (void *) &altitude.current, (TickType_t) 10) != pdPASS) {
                uartSend("altCurrentRxFail\r\n");
            }
        }
        while(uxQueueMessagesWaiting(xMeasuredYawQueue) > 0) {
            if(xQueueReceive(xMeasuredYawQueue, (void *) &yaw.current, (TickType_t) 10) != pdPASS) {
                uartSend("yawRxFail\r\n");
            }
        }

        // Calculate the errors
        pidCalcErrors(&altitude);
        pidCalcErrors(&yaw);

        if (altitude.integralError >  MAIN_ERROR_MAX) {altitude.integralError =  MAIN_ERROR_MAX;}
        if (altitude.integralError < -MAIN_ERROR_MAX) {altitude.integralError = -MAIN_ERROR_MAX;}
        if (yaw.integralError >  TAIL_ERROR_MAX) {yaw.integralError =  TAIL_ERROR_MAX;}
        if (yaw.integralError < -TAIL_ERROR_MAX) {yaw.integralError = -TAIL_ERROR_MAX;}

        // Calculate the PWM duty cycles
        pwmUpdateMessage_t pwm;
        pwm.main = pidCalcDutyCycle(&altitude);
        pwm.tail = pidCalcDutyCycle(&yaw);

        // Send calculated values to the the pwm update task
        xQueueSend(xPWMQueue, (void *) &pwm, (TickType_t) 10);

        // Print a bunch of shit
        if (i % 10 == 0) {
            char str[14];
            usprintf(str, "Alt: %d [%d] %4d\r\n", altitude.current, altitude.target, pwm.main);
            uartSend(str);
            usprintf(str, "Yaw: %d [%d] %4d, %4d\r\n\r\n", yaw.current, yaw.target, pwm.tail, yaw.integralError);
            uartSend(str);
        }
        //Task delay
        vTaskDelay(PID_TASK_DELAY / portTICK_RATE_MS);
    }
}

// Current PID error calculators
void pidCalcErrors(pidHandle_t* h) {
    int prevAltError = h->propError;
    h->propError = h->target - h->current; // calculate proportional error
    if(h->propError > 180) {
        h->propError -= 360;
    }
    if (h->propError <= -180) {
        h->propError += 360;
    }
    h->integralError += h->propError; // calculate total integral error
    h->diffError = h->propError - prevAltError; // calculate differential error
}

// Uses PID control to calculate duty cycle for each rotor
int16_t pidCalcDutyCycle(pidHandle_t* h) {
    // Update main rotor duty cycle, impose limits
    int16_t dutyCycle =
            (h->propError * h->kp/1000) +
            (h->integralError * h->ki/1000) +
            (h->diffError * h->kd/1000) +
            h->offset;
    if      (dutyCycle > MAIN_MAX_DUTY) {dutyCycle = MAIN_MAX_DUTY;}
    else if (dutyCycle < MAIN_MIN_DUTY) {dutyCycle = MAIN_MIN_DUTY;}
    return dutyCycle;
}
/*--------------------------------------------------------------*/
