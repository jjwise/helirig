/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 pwm.h

 Setup of PWM generators for main and tail rotors and
 Update PWM duty cycle functions for both. PWM freq is
 kept constant at PWM_FREQ_HZ
 Main rotor PWM signal: Pin PC5  (M0PWM7).
 Tail rotor PWM signal: Pin PF1  (M1PWM5).
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "pwm.h"
#include "uart.h"
/*--------------------------------------------------------------*/

//Recieves PWM vaues from queue and updates the main and tail rotor PWM
void pwmTask(void *pvParameters) {
    while(1) {
        pwmUpdateMessage_t recievedMessage;
        while(uxQueueMessagesWaiting(xPWMQueue) > 0) {
            if(xQueueReceive(xPWMQueue, (void *) &recievedMessage, (TickType_t) 10) != pdPASS) {
                uartSend("pwmRxFail\r\n");
            }
            pwmUpdateTail(recievedMessage.tail);
            pwmUpdateMain(recievedMessage.main);
        }
        vTaskDelay(PWM_UPDATE_RATE / portTICK_RATE_MS);
    }
}

// Update main PWM duty cycle
void pwmUpdateMain (uint32_t duty) {
    // Calculate the PWM period corresponding to the freq.
    uint32_t pwmPeriod = PWMGenPeriodGet(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, (pwmPeriod * duty) / 100);
}

// Update tail PWM duty cycle
void pwmUpdateTail (uint32_t duty) {
    // Calculate the PWM period corresponding to the freq.
    uint32_t pwmPeriod = PWMGenPeriodGet(PWM_TAIL_BASE, PWM_TAIL_GEN);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, (pwmPeriod * duty) / 100);
}

// Setup of PWM generators
void pwmInit (void) {
    // Enable GPIO peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    // Enable PWM peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);

    // Set the clock divider for PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // GPIO configuration
    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);

    // PWM configuration
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                   PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                   PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Enable
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    // Set PWM frequency
    uint32_t pwmPeriphFreq = SysCtlClockGet() / 64;
    uint32_t pwmPeriod = pwmPeriphFreq / PWM_FREQ;
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, pwmPeriod);
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, pwmPeriod);
}
/*--------------------------------------------------------------*/
