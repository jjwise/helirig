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

#ifndef PWM_H_
#define PWM_H_

/* Definitions -------------------------------------------------*/
#define PWM_FREQ           100
#define PWM_UPDATE_RATE    40 // ms between duty cycle update
// Duty cycle limits
#define MAIN_MAX_DUTY       98          // Maximum duty cycle
#define MAIN_MIN_DUTY       2           // Minimum duty cycle
#define TAIL_MAX_DUTY       60          // Maximum duty cycle
#define TAIL_MIN_DUTY       2           // Minimum duty cycle
/*--------------------------------------------------------------*/

/* Includes -------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
/*--------------------------------------------------------------*/

/* Type Definitions -------------------------------------------------*/
typedef struct pwmUpdateMessage_t {
    int16_t tail;
    int16_t main;
} pwmUpdateMessage_t;
/*--------------------------------------------------------------*/

/* Globals -------------------------------------------------*/
extern QueueHandle_t xPWMQueue;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/
//PWM task
void pwmTask(void *pvParameters);

// Setup of PWM generators
void pwmInit (void);

// Update main PWM duty cycle
void pwmUpdateMain (uint32_t duty);

// Update tail PWM duty cycle
void pwmUpdateTail (uint32_t duty);
/*--------------------------------------------------------------*/

#endif /* PWM_H_ */
