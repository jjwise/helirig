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
#ifndef PID_H_
#define PID_H_
/* Macro Definitions -------------------------------------------*/
//Heli 2 WORKING GAINS:
#define MAIN_PROP_GAIN      180
#define MAIN_DIFF_GAIN      -50
#define MAIN_INT_GAIN       5
#define MAIN_ERROR_MAX      (30*1000 / MAIN_INT_GAIN)
#define MAIN_DUTY_OFFSET    20    // Duty cycle offset to offset gravity

// Tail rotor
#define TAIL_PROP_GAIN      180
#define TAIL_DIFF_GAIN      -200
#define TAIL_INT_GAIN       8
#define TAIL_DUTY_OFFSET    5
#define TAIL_ERROR_MAX      (40*1000/ TAIL_INT_GAIN)  // Max duty component / ki

#define PID_TASK_DELAY      40

//#define MAIN_PROP_GAIN      180/2
//#define MAIN_DIFF_GAIN      -50/2
//#define MAIN_INT_GAIN       5
//#define MAIN_ERROR_MAX      (50*1000 / MAIN_INT_GAIN)
//#define MAIN_DUTY_OFFSET    20    // Duty cycle offset to offset gravity
//
//// Tail rotor
//#define TAIL_PROP_GAIN      200/2
//#define TAIL_DIFF_GAIN      -200
//#define TAIL_INT_GAIN       20/2
//#define TAIL_DUTY_OFFSET    5
//#define TAIL_ERROR_MAX      (50*1000/ TAIL_INT_GAIN)  // Max duty component / ki
//
//#define PID_TASK_DELAY      40
/*--------------------------------------------------------------*/

/* Type definitions --------------------------------------------*/
typedef struct pidHandle_t {
    int32_t current;        // Current altitude (percent)
    int32_t target;         // Target altitude
    int32_t propError;      // Proportional error (degrees)
    int32_t integralError;  // Integral error
    int32_t diffError;      // differential error
    int16_t duty;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t offset;
} pidHandle_t;

/* External globals ----------------------------------------*/
extern QueueHandle_t xYawDegreesQueue;
extern QueueHandle_t xAltQueue;
extern QueueHandle_t xControlTargetQueue;
/*--------------------------------------------------------------*/

/* Function Prototypes------------------------------------------*/
// Task: defines pid handles for main and tail rotors. Receieves current
// and target values for each rotor. Sends calculated pwm values to the
// pwm queue
void pidTask(void *pvParameters);
// Current PID error calculators
void pidCalcErrors(pidHandle_t* h);
// Uses PID control to calculate duty cycle for each rotor
int16_t pidCalcDutyCycle(pidHandle_t* h);
/*--------------------------------------------------------------*/

#endif /* PID_H_ */
