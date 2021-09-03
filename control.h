/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 control.h

 FSM controller. Each state is implemented as a task (slightly
 unnecessary but shows use of task suspention and deletion). Only one
 task (state) is running at any one point in time. State changes are event
 driven (button pushes, or change in measured values). Creates target
 values that are sent to the PID controller
 ----------------------------------------------------------------*/
#ifndef CONTROL_H_
#define CONTROL_H_

/* Definitions -------------------------------------------------*/
#define CONTROL_UPDATE_RATE     100 // delay time in ms
#define YAW_TARGET_STEP         15
#define ALT_TARGET_STEP         10
#define PATTERN_LENGTH          4
#define TAIL_YAWREF_DUTY        50  // Duty cycle when finding reference
/*--------------------------------------------------------------*/

/* Includes -------------------------------------------------*/
#include "userInput.h"
/*--------------------------------------------------------------*/

/* Type Definitions -------------------------------------------------*/
typedef struct controlTargetMessage_t {
    int32_t yaw;
    int32_t altitude;
} controlTargetMessage_t;
/*--------------------------------------------------------------*/

/* Globals -------------------------------------------------*/
extern SemaphoreHandle_t ctrlYawRefSmph;
extern QueueHandle_t xUserInputEventQueue;
extern QueueHandle_t xMeasuredYawQueue;
extern QueueHandle_t xMeasuredAltitudeQueue;
extern TaskHandle_t pidTaskHandle;
extern TaskHandle_t controlFlyingTaskHandle;
extern TaskHandle_t controlLandedTaskHandle;
extern TaskHandle_t controlLandingTaskHandle;
extern TaskHandle_t controlYawRefTaskHandle;
extern TaskHandle_t controlSpecialTaskHandle;

/*--------------------------------------------------------------*/

/* Function prototypes ----------------------------------------*/

//Updates the altitude and yaw targets of the heli if in this
//state. Targets are updated through button pushes.
void controlFlyingTask(void *pvParameters);

//Task that runs when heli in landed state, can transition to
//flying or special task state from here.
void controlLandedTask(void *pvParameters);

//State that brings helicopter gently to landed
void controlLandingTask(void *pvParameters);

//State for finding the reference point for the yaw. Is
//blocked by until a semaphore is given by the yaw ref ISR.
void controlYawRefTask(void *pvParameters);

//Special states for heli. First state brings the helicopter gently
//to an alttude of 50% and maintains this altitude. Second state
//maintains a slow spin indefinitely.
void controlSpecialTask(void *pvParameters);

//Updates the target altitude and yaw depending on the recieved button pushes.
void controlUpdateTarget(controlTargetMessage_t* target, userInputEventMessage_t recievedEvent);

//Initialisation function for transition from landed to flying state.
//initial yaw target is set to the helicopter's current position.
void controlStartFlying(void);

//creates tasks required to transition from landing to landed
void controlStartLanded(void);

//Creates tasks required for transition to landing state
void controlStartLanding(void);

//Creates tasks required for transition to the special state
void controlStartSpecial(uint8_t);

/*--------------------------------------------------------------*/

#endif /* CONTROL_H_ */
