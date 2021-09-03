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

 /* Includes ----------------------------------------------------*/
#include "main.h"
#include "control.h"
#include "uart.h"
#include "userInput.h"
#include "pid.h"
#include "yaw.h"
#include "pwm.h"
/*--------------------------------------------------------------*/

/* Function definitions ----------------------------------------*/

//Task that runs when heli in landed state, can transition to
//flying or special task state from here.
void controlLandedTask(void *pvParameters) {
    uint8_t patternStatus1 = 0;
    uint8_t patternStatus2 = 0;
    pwmUpdateMessage_t pwm = {0};
    const userInputEventMessage_t pattern1[PATTERN_LENGTH] = {
                        {BUT_PUSHED, LEFT},
                        {BUT_RELEASED, LEFT},
                        {BUT_PUSHED, RIGHT},
                        {BUT_RELEASED, RIGHT}};

    const userInputEventMessage_t pattern2[PATTERN_LENGTH] = {
                        {BUT_PUSHED, RIGHT},
                        {BUT_RELEASED, RIGHT},
                        {BUT_PUSHED, LEFT},
                        {BUT_RELEASED, LEFT}};
    while (1) {
        // Sustain idle
        xQueueSend(xPWMQueue, (void *) &pwm, (TickType_t) 10);

        //Check for user input
        while(uxQueueMessagesWaiting(xUserInputEventQueue) > 0) {
            // Get user input event
            userInputEventMessage_t recievedEvent;
            xQueueReceive(xUserInputEventQueue, &recievedEvent, 5);

            // Check for user change to flying mode
            if(recievedEvent.name == SW1 && recievedEvent.action == SWITCHED_ON) {
                controlStartFlying();
            }
            // Check for pattern 1
            userInputEventMessage_t next1 = pattern1[patternStatus1];
            if(recievedEvent.name == next1.name && recievedEvent.action == next1.action) {
                patternStatus1++;
                if(patternStatus1 == PATTERN_LENGTH) {
                    controlStartSpecial(1);
                }
            } else { patternStatus1 = 0; }

            // Check for pattern 2
            userInputEventMessage_t next2 = pattern2[patternStatus2];
            if(recievedEvent.name == next2.name && recievedEvent.action == next2.action) {
                patternStatus2++;
                if(patternStatus2 == PATTERN_LENGTH) {
                    controlStartSpecial(2);
                }
            } else { patternStatus2 = 0; }
        }

        // Task delay
        vTaskDelay(CONTROL_UPDATE_RATE / portTICK_RATE_MS);
    }
}

//Updates the altitude and yaw targets of the heli if in this
//state. Targets are updated through button pushes.
void controlFlyingTask(void *yawInitial) {
    //initialise yaw and altitude to 0
    controlTargetMessage_t target = {0};
    //set target yaw to current yaw position
    target.yaw = *(int32_t*) yawInitial;
    target.yaw -= target.yaw % YAW_TARGET_STEP;
    xQueueSend(xControlTargetQueue, (void *) &target, 5);

    while (1) {
        // Check for user inputs
        while(uxQueueMessagesWaiting(xUserInputEventQueue) > 0) {
            // Get the user input event
            userInputEventMessage_t recievedEvent;
            xQueueReceive(xUserInputEventQueue, &recievedEvent, 5);
            // Check for user change to flying mode
            if(recievedEvent.name == SW1 && recievedEvent.action == SWITCHED_OFF) {
                controlStartLanding();
            } else {
                controlUpdateTarget(&target, recievedEvent);
            }
        }

        // Send the target to the  PID task
        xQueueSend(xControlTargetQueue, (void *) &target, 5);

        // Task delay
        vTaskDelay(CONTROL_UPDATE_RATE / portTICK_RATE_MS);
    }
}

//State that brings helicopter gently to landed
void controlLandingTask(void *pvParameters) {
    int32_t altitude;
    controlTargetMessage_t target = {0};
    while (1) {
        if (xQueueReceive(xMeasuredAltitudeQueue, (void *) &altitude, (TickType_t) 10) == pdPASS) {
            target.altitude = altitude - 20;
            if(altitude == 0) {
                vTaskDelete(pidTaskHandle);
                controlStartLanded();
            }
        }
        // Send target to PID task
        xQueueSend(xControlTargetQueue, (void *) &target, 5);

        // Task delay
        vTaskDelay(CONTROL_UPDATE_RATE / portTICK_RATE_MS);
    }
}

//State for finding the reference point for the yaw. Is
//blocked by until a semaphore is given by the yaw ref ISR.
void controlYawRefTask(void *pvParameters) {
    uartSend("LOOKING FOR REF!\r\n");

    // Set the tail pwm duty cycle to search for ref
    pwmUpdateMessage_t pwm;
    pwm.tail = TAIL_YAWREF_DUTY;
    pwm.main = 0;
    xQueueSend(xPWMQueue, (void *) &pwm, (TickType_t) 10);

    // Wait for ISR to give semaphore
    xSemaphoreTake(ctrlYawRefSmph, (TickType_t) 0xffff);

    // Clean up
    xQueueReset(xMeasuredAltitudeQueue);
    xQueueReset(xMeasuredYawQueue);
    xQueueReset(xUserInputEventQueue);
    IntDisable(YAW_REF_INT_GROUP);

    // Give semaphore to yaw calculation task
    xSemaphoreGive(ctrlYawRefSmph);

    // Reference has been found:
    uartSend("REF FOUND!\r\n");

    // Switch to Landed mode
    controlStartLanded();
}

//Special states for heli. First state brings the helicopter gently
//to an alttude of 50% and maintains this altitude. Second state
//maintains a slow spin indefinitely.
void controlSpecialTask(void *patternNumberPTR) {
    uint8_t* patternNumber = (uint8_t *) patternNumberPTR;
    int32_t altitude;
    int32_t yaw;
    controlTargetMessage_t target;
    target.altitude = 15;
    target.yaw = 0;

    while (*patternNumber == 1) {
        // Increment target up to 50% and stay
        if (xQueueReceive(xMeasuredAltitudeQueue, (void *) &altitude, (TickType_t) 10) == pdPASS) {
            if(altitude >= 50 ) {
                target.altitude = 50;
            } else if(altitude >= 40 ) {
                target.altitude = 55;
            } else if(altitude >= 30) {
                target.altitude = 45;
            } else if(altitude >= 20) {
                target.altitude = 35;
            } else if(altitude >= 10) {
                target.altitude = 25;
            }
        }

        // Check for user input
        while(uxQueueMessagesWaiting(xUserInputEventQueue) > 0) {
            // Get the user input event
            userInputEventMessage_t recievedEvent;
            xQueueReceive(xUserInputEventQueue, &recievedEvent, 5);
            // Any user input goes back to landing
            if(recievedEvent.name != SW1) {
                controlStartLanding();
            }
        }
        // Send the target to the  PID task
        xQueueSend(xControlTargetQueue, (void *) &target, 5);

        // Task delay
        vTaskDelay(CONTROL_UPDATE_RATE / portTICK_RATE_MS);
    }

    while (*patternNumber == 2) {
        // Increment target up to 50% and stay
        if (xQueueReceive(xMeasuredYawQueue, (void *) &yaw, (TickType_t) 10) == pdPASS) {
            if (abs(yaw) >= abs(target.yaw) - 5 && abs(yaw) <= abs(target.yaw) + 5) {
                userInputEventMessage_t dummy;
                dummy.name = RIGHT;
                dummy.action = BUT_PUSHED;
                controlUpdateTarget(&target, dummy);
            }
        }

        // Check for user input
        while(uxQueueMessagesWaiting(xUserInputEventQueue) > 0) {
            // Get the user input event
            userInputEventMessage_t recievedEvent;
            xQueueReceive(xUserInputEventQueue, &recievedEvent, 5);
            // Any user input goes back to landing
            if(recievedEvent.name != SW1) {
                controlStartLanding();
            }
        }

        // Send the target to the  PID task
        xQueueSend(xControlTargetQueue, (void *) &target, 5);

        // Task delay
        vTaskDelay(CONTROL_UPDATE_RATE / portTICK_RATE_MS);
    }
}

//Updates the target altitude and yaw depending on the recieved button pushes.
void controlUpdateTarget(controlTargetMessage_t* target, userInputEventMessage_t recievedEvent) {
    if (recievedEvent.action == BUT_PUSHED) {
        switch (recievedEvent.name) {
        case UP:
            target->altitude += ALT_TARGET_STEP;
            if (target->altitude > 100) { target->altitude = 100; }
            break;
        case DOWN:
            target->altitude -= ALT_TARGET_STEP;
            if (target->altitude < 0) { target->altitude = 0; }
            break;
        case RIGHT:
            target->yaw += YAW_TARGET_STEP;
            if(target->yaw >= 180){ target->yaw -= 360; }
            break;
        case LEFT:
            target->yaw -= YAW_TARGET_STEP;
            if(target->yaw < -180){ target->yaw += 360; }
        }
    }
}

//Initialisation function for transition from landed to flying state.
//initial yaw target is set to the helicopter's current position.
void controlStartFlying(void){
    uartSend("Flying\r\n");
    int32_t yawInitial = 0;
    //don't need to do this here but just wanted to use task parameters
    while(uxQueueMessagesWaiting(xMeasuredYawQueue) > 0) {
        if(xQueueReceive(xMeasuredYawQueue, (void *) &yawInitial, (TickType_t) 10) != pdPASS) {
            uartSend("yawRxFail\r\n");
        }
    }
    // Run the Flying and PID tasks
    if (pdTRUE != xTaskCreate(pidTask, "PID calculation task", TASK_STACK_DEPTH, NULL, 6, &pidTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
    if (pdTRUE != xTaskCreate(controlFlyingTask, "System control task", TASK_STACK_DEPTH, &yawInitial, 2, &controlFlyingTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
    vTaskDelete(NULL);
}

//Creates tasks required for transition to the special state
void controlStartSpecial(uint8_t patternNumber){
    uartSend("Special\r\n");
    // Run the Special and PID tasks
    if (pdTRUE != xTaskCreate(pidTask, "PID calculation task", TASK_STACK_DEPTH, NULL, 6, &pidTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
    if (pdTRUE != xTaskCreate(controlSpecialTask, "System control task", TASK_STACK_DEPTH, &patternNumber, 2, &controlSpecialTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
    vTaskDelete(NULL);
}

//Creates tasks required for transition to landing state
void controlStartLanding(void){
    uartSend("Landing\r\n");
    // Start landing task (do not stop PID controller)
    if (pdTRUE != xTaskCreate(controlLandingTask, "System control task", TASK_STACK_DEPTH, NULL, 2, &controlLandingTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
    vTaskDelete(NULL);
}

//creates tasks required to transition from landing to landed
void controlStartLanded(void){
    // Stop PID task and run landed task
    uartSend("Landed\r\n");
    if (pdTRUE != xTaskCreate(controlLandedTask, "System control task", TASK_STACK_DEPTH, NULL, 2, &controlLandedTaskHandle))
    { while(1);}
    vTaskDelete(NULL);
}
