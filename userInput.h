/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 userInput.h

 Adaption of buttons4.h by P.J. Bones UCECE, 7.2.2018

 This module supports hardware on the tiva/orbit kit for the
 helirig setup. The"up, down, left, right, soft reset" buttons
 and two sliders are supported.
----------------------------------------------------------------*/

#ifndef USERINPUT_H_
#define USERINPUT_H_

/* Definitions -------------------------------------------------*/
#define NUM_BUT_POLLS        5
#define BUTTON_POLL_RATE     20 // delay time in ms
/*--------------------------------------------------------------*/

/* Type Definitions -------------------------------------------------*/
typedef struct userInputEventMessage_t {
    enum userInputActions action;
    enum userInputNames name;
} userInputEventMessage_t;

typedef struct userInputAction_t {
    enum userInputNames name;
    bool isSwitch;           // Is the input being handled a switch
    bool normalState;        // Un-pressed electrical state
    bool measuredState;      // Current measured electrical state
    bool prevMeasuredState;  // Previous measured electrical state
    bool isStable;           // Current electrical stability
    bool hasChanged;         // Flag for change in state
    uint8_t noChangeCount;   // Number of polls since last change
} userInputHandle_t;
/*--------------------------------------------------------------*/

/* Globals -------------------------------------------------*/
extern QueueHandle_t xUserInputEventQueue;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/
// initButtons: Initialize the variables associated with the
// set of buttons defined by the constants above.
void userInputInit (void);

// Task to poll all buttons once and updates
// variables associated with the buttons if necessary.
// It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
void userInputPollTask (void *pvParameters);

// Queues user events for controller task
void userInputQueueEvent(userInputHandle_t * b);

// Updates the measured state of each pin
void userInputUpdate(userInputHandle_t * h);
/*--------------------------------------------------------------*/

#endif /* USERINPUT_H_ */
