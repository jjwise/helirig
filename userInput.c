/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 userInput.c

 This module supports hardware on the tiva/orbit kit for the
 heli-rig setup. The"up, down, left, right, soft reset" buttons
 and two sliders are supported.
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "userInput.h"
#include "uart.h"
/*--------------------------------------------------------------*/

// Task to poll all buttons once and updates
// variables associated with the buttons if necessary.
// It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
void userInputPollTask(void *pvParameters) {
    // Array of input handlers
    userInputHandle_t inputs[NUM_INPUTS];

    // Fill the array
    int i;
    for (i = 0; i < NUM_INPUTS; i++) {
        // Setup the button objects
        userInputHandle_t newHandle;
        newHandle.name = (enum userInputNames)i;
        userInputUpdate(&newHandle); // Set the initial state
        newHandle.isStable = true;
        newHandle.hasChanged = false;
        inputs[i] = newHandle;
    }

    // Set normal states
    inputs[UP].normalState    = BUT_UP_NORMAL;
    inputs[DOWN].normalState  = BUT_DOWN_NORMAL;
    inputs[LEFT].normalState  = BUT_LEFT_NORMAL;
    inputs[RIGHT].normalState = BUT_RIGHT_NORMAL;
    inputs[SW1].normalState   = (GPIOPinRead (SW1_PORT_BASE, SW1_PIN) == SW1_PIN);
    inputs[SW2].normalState   = (GPIOPinRead (SW2_PORT_BASE, SW2_PIN) == SW2_PIN);

    inputs[SW1].isSwitch  = true;
    inputs[SW2].isSwitch  = true;

    while (1) {
        // De-bounce algorithm: Only set the state change flag once the pin has
        // not changed for a set number of poll cycles

        // Iterate through the buttons.
        int i;
        for (i = 0; i < NUM_INPUTS; i++) {
            userInputHandle_t* h = &inputs[i];
            userInputUpdate(h);
            // input must be stable to queue an event
            if (h->isStable && h->hasChanged) {
                // if the pin has changed and becomes stable, queue an event
                userInputQueueEvent(h);
                h->hasChanged = false; // Reset the change flag
            }
        }

        vTaskDelay(BUTTON_POLL_RATE / portTICK_RATE_MS);
    }
}

// Queues user events for controller task to consume
void userInputQueueEvent(userInputHandle_t * h) {
    userInputEventMessage_t event;
    event.name = h->name;
    if (h->measuredState == h->normalState) {
        event.action = (h->isSwitch) ? SWITCHED_OFF:BUT_RELEASED;
        xQueueSend(xUserInputEventQueue, (void *) &event, (TickType_t) 10 != pdPASS);
    } else {
        event.action = (h->isSwitch) ? SWITCHED_ON:BUT_PUSHED;
        xQueueSend(xUserInputEventQueue, (void *) &event, (TickType_t) 10 != pdPASS);
    }
}

// Updates the measured state of each pin
void userInputUpdate(userInputHandle_t * h){
    // Update the measured state of each pin
    switch (h->name) {
    case UP :
        h->measuredState = (GPIOPinRead (BUT_UP_PORT_BASE, BUT_UP_PIN) == BUT_UP_PIN);
        break;
    case DOWN :
        h->measuredState = (GPIOPinRead (BUT_DOWN_PORT_BASE,  BUT_DOWN_PIN) == BUT_DOWN_PIN);
        break;
    case LEFT :
        h->measuredState = (GPIOPinRead (BUT_LEFT_PORT_BASE,  BUT_LEFT_PIN) == BUT_LEFT_PIN);;
        break;
    case RIGHT :
        h->measuredState = (GPIOPinRead (BUT_RIGHT_PORT_BASE, BUT_RIGHT_PIN) == BUT_RIGHT_PIN);
        break;
    case SW1 :
        h->measuredState = (GPIOPinRead (SW1_PORT_BASE, SW1_PIN) == SW1_PIN);
        break;
    case SW2 :
        h->measuredState = (GPIOPinRead (SW2_PORT_BASE, SW2_PIN) == SW2_PIN);
        break;
    }

    // Check for change in state
    if (h->measuredState != h->prevMeasuredState){
        h->noChangeCount = 0;
        h->hasChanged = true;
    } else {
        h->noChangeCount++;
    }

    // Check if the pin is stable
    if (h->noChangeCount >= NUM_BUT_POLLS){
        h->isStable = true;
    }

    // Update the previous measured state
    h->prevMeasuredState = h->measuredState;
}

// Initialize the variables associated with the
// set of buttons defined by the constants in main.h.
void userInputInit(void) {
    // UP button (active HIGH)
    SysCtlPeripheralEnable (BUT_UP_PERIPH);
    GPIOPinTypeGPIOInput (BUT_UP_PORT_BASE, BUT_UP_PIN);
    GPIOPadConfigSet (BUT_UP_PORT_BASE,
                      BUT_UP_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPD);

    // DOWN button (active HIGH)
    SysCtlPeripheralEnable (BUT_DOWN_PERIPH);
    GPIOPinTypeGPIOInput (BUT_DOWN_PORT_BASE, BUT_DOWN_PIN);
    GPIOPadConfigSet (BUT_DOWN_PORT_BASE,
                      BUT_DOWN_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPD);

    // LEFT button (active LOW)
    SysCtlPeripheralEnable (BUT_LEFT_PERIPH);
    GPIOPinTypeGPIOInput (BUT_LEFT_PORT_BASE, BUT_LEFT_PIN);
    GPIOPadConfigSet (BUT_LEFT_PORT_BASE,
                      BUT_LEFT_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU);

    // RIGHT button (active LOW)
      // Note that PF0 is one of a handful of GPIO pins that need to be
      // "unlocked" before they can be reconfigured.  This also requires
      // #include "inc/tm4c123gh6pm.h"
    SysCtlPeripheralEnable (BUT_RIGHT_PERIPH);
    //---Unlock PF0 for the right button:
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R  |= GPIO_PIN_0; //PF0 unlocked
    GPIO_PORTF_LOCK_R = GPIO_LOCK_M;
    GPIOPinTypeGPIOInput (BUT_RIGHT_PORT_BASE, BUT_RIGHT_PIN);
    GPIOPadConfigSet (BUT_RIGHT_PORT_BASE,
                      BUT_RIGHT_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU);

    // SW1 Slider switch
    SysCtlPeripheralEnable (SW1_PERIPH);
    GPIOPinTypeGPIOInput (SW1_PORT_BASE, SW1_PIN);
    GPIOPadConfigSet (SW1_PORT_BASE, SW1_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPD);

    // SW2 Slider switch
    SysCtlPeripheralEnable (SW2_PERIPH);
    GPIOPinTypeGPIOInput (SW2_PORT_BASE, SW2_PIN);
    GPIOPadConfigSet (SW2_PORT_BASE,
                      SW2_PIN,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPD);
}
