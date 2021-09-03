/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 yaw.h

 Yaw position is encoded with a quadrature encoder.
 The yawIntHandler interrupt is generated when either
 pin of the encoder changes, the internal YawPosition
 variable is then updated accordingly. yawDegrees is
 then calculated by yawCalcDeg in the main loop.
----------------------------------------------------------------*/
#ifndef YAW_H_
#define YAW_H_
/* Macro Definitions -------------------------------------------*/
// Yaw scaling
#define YAW_SCALE_NUM       1607
#define YAW_SCALE_DEN       1000
#define YAW_REF_TASK_RATE   10
#define YAW_TASK_RATE       10    // (ms) Duration to suspend task
/*--------------------------------------------------------------*/

/* Type Definitions --------------------------------------------*/
typedef struct encoderHandle_t {
    uint8_t pins;    // LSB encodes pinA state, next bit pinB
    int32_t prevState;    // Previous quadrature encoded state
    int32_t currentState; // Current quadrature encoded state
} encoderHandle_t;
/*--------------------------------------------------------------*/

/* Globals --------------------------------------------*/
extern QueueHandle_t xMeasuredYawQueue;
extern QueueHandle_t xYawEncoderQueue;
extern QueueHandle_t xMeasuredAltitudeQueue;
extern SemaphoreHandle_t ctrlYawRefSmph;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/
// Task to convert yaw data from quadrature encoder into angle
// and send the angle to the flight controller task through a queue
void yawCalculateTask(void *pvParameters);
// Initialization of Yaw GPIO and interrupt handler
void yawInit (void);
// Interrupt handler for quadrature encoder (both pins)
void yawRefIntHandler(void);
// Interrupt handler for yaw reference
void yawIntHandler(void);
// Calculates the change in yaw position after a change in
// state of either quadrature encoder pin
int8_t yawChange(encoderHandle_t* yawState);
// Calculates current yaw position in degrees (-180 -> 180)
int32_t yawCalcDeg (int32_t yawPosition);
/*--------------------------------------------------------------*/

#endif /* YAW_H_ */
