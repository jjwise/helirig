/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 main.c

Creates FreeRTOS tasks and queues. Starts FreeRTOS scheduler.
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "altitude.h"
#include "yaw.h"
#include "uart.h"
#include "pid.h"
#include "pwm.h"
#include "userInput.h"
#include "control.h"
/*--------------------------------------------------------------*/

extern QueueHandle_t xUserInputEventQueue = NULL;
extern QueueHandle_t xControlTargetQueue = NULL;
extern QueueHandle_t xAltitudeADCQueue = NULL;
extern QueueHandle_t xMeasuredAltitudeQueue = NULL;
extern QueueHandle_t xYawEncoderQueue = NULL;
extern QueueHandle_t xMeasuredYawQueue = NULL;
extern QueueHandle_t xPWMQueue = NULL;
extern QueueHandle_t xTelemetryQueue = NULL;

extern TaskHandle_t controlFlyingTaskHandle = NULL;
extern TaskHandle_t controlLandedTaskHandle = NULL;
extern TaskHandle_t controlLandingTaskHandle = NULL;
extern TaskHandle_t controlYawRefTaskHandle = NULL;
extern TaskHandle_t controlSpecialTaskHandle = NULL;
extern TaskHandle_t pidTaskHandle = NULL;

extern SemaphoreHandle_t ctrlYawRefSmph = NULL;
/*--------------------------------------------------------------*/

//Sets crystal frequency, initialises periphs, creates tasks and queues,
//starts FreeRTOS
int main(void) {
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize the world
    altitudeInitADC();
    userInputInit();
    yawInit();
    pwmInit();
    uartInit();

    // Create FreeRTOS stuff
    createQueues();
    createTasks();

    ctrlYawRefSmph = xSemaphoreCreateBinary();
    //xSemaphoreTake(ctrlYawRefSmph, 100);

    vTaskStartScheduler();      // Start FreeRTOS!!

    while(1);                   // Should never get here since the RTOS should never "exit".
}

void taskCpuUsage(void *pvParameters) {
    while(1) {
        static char runtime_stats_buffer[1024];
        vTaskGetRunTimeStats(runtime_stats_buffer);
        //uartSend(runtime_stats_buffer);
        vTaskDelay(1000 / portTICK_RATE_MS); //show every second
    }
}

//Creates FreeRTOS queues
void createQueues(void) {
    xUserInputEventQueue = xQueueCreate(3, sizeof(userInputEventMessage_t));
    xControlTargetQueue = xQueueCreate(3, sizeof(controlTargetMessage_t));
    xMeasuredAltitudeQueue = xQueueCreate(5, sizeof(int32_t));
    xAltitudeADCQueue = xQueueCreate(12, sizeof(uint32_t));
    xYawEncoderQueue = xQueueCreate(40, sizeof(uint8_t));
    xMeasuredYawQueue = xQueueCreate(5, sizeof(int32_t));
    xPWMQueue = xQueueCreate(10, sizeof(pwmUpdateMessage_t));
    xTelemetryQueue = xQueueCreate(20, sizeof(telemetryMessage_t));
}

//Creates FreeRTOS tasks
void createTasks(void) {
    if (pdTRUE != xTaskCreate(uartTaskTelemetry, "telemetry", TASK_STACK_DEPTH, NULL, 1, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(userInputPollTask, "User input polling task", TASK_STACK_DEPTH, NULL, 2, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(altitudeADCSamplingTask, "Altitude sampling task", TASK_STACK_DEPTH, NULL, 6, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(altitudeAvgTask, "Altitude averaging task", TASK_STACK_DEPTH, NULL, 5, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(yawCalculateTask, "Calculates yaw Angle", TASK_STACK_DEPTH, NULL, 5, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(pwmTask, "PWM duty setting task", TASK_STACK_DEPTH, NULL, 3, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(controlYawRefTask, "System control task", TASK_STACK_DEPTH, NULL, 2, &controlLandedTaskHandle))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.

    if (pdTRUE != xTaskCreate(taskCpuUsage, "Task CPU Usage", TASK_STACK_DEPTH, NULL, 2, NULL))
    { while(1);}               // Oh no! Must not have had enough memory to create the task.
}
