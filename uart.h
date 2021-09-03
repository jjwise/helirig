/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 uart.h

 Module handles:
 - UART initialization
 - Format and selection of data for transmission
 - transmission of data over USB using UART communication
----------------------------------------------------------------*/

#ifndef UART_H_
#define UART_H_

/* Definitions -------------------------------------------------*/
#define UART_MAX_STR_LEN        16
#define UART_BAUD_RATE          9600
#define TELEMETRY_TASK_RATE     500 // 1Hz
#define MAX_TELEMETRY_CHAR      14
/*--------------------------------------------------------------*/

/* Type Definitions -------------------------------------------------*/
typedef struct telemetryMessage_t {
    char str[MAX_TELEMETRY_CHAR];
} telemetryMessage_t;
/*--------------------------------------------------------------*/

/* Globals -------------------------------------------------*/
extern QueueHandle_t xTelemetryQueue;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/

// initialization of UART
void uartInit(void);

// Transmit a string via UART0
void uartSend(char* payload);

// Task to send telemetry data over UART, not used
void uartTaskTelemetry (void *pvParameters);
/*--------------------------------------------------------------*/

#endif /* UART_H_ */
