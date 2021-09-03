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

/* Includes ----------------------------------------------------*/
#include "main.h"
#include "uart.h"
/*--------------------------------------------------------------*/

// initialization of UART
void uartInit(void) {
    // Enable UART and GPIO peripherals
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);

    // GPIO configuration
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    // UART configuration
    UARTConfigSetExpClk(UART_USB_BASE,
                        SysCtlClockGet(),
                        UART_BAUD_RATE,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    // Enable UART
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

// Transmit a string via UART0
void uartSend(char* payload) {
    // Loop while there are more characters to send.
    while(*payload) {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *payload);
        payload++;
    }
}

// Task to send system information over USB.
void uartTaskTelemetry(void *pvParameters){
    while(1) {
        while(uxQueueMessagesWaiting(xTelemetryQueue) > 0) {
            telemetryMessage_t recievedMessage;
            if(xQueueReceive(xTelemetryQueue, (void *) &recievedMessage, (TickType_t) 10) != pdPASS) {
                uartSend("teleRxFail\r\n");
            }
            uartSend(recievedMessage.str);
        }
        vTaskDelay(TELEMETRY_TASK_RATE / portTICK_RATE_MS);
    }
}
