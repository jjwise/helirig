/*---------------------------------------------------------------
               ________________
                 _____|___
                / |__|    \-----__|__
                \_________/------ |
                ____|___|____

-----------------------------------------------------------------
 ENCE 464 Group 13
 main.h
 Hardware specific definitions
----------------------------------------------------------------*/

/* Includes ----------------------------------------------------*/
// Standard
#include <stdint.h>
#include <stdbool.h>
// hardware specific
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
// Drivers
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include "ustdlib.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
/*--------------------------------------------------------------*/

/* Constants ---------------------------------------------------*/
enum flightModes {LANDED, FLYING, LANDING, YAWREF, SPECIAL};
enum userInputNames {UP, DOWN, LEFT, RIGHT, SW1, SW2, NUM_INPUTS};
enum userInputActions {BUT_RELEASED, BUT_PUSHED, SWITCHED_ON, SWITCHED_OFF};
/*--------------------------------------------------------------*/

/* User interface definitions ----------------------------------*/
// UP button
#define BUT_UP_PERIPH       SYSCTL_PERIPH_GPIOE
#define BUT_UP_PORT_BASE    GPIO_PORTE_BASE
#define BUT_UP_PIN          GPIO_PIN_0
#define BUT_UP_INT_GROUP    INT_GPIOB
#define BUT_UP_NORMAL       false

// DOWN button
#define BUT_DOWN_PERIPH     SYSCTL_PERIPH_GPIOD
#define BUT_DOWN_PORT_BASE  GPIO_PORTD_BASE
#define BUT_DOWN_PIN        GPIO_PIN_2
#define BUT_DOWN_INT_GROUP  INT_GPIOD
#define BUT_DOWN_NORMAL     false

// LEFT button
#define BUT_LEFT_PERIPH     SYSCTL_PERIPH_GPIOF
#define BUT_LEFT_PORT_BASE  GPIO_PORTF_BASE
#define BUT_LEFT_PIN        GPIO_PIN_4
#define BUT_LEFT_INT_GROUP  INT_GPIOF
#define BUT_LEFT_NORMAL     true

// RIGHT button
#define BUT_RIGHT_PERIPH    SYSCTL_PERIPH_GPIOF
#define BUT_RIGHT_PORT_BASE GPIO_PORTF_BASE
#define BUT_RIGHT_PIN       GPIO_PIN_0
#define BUT_RIGHT_INT_GROUP INT_GPIOB
#define BUT_RIGHT_NORMAL    true

// SW1 slider switch
#define SW1_PERIPH          SYSCTL_PERIPH_GPIOA
#define SW1_PORT_BASE       GPIO_PORTA_BASE
#define SW1_PIN             GPIO_PIN_7
#define SW1_NORMAL          false

// SW2 slider switch
#define SW2_PERIPH          SYSCTL_PERIPH_GPIOA
#define SW2_PORT_BASE       GPIO_PORTA_BASE
#define SW2_PIN             GPIO_PIN_6
#define SW2_NORMAL          false

//RESET BUTTON
#define BUT_RESET_BASE      GPIO_PORTA_BASE
#define BUT_RESET_PIN       GPIO_PIN_6
#define BUT_RESET_NORMAL    true
/*--------------------------------------------------------------*/

/* PWM Hardware Details ----------------------------------------*/
// Main rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

// Tail rotor PWM: PF1
#define PWM_TAIL_BASE           PWM1_BASE
#define PWM_TAIL_GEN            PWM_GEN_2
#define PWM_TAIL_OUTNUM         PWM_OUT_5
#define PWM_TAIL_OUTBIT         PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1
/*--------------------------------------------------------------*/

/* Measurement input hardware detail ---------------------------*/
// Altitude ADC input: ADC0, Channel 9, pin PE4 (AIN9)
#define ALTITUDE_ADC_BASE       ADC0_BASE
#define ALTITUDE_ADC_PERIPH     SYSCTL_PERIPH_ADC0
#define ALTITUDE_ADC_CHANNEL    ADC_CTL_CH9
#define ALTITUDE_ADC_SEQ_NUM    3
#define ALTITUDE_ADC_TRIGGER    ADC_TRIGGER_PROCESSOR
#define ALTITUDE_ADC_STEP       0
#define ALTITUDE_ADC_PRIORITY   0

// Yaw reference pin:
#define YAW_REF_BASE            GPIO_PORTC_BASE
#define YAW_REF_PERIPH          SYSCTL_PERIPH_GPIOC
#define YAW_REF_PIN             GPIO_PIN_4
#define YAW_REF_INT_GROUP       INT_GPIOC

// Yaw quadrature encoder:
#define YAW_QUAD_BASE           GPIO_PORTB_BASE
#define YAW_QUAD_PERIPH         SYSCTL_PERIPH_GPIOB
#define YAW_QUAD_PIN_A          GPIO_PIN_0
#define YAW_QUAD_PIN_B          GPIO_PIN_1
#define YAW_QUAD_INT_GROUP      INT_GPIOB
/*--------------------------------------------------------------*/

/* USB UART Communications -------------------------------------*/
// UART0, Rx:PA0 , Tx:PA1
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX
/*--------------------------------------------------------------*/


void createQueues(void);
void createTasks(void);
