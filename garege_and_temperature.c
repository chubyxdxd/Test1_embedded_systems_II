#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "driverlib/adc.h"
#include "driverlib/pwm.h"

#define BUFFER_SIZE 50
#define PWM_FREQUENCY 1000
uint32_t FS = 120000000;
int cnt=0;
int flag=0;
void Timer0IntHandler(void)
{
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  if(flag==1){
    //LED ROJO
    enviar_string("LED ROJO\r\n");
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);
  }else if(flag==2){
    //NO HACE NADA
    enviar_string("NO HACE NADA\r\n");
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);
  }else if(flag==3){
    //MOTOR
    enviar_string("MOTOR\r\n");
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0);
  }
}


void enviar_string(const char* str)
{
    while (*str)
    {
        UARTCharPut(UART0_BASE, *str);
        str++;
    }
}

void delay_ms(uint32_t ms)
{
    SysCtlDelay((120000000 / 3 / 1000) * ms);
}

void configurar_pwm(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);
    
    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmLoad);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (pwmLoad * 100) / 100);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

int main(void)
{
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, 120000000, 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
    
    configurar_pwm();

        /* TIMERS */
    IntMasterEnable();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, FS*4);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    IntEnable(INT_TIMER0A);
    /*FINAL*/

    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);
    
    char buffer[BUFFER_SIZE];
    uint8_t buffer_index = 0;
    int flagmotor=0;
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0);
    
    while (1)
    {
        if(UARTCharsAvail(UART0_BASE))
	  {
	    //	    enviar_string("Conectado: ");
	    char c = UARTCharGet(UART0_BASE);
	    
	    if(c == '\n' || c == '\r')
	      {
		buffer[buffer_index] = '\0';
		enviar_string("Recibido: ");
		enviar_string(buffer);
		/* enviar_string("F");  */
		enviar_string("\r\n");
		
		if(strcmp(buffer, "11") == 0) {
		  //		  enviar_string("Hola");
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
		  GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
		} else if(strcmp(buffer, "00") == 0){
		  //		  enviar_string("Adios");
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
		  GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
		} else if(strcmp(buffer, "1") == 0){ // <2
		  //LED ROJO
		  //enviar_string("caso 1\r\n");
		  flag=1;
		} else if(strcmp(buffer, "2") == 0){
		  // NO HACE NADA
		  //		  enviar_string("caso 2\r\n");
		  flag=2;		  
		} else if(strcmp(buffer, "3") == 0){
		  //		  enviar_string("caso 3\r\n");
		  flag=3;
		}	  	  
		buffer_index = 0;
	      }
	    else
	      {
		if(buffer_index < BUFFER_SIZE - 1)
		  {
		    buffer[buffer_index++] = c;
		  }
	      }
	  }
    }
}

