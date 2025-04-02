//todo
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#define BUFFER_SIZE 50
#define PWM_FREQUENCY 1000

// Función para enviar una cadena por UART
void enviar_string(const char* str)
{
    while(*str)
    {
        UARTCharPut(UART0_BASE, *str);
        str++;
    }
}

// Handler para Timer0: togglea el LED en PC4 (cada 120ms)
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}

// Handler para Timer1: togglea el LED en PC5 (cada 250ms)
void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
    // Configurar el reloj del sistema a 120 MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // --- Configuración de UART (PA0 y PA1) ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, 120000000, 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // --- Configuración del ADC en PK3 (canal 19) para leer un potenciómetro ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH19);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    // --- Configuración del PWM en PK4 (M0PWM6) ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // PK4 se configura como salida PWM
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);
    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmLoad);
    // Iniciar con ciclo de trabajo 0%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    // --- Configuración de LEDs incorporados (para otros fines, no se usan en el toggle de PC4/PC5) ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // --- Configuración de pines para los LEDs que toggleará el timer: PC4 y PC5 ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // --- Configuración de Timer0 y Timer1 para los toggles ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Timer0: Período de 120ms
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Cálculo: (120e6 * 0.12) = 14,400,000 ciclos (resta 1)
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() * 120) / 100 - 1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Timer1: Período de 250ms
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Cálculo: (120e6 * 0.25) = 30,000,000 ciclos (resta 1)
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() * 250) / 100 - 1);
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1IntHandler);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);

    // Habilitar interrupciones globales
    IntMasterEnable();

    // --- Bucle principal: Lectura del ADC, actualización del PWM y envío por UART ---
    uint32_t adcValue = 0;
    char buffer[BUFFER_SIZE];
    uint8_t buffer_index = 0;
    int valorUART = 0;

    while(1)
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
		
                if(strcmp(buffer, "1") == 0) {
		  //		  enviar_string("Hola");
		  enviar_string("prender\r\n");
		  GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
		  PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
                } else{
		  //		  enviar_string("Adios");
		  enviar_string("apagar\r\n");
		  GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0);
		  PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
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
        // Manejo de recepción UART: se acumulan los caracteres en el buffer gettiva	            
        // Disparar conversión del ADC
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false)) {}
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);
	
        // Actualizar PWM en PK4 proporcional al valor del potenciómetro (rango 0–4095)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (pwmLoad * adcValue) / 4096);
	
        // (Opcional) Puedes enviar datos por UART o actualizar otros elementos aquí.        
        // Se incluye un pequeño retardo para evitar saturar el bucle.
	//        SysCtlDelay((SysCtlClockGet() / 3 / 1000) * 7000);

    }
}
// ESTO

