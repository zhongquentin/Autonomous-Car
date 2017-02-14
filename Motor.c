#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
   // ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    //ROM_UARTIntClear(UART0_BASE, ui32Status);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //

    //while(ROM_UARTCharsAvail(UART0_BASE))
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //


       // ROM_UARTCharPutNonBlocking(UART0_BASE,
         //                          ROM_UARTCharGetNonBlocking(UART0_BASE));

    	//char input = ROM_UARTCharGet(UART0_BASE);
    	char input = ROM_UARTCharGet(UART1_BASE);

//    	ROM_UARTCharPutNonBlocking(UART0_BASE,
//    	                                   input);

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // assert ENA signal high Accelerate
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, ); // CS
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); // ENB high
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // INA high (OLD)****
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); // INA low
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high

		if(input == 'a')
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high
		if(input == 's')
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low


        if(input == '0' || input == 'q')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); 					  //0 Motor stopped
        if(input == '1')
        	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
        	                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 1 / 10); //10%
        if(input == '2')
        	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
        	                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 1 / 5);  //20%
		if(input == '3' || input == 'd')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 10);  //30%
		if(input == '4' || input == 'f')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 4 / 10); //40%
		if(input == '5' || input == 'g')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 5 / 10); //50%
		if(input == '6' || input == 'h')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 6 / 10); //60%
		if(input == '7' || input == 'i')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 7 / 10); //70%
		if(input == '8' || input == 'j')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 8 / 10); //80%
		if(input == '9' || input == 'k')
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 9 / 10);  //90%

        //
        // Blink the LED to show a character transfer is occuring.
        //

        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) == 0)
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        else
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        //SysCtlDelay(SysCtlClockGet() / 5 * 3);

        //
        // Turn off the LED
        //
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(*pui8Buffer != '\0')
    {
        //
        // Write the next character to the UART.
        //
        //ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        ROM_UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************s
int
main(void)
{

    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1); // INA OLD BADDDDD
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2); // ENA
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); // INA
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // ENB
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5); // INB

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // LED
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //bluetooth
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

    //
    // The PWM peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 3125); //62500 1kHz

    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 10); // neutral or 0%

    //
    // Enable the PWM0 Bit0 (PD0) output signal.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    //
    // Enable the PWM generator block.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    //
    // Set GPIO A0 and A1 as UART pins.
    //
   // GPIOPinConfigure(GPIO_PA0_U0RX);
   // GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinConfigure(GPIO_PB0_U1RX); //bluetooth
    GPIOPinConfigure(GPIO_PB1_U1TX); //bluetooth
   // ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //bt

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
//    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                             UART_CONFIG_PAR_NONE));

    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));

    //ROM_IntEnable(INT_UART0);
    ROM_IntEnable(INT_UART1);
    //ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //bt

    UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    while(1)
    {
    }
}
