//*****************************************************************************
                              /****** EEC 195B NATCAR ******/
// Luke Alcantara
// Quentin Zhong
// Christopher Gan
// Code for the camera and servo together and motor and bluetooth

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"

volatile uint32_t pui32ADC0Value[129];  //circular buffer
volatile uint32_t pingBuffer[129];
volatile uint32_t pongBuffer[129];
volatile uint32_t whiteblack[129];
volatile uint32_t threshold=180;
volatile int32_t slope[129];
volatile uint32_t pingFull = 0;
volatile uint32_t pongFull = 0;
volatile uint32_t clk_counter;
volatile int32_t maxIndex = 0;
volatile int32_t minIndex = 0;
volatile int32_t meanIndex = 0;

volatile uint32_t sum = 0;
volatile int32_t mean = 0;
volatile int32_t max = 0;
volatile int32_t min = 3000;
volatile uint32_t Vpp = 0;
volatile uint32_t count = 0;
volatile uint32_t toggle = 0; //check if last buffer written to was ping or pong
char letter;
char letter2;
volatile uint32_t count;
volatile uint32_t run = 1;
volatile uint32_t timer_rate = 500;

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
void
Timer0IntHandler(void) //timer interrupt handler
{
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.

	//UARTprintf("\nTEST1\n");
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); // assert SI signal high
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // assert test signal high
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // assert CLK signal high
    clk_counter = 1;
    //<<========  UPDATE BUFFER POINTER

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert SI signal LOW
    ADCProcessorTrigger(ADC0_BASE, 3); // Start an ADC conversion using software triggering
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // deassert CLK signal LOW

    //ROM_IntMasterDisable();
    //UARTprintf("\rTimerTest\n");
    //ROM_IntMasterEnable();

}

//*****************************************************************************
void
adcHandler(void) //adc interrupt handler
{

	ADCIntClear(ADC0_BASE, 3); //clear interrupt request

	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // assert CLK signal HIGH

	ADCSequenceDataGet(ADC0_BASE, 3, &pui32ADC0Value[clk_counter]); //address of element

	if (toggle == 0)// Store in Ping or Pong buffer if empty-Ping has priority
	{
		pingBuffer[clk_counter] = (pui32ADC0Value[clk_counter]);
	}
	else
	{
		pongBuffer[clk_counter] = (pui32ADC0Value[clk_counter]);
	}

	clk_counter = clk_counter + 1; //increment counter


	if (clk_counter < 129)// Check if not full
	{
		ADCProcessorTrigger(ADC0_BASE, 3); // Start an ADC conversion using software triggering
	}
	else
	{
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); // deassert test signal LOW
		if (toggle == 0){
			pingFull = 1; //switch pingfull and pongfull flags
			pongFull = 0;
			clk_counter = 0; //restart clock counter
			toggle = 1;

		}
		else {
			pongFull = 1; //switch pingfull and pongfull flags
			pingFull = 0;
			clk_counter = 0; //restart clock counter
			toggle = 0;
		}
	}

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // deassert CLK signal LOW
}

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
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low  //brake

		if(input == 'a')
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // INB high
		if(input == 's')
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // INB low


        if(input == '0' || input == 'q')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 0); 					  //0 Motor stopped
        if(input == '1')
        	 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
        	                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 1 / 10); //10%
        if(input == '2')
        	 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
        	                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 1 / 5);  //20%
		if(input == '3' || input == 'd')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 3 / 10);  //30%
		if(input == '4' || input == 'f')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 4 / 10); //40%
		if(input == '5' || input == 'g')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 5 / 10); //50%
		if(input == '6' || input == 'h')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 6 / 10); //60%
		if(input == '7' || input == 'i')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 7 / 10); //70%
		if(input == '8' || input == 'j')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 8 / 10); //80%
		if(input == '9' || input == 'k')
			 PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
			                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 9 / 10);  //90%

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

void
vt(void)
{
	int j = 1;

	if (pingFull)
	{
		for (j=1; j < 129; j++)
		{
			if (pingBuffer[j]<=threshold) // check if less than theshold, or greater than, and write a 1 or 0
			{
				whiteblack[j]=0;
			}
			else if (pingBuffer[j]>threshold)
			{
				whiteblack[j]=1;
			}
		}

		for (j =1; j < 129; j++)
		{
			UARTprintf("%d",whiteblack[j]); // simply prints out if 1 or 0
		}
		UARTprintf("\n");
	}

	else if (pongFull)
	{
		for (j=1; j < 129; j++)
		{
			if (pongBuffer[j]<=threshold) // check if less than theshold, or greater than, and write a 1 or 0
			{
				whiteblack[j]=0;
			}
			else if (pongBuffer[j]>threshold)
			{
				whiteblack[j]=1;
			}
		}
	for (j =1; j<129; j++)
		{
			UARTprintf("%d",whiteblack[j]); // simply prints out if 1 or 0
		}
		UARTprintf("\n");
	}
}

//*****************************************************************************

void
st(void)
{
	int i = 1;
	max = 0;
	min = 3000;
	maxIndex = 0;
	minIndex = 0;

	if (pingFull)
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0;
			}
			else
			{

				if (pingBuffer[i-1] > pingBuffer[i+1]) // get value before and after
				{
					slope[i] = ((pingBuffer[i-1]-pingBuffer[i+1]) >> 1)*(-1); //slope equation if a negative #
				}
				else
					{
					slope[i] = (pingBuffer[i+1]-pingBuffer[i-1]) >> 1;
					}
			}

			if (slope[i]>max)
			{
				max = slope[i]; //continuously re-update the max
				maxIndex = i;
			}
			if (slope[i]<min)
			{
				min = slope[i]; //continuously re-update the min
				minIndex = i;
			}
		}
	}
	else if (pongFull)
	{
		for (i=1; i < 129; i++)
		{
			if (i<3 || i > 127)
			{
				slope[i] = 0;
			}

			else
			{
				if (pongBuffer[i-1] > pongBuffer[i+1])
				{
					slope[i] = ((pongBuffer[i-1]-pongBuffer[i+1]) >> 1)*(-1);
				}
				else
				{
					slope[i] = (pongBuffer[i+1]-pongBuffer[i-1]) >> 1;
				}
			}

			if (slope[i]>max)
			{
				max = slope[i]; //continuously re-update the max
				maxIndex = i;

			}
			if (slope[i]<min)
			{
				min = slope[i]; //continuously re-update the min
				minIndex = i;
			}

			//UARTprintf("%i ", slope[i]);
		}
	}

	meanIndex = (minIndex + maxIndex) >> 1; //TODO: change to bit shift

	for(i = 1; i < 128; i++)
	{
		if (i < minIndex && i > maxIndex)
		{
			UARTprintf("1");
		}
		else
		{
			UARTprintf("0");
		}
	}
	UARTprintf("\n");
	/*if((meanIndex >= 0) && (meanIndex < 15))
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) >> 2); //FULLY CCW
	if(meanIndex >= 15 && meanIndex < 29)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * 10 / 36); //3/4 FULLY CCW
	if(meanIndex >= 29 && meanIndex < 43)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 31); //1/2 FULLY CCW
	if(meanIndex >= 43 && meanIndex < 57)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 5 / 14); //1/4 FULLY CCW
	if(meanIndex >= 57 && meanIndex < 71)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 3 >> 3); //NEUTRAL 13
	if(meanIndex >= 71 && meanIndex < 85)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) << 2 / 10); //1/4 FULLY CW 12
	if(meanIndex >= 85 && meanIndex < 99)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 23); //1/2 FULLY CW 11
	if(meanIndex >= 99 && meanIndex < 113)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)* 10 / 21); //3/4 FULLY CW
	if(meanIndex >= 113 && meanIndex < 128)
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
							 PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) >> 1); //FULLY CW*/
//
//	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
//								 (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / (4*(1/((meanIndex + 128) >> 7)))));
	if (meanIndex > 100)
	{
		meanIndex = 100;
	}
	else if (meanIndex < 20)
	{
		meanIndex = 20;
	}
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
									 ((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) * (meanIndex + 128)) >> 9));
}
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


//*****************************************************************************
int
main(void)
{

    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();


    UARTprintf("\nLAB 4.:\n");
	UARTprintf("\nLuke Alcantara\nQuentin Zhong\nChristopher Gan\n");
	//UARTprintf("Enter 'p' to print, 'c' to continue, or 'q' to quit\n");

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //motor
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2); // ENA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); // INA
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // ENB
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5); // INB
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // LED

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //bluetooth

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Enable GPIO Port E/F (and ADC port e pin3)
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntRegister(ADC0_BASE,3,adcHandler);
	ADCIntEnable(ADC0_BASE,3);
	ADCIntClear(ADC0_BASE, 3);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//PWM peripheral enable

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //enable the timer
    ROM_IntMasterEnable();

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/timer_rate);

    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

     SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //motor

     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//motor

     GPIOPinConfigure(GPIO_PC4_M0PWM6);
     GPIOPinConfigure(GPIO_PB7_M0PWM1);//motor

     GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
     GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//motor

     PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);
     PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);//motor

     PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 12500);//250MHz servo
     PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 3125); //62500 1kHz//motor

     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) *3 >> 3);//neutral
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * 3 / 10);  //30%

     PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
     PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);//motor

     PWMGenEnable(PWM0_BASE, PWM_GEN_3);
     PWMGenEnable(PWM1_BASE, PWM_GEN_0);//motor

     GPIOPinConfigure(GPIO_PB0_U1RX); //bluetooth
     GPIOPinConfigure(GPIO_PB1_U1TX); //bluetooth
     ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //bt
     ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                      UART_CONFIG_PAR_NONE));
     ROM_IntEnable(INT_UART1);
     ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //bt
     UARTSend((uint8_t *)"\033[2JEnter text: ", 16);
    while(run==1)
    {
    	if (pingFull | pongFull)
    	{
    		st();
    		/*if (ROM_UARTCharsAvail(UART0_BASE)) //check if any available input from the keyboard
    		{
    			letter = UARTgetc(); //grab the first char at the top
    			if (letter == 'v') //check if keyboard 'v', call voltage threshold
    			{
    				vt();
    			}
    			if ((letter != 'v') || (letter == 's')) // call slope threshold as default
    			{
					st();
    			}

    		}

    		else //nothing entered on keyboard
    		{
    			st();
    		}*/
    	}

    }
}
