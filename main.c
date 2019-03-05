/*
 * main.c

 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "ADC_control.h"
#include "inc/hw_types.h"
#include "string.h"
#include "inc/hw_gpio.h"

// PE1 PE2 PE3
int height = 0;
int time_count = 0;
bool count_flag = true;

void LED_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3);
}

void Recive_UART1_Config(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0| GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void UART1IntHandler(void)
{
    uint32_t ui32Status;
    char rx_buffer;
    static bool receive_flag=false;
    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);
    while(UARTCharsAvail(UART1_BASE))
    {
        rx_buffer = (uint8_t)(UARTCharGetNonBlocking(UART1_BASE));
        if(rx_buffer==0xEA)
        {
            receive_flag = true;
        }
        if(receive_flag&&rx_buffer!=0xEA)
        {
        	switch(rx_buffer)
        	{
        	case 0x01:
        		ADCsend = false;
        		UARTCharPutNonBlocking(UART2_BASE, 0xEA);
        		UARTCharPutNonBlocking(UART2_BASE, 0xAD);
        		UARTCharPutNonBlocking(UART2_BASE, 0xA1);
        		UARTCharPutNonBlocking(UART2_BASE, 0x02);
        		UARTCharPutNonBlocking(UART2_BASE, 0x11);
        		UARTCharPutNonBlocking(UART2_BASE, 0x11);
        		UARTCharPutNonBlocking(UART2_BASE, 0xEF);
        		speed = (int)(500*1.8/360*3.14*3.3*3.3);
        		time_count = 0;
        		count_flag = true;
        		break;
        	case 0x02:
        		ADCsend = false;
        		UARTCharPutNonBlocking(UART2_BASE, 0xEA);
        		UARTCharPutNonBlocking(UART2_BASE, 0xAD);
          		UARTCharPutNonBlocking(UART2_BASE, 0xA1);
        		UARTCharPutNonBlocking(UART2_BASE, 0x02);
        		UARTCharPutNonBlocking(UART2_BASE, 0x44);
        		UARTCharPutNonBlocking(UART2_BASE, 0x44);
        		UARTCharPutNonBlocking(UART2_BASE, 0xEF);
        		speed = (int)(500*1.8/360*3.14*3.3*3.3);
        		time_count = 0;
        		count_flag = true;
        		break;
        	case 0x03:
        		UARTCharPutNonBlocking(UART2_BASE, 0xEA);
        		UARTCharPutNonBlocking(UART2_BASE, 0xAD);
        		UARTCharPutNonBlocking(UART2_BASE, 0xA1);
        		UARTCharPutNonBlocking(UART2_BASE, 0x01);
        		UARTCharPutNonBlocking(UART2_BASE, 0x00);
        		UARTCharPutNonBlocking(UART2_BASE, 0xEF);
        		speed = (int)(500*1.8/360*3.14*3.3*3.3);
        		ADCsend = true;
        		count_flag = false;
        		break;

        	case 0x04:
        		UARTCharPutNonBlocking(UART2_BASE, 0xEA);
        		UARTCharPutNonBlocking(UART2_BASE, 0xAD);
        		UARTCharPutNonBlocking(UART2_BASE, 0xA1);
        	    UARTCharPutNonBlocking(UART2_BASE, 0x02);
        	    UARTCharPutNonBlocking(UART2_BASE, 0x55);
        	    UARTCharPutNonBlocking(UART2_BASE, 0x55);
          		UARTCharPutNonBlocking(UART2_BASE, 0xEF);
          		speed = (int)(500*1.8/360*3.14*3.3*3.3);
          		time_count = 0;
          		count_flag = true;
        	    ADCsend = false;
        	    break;
        	}
        	receive_flag=false;
        }

    }
}

void Key_PF4_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void KEY_PF4_Pros(void)
{
    uint8_t ReadPin;
    ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
    if((ReadPin&GPIO_PIN_4)!=GPIO_PIN_4)
    {
        SysCtlDelay(20*(40000000/3000));
        ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
        if((ReadPin&GPIO_PIN_4)!=GPIO_PIN_4)
        {
        	ADCsend = false;
        	UARTCharPutNonBlocking(UART2_BASE,0xEA);
        	UARTCharPutNonBlocking(UART2_BASE,0xAD);
        	UARTCharPutNonBlocking(UART2_BASE,0xA1);
        	UARTCharPutNonBlocking(UART2_BASE,0x01);
        	UARTCharPutNonBlocking(UART2_BASE,0x11);
        	UARTCharPutNonBlocking(UART2_BASE,0xEF);
        	speed = 500*1.8/360*3.14*3.3*3.3;
        	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
        	SysCtlDelay(100*(SysCtlClockGet()/3000));
        	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0x0);
        	time_count=0;
        	count_flag = true;
        }
     while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
     }
}

void Timer1_Config(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	//TIMER1_BASE A
	// Configure the two 32-bit periodic timers. //周期性计时
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/100);     //10 ms 记一次
	IntPrioritySet(INT_TIMER1A ,0x00); //b '000'0 0000 //工程中的最高优先级
	// Setup the interrupts for the timer timeouts.
	//
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//TimerIntEnable()
	// Enable the timers.
	//
	TimerEnable(TIMER1_BASE, TIMER_A);
	//UARTprintf(" \n Timer1 ok ");
}

void Timer1IntHandler(void)   //10ms执行一次
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //countX++;
	if(count_flag)
	{
		time_count++;
	}
}

void main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    FPULazyStackingEnable();
    FPUEnable();
    IntMasterEnable();
    LED_Init();
    // Init uart.
    ConfigureUART0();
    Recive_UART2_Config();
    Recive_UART1_Config();
    ADC_Init();
    Key_PF4_Init();
    Timer1_Config();
    while(1)
    {
    	// Delay some time.
    	SysCtlDelay(20*(SysCtlClockGet()/3000));

    	// Trigger a ADC sample interrupt.
    	ADC_Trig();

    	// Wait for sample finish.
    	while(true != bDataReady)
    		;
		// Reset the flag after sample finish.
		bDataReady = false;

		// Print the voltage.

		DataHander();
		height = (uint8_t)((pitch/360.0)*2*3.14*30);
		UARTprintf("n3.val=%d", height);
		UARTprintf("%c%c%c", 0xFF, 0xFF, 0xFF);
		UARTprintf("addt 6,1,%d", height);
		UARTprintf("%c%c%c", 0xFF, 0xFF, 0xFF);
		UARTprintf("n2.val=%d", (int)(speed*0.8));
		UARTprintf("%c%c%c", 0xFF, 0xFF, 0xFF);
		UARTprintf("n0.val=%d", (int)(0.01*time_count));
		UARTprintf("%c%c%c", 0xFF, 0xFF, 0xFF);
		UARTprintf("n1.val=%d", (int)(0.01*time_count)*(int)speed);
		UARTprintf("%c%c%c", 0xFF, 0xFF, 0xFF);
		KEY_PF4_Pros();
    }
}


