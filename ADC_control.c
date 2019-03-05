/*
 * ADC_control.c
 *
 *  Created on: 2018年7月17日
 *      Author: CZW
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
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




#define ADDRESS_ID 0xA1
#define SEND_ID 0xAD

int8_t str[6]={0xEA,SEND_ID,ADDRESS_ID,0X01,0XFF,0XEF};

//EA SEND_ID ADDRESS_ID 01 00 FF

volatile bool  RunAhead = false;
volatile bool  RunLeft = false;
volatile bool  RunRight = false;
volatile bool  RunRear = false;
volatile bool  RunLeft_rear = false;
volatile bool  RunLeft_ahead = false;
volatile bool  RunRight_rear = false;
volatile bool  RunRight_ahead = false;
volatile bool  ADCsend = true;
//volatile bool ADCflag = false;
int pitch = 0;
volatile bool  bDataReady = true;     // The flag indicates is data ready.
uint32_t X_Ch0Value, Y_Ch1Value,X_Ch2Value;          // Savve the voltage value of ch0 and ch1.
//uint32_t Sign;

int speed=0;
void ADC_Init(void);
void ADC0IntHander(void);
void ADC_Trig(void);

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void ConfigureUART0(void)//串口
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}

void Recive_UART2_Config(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinConfigure(GPIO_PD7_U2TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	UARTConfigSetExpClk(UART2_BASE,SysCtlClockGet(), 9600,
	                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                                 UART_CONFIG_PAR_NONE));
    IntPrioritySet(INT_UART2 ,0x11); //b '000'0 0000 //工程中的最高优先级
    IntEnable(INT_UART2);
    UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);

}


void UART2IntHandler(void)
{
	uint32_t ui32Status;
	uint8_t rx_buffer;
	static bool receive_flag = false;
	static bool receive_pitch_flag = false;
	static int index=0;
	ui32Status = UARTIntStatus(UART2_BASE, true);
	UARTIntClear(UART2_BASE, ui32Status);
	while(UARTCharsAvail(UART2_BASE))
	{
		rx_buffer = (uint8_t)(UARTCharGetNonBlocking(UART2_BASE));
		if(rx_buffer==0xDA)
		{
			receive_flag = true;
			index=0;
		}
		if(receive_flag)
		{
			index += 1;
			if(index==2)
			{
				switch(receive_flag)
				{
				case 0x01:
					receive_pitch_flag=true;
					break;
				}
			}
		}
		if(index==3&&receive_pitch_flag)
		{
			pitch = rx_buffer;
			receive_flag = false;
			receive_pitch_flag = false;
			index = 0;
		}
	}
}

//
// ADC0 interrupt handler.
//
void ADC0IntHander(void)
{
	uint32_t ui32AdcValue[3] = {0};

	// Clear the interrupt flag.
    ADCIntClear(ADC0_BASE, 0);

    // Get the register value.
    ADCSequenceDataGet(ADC0_BASE, 0, ui32AdcValue);

    // Calculate the voltage, unit in mv.
    X_Ch0Value = ((float)ui32AdcValue[0])*(3300.0/4096.0);
    Y_Ch1Value = ((float)ui32AdcValue[1])*(3300.0/4096.0);
    X_Ch2Value = ((float)ui32AdcValue[2])*(3300.0/4096.0);
	// Set the flag.
	bDataReady = true;
	//UARTprintf("ch0:%4umv, ch1:%4umv\n, ch2:%4umv\n", X_Ch0Value,Y_Ch1Value,X_Ch2Value);
	//ADCflag = true;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
}

void DataHander(void)
{
	if(ADCsend == true)
	{
		int i;
		if((X_Ch0Value > 1700) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value > 1680) && (X_Ch2Value < 1750))
		{
			str[4]=0x11;
			RunAhead = true;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(500*1.8/360*3.14*3.3*3.3);

		}
		else if((X_Ch0Value < 1620) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value > 1680) && (X_Ch2Value < 1750))
		{
			str[4]=0x22;
			RunRear = true;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(-500*1.8/360*3.14*3.3*3.3);

		}
		else if((Y_Ch1Value > 1700) && (X_Ch0Value > 1620) && (X_Ch0Value < 1700) &&
				(X_Ch2Value > 1680) && (X_Ch2Value < 1750))
		{
			str[4]=0x33;
			RunRight = true;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(250*1.8/360*3.14*3.3*3.3);

		}
		else if((Y_Ch1Value < 1500) && (X_Ch0Value > 1620) && (X_Ch0Value < 1700) &&
				(X_Ch2Value > 1680) && (X_Ch2Value < 1750))
		{
			str[4]=0x44;
			RunLeft = true;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(250*1.8/360*3.14*3.3*3.3);
		}
		else if((Y_Ch1Value > 1500) && (Y_Ch1Value < 1700) && (X_Ch0Value > 1620) && (X_Ch0Value < 1700) &&
				(X_Ch2Value > 1680) && (X_Ch2Value < 1750))//停
		{
			str[4]=0x00;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(0*1.8/360*3.14*3.3*3.3);
		}

		if((X_Ch0Value > 1700) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value > 1750))  //前加速
		{
			str[4]=0xA1;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(750*1.8/360*3.14*3.3*3.3);
		}

		if((X_Ch0Value > 1700) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value < 1680)) //前减速
		{
			str[4]=0xB1;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(250*1.8/360*3.14*3.3*3.3);
		}
		if((X_Ch0Value < 1620) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value > 1750))  //后加速
		{
			str[4]=0xA2;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(-750*1.8/360*3.14*3.3*3.3);
		}

		if((X_Ch0Value < 1620) && (Y_Ch1Value < 1700) && (Y_Ch1Value > 1500) &&
				(X_Ch2Value < 1680)) //后减速
		{
			str[4]=0xB2;
			for(i=0;i<6;i++)
				UARTCharPutNonBlocking(UART2_BASE,str[i]);
			speed = (int)(-250*1.8/360*3.14*3.3*3.3);
		}
	}

}


//
// Init the adc.
//
void ADC_Init(void)
{
	//
	// Enable the peripheral adc0.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
			;

	//
	// Init the peripheral GPIOE.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
		;

	//
	// Configure the GPIOE2,3 to peripheral function.
	//
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 |GPIO_PIN_3|GPIO_PIN_1);

	//
	// Sets the clock configuration for the ADC: use piosc, full rate.
	//
	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);

	//
	// Set the reference voltage.
	// Attention: tm4c123x default reference voltage is 3.3v, can't change.
	//
	ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

	//
	// Enable the first sample sequencer to capture the value of channel 0
	// and channel 1 when the processor trigger occurs.
	//
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

	//
	// Configure a step of the sample sequencer.
	//! \param ui32Base is the base address of the ADC module.
	//! \param ui32SequenceNum is the sample sequence number.
	//! \param ui32Step is the step to be configured.
	//! \param ui32Config is the configuration of this step;
	// *Specific information refer to data sheet and drverlib.
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2 |ADC_CTL_END | ADC_CTL_IE);
	//
	// Configure the hardware over sample value: 64.
	// param ui32Factor is the number of samples to be averaged.
	//
	ADCHardwareOversampleConfigure(ADC0_BASE, 64);

	//
	// Enables sample sequence0.
	//
	ADCSequenceEnable(ADC0_BASE, 0);

	//
	// Register the interrupt handler.
	//
	ADCIntRegister(ADC0_BASE, 0, ADC0IntHander);

	//
	// Enable the interrupt sequence0 of ADC0.
	//
	ADCIntEnable(ADC0_BASE, 0);

	//
    // set the interrut priority.
	//
    IntPrioritySet (INT_ADC0SS0, 0x10);

    //
    // Enable the interrupt of sequence0.
    //
	IntEnable(INT_SSI0);
}

//
// Trigger a interrupt of ADC0.
//
void ADC_Trig(void)
{
	ADCProcessorTrigger(ADC0_BASE, 0);
}

