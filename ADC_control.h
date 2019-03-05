/*
 * ADC_control.h
 *
 *  Created on: 2018Äê7ÔÂ17ÈÕ
 *      Author: CZW
 */

#ifndef ADC_CONTROL_H_
#define ADC_CONTROL_H_
#define ADDRESS_ID 0xA1
#define SEND_ID 0xAD


extern volatile bool  RunAhead;
extern volatile bool  RunLeft;
extern volatile bool  RunRight;
extern volatile bool  RunRear;

extern volatile bool  RunLeft_rear;
extern volatile bool  RunLeft_ahead;
extern volatile bool  RunRight_rear;
extern volatile bool  RunRight_ahead;
extern volatile bool  ADCsend;
//extern volatile bool ADCflag;
extern volatile bool  bDataReady;     // The flag indicates is data ready.
extern uint32_t X_Ch0Value,Y_Ch1Value,X_Ch2Value;          // Savve the voltage value of ch0 and ch1.
//extern uint32_t Sign;

extern int pitch;
extern int speed;

extern void ADC_Init(void);
extern void ADC0IntHander(void);
extern void ADC_Trig(void);
extern void ConfigureUART0(void);
extern void DataHander(void);
extern void Recive_UART2_Config(void);
extern void UART2IntHandler(void);



#endif /* ADC_CONTROL_H_ */
