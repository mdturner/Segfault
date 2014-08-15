//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------
 
#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include <I2CHW_1Common.h>  
#include <I2CHW_1mstr.h> 
#include <stdlib.h>
#include <math.h>
//#include  "PWM8.h" 
#define SLAVE_ADDRESS 0x52
#define SLAVE_ADDRESS2 0x53
#define PI 3.14159
#define NLOOPS 50
#define NSWITCH 50
#define TIMEOUT 10
 
#define DEBUG
 
//Towing constants
#define T_ANGLE_LIMIT 20
#define T_TURN_LIMIT 10
#define T_S 3
#define T_P 5
 
//Balance constants
#define B_OFFSET_ANGLE 5
#define B_TURN_LIMIT 5
#define B_S 6 //Steering 6
#define B_P 9 //Gyro 12
#define B_I 18 //Accelerometer 24
#define B_D 0 //dGyro
#define TIME_STEP 0.009765625
#define OS_LIMIT 80 //Speed limit
#define OS_P 0.0
#define OS_I 0.03
#define OS_R 1.0
#define OS_ANGLE_MAX 0.02
 
//Loop constants
#define A_LP_S 0.1
#define A_LP 0.01 //Low pass frequency (in 100 Hz) 0.01
#define A_HP 0.99
 
BYTE    Zero=0x00;
BYTE	Initialize[2];
BYTE    rxBuffer[6];  
BYTE    status;
int msg;
 
 
// For Nunchuck
int Ax, Ay, Az;
float ThetaX,ThetaY,ThetaX_LP,ThetaY_LP,ThetaX_LP_Old;
 
// For WMP
int Roll;
float OmegaY,OmegaY_Old,OmegaY_HP,OmegaY_HP_Old,dOmegaY_HP;
//int Yaw, Pitch;
//float OmegaX,OmegaZ;
 
// For main program
BYTE line;
int nLoops;
int watchtime;
char timeout;
int nSwitch;
 
// For output
float Angle,Vel,Drive,Duty_1,Duty_2,ThetaX_Out,ThetaY_Out;
float os_sum,os_angle,os_angle_old;
 
float CoerceRange(float x_min, float x_max, float x) {
	if (x<x_min) x=x_min;
	if (x>x_max) x=x_max;
	return x;
}
 
void Nunchuck() {
	Ax=(rxBuffer[2]*2)+((rxBuffer[5]>>4)&0x1)-255;
	Ay=(rxBuffer[3]*2)+((rxBuffer[5]>>5)&0x1)-255;
	Az=((rxBuffer[4]&~0x01)*2)+((rxBuffer[5]>>6)&0x03)-255;
 
	//Exact way
	//ThetaX=-atan(Az/sqrt(Ax*Ax+Ay*Ay))*180.0/PI;
	//ThetaY=atan(Ax/sqrt(Ay*Ay+Az*Az))*180.0/PI;
 
	//Good enough way
	ThetaX=Az*90/103*2/PI;
	ThetaY=Ax*90/103*2/PI;
 
	//Low pass filter
	ThetaX_LP_Old=ThetaX_LP;
	ThetaX_LP*=(1-A_LP);
	ThetaX_LP+=A_LP*ThetaX;
 
	ThetaY_LP*=(1-A_LP_S);
	ThetaY_LP+=A_LP_S*ThetaY;
 
	//LCD Stuff 
	/* 
	LCD_1_Position(0,0);
	LCD_1_PrCString("       ");
	LCD_1_Position(0,0);
	LCD_1_PrString(ftoa(ThetaX, &msg));
	LCD_1_Position(0,7);
	LCD_1_PrCString("         ");
	LCD_1_Position(0,8);
	LCD_1_PrString(ftoa(ThetaY, &msg));
	*/
 
	//UART stuff
	/*
	UART_1_CPutString("Theta X_LP: ");
	UART_1_PutString(ftoa(ThetaX_LP, &msg));
	UART_1_CPutString("     Theta X: ");
	UART_1_PutString(ftoa(ThetaX, &msg));
	UART_1_CPutString("     Theta Y: ");
	UART_1_PutString(ftoa(ThetaY, &msg));
	UART_1_CPutString("     Time: ");
	*/
 
 
}
 
void WMP() {
	Roll = (rxBuffer[4]&~0x3)*64+rxBuffer[1];
 
	if (rxBuffer[4]&0x2) OmegaY = ((float) Roll-8192.)/20.*1.45;
		else OmegaY = ((float) Roll-8192.)/20.*6.59;
 
	OmegaY_HP+=OmegaY-OmegaY_Old;
	OmegaY_HP*=A_HP;
	dOmegaY_HP=OmegaY_HP-OmegaY_HP_Old;
	OmegaY_Old=OmegaY;
	OmegaY_HP_Old=OmegaY_HP;
 
	// X & Z
	/*
	Yaw = (rxBuffer[3]&~0x3)*64+rxBuffer[0];
	Pitch = (rxBuffer[5]&~0x3)*64+rxBuffer[2];
	if (rxBuffer[3]&0x1) OmegaX = ((float) Pitch-8192)/20;
		else OmegaX = (Pitch-8192)/4;
	if (rxBuffer[3]&0x2) OmegaZ = ((float) Yaw-8192)/20;
		else OmegaZ = (Yaw-8192)/4;
	*/
 
	//Uart Stuff
	/*
	//UART_1_CPutString("Omega X: ");
	//UART_1_PutString(ftoa(OmegaX, &msg));
	UART_1_CPutString("   Omega Y: ");
	UART_1_PutString(ftoa(OmegaY, &msg));
	UART_1_CPutString("   Omega Y HP: ");
	UART_1_PutString(ftoa(OmegaY_HP, &msg));
	//UART_1_CPutString("    Omega Z: ");
	//UART_1_PutString(ftoa(OmegaZ, &msg));
	*/
}
 
void Tow() {
	/*
	if (fabs(ThetaX_LP)<ANGLE_LIMIT) 
		{
		LED_1_Switch(1);
		//UART_1_CPutString("%");
		}
	else {
		LED_1_Switch(0);
	}
	*/
 
	//UART_1_PutString(ftoa(ThetaX_LP, &msg));
	//UART_1_PutCRLF();
 
	LED_1_Switch(1);
 
	ThetaX_Out=CoerceRange(-T_ANGLE_LIMIT,T_ANGLE_LIMIT,ThetaX_LP);
	ThetaY_Out=CoerceRange(-T_TURN_LIMIT,T_TURN_LIMIT,ThetaY_LP);
 
	Duty_1=(T_P*ThetaX_Out+T_S*ThetaY_Out+100)/200;
	Duty_1=CoerceRange(0,1,Duty_1);
 
	Duty_2=(T_P*ThetaX_Out-T_S*ThetaY_Out+100)/200;
	Duty_2=CoerceRange(0,1,Duty_2);
 
	PWM8_1_WritePulseWidth(Duty_1*255);
	PWM8_2_WritePulseWidth(Duty_2*255);
}
 
void Balance() {
	LED_1_Switch(1);
	//Angle=OmegaY_HP*TIME_STEP+ThetaX_LP;
	Angle=(1-A_LP)*(Angle+OmegaY_HP*TIME_STEP)+A_LP*(ThetaX-B_OFFSET_ANGLE)+os_angle;
	//Vel=OmegaY_HP; 
	Vel=(ThetaX_LP-ThetaX_LP_Old)/TIME_STEP+OmegaY_HP+(os_angle-os_angle_old)/TIME_STEP; 
 
	//Drive+=(B_P*ThetaX_LP+B_D*OmegaY_HP)*TIME_STEP;
	//Drive+=(B_P*Angle+B_D*OmegaY_HP)*TIME_STEP;
	Drive+=(B_I*Angle+B_P*Vel)*TIME_STEP+B_D*dOmegaY_HP;
 
	Drive=CoerceRange(-100,100,Drive);
 
	/*
	os_angle_old=os_angle;
	if (Drive>OS_LIMIT) {
		os_sum+=(Drive-OS_LIMIT)*TIME_STEP;
		os_angle=OS_P*(Drive-OS_LIMIT)+OS_I*os_sum;
		}
	else {
		os_sum=0;
		os_angle+=-OS_R*TIME_STEP;
		}
	os_sum=CoerceRange(0,OS_ANGLE_MAX/OS_I,os_sum);
	os_angle=CoerceRange(0,OS_ANGLE_MAX,os_angle);
	*/
 
	ThetaY_Out=CoerceRange(-B_TURN_LIMIT,B_TURN_LIMIT,ThetaY_LP);
	Duty_1=(Drive+B_S*ThetaY_Out+100)/200;
	Duty_1=CoerceRange(0,1,Duty_1);
	Duty_2=(Drive-B_S*ThetaY_Out+100)/200;
	Duty_2=CoerceRange(0,1,Duty_2);
	PWM8_1_WritePulseWidth(Duty_1*255);
	PWM8_2_WritePulseWidth(Duty_2*255);
}
 
void Neutral() {
LED_1_Switch(0);
}
 
void main()
{		
 Initialize[0]=0xfe;
 Initialize[1]=0x05;
 
 LCD_1_Start();
 I2CHW_1_Start();  
 I2CHW_1_EnableMstr();  
 M8C_EnableGInt;  
 I2CHW_1_EnableInt(); 
 
 LED_1_Start();
 
 UART_1_Start(UART_1_PARITY_NONE);
 
 DigInv_1_Start();
 DigInv_2_Start();
 
 PWM8_1_WritePulseWidth(127);  
 PWM8_2_WritePulseWidth(127);  
 
 /* ensure interrupt is disabled */  
 PWM8_1_DisableInt();  
 PWM8_2_DisableInt();  
 
 /* start the PWM8! */  
 PWM8_1_Start();  
 PWM8_2_Start();  
 
 LCD_1_Position(0,0);
 LCD_1_PrHexByte(0xBE);
 
/* Enable the global and local interrupts */ 
 
   SleepTimer_1_Start();  
   SleepTimer_1_SetInterval(SleepTimer_1_512_HZ);   // Set interrupt to a  
   SleepTimer_1_EnableInt();                     // 64 Hz rate  
 
/* Initialize */
		UART_1_CPutString("On");
		UART_1_PutCRLF();
		/* Send the contents of the data in txBuffer */;  
        I2CHW_1_bWriteBytes(SLAVE_ADDRESS2, Initialize, 2, I2CHW_1_CompleteXfer);  
        /* Wait until the data is transferred */  
        while(!(I2CHW_1_bReadI2CStatus() & I2CHW_WR_COMPLETE));  
 
		UART_1_CPutString("Initialized");
		UART_1_PutCRLF();
 
		LCD_1_Position(0,0);
		LCD_1_PrHexByte(I2CHW_1_bReadI2CStatus());
 
        I2CHW_1_ClrWrStatus();  
 
 
 
    /* Send and Receive forever*/  
    while( 1 )  
    {  
		timeout=FALSE;
		/* Request Data */;  
        I2CHW_1_bWriteBytes(SLAVE_ADDRESS, &Zero, 1, I2CHW_1_CompleteXfer);  
        /* Wait until the data is transferred */  
		watchtime = SleepTimer_1_bGetTimer();
 
		//while(!(I2CHW_1_bReadI2CStatus() & I2CHW_WR_COMPLETE)); 
        while((!(I2CHW_1_bReadI2CStatus() & I2CHW_WR_COMPLETE))&&(SleepTimer_1_bGetTimer()>(watchtime-TIMEOUT))); 
		if (SleepTimer_1_bGetTimer()<=(watchtime-TIMEOUT)) timeout=TRUE;
        /* Clear Write Complete Status bit */  
        I2CHW_1_ClrWrStatus();  
 
		//LCD_1_Position(0,3);
		//LCD_1_PrHexByte(I2CHW_1_bReadI2CStatus());
		SleepTimer_1_TickWait(1); 
 
		/* Read from the slave and place in rxBuffer */;  
        I2CHW_1_fReadBytes(SLAVE_ADDRESS,rxBuffer, 6, I2CHW_1_CompleteXfer);  
        /* Wait until the data is read */   
		watchtime = SleepTimer_1_bGetTimer();
 
		//while(!(I2CHW_1_bReadI2CStatus() & I2CHW_RD_COMPLETE)); 
		while((!(I2CHW_1_bReadI2CStatus() & I2CHW_RD_COMPLETE))&&(SleepTimer_1_bGetTimer()>(watchtime-TIMEOUT))); 
		if (SleepTimer_1_bGetTimer()<=(watchtime-TIMEOUT)) timeout=TRUE;
		/* Clear Read Complete Status bit */  
        I2CHW_1_ClrRdStatus();  
 
		//LCD_1_Position(0,6);
		//LCD_1_PrHexByte(I2CHW_1_bReadI2CStatus());
		SleepTimer_1_TickWait(1); 
 
		// Check source - 0 = Nunchuck, 1 = WM+
		line=(0x02&rxBuffer[5])>>1;
 
 
		if (timeout) {
			//Neutral();
			UART_1_CPutString("~");
			UART_1_PutCRLF();
			UART_1_PutString(ftoa(255-SleepTimer_1_bGetTimer(), &msg));
			SleepTimer_1_SetTimer(255);
			UART_1_PutCRLF();}
		else if (line==0x00) Nunchuck();
		else if (line==0x01) {
			WMP();
 
			while ((255-SleepTimer_1_bGetTimer())%5!=0) SleepTimer_1_TickWait(1);
 
			UART_1_PutString(ftoa(255-SleepTimer_1_bGetTimer(), &msg));
			SleepTimer_1_SetTimer(255);
			UART_1_PutCRLF();
 
			if (nLoops>NLOOPS) {
				LED_1_Switch(1);
				if ((PRT1DR & 0x01)==0) {
					if (nSwitch>NSWITCH) {
						Drive=0;
						Angle=ThetaX_LP;
						os_sum=0;
						os_angle=0;
						}
					else {
						nSwitch++;
						Balance();
						}
					}
				else if (PRT1DR & 0x02) {
					Drive=0;
					Angle=ThetaX_LP;
					Tow();
					nSwitch=0;
					os_sum=0;
					os_angle=0;
					}
				else {
					Balance();
					nSwitch=0;
					}
			}
			else {
				nLoops++;
				UART_1_CPutString("=");
				UART_1_PutCRLF();
			}
		}
 
    }  
}
