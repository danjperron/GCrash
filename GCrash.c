/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GCrash
//
// Simple program to access MPU6050 accelerometer and report
// data via bluetooth using a HC04 serial to bluetooth adapter

//   Date: 12 July 2013
//   programmer: Daniel Perron
//   Version: 1.0
//   Processor: PIC12F1840
//   Software: Microchip MPLAB IDE v8.90  with Hitech C (freeware version)
//   
                   

///////////////////  How to program the IC.
//
//    1 - Use MPLAB with pickit 3  (need version 8.90 at least)
//  or
//    2 - Use standalone pickit2 v 2.61 (Select the I.C. and load the corresponding hex file).
//  or
//    3 - Use Raspberry Pi board  with burnVLP.py . Software available from https://github.com/danjperron/A2D_PIC_RPI 
//        

////////////////////////////////////  GPL LICENSE ///////////////////////////////////
/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*  COMMUNICATION PROTOCOL
    
      using TTL UART with fix baud at 9600, 8 bits data, no parity.
 
      ASCII COMMAND character
         S  -> STOP   Stop and wait
         G  -> GO     Start to output data when system sense the drop.  Use STOP to end it.
         I  -> INFO   Output the current accelerometer data.
        
       Record Format:
          Time(msec)\tGx\t\Gy\t\Gz\tVoltage\n

     
//////////////////////////  PORT DESCRIPTION
/*

 RA0	ANALOG	Power Voltage (Vref = 2.048V) 2 resistors , equal value, divider to Battery 	
 RA1	I/O    	I2C SCL
 RA2	I/O     I2C SDA
 RA3	IN		MCLR  Master Clear  (Reset cpu when is low)
 RA4	OUT		UART TX
 RA5	IN		UART RX
*/




#include <htc.h>
#include <stddef.h>
#include <conio.h>
#include <stdlib.h>
#include "i2cMaster.h"
#include "MPU6050.h"

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
 #define _XTAL_FREQ 32000000
#endif

#ifndef BORV_LO
#define BORV_LO BORV_19
#endif

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_ON & PLLEN_OFF & BORV_LO & LVP_OFF);
__IDLOC(0000);

typedef unsigned char UInt8;
typedef unsigned short UInt16;

near bit  TimerSecFlag;
near volatile unsigned char RunStop;
near volatile unsigned long Timer;
near volatile unsigned int Timerms;



static void interrupt isr(void){

// assume that all memories reside in first bank
// need to be check after compilation
// only one interrupt so we won't check which one it is
//	if(TMR0IF){
 TMR2IF=0; 
Timer++;
 Timerms++;
 if(Timerms>=1000)
   {
     Timerms=0;
     TimerSecFlag=1;
  }
}



void Init1msTimer()   
{

// we want 1mss period so
// clock of 32Mhz give  125 ns   1ms / 125ns = 8000
// 8000  prescaler of 64 = 125 start count
T2CON=0;
 PR2 = 125; 
TMR2=0;
T2CON= 0b00000111; // Timer on prescaller 64
 // Enable IRQ
TMR2IF=0;
 Timerms=0;
 Timer=0;
 TimerSecFlag=0;
TMR2IE=1;
PEIE = 1;
GIE=1;
}


void A2DInit()
{
ADCON1= 0b10100011;  // right justified, fosc/32 vref internal fixed.
ADCON0= 0b00000001; // enable a/d
ADIE=0;
ADIF=0;
FVRCON=0b11000010;  // Vref internal 2.048V on ADC
}

void putch(char char_out)
{
   while(!TRMT);  // wait until transmit done
   TXREG= char_out;
}

// from  	http://www.azillionmonkeys.com/qed/sqroot.html
static unsigned short isqrt(unsigned long val) {
	    unsigned long temp, g=0, b = 0x8000, bshft = 15;
	    do {
	        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
	           g += b;
	           val -= temp;
	        }
	    } while (b >>= 1);
	    return g;
	}

void printGForce(long RawG)
{
  char buffer[16];
  unsigned char T;
  RawG *= 1600;
  RawG /= 32768;

  ltoa(buffer,RawG /100,10);cputs(buffer);
  putch('.');

  if(RawG<0)
     T = (unsigned char) ((-RawG) % 100);
   else
     T = (unsigned char) RawG % 100;

  if(T < 10)
    putch('0');
  ultoa(buffer,T,10);cputs(buffer);  
    
}



 main(void){
    char buffer[16];
    unsigned long Accel,AccX,AccY,AccZ;
    long temp;
 	UInt8 Counter=0;  
    unsigned long m_Timer;
	OSCCON		= 0b11110000;	// 32MHz
	OPTION_REG	= 0b00000011;	// pullups on, TMR0 @ Fosc/4/16 ( need to interrupt on every 80 clock 16*5)
	ANSELA		= 0b00001;	    // RA0 Analog 
	PORTA   	= 0b00100000;	
	WPUA		= 0b00111111;	// pull-up ON 
	TRISA		= 0b00011111;	// ALL INPUT  RA5 OUTPUT
//	VREGCON		= 0b00000010;	// lower power sleep please
    INTCON		= 0b00000000;	// no interrupt  


   // set serial com with 9600 baud

    APFCON = 0b10000100;   // Set Rx/Tx to pin 2&3
    TXSTA = 0b10000010;
    RCSTA = 0;
    BRGH =0;   
    BRG16 = 0;
    SYNC =0;
    SPBRGL = 51;
    SPBRGH =0;
    TXEN =1;   // enable transmitter
    SPEN = 1;  // enable serial port
    CREN = 1;  // enable receiver
    RCIE =0;   // disable received interrup;
    TXIE =0;   // disable transmit interrupt
    RCIF =0;   // clear received flag
    TXIF = 0;
    SCKP = 0;
    ABDEN = 0;

   // wait for  serial uart  i/o pin toggle ( at least one full caracter length) 
       __delay_ms(100);

   cputs("GCrash V1.0\r\n");

  
  Init1msTimer();

  TimerSecFlag=0;while(!TimerSecFlag);
  cputs("Test MPU6050 communication\n\r");
  i2c_Init();
  MPU6050_Test_I2C();
  Setup_MPU6050();

  cputs("Ready\r\n");
TimerSecFlag=0;

      
 while(1)
{
 // if(TimerSecFlag)
  {
  di();
  m_Timer = Timer;
  ei();

    TimerSecFlag=0;
    Get_Accel_Values();

#define iabs(A)  (A<0 ? (unsigned long) (-A) : (unsigned long) A)
    
   
    
    AccX = iabs(ACCEL_XOUT);
    
    AccY = iabs(ACCEL_YOUT);
    

   temp = (long) ACCEL_ZOUT;
   temp+=204;
    
    AccZ = iabs(temp);

    Accel = isqrt(AccX + AccY + AccZ);

  //cputs("\r\n************\r\nTimer\t X \t Y \t Z\r\n");
  ultoa(buffer,m_Timer,10);cputs(buffer);
  putch('\t');
  printGForce((long)ACCEL_XOUT);
  putch('\t');
  printGForce((long)ACCEL_YOUT);
  putch('\t');
  printGForce((long)ACCEL_ZOUT);
  putch('\t');
  
 /* itoa(buffer,ACCEL_XOUT,10);cputs(buffer);
  putch('\t');
  itoa(buffer,ACCEL_YOUT,10);cputs(buffer);
  putch('\t');
  itoa(buffer,ACCEL_ZOUT,10);cputs(buffer);
  cputs("\r\n");
  cputs("AccX= ");
  ultoa(buffer,AccX,10);cputs(buffer);
 cputs("\r\nAccY= ");
  ultoa(buffer,AccY,10);cputs(buffer);
 cputs("\r\nAccY= ");
  ultoa(buffer,AccZ,10);cputs(buffer);
 cputs("\r\nSquare it");
 */
   AccX*=AccX;
   AccY*=AccY;
   AccZ*=AccZ;
/*  cputs("\r\n");
  cputs("AccX^2= ");
ultoa(buffer,AccX,10);
  cputs(buffer);
 cputs("\r\nAcc^Y= ");
 ultoa(buffer,AccY,10);cputs(buffer);
 cputs("\r\nAcc^Z= ");
 ultoa(buffer,AccZ,10);cputs(buffer);
 cputs("\r\nSumIt");
*/
   Accel = AccX + AccY + AccZ;
//  cputs("\r\nSum=");
//  ultoa(buffer,Accel,10);cputs(buffer);
//  cputs("\r\n Square root =");
    Accel = isqrt(Accel);
 // ultoa(buffer,Accel,10);cputs(buffer);
 // cputs("\r\n G Force =");

  printGForce(Accel);
/*     
  Accel *= 1600;
  Accel /= 32768;

  ultoa(buffer,Accel /100,10);cputs(buffer);
  putch('.');
  if((Accel % 100) < 10)
    putch('0');
  ultoa(buffer,Accel % 100,10);cputs(buffer);  
*/
    cputs("\r\n");
 // cputs("\r\n************\r\n");
  
//   printf("%lu\t%d\t%d\t%d\r\n",m_Timer,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT);
//  printf("%lu\t",m_Timer);
//  printf("%lu.%02lu",Accel/100,Accel%100);
//  printf("\r\n");
  }
}
  
}