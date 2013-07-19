/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GCrash
//
// Simple program to access MPU6050 accelerometer and report
// data via bluetooth using a HC04 serial to bluetooth adapter

//   Date: 18 July 2013
//   programmer: Daniel Perron
//   Version: 1.01
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
    
      using TTL UART with fix baud at 115200, 8 bits data, no parity.
 
      ASCII COMMAND character
       [esc] 			-> IDLE  Go to idle mode (do nothing)
        G,g or enter	-> READY  Be ready for  free fall detection
        I or i          -> INFO   Display Acceleration info in G.
        D or d          -> DROP   Force free fall detection
        H or h          -> HIT    Force Crash detection

      Command available in IDLE mode only
        ?               -> display Release version.
        V or v          -> Display Battery Voltage.

        
       Record Format:   Everything in HEX Format (for speed, The PIC is too slow decode into decimal ascii)
               TTTTXXXXYYYYZZZZ\r\n
        where TTTT is time in ms
         and  XXXX is acceleration on X axis (signed short integer)
         and  YYYY is acceleration on Y axis (signed short integer)
         and  ZZZZ is acceleration on Z axis (signed short integer)


     
//////////////////////////  PORT DESCRIPTION
/*

 RA0    IN      not used 
 RA1	I/O    	I2C SCL
 RA2	I/O		I2C SDA
 RA3	IN		MCLR  Master Clear  (Reset cpu when is low)
 RA4	OUT	UART TX
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

#define iabs(A)  (A<0 ?  (-A) :  A)


__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_ON & PLLEN_OFF & BORV_LO & LVP_OFF);
__IDLOC(0000);


near volatile unsigned short Timerms;       //  Interrupt Timer counter in ms 
near volatile unsigned short TimerCrash;   // Interrupt Timer counter in ms  

// MODE OF OPERATION
//  
//  IDLE => Do nothing
//  INFO =>  display every second the MPU6050 Data
//  READY => Wait until the unit  drop. 
//  DROP =>  output every 1/100 of second the peak G force
//  HIT     =>  output for an other 0.5 second the data and at the end display the Peak acceleration force.




#define MODE_IDLE      			0
#define MODE_READY  	      	1
#define MODE_DROP		      	2
#define MODE_HIT			 	3
#define MODE_DONE		      	4
#define MODE_INFO				100

near volatile unsigned char Mode= 0xff;
near volatile unsigned char NewMode = MODE_IDLE;

// serial buffer 
#define SERIAL_BUFFER_SIZE 32
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
char SerialBuffer[SERIAL_BUFFER_SIZE];






// This is the Peak detected data
// The MPU6050 is scan every ms and the biggest overall  GForce is kept. This is reset every time the data is read by the main program
// We scan the MPU6050 every ms put we output signal every  at the maximum speed of the serial and string format which will  more than 1ms


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
TMR2IE=1;
PEIE = 1;
GIE=1;
}




void CalculateSumOfSquare(void);

char FindDrop(void);

static void interrupt isr(void){
// check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
        TXREG= SerialBuffer[OutFiFo];
        OutFiFo++;
       if(OutFiFo >= SERIAL_BUFFER_SIZE)
         OutFiFo=0;
      }
     else 
   if(OutFiFo == InFiFo)
     {
       // no more char to send
       // just disable interrupt
       TXIE=0;
     }
  }

// Timer 1 ms
	if(TMR2IF){
 TMR2IF=0; 
 Timerms++;
}
}



void putch(char char_out)
{
   unsigned char temp;

// increment InFiFo and  loop resize if need it.
   temp = InFiFo + 1;
   if(temp >= SERIAL_BUFFER_SIZE)
     temp = 0;

//  wait  if buffer full
  while(temp == OutFiFo);      

// ok write the buffer
  SerialBuffer[InFiFo]=char_out;
// now tell the interrupt routine we have a new char
InFiFo= temp;
// and enable interrupt
 TXIE=1;    
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


void printCentiValue(long value)
{
  char buffer[16];
  long _lvalue;
  unsigned short T;

  if(value <0)
   {
    putch('-');
    _lvalue = (-value); 
   }
  else
   _lvalue = value;
  
  ltoa(buffer,_lvalue /100,10);cputs(buffer);
  putch('.');

  T = (unsigned short) _lvalue % 100;

  if(T < 10)
    putch('0');
  utoa(buffer,T,10);cputs(buffer);  
   
}


void printGForce(long RawG)
{
  RawG *= 1600;
  RawG /= 32768;
  printCentiValue(RawG);
}

 void printUShort( unsigned short value)
{
   char buffer[16];
   utoa(buffer,value,10); cputs(buffer);
}
     



void CalculateSumOfSquares(void)
{

unsigned long GxSquare, GySquare, GzSquare;
long temp;

 // Calculate The Total Force
 // GForce = sqrt(x**2 + y**2 + z**2)
 // First  set them all positive. Sum of three 15 bits square won't overflow 32 bits unsigned
  
     
// iabs  ~9us
    GxSquare = (unsigned long) iabs( CurrentData.Gx);
    GySquare = (unsigned long) iabs(CurrentData.Gy);
    GzSquare = (unsigned long) iabs(CurrentData.Gz);



  GxSquare *= GxSquare;
  GySquare *= GySquare;
  GzSquare *= GzSquare;
  CurrentData.SumSquare =  GxSquare + GySquare + GzSquare;

if(CurrentData.SumSquare > PeakData.SumSquare)
   PeakData = CurrentData;
}

 
char FindCrash(void)
{
   // if all absolute acceleration axis are bigger than 1.5 G this is a hit
  // 1.5 G is equivalent to 1.5 *  32768/16 => 3072
     const short OneAndHalfG = 3072;
  if((iabs(CurrentData.Gx) > OneAndHalfG)  || (iabs(CurrentData.Gy) > OneAndHalfG) || (iabs(CurrentData.Gz) > OneAndHalfG))
            return 1;
   return 0;

}

char FindDrop(void)
{
   // if all absolute acceleration axis are less than 0.5 G this is a drop
  // 0.5 G is equivalent to 0.5 *  32768/16 => 1024
 const short HalfG = 1024;

  if(iabs(CurrentData.Gx) < HalfG)
    if(iabs(CurrentData.Gy) < HalfG)
      if(iabs(CurrentData.Gz) < HalfG)
            return 1;
return 0;
} 



void DisplayInfo(GForceStruct * gs)
{
  unsigned short  Gt;
  // assuming that interrupt routine is off
  Get_Accel_Values();  // values return in global variable  ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT
  CalculateSumOfSquares();
  // now let's display the info
  cputs(" Time(ms)=");
  printUShort(gs->Timer);
  cputs("  Gx=");
  printGForce(gs->Gx);
  cputs("  Gy=");
  printGForce(gs->Gy);
  cputs("  Gz=");
  printGForce(gs->Gz);
  // now  for the full force we will do the square root of the sum of the squares.
  Gt = isqrt(gs->SumSquare);
  cputs("  G=");
  printGForce(Gt);
  cputs("\r\n");  
}


void putHexNibble(unsigned char value)
{
   value &= 0xf;
   if(value > 9)
     value +=  'A' - 10;
 else
    value += '0';
   putch(value);

}



void putHex(unsigned short value)
{

    putHexNibble(value >> 12);
    putHexNibble(value >> 8);
    putHexNibble(value >> 4);
    putHexNibble(value);
}

void DisplayData(void)
{
// data will be display in hex since atoi is way too slow
// the format will be  TTTTXXXXYYYYZZZZ\r\n  (total of 18 characters).
// where 
// TTTT is 16 bits hex   time in ms.
// XXXX is 16 bits hex   Raw Gx value
// XXXX is 16 bits hex   Raw Gy value
// XXXX is 16 bits hex   Raw Gz value

   putHex(CurrentData.Timer);
   putHex(CurrentData.Gx);
   putHex(CurrentData.Gy);
   putHex(CurrentData.Gz);
   cputs("\r\n");

}


void printVersion(void)
{
   cputs("GCrash V1.01\r\n");
}


void printVoltage(void)
{
  // A/D use VDD has REFERENCE  and WE will check 2.048V Reference
  // so formula is   A/D Value = 2.048V * 1024 / VDD
  //         
  //  then
  //
  //       VDD =  2.048 * 1024 / (A/D Value)
  //  with the CentiValue
  //
  //       VDD = 2097152 / (A/D Value)
  //   

   unsigned short value;
   long  VDDValue;
   // Enable A/D And VFR
    // Right Justified, clock / 64 , VRef=VDD
    ADCON1= 0b11100000;
    // FIXED VOLTAGE REFERENCE  2.048V For A/D
    FVRCON= 0b11000010;
    // A/D Always on FVR
    ADCON0= 0b01111101;
    //wait a little
    __delay_ms(100);
   // start conversion
     ADGO=1;
   // wait
    while(ADGO);
    value = ADRES;
   cputs("VDD = ");
    if(value ==0)
      cputs("---");
    else
     {
    
       VDDValue = (long)209715 / ((long)value);
       printCentiValue(VDDValue);
     }
    cputs("V\r\n");

// disable A/D and VFR
   ADCON0=0;
   FVRCON=0;
}


 main(void){
    unsigned short m_Timer; 
    char UserKey;

	OSCCON		= 0b11110000;	// 32MHz
	OPTION_REG	= 0b00000011;	// pullups on, TMR0 @ Fosc/4/16 ( need to interrupt on every 80 clock 16*5)
	ANSELA		= 0b00000;	    // NO Analog 
	PORTA   	= 0b00100000;	
	WPUA		= 0b00111111;	// pull-up ON 
	TRISA		= 0b00011111;	// ALL INPUT  RA5 OUTPUT
//	VREGCON		= 0b00000010;	// lower power sleep please
    INTCON		= 0b00000000;	// no interrupt  


//  A/D & FVR OFF
     ADCON0=0;
     FVRCON=0;

   // set serial com with 115200 baud

    APFCON = 0b10000100;   // Set Rx/Tx to pin 2&3
    TXSTA = 0b10000010;
    RCSTA = 0;
//    BRGH =0;   //9600
    BRGH=1;      //115200  

    BRG16 = 0;
    SYNC =0;
 //   SPBRGL = 51;  //9600
    SPBRGL = 16; //115200

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
// reset interrupt fifo buffer
    InFiFo=0;
    OutFiFo=0;
    GIE = 1;
    PEIE =1;  // enable peripheral



     Init1msTimer() ;



 // wait for  serial uart  i/o pin toggle ( at least one full caracter length) 
  __delay_ms(100);
  printVersion();
  printVoltage();

  // wait 2 seconds 
 __delay_ms(2000);
  cputs("Test MPU6050 communication\n\r");
  i2c_Init();
  MPU6050_Test_I2C();
  Setup_MPU6050();

  Mode= RCREG; // clear receive buffer;
  Mode= 255;

 while(1)
{

 // check user command
 if(RCIF)
   { 
     UserKey = RCREG;  // get user command
     
     if ((UserKey == 'G' ) || (UserKey == 'g' ) || (UserKey == 0xd))
         NewMode=MODE_READY;
    else if (UserKey== 27)
         NewMode= MODE_IDLE;
    else if ((UserKey== 'I') || (UserKey== 'i'))
         NewMode= MODE_INFO;
    else if  (UserKey =='d')
         NewMode = MODE_DROP;
    else if ((UserKey == 'h' ) || (UserKey == 'H'))
         NewMode = MODE_DONE;
    else
       if(Mode == MODE_IDLE)
        {
           if  ((UserKey == 'V')  || (UserKey == 'v'))
                    printVoltage();
           else if (UserKey =='?')
                    printVersion(); 
       }

  }



  if(Mode != NewMode)
  {
     Mode=NewMode;

     switch(Mode)
    {
       case MODE_IDLE:   cputs("\r\nIDLE\r\n");
                                      break;
       case MODE_READY: cputs("\r\nREADY\r\n");
						    // clear the  datas
                                        PeakData.SumSquare=0;
                                        CurrentData.SumSquare=0;
                                        TimerCrash=65535;
                                        break;
       case MODE_DONE:   
 							cputs("*****\r\nMaximum Peak ");
                          
  							DisplayInfo(&PeakData);
 							cputs("*****\r\nDONE\r\n");  
                                        break;
    } 
}

  if(Mode == MODE_DROP)
     {
       while(GotInt_MPU6050()==0);
         Get_Accel_Values();
         CalculateSumOfSquares();
         DisplayData();
         if(FindCrash())
                TimerCrash=CurrentData.Timer + 1000;
          if(TimerCrash < CurrentData.Timer)
                NewMode= MODE_DONE;
          continue;
     }
   if(Mode == MODE_READY)
    {
       di();
        Timerms=0;
        ei();
       Get_Accel_Values();
       if(FindDrop())
         {
            CalculateSumOfSquares();
            DisplayData();
            NewMode= MODE_DROP;
           continue;
         }
    }
   if(Mode == MODE_INFO)
   {
      Get_Accel_Values();
      DisplayInfo(&CurrentData);
   }
    
}

}


