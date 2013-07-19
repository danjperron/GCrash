#include <htc.h>
#include <stddef.h>
#include <conio.h>
#include <stdlib.h>
#include "i2cMaster.h"
#include "MPU6050.h"

int MPU6050_Test_I2C(void)
{
    unsigned char Data = 0x00;
    LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);
 
    if(Data == 0x68)
    {
      cputs("I2C Read Test Passed, MPU6050 Address: 0x68\r\n");       
      return(1);   
   }
     
   cputs("ERROR: I2C Read Test Failed, Stopping\r\n");
 
    return(0);
}


void Setup_MPU6050(void)
{
   int loop;
   unsigned char TheReg;
  
// I need code space. Just create a table with register to clear
const unsigned char MPU6050RegTable[]= {
    MPU6050_RA_FF_THR,    		//Freefall threshold of |0mg|  LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
    MPU6050_RA_FF_DUR,    		//Freefall duration limit of 0   LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
    MPU6050_RA_MOT_THR,		//Motion threshold of 0mg     LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
    MPU6050_RA_MOT_DUR,    		//Motion duration of 0s    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
    MPU6050_RA_ZRMOT_THR,    	//Zero motion threshold    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
    MPU6050_RA_ZRMOT_DUR,    	//Zero motion duration threshold    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
    MPU6050_RA_FIFO_EN,    		//Disable sensor output to FIFO buffer    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);
    MPU6050_RA_I2C_MST_CTRL,    //AUX I2C setup    //Sets AUX I2C to single master control, plus other config    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
    MPU6050_RA_I2C_SLV0_ADDR,  //Setup AUX I2C slaves    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    MPU6050_RA_I2C_SLV0_REG,   	//    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
    MPU6050_RA_I2C_SLV0_CTRL,  	//    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    MPU6050_RA_I2C_SLV1_ADDR, // LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    MPU6050_RA_I2C_SLV1_REG,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
    MPU6050_RA_I2C_SLV1_CTRL,  //LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    MPU6050_RA_I2C_SLV2_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    MPU6050_RA_I2C_SLV2_REG,    //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
    MPU6050_RA_I2C_SLV2_CTRL,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    MPU6050_RA_I2C_SLV3_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    MPU6050_RA_I2C_SLV3_REG,    //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
    MPU6050_RA_I2C_SLV3_CTRL,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    MPU6050_RA_I2C_SLV4_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    MPU6050_RA_I2C_SLV4_REG,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
    MPU6050_RA_I2C_SLV4_DO,     //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
    MPU6050_RA_I2C_SLV4_CTRL,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    MPU6050_RA_I2C_SLV4_DI,      //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);
    MPU6050_RA_INT_PIN_CFG,     //MPU6050_RA_I2C_MST_STATUS //Read-only    //Setup INT pin and AUX I2C pass through    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
    MPU6050_RA_INT_ENABLE,    //Enable data ready interrupt      LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);
    MPU6050_RA_I2C_SLV0_DO,  //Slave out, dont care    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
    MPU6050_RA_I2C_SLV1_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
    MPU6050_RA_I2C_SLV2_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
    MPU6050_RA_I2C_SLV3_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
    MPU6050_RA_I2C_MST_DELAY_CTRL, //More slave config      LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    MPU6050_RA_SIGNAL_PATH_RESET,  //Reset sensor signal paths    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    MPU6050_RA_MOT_DETECT_CTRL,     //Motion detection control    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    MPU6050_RA_USER_CTRL,                 //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
    MPU6050_RA_CONFIG,                       //Disable FSync, 256Hz DLPF    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
    MPU6050_RA_FF_THR,				   //Freefall threshold of |0mg|    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
    MPU6050_RA_FF_DUR,			       //Freefall duration limit of 0    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
    MPU6050_RA_MOT_THR,                 //Motion threshold of 0mg    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
    MPU6050_RA_MOT_DUR,			    //Motion duration of 0s    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
    MPU6050_RA_ZRMOT_THR,	    //Zero motion threshold    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
    MPU6050_RA_ZRMOT_DUR,      //Zero motion duration threshold    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
    MPU6050_RA_FIFO_EN,		    //Disable sensor output to FIFO buffer    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);
    MPU6050_RA_I2C_MST_CTRL,      //AUX I2C setup    //Sets AUX I2C to single master control, plus other config    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
    MPU6050_RA_I2C_SLV0_ADDR,    //Setup AUX I2C slaves    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    MPU6050_RA_I2C_SLV0_REG,    //LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
    MPU6050_RA_I2C_SLV0_CTRL,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    MPU6050_RA_I2C_SLV1_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    MPU6050_RA_I2C_SLV1_REG,    //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
    MPU6050_RA_I2C_SLV1_CTRL,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    MPU6050_RA_I2C_SLV2_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    MPU6050_RA_I2C_SLV2_REG,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
    MPU6050_RA_I2C_SLV2_CTRL,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    MPU6050_RA_I2C_SLV3_ADDR,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    MPU6050_RA_I2C_SLV3_REG,   //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
    MPU6050_RA_I2C_SLV3_CTRL, //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    MPU6050_RA_I2C_SLV4_ADDR, //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    MPU6050_RA_I2C_SLV4_REG,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
    MPU6050_RA_I2C_SLV4_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
    MPU6050_RA_I2C_SLV4_CTRL, //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    MPU6050_RA_I2C_SLV4_DI,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);
    MPU6050_RA_I2C_SLV0_DO,  //    //Slave out, dont care    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
    MPU6050_RA_I2C_SLV1_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
    MPU6050_RA_I2C_SLV2_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
    MPU6050_RA_I2C_SLV3_DO,  //    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
    MPU6050_RA_I2C_MST_DELAY_CTRL,  //More slave config    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    MPU6050_RA_SIGNAL_PATH_RESET,      //Reset sensor signal paths    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    MPU6050_RA_MOT_DETECT_CTRL,    //Motion detection control    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    MPU6050_RA_USER_CTRL,    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
    MPU6050_RA_INT_PIN_CFG,    //MPU6050_RA_I2C_MST_STATUS //Read-only    //Setup INT pin and AUX I2C pass through    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
    MPU6050_RA_INT_ENABLE,    //Enable data ready interrupt    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);
    MPU6050_RA_FIFO_R_W,       // LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    0xff
};   



//    //Sets sample rate to 8000/1+7 = 1000Hz
//    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);
    //Sets sample rate to 8000/1+15 = 500Hz
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 15);
    //Disable gyro self tests, scale of 500 degrees/s
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);
    //Disable accel self tests, scale of +-16g, no DHPF
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x18);

    loop=0;
   do
   {
       TheReg = MPU6050RegTable[loop++];
       if(TheReg==0xff) break;
       LDByteWriteI2C(MPU6050_ADDRESS,TheReg,0);
    }while(1);



    //Sets clock source to gyro reference w/ PLL
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
 
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address
 LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x01);
    cputs("\r\nMPU6050 Setup Complete\r\n");

}






GForceStruct  CurrentData;
GForceStruct  PeakData;
extern near volatile unsigned short Timerms;

void Get_Accel_Values(void)
{
char cval[6];
LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, cval, 6);
      di();
     CurrentData.Timer = Timerms;
      ei(); 
	CurrentData.Gx = ((cval[0]<<8)|cval[1]);
	CurrentData.Gy = ((cval[2]<<8)|cval[3]);
	CurrentData.Gz = ((cval[4]<<8)|cval[5]);
// got a problem with Z axis Zero G at -204 numeric value
//    CurrentData.Gz+=204;
   }




unsigned char GotInt_MPU6050(void)
{
  unsigned char uc_temp;

// Do we have a new data

	LDByteReadI2C(MPU6050_ADDRESS,MPU6050_RA_INT_STATUS, &uc_temp, 1);

  return ((uc_temp & 1) == 1 ? 1 : 0);
}	
