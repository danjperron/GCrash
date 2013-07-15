
#define I2C_READ	0x01		/* read bit used with address */
#define I2C_WRITE	0x00		/* write bit used with address */


// info extract from
// http://www.hobbytronics.co.uk/hi-tech-c-i2c-master

// Initialise MSSP port. (12F1822 - other devices may differ)
void i2c_Init(void);

// i2c_Wait - wait for I2C transfer to finish
void i2c_Wait(void);

// i2c_Start - Start I2C communication
void i2c_Start(void);

// i2c_Restart - Re-Start I2C communication
void i2c_Restart(void);


// i2c_Stop - Stop I2C communication
void i2c_Stop(void);

// i2c_Write - Sends one byte of data
void i2c_Write(unsigned char data);

// i2c_Address - Sends Slave Address and Read/Write mode
// mode is either I2C_WRITE or I2C_READ
void i2c_Address(unsigned char address, unsigned char mode);

// i2c_Read - Reads a byte from Slave device
unsigned char i2c_Read(unsigned char ack);


// Main Function
void LDByteWriteI2C(unsigned char i2cSlaveAddress, unsigned char LowAdd, unsigned char data);
void LDByteReadI2C(unsigned char i2cSlaveAddress, unsigned char Address, unsigned char *Data, unsigned char Length);

 