#ifndef IIC_H
#define IIC_H


void WriteSDA1(void);
void WriteSDA0(void);
void WriteSCL1(void);
void WriteSCL0(void);
void Delay_1u(unsigned int microSecs);
void i2c_start();
void ReadSDA(void);
void I2C_Stop_1(void);
void SEND_0_1(void);
void SEND_1_1(void);
char Check_Acknowledge_1(void);
void Write_Acknowledge_1(char stat);
void WriteI2CByte_1(char b);
char ReadI2CByte_1(unsigned int ack);


#endif