#ifndef MS5611_H
#define MS5611_H

#define F_CPU 4000000UL // 4 MHz external XTAL 
#define SCL_CLOCK 100000L // I2C clock in Hz 
#define ADDR_W 0xEE // Module address write mode 
#define ADDR_R 0xEF // Module address read mode 
#define CMD_RESET 0x1E // ADC reset command 
#define CMD_ADC_READ 0x00 // ADC read command 
#define CMD_ADC_CONV 0x40 // ADC conversion command 
#define CMD_ADC_D1 0x48  // ADC D1 conversion 
#define CMD_ADC_D2 0x58 // ADC D2 conversion 
#define CMD_ADC_256 0x00 // ADC OSR=256 
#define CMD_ADC_512 0x02 // ADC OSR=512 
#define CMD_ADC_1024 0x04 // ADC OSR=1024 
#define CMD_ADC_2048 0x06 // ADC OSR=2048 
#define CMD_ADC_4096 0x08 // ADC OSR=4096 
#define CMD_PROM_RD 0xA0 // Prom read command


void ms5611_Reset(void);
void MS5611_init(void);
void get_ms5611_Temperature(void);
void get_ms5611_Pressure(void);
uint32 MS561101BA_DO_CONVERSION(uint8 command);
void MS561101BA_GetTemperature(uint8 OSR_Temp);
void get_ms5611_data(void);

extern float Temperature;
extern float Pressure; 
#endif