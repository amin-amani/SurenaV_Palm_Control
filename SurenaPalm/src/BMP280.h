#ifndef BMP280_H
#define BMP280_H 
#include "math.h"
//ref:https://github.com/1ukast/BMP280_STM32
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1 /**< R calibration  0xE1-0xF0 */
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA
#define BMP280_RESET_VALUE    0xB6

typedef enum 
{
	oversampling_skipped = 0b000,
	oversampling_x1 = 0b001,
	oversampling_x2 = 0b010,
	oversampling_x4 = 0b011,
	oversampling_x8 = 0b100,
	oversampling_x16 = 0b101
}Oversampling_t;
typedef enum 
{
	mode_sleep = 0x00, mode_forced = 0x01, mode_normal = 0x03
}PowerMode_t;

typedef enum 
{
	standby_time_500us = 0b000,
	standby_time_62500us = 0b001,
	standby_time_125ms = 0b010,
	standby_time_250ms = 0b011,
	standby_time_500ms = 0b100,
	standby_time_1000ms = 0b101,
	standby_time_2000ms = 0b110,
	standby_time_4000ms = 0b111
}StandbyTime_t;

typedef enum 
{
	filter_off = 0b000,
	filter_coeff_2 = 0b001,
	filter_coeff_4 = 0b010,
	filter_coeff_8 = 0b011,
	filter_coeff_16 = 0b100
}FilterSetting_t;


typedef struct
{
uint8_t (*spiSendFunction)(uint8_t);
void (*SPIChpSelectFunction)(bool value)
} BMP280_config_t;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct Measurement
	{
		float temperature ;
		float pressure ;
		float altitude ;
	} measurement;


	struct CompensationParameters
	{
		uint16_t dig_t1;
		int16_t dig_t2;
		int16_t dig_t3;
		uint16_t dig_p1;
		int16_t dig_p2;
		int16_t dig_p3;
		int16_t dig_p4;
		int16_t dig_p5;
		int16_t dig_p6;
		int16_t dig_p7;
		int16_t dig_p8;
		int16_t dig_p9;
	} compensationParameters;
static BMP280_config_t BMPConfig;
float p_reference = 0;
    	int32_t t_fine = 0;
//====================================================================================================================
void BMP280Init(BMP280_config_t config)
{
BMPConfig=config;
}
//====================================================================================================================
uint8_t BMP280ReadRegister(uint8_t registerAddress)
{
uint8_t result=0;
BMPConfig.SPIChpSelectFunction(0);
BMPConfig.spiSendFunction(registerAddress|0x80);
result = BMPConfig.spiSendFunction(0xff);
BMPConfig.SPIChpSelectFunction(1);
return result;
}

void BMP280ReadMultiRegister(uint8_t registerAddress,uint8_t*buffer,int count)
{
uint8_t result=0,i;
BMPConfig.SPIChpSelectFunction(0);
BMPConfig.spiSendFunction(registerAddress|0x80);
for(i=0;i<count;i++)
{
buffer[i] = BMPConfig.spiSendFunction(0xff);
}
BMPConfig.SPIChpSelectFunction(1);
}
//====================================================================================================================

uint8_t BMP280WriteRegister(uint8_t registerAddress,uint8_t value)
{
uint8_t result=0;
BMPConfig.SPIChpSelectFunction(0);
BMPConfig.spiSendFunction(registerAddress&0x7F);
result = BMPConfig.spiSendFunction(value);
BMPConfig.SPIChpSelectFunction(1);
return result;
}
//====================================================================================================================
void BMP280SetPressureOversampling(Oversampling_t osrs_p)
{
	uint8_t ctrl = BMP280ReadRegister(BMP280_REGISTER_CONTROL);
	ctrl = (ctrl & 0b11100011) | (osrs_p << 2);
	BMP280WriteRegister(BMP280_REGISTER_CONTROL, ctrl);
}
//====================================================================================================================
void BMP280SetTemperatureOversampling(Oversampling_t osrs_t)
{
	uint8_t ctrl = BMP280ReadRegister(BMP280_REGISTER_CONTROL);
	ctrl = (ctrl & 0b00011111) | (osrs_t << 5);
	BMP280WriteRegister(BMP280_REGISTER_CONTROL, ctrl);
}
//====================================================================================================================

void setPowerMode(PowerMode_t mode)
{
	uint8_t ctrl = BMP280ReadRegister(BMP280_REGISTER_CONTROL);
	ctrl = (ctrl & 0b11111100) | mode;
	BMP280WriteRegister(BMP280_REGISTER_CONTROL, ctrl);
}
//====================================================================================================================
void BMP280SetStandbyTime(StandbyTime_t t_sb)
{
	uint8_t conf = BMP280ReadRegister(BMP280_REGISTER_CONFIG);
	conf = (conf & 0b00011111) | (t_sb << 5);
	BMP280WriteRegister(BMP280_REGISTER_CONFIG, conf);
}
//====================================================================================================================

void BMP280SetFilterCoefficient(FilterSetting_t filter)
{
	uint8_t conf = BMP280ReadRegister(BMP280_REGISTER_CONFIG);
	conf = (conf & 0b11100011) | (filter << 2);
	BMP280WriteRegister(BMP280_REGISTER_CONFIG, conf);
}
//====================================================================================================================
void BMP280Reset()
{
	BMP280WriteRegister(BMP280_REGISTER_SOFTRESET, BMP280_RESET_VALUE);
}
//====================================================================================================================
uint8_t BMP280GetID()
{
	return BMP280ReadRegister(BMP280_REGISTER_CHIPID);
}
//====================================================================================================================
void BMP280ReadCompensationParameters()
{
	uint8_t buf[24];
	BMP280ReadMultiRegister(BMP280_REGISTER_DIG_T1, buf, 24);
	compensationParameters.dig_t1 = ((buf[1] << 8) | buf[0]);
	compensationParameters.dig_t2 = ((buf[3] << 8) | buf[2]);
	compensationParameters.dig_t3 = ((buf[5] << 8) | buf[4]);
	compensationParameters.dig_p1 = ((buf[7] << 8) | buf[6]);
	compensationParameters.dig_p2 = ((buf[9] << 8) | buf[8]);
	compensationParameters.dig_p3 = ((buf[11] << 8) | buf[10]);
	compensationParameters.dig_p4 = ((buf[13] << 8) | buf[12]);
	compensationParameters.dig_p5 = ((buf[15] << 8) | buf[14]);
	compensationParameters.dig_p6 = ((buf[17] << 8) | buf[16]);
	compensationParameters.dig_p7 = ((buf[19] << 8) | buf[18]);
	compensationParameters.dig_p8 = ((buf[21] << 8) | buf[20]);
	compensationParameters.dig_p9 = ((buf[23] << 8) | buf[22]);
}
void BMP280Measure()
{
	uint8_t data[6];
	BMP280ReadMultiRegister(BMP280_REGISTER_PRESSUREDATA, data, 6);

	int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	int32_t adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	// measurement.temperature = (float) compensate_temperature(adc_T) / 100.0;
	// measurement.pressure = (float) compensate_pressure(adc_P) / 256.0;

	if (p_reference > 0)
	{
		measurement.altitude = (1.0
				- pow(measurement.pressure / p_reference, 0.1903)) * 4433076.0;
	}
}
void BMP280SetReferencePressure(uint16_t samples, uint8_t delay)
{
	
	float sum = 0;
    char i;
	for ( i = 0; i < samples; i++)
	{
		BMP280Measure();
		sum += measurement.pressure;

	}
	p_reference = sum / samples;

}
#endif
//5859 8311 3935 2750 