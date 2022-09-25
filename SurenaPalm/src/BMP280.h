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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct
{
uint8_t (*spiSendFunction)(uint8_t);
void (*SPIChpSelectFunction)(bool value)
} BMP280_config_t;

static BMP280_config_t BMPConfig;
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
BMPConfig.spiSendFunction(registerAddress);
result = BMPConfig.spiSendFunction(0xff);
BMPConfig.SPIChpSelectFunction(1);
return result;
}