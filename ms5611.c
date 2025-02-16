// author: dogu
#include <ms5611.h>
#include "math.h"

#include "main.h"

// extern I2C_HandleTypeDef hi2c2;
// #define ms5611_i2c &hi2c2

I2C_HandleTypeDef *ms5611_i2c;
static uint8_t ms5611_addr;

static uint16_t prom[6];

// min OSR by default
static uint8_t pressAddr = PRESSURE_OSR_1024;
static uint8_t tempAddr = TEMP_OSR_1024;
static uint8_t convDelay = CONVERSION_OSR_1024;

static int32_t temperature;
static int32_t pressure;
static float altitude_m;

void (*ms5611_delay)(int ms);

void ms5611_config(I2C_HandleTypeDef *i2c,uint8_t addr, uint8_t osr, void(*delay_func)(int ms)){

	ms5611_i2c = i2c;
	ms5611_addr = addr;
	ms5611_setOSR(osr);
	(ms5611_delay) = (delay_func);
}

void ms5611_i2c_write_byte(uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(ms5611_i2c, ms5611_addr << 1, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void ms5611_i2c_read_byte(uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(ms5611_i2c, ms5611_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                     data, 1, 100);
}

uint16_t ms5611_i2c_read_word(uint8_t reg) {
    uint8_t data[2];
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(ms5611_i2c, ms5611_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                     data, 2, 100);
    return (data[0] << 8) | data[1];
}

uint32_t ms5611_i2c_read_24bits(uint8_t reg) {
    uint8_t data[3];
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(ms5611_i2c, ms5611_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                     data, 3, 100);
    return (data[0] << 16) | (data[1] << 8) | data[2];
}

//void ms5611_delay(uint32_t time) {
//// HAL_Delay or osDelay
//#ifdef USE_OS_DELAY
//    HAL_Delay(time);
//#endif
//}

int ms5611_init() {
    ms5611_i2c_write_byte(CMD_RESET, CMD_RESET);
    ms5611_delay(3);


    // prom[0] = ms5611_read16bits(CMD_PROM_C1);
    // prom[1] = ms5611_read16bits(CMD_PROM_C2);
    // prom[2] = ms5611_read16bits(CMD_PROM_C3);
    // prom[3] = ms5611_read16bits(CMD_PROM_C4);
    // prom[4] = ms5611_read16bits(CMD_PROM_C5);
    // prom[5] = ms5611_read16bits(CMD_PROM_C6);

    for (int i = 0; i < 6; i++) {
        prom[i] = ms5611_i2c_read_word(CMD_PROM_C1 + i * 2);
    }


    return 1;
}

uint32_t ms5611_read_rawTemp() {
    ms5611_i2c_write_byte(tempAddr, tempAddr);
    ms5611_delay(convDelay);
    return ms5611_i2c_read_24bits(0x00);
}

uint32_t ms5611_read_rawPress() {
    ms5611_i2c_write_byte(pressAddr, pressAddr);
    ms5611_delay(convDelay);
    return ms5611_i2c_read_24bits(0x00);
}

void ms5611_calculate() {
    int32_t dT;
    int64_t TEMP, OFF, SENS, P;
    uint32_t D1, D2;
    float press, r, c;

    // D1 = ms5611_readRawPressure();
    // D2 = ms5611_readRawTemp();

    D1 = ms5611_read_rawPress();
    D2 = ms5611_read_rawTemp();


    dT = D2 - ((long)prom[4] * 256);
    TEMP = 2000 + ((int64_t)dT * prom[5]) / 8388608;
    OFF = (int64_t)prom[1] * 65536 + ((int64_t)prom[3] * dT) / 128;
    SENS = (int64_t)prom[0] * 32768 + ((int64_t)prom[2] * dT) / 256;

    if (TEMP < 2000) {  // second order temperature compensation
        int64_t T2 = (((int64_t)dT) * dT) >> 31;
        int64_t Aux_64 = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = (5 * Aux_64) >> 1;
        int64_t SENS2 = (5 * Aux_64) >> 2;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P = (D1 * SENS / 2097152 - OFF) / 32768;
    temperature = TEMP;
    pressure = P;

    press = (float)pressure;
    r = press / 101325.0;
    c = 1.0 / 5.255;
    altitude_m = (1 - pow(r, c)) * 44330.77;
}

void ms5611_getTemperatureAndPressure(float *temp, float *press,
                                      float *alt) {
    ms5611_calculate();
    // *temperature = (float)temperature / 100.0;
    // *pressure = (float)pressure / 100.0;
    *temp = temperature / 100.0;
    *press = pressure / 100.0;
    *alt = altitude_m;
}

void ms5611_setOSR(OSR osr)
{
	switch(osr)
	{
		default:
		case OSR_256:
			pressAddr = PRESSURE_OSR_256;
			tempAddr = TEMP_OSR_256;
			convDelay = CONVERSION_OSR_256;
			break;
		case OSR_512:
			pressAddr = PRESSURE_OSR_512;
			tempAddr = TEMP_OSR_512;
			convDelay = CONVERSION_OSR_512;
			break;
		case OSR_1024:
			pressAddr = PRESSURE_OSR_1024;
			tempAddr = TEMP_OSR_1024;
			convDelay = CONVERSION_OSR_1024;
			break;
		case OSR_2048:
			pressAddr = PRESSURE_OSR_2048;
			tempAddr = TEMP_OSR_2048;
			convDelay = CONVERSION_OSR_2048;
			break;
		case OSR_4096:
			pressAddr = PRESSURE_OSR_4096;
			tempAddr = TEMP_OSR_4096;
			convDelay = CONVERSION_OSR_4096;
			break;
	}
}
