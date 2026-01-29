#include "main.h"
#include "compass.h"
#include <math.h>
#include <stdint.h>

#define OFFSET -240
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
uint8_t addr = 0x0C; // 0x0C is the address of the compass 4 click

static HAL_StatusTypeDef writeSingleI2cCommand(uint8_t sensorAdr, uint8_t regAdr, uint8_t command);
static HAL_StatusTypeDef readSingleI2cCommand(uint8_t sensorAdr, uint8_t regAdr, uint8_t *received);


HAL_StatusTypeDef compass4clickInit()
{


    uint8_t regAdr = 0x32;
    HAL_StatusTypeDef status = writeSingleI2cCommand(addr, regAdr, 0x0A); // softreset the sensor
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(100);
    regAdr = 0x31;
    status = writeSingleI2cCommand(addr, regAdr, 0b01001010); // activate 200Hz continuous measurement and low noise drive
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10);
    regAdr = 0x30;
    status = writeSingleI2cCommand(addr, regAdr, 0b00101010); // enable noise supression and set watermark level to 10
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10);
    return status;
}

/**
 * @brief function to calculate the heading direction of the compass
 *
 * @param ret Heading direction in degrees
 * @return ** HAL_StatusTypeDef if !=HAL_OK there occurred an ERROR
 */
HAL_StatusTypeDef heading(int16_t *ret)
{
    int16_t deg = 0;	// Variable to hold converted data
    HAL_StatusTypeDef status;	//check errors
    uint8_t dry;   		// Data ready
    uint8_t raw;   		// Variable to read raw data
    int16_t X = 0; 		// Variable to hold raw data
    int16_t Y = 0; 		// Variable to hold raw data
    int16_t Z = 0; 		// Variable to hold raw data

    uint8_t regAdr = 0x10;
    status = readSingleI2cCommand(addr, regAdr, &dry); // read ST1
    if (status != HAL_OK)
    {
        /* If read fails, try to re-initialize the sensor once and retry the read */
        HAL_StatusTypeDef init_status = compass4clickInit();
        if (init_status != HAL_OK)
        {
            return init_status;
        }
        /* retry ST1 read once after init */
        status = readSingleI2cCommand(addr, regAdr, &dry);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    if (dry > 0x00)	//when data is ready
    {

        regAdr = 0x12; // Register address for high X data
        status = readSingleI2cCommand(addr, regAdr, &raw); 	//read reg
        if (status != HAL_OK)
        {
            return status;
        }

        X |= raw << 8;	//set the 8_high bits in front of the 16_bit data

        raw = 0;

        regAdr = 0x11; // Register address for low X data
        status = readSingleI2cCommand(addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        X |= raw;	//add the low 8_bits to the 16_bits data

        raw = 0;

        regAdr = 0x14; // Register address for high Y data
        status = readSingleI2cCommand(addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Y |= raw << 8;
        raw = 0;

        regAdr = 0x13; // Register address for low Y data
        status = readSingleI2cCommand(addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Y |= raw;
        raw = 0;

        regAdr = 0x16; // Register address for high Z data
        status = readSingleI2cCommand(addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Z |= raw << 8;
        raw = 0;

        regAdr = 0x15; // Register address for low Z data
        status = readSingleI2cCommand(addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Z |= raw;
        raw = 0;

status = readSingleI2cCommand(addr, 0x18, &raw);    //after reading reg 18 the sensor can measure new data
        if (status != HAL_OK)
        {
            return status;
        }

        if (raw & 0x08)
        {
            status = HAL_ERROR;
            return status;
        }
        raw = 0;

        X = X ^ 0xffff;								//create 2 complement data
        X = X + 0x0001;

        Y = Y ^ 0xffff;
        Y = Y + 0x0001;

        Z = Z ^ 0xffff;
        Z = Z + 0x0001;

        X = X * 0.15;								//calculate Tesla
        Y = Y * 0.15;
        Z = Z * 0.15;
        deg = (atan2(Y, X) - 0.1) * 180 / 3.1415;	//convert magnetic data to degree
        if (deg < 0)
            deg += 360;
        if (deg > 360)
            deg -= 360;
    }

    X = 0;		//Reset
    Y = 0;
    Z = 0;

    deg = (deg + OFFSET) % 360;
        if (deg < 0) {
            deg += 360;
        }

    *ret = deg;
    return status;
}

/**
 * @brief this function will read 8bit from a sensor register
 *
 * @param hi2c1 	i2C handler
 * @param sensorAdr Adress of the sensor which is spoken to
 * @param regAdr 	Adress of the Register we want to read
 * @param received  Data from the register
 * @return ** HAL_StatusTypeDef if !=HAL_OK there occurred an ERROR
 */
static HAL_StatusTypeDef readSingleI2cCommand(uint8_t sensorAdr, uint8_t regAdr, uint8_t *received)
{
    HAL_StatusTypeDef ret;
    uint8_t rawSensVal[1] = {0x00};
    uint8_t readAddress = (sensorAdr << 1) | 0x01; // R-Bit set to 1

    ret = HAL_I2C_Mem_Read(&hi2c1, readAddress, regAdr, sizeof(regAdr), rawSensVal, sizeof(rawSensVal), 1000);
    *received = rawSensVal[0];

    return ret;
}

/**
 * @brief This function will write a 8bit command in the sensor register
 *
 * @param hi2c1 	i2C handler
 * @param sensorAdr Address of the sensor which is spoken to
 * @param regAdr 	Address of the Register we want to talk
 * @param command 	Command we want to send
 * @return ** HAL_StatusTypeDef 	if !=HAL_OK there occurred an ERROR
 */
static HAL_StatusTypeDef writeSingleI2cCommand(uint8_t sensorAdr, uint8_t regAdr, uint8_t command)
{
    HAL_StatusTypeDef ret;

    uint8_t writeAddress = (sensorAdr << 1); // R/W-Bit set to 0

    uint8_t txBuf[3];

    txBuf[0] = regAdr;
    txBuf[1] = command;
    txBuf[2] = command;

    ret = HAL_I2C_Master_Transmit(&hi2c1, writeAddress, txBuf, ARRAY_SIZE(txBuf), HAL_MAX_DELAY);

    return ret;
}
