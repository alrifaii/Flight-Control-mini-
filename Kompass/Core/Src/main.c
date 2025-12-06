/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Kompass 4 click Sensor
 * @autor		   : Julian Bauer
 ******************************************************************************
 * @attention
 *
 *
 *@Preperation to run the project:
 *Pin D8 (Reset Pin from the board) is not connected to the STM
 *Plase a Jumper on Pin D8 and D12
 *
 *@optional: use sunglasses, the led toggles because of Uart & RST Pin
 *
 * ©Bauer Julian
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define OFFSET -240
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t addr = 0x0C; // 0x0C is the address of the compass 4 click
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef writeSingleI2cCommand(I2C_HandleTypeDef hi2c1, uint8_t sensorAdr, uint8_t regAdr, uint8_t command);
HAL_StatusTypeDef readSingleI2cCommand(I2C_HandleTypeDef hi2c1, uint8_t sensorAdr, uint8_t regAdr, uint8_t *received);
HAL_StatusTypeDef compass4clickInit();
HAL_StatusTypeDef heading(int16_t *deg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    HAL_StatusTypeDef status = compass4clickInit();
    if (status != HAL_OK)
    {
        printf("Compass Init ERROR\n\r");
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int16_t deg = 0;
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        status = heading(&deg);

        printf("Heading: %d\n\r", deg);
        HAL_Delay(50);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief function to initialize all the settings to work with the compass
 *
 * @return ** HAL_StatusTypeDef if !=HAL_OK there occurred an ERROR
 */
HAL_StatusTypeDef compass4clickInit()
{

    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1); 	// Resetpin needs to be high, use a jumper on D8 & D12

    uint8_t regAdr = 0x32;
    HAL_StatusTypeDef status = writeSingleI2cCommand(hi2c1, addr, regAdr, 0x0A); // softreset the sensor
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(100);
    regAdr = 0x31;
    status = writeSingleI2cCommand(hi2c1, addr, regAdr, 0b01001010); // activate 200Hz continuous measurement and low noise drive
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10);
    regAdr = 0x30;
    status = writeSingleI2cCommand(hi2c1, addr, regAdr, 0b00101010); // enable noise supression and set watermark level to 10
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
    status = readSingleI2cCommand(hi2c1, addr, regAdr, &dry); // read ST1
    if (status != HAL_OK)
    {
        printf("Read ST1 ERROR\n");
        return status;
    }

    if (dry > 0x00)	//when data is ready
    {

        regAdr = 0x12; // Register address for high X data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);	//read reg
        if (status != HAL_OK)
        {
            return status;
        }

        X |= raw << 8;	//set the 8_high bits in front of the 16_bit data

        raw = 0;

        regAdr = 0x11; // Register address for low X data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        X |= raw;	//add the low 8_bits to the 16_bits data

        raw = 0;

        regAdr = 0x14; // Register address for high Y data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Y |= raw << 8;
        raw = 0;

        regAdr = 0x13; // Register address for low Y data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Y |= raw;
        raw = 0;

        regAdr = 0x16; // Register address for high Z data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Z |= raw << 8;
        raw = 0;

        regAdr = 0x15; // Register address for low Z data
        status = readSingleI2cCommand(hi2c1, addr, regAdr, &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        Z |= raw;
        raw = 0;

        status = readSingleI2cCommand(hi2c1, addr, 0x18, &raw);	//after reading reg 18 the sensor can measure new data
        if (status != HAL_OK)
        {
            return status;
        }

        if (raw & 0x08)
        {
            printf("Magnetic sensor overflow\n");
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
HAL_StatusTypeDef readSingleI2cCommand(I2C_HandleTypeDef hi2c1, uint8_t sensorAdr, uint8_t regAdr, uint8_t *received)
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
HAL_StatusTypeDef writeSingleI2cCommand(I2C_HandleTypeDef hi2c1, uint8_t sensorAdr, uint8_t regAdr, uint8_t command)
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
