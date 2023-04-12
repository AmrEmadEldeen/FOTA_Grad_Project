/* USER CODE BEGIN Header */
/**
 **********
 * @file           : main.c
 * @brief          : Main program body
 **********
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **********
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

uint8_t supported_commands[5] = {
		BL_GET_VER,
		BL_GET_HELP,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WRITE_DATA,
};

uint32_t SIZE = 8;
uint32_t ADDRESS = 0x08020000;

uint32_t CommandCounter = 0 ;
uint32_t RxCounter = 0 ;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t num = 0 ;
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
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */

	HAL_CAN_Start(&hcan1);

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	TxHeader.DLC = 1;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x99;
	TxHeader.TransmitGlobalTime = DISABLE;

	//   HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0)
		{
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

			bootloader_can_read_data();

		}

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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 18;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x103;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;


	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/****************************************************************************************
 ********************* implementation of bootloader command handle functions *************
 *****************************************************************************************/

/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
static void bootloader_handle_gethelp_cmd(void)
{
	TxHeader.StdId = BL_GET_HELP;

	for(uint32_t i = 0 ; i < 5 ; i++)
	{
		TxData[i] = supported_commands[i];
	}

	//send the supported commands to the node_mcu
	bootloader_can_write_data(1);
}

/*Helper function to handle BL_GET_VER command */
static void bootloader_handle_getver_cmd(void)
{
	uint8_t app_version;

	app_version = get_app_version();

	TxHeader.StdId = BL_GET_VER;

	TxData[0] = app_version;

	bootloader_can_write_data(1);
}

/*Helper function to handle BL_GO_TO_ADDR command */
static void bootloader_handle_go_cmd(void)
{
	TxHeader.StdId = BL_GO_TO_ADDR;

	uint8_t VERIFICATION_ADDRESS = (uint8_t) ADDR_INVALID;

	uint32_t*ptr_address = &RxData;

	uint32_t address = *(ptr_address);

	VERIFICATION_ADDRESS = verify_address(address);

	if( VERIFICATION_ADDRESS == (uint8_t)ADDR_VALID )
	{
		TxData[0] = (uint8_t)VERIFICATION_ADDRESS;

		//tell node_mcu that address is fine
		bootloader_can_write_data(1);

		/*jump to "go" address.
        we dont care what is being done there.
        host must ensure that valid code is present over there
        Its not the duty of bootloader. so just trust and jump */

		/* Not doing the below line will result in hardfault exception for ARM cortex M */
		//watch : https://www.youtube.com/watch?v=VX_12SjnNhY

		uint32_t go_address = address;

		go_address+=1; //make T bit =1

		void (*lets_jump)(void) = (void *)go_address;

		lets_jump();

	}
	else
	{
		TxData[0] = (uint8_t)VERIFICATION_ADDRESS;

		//tell host that address is invalid
		bootloader_can_write_data(1);
	}
}

/*Helper function to handle BL_FLASH_ERASE command */
static void bootloader_handle_flash_erase_cmd(void)
{
	uint8_t ERASE_STATUS = (uint8_t)FLASH_ERASE_FAILED;

	uint32_t*ptr = &RxData;
	uint32_t initial_sector = *(ptr);
	uint32_t number_of_sectors = *(ptr+1);

	ERASE_STATUS = execute_flash_erase(initial_sector,number_of_sectors);

	TxHeader.StdId = BL_FLASH_ERASE;

	if(ERASE_STATUS == (uint8_t)FLASH_ERASE_SUCCESS)
	{
		TxData[0] = (uint8_t)FLASH_ERASE_SUCCESS;
	}
	else if(ERASE_STATUS == (uint8_t)FLASH_ERASE_FAILED)
	{
		TxData[0] = (uint8_t)FLASH_ERASE_FAILED;
	}
	else
	{
		TxData[0] = (uint8_t)INVALID_SECTOR;
	}

	bootloader_can_write_data(1);
}

static void bootloader_handle_mem_write_size_cmd(void)
{
	uint32_t*ptr_size = &RxData;

	SIZE = *(ptr_size);
}

static void bootloader_handle_mem_write_address_cmd(void)
{
	uint32_t*ptr_address = &RxData;

	ADDRESS = *(ptr_address);
}

/*Helper function to handle BL_MEM_WRITE_DATA command */
static void bootloader_handle_mem_write_data_cmd(void)
{
	uint8_t WRITE_STATUS = (uint8_t)FLASH_WRITE_FAILED;

	uint8_t VERIFICATION_ADDRESS = (uint8_t)ADDR_INVALID;

	TxHeader.StdId = BL_MEM_WRITE_DATA;

	uint32_t payload_len = (SIZE/4);

	uint32_t mem_address = ADDRESS;

	VERIFICATION_ADDRESS = verify_address(mem_address);

	if( VERIFICATION_ADDRESS == (uint8_t)ADDR_VALID )
	{
		//execute mem write
		WRITE_STATUS = execute_mem_write(mem_address, payload_len);

		if(WRITE_STATUS == (uint8_t)FLASH_WRITE_SUCCESS)
		{
			TxData[0] = (uint8_t)FLASH_WRITE_SUCCESS;
		}
		else
		{
			TxData[0] = (uint8_t)FLASH_WRITE_FAILED;
		}
	}
	else
	{
		TxData[0] = (uint8_t)ADDR_INVALID;
	}

	//inform host about the status
	bootloader_can_write_data(1);
}


/* USER CODE BEGIN 4 */
/* This function read the data from can1 */
void bootloader_can_read_data(void)
{
	switch(RxHeader.StdId)
	{
	case BL_GET_HELP:
		bootloader_handle_gethelp_cmd();
		break;
	case BL_GET_VER:
		bootloader_handle_getver_cmd();
		break;
	case BL_FLASH_ERASE:
		bootloader_handle_flash_erase_cmd();
		break;
	case BL_MEM_WRITE_SIZE:
		bootloader_handle_mem_write_size_cmd();
		break;
	case BL_MEM_WRITE_ADDRESS:
		bootloader_handle_mem_write_address_cmd();
		break;
	case BL_MEM_WRITE_DATA:
		bootloader_handle_mem_write_data_cmd();
		break;
	case BL_GO_TO_ADDR:
		bootloader_handle_go_cmd();
		break;
	case FIRMWARE_OVER_THE_AIR:
		UpdateAPP();
		break;
	}
}

/* This function write data into can1 */
void bootloader_can_write_data(uint32_t length_of_data)
{
	for(uint32_t i = 0 ; i < length_of_data ; i++)
	{
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData[i], &TxMailbox);
	}

	memset(TxData,0,8);
}

/*Just returns the macro value */
uint8_t get_app_version(void)
{
	return (uint8_t) APP_VERSION;
}

//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{
	/*so the valid addresses which we can jump to are
	 external memory & SRAM & flash memory */

	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return (uint8_t) ADDR_VALID;
	}
	else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return (uint8_t) ADDR_VALID;
	}
	else if ( go_address >= SRAM3_BASE && go_address <= SRAM3_END)
	{
		return (uint8_t) ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return (uint8_t) ADDR_VALID;
	}
	else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return (uint8_t) ADDR_VALID;
	}
	else
		return (uint8_t) ADDR_INVALID;
}

uint8_t execute_flash_erase(uint32_t initial_sector_number , uint32_t number_of_sector)
{
	//we have totally 12 sectors in one bank .. sector[0 to 11]
	//number_of_sector has to be in the range of 0 to 11
	// if sector_number = 0xff , that means mass erase !

	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError = 0;
	uint8_t erase_status = 0x01;

	if(number_of_sector > 12)
		return (uint8_t)INVALID_SECTOR;

	if( (initial_sector_number == 0xFFFFFFFF ) || (number_of_sector <= 12) )
	{
		if(number_of_sector == (uint32_t) 0xFFFFFFFF)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			/*Here we are just calculating how many sectors needs to erased */
			uint32_t remanining_sector = 12 - number_of_sector;
			if( number_of_sector > remanining_sector)
			{
				number_of_sector = remanining_sector;
			}
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = initial_sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our MCU will work on this voltage range
		erase_status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return (uint8_t)erase_status;
	}

	return (uint8_t)INVALID_SECTOR;
}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint32_t mem_address, uint32_t len)
{
	HAL_StatusTypeDef write_status = HAL_ERROR;

	uint32_t*ptr_data = &RxData;

	//We have to unlock flash module to get control of registers
	HAL_FLASH_Unlock();

	for(uint32_t i = 0 ; i < len ; i = i + 2)
	{
		//Here we program the flash byte by byte
		write_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,mem_address+(i*4),ptr_data[0]);

		write_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,mem_address+(i*4)+4,ptr_data[1]);

		ADDRESS = ADDRESS + 8;
	}

	HAL_FLASH_Lock();

	return (uint8_t)write_status;
}

uint8_t UpdateAPP(void)
{
	HAL_StatusTypeDef erase_status = HAL_ERROR;
	HAL_StatusTypeDef write_status = HAL_ERROR;

	uint32_t size = RxData[0];
	uint32_t*ptr = (uint32_t)0x08020000;
	erase_status = execute_flash_erase(2,8);
	if(erase_status == FLASH_ERASE_SUCCESS)
	{
		write_status = execute_mem_write(ptr,size);
	}
	else
	{
		return (uint8_t)erase_status;
	}

	return (uint8_t)write_status;
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

#ifdef  USE_FULL_ASSERT
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
