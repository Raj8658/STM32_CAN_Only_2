/* main.c - Transmitter (STM32F407VG Discovery) */
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t txData[1] = {0};

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN2_Init(void);
void Error_Handler(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN2_Init();

    // CAN Filter Configuration (Allow all messages)
    CAN_FilterTypeDef can_filter = {
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14
    };
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);

    HAL_CAN_Start(&hcan2);

    // Configure CAN message header
    TxHeader.DLC = 1;           // Data length = 1 byte
    TxHeader.IDE = CAN_ID_STD;  // Standard ID
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x103;     // CAN ID = 0x103

    while (1) {
        // Read button with debouncing
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Button pressed (PA0 = LOW)
            HAL_Delay(50);  // Debounce delay
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Still pressed
                txData[0] = 1;
            }
        } else {
            txData[0] = 0;
        }

        // Send CAN message
        if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, txData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }
        HAL_Delay(50);  // Throttle message rate
    }
}

void MX_CAN2_Init(void) {
    // Enable CAN clocks
    __HAL_RCC_CAN1_CLK_ENABLE();  // CAN2 requires CAN1 clock
    __HAL_RCC_CAN2_CLK_ENABLE();

    // Configure CAN peripheral
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 6;      // APB1 = 42 MHz → 42 MHz / (6 * (8 + 3 + 1)) = 583 kbps
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_8TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = DISABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_GPIO_Init(void) {
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PA0 (Button) as input with pull-up
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PB12 (CAN2_RX) and PB13 (CAN2_TX) as alternate function
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure power supply
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Configure HSE and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;     // HSE = 8 MHz → PLLM divides to 1 MHz
    RCC_OscInitStruct.PLL.PLLN = 336;   // PLLN multiplies to 336 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // PLLP divides to 168 MHz (SYSCLK)
    RCC_OscInitStruct.PLL.PLLQ = 7;     // PLLQ for peripheral clocks
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Configure clock dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;    // HCLK = 168 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;     // APB1 = 42 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;     // APB2 = 84 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
        // Blink LED or indicate error here
    }
}
