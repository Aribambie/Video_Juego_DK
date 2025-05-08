/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "Bitmap.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//Sprites para el fondo y las escaleras
extern uint8_t fondo[];
extern uint8_t E1[];
extern uint8_t E2[];
extern uint8_t E3[];
extern uint8_t E4[];
extern uint8_t E5[];
extern uint8_t E6[];

/* USER CODE BEGIN PV */
extern uint8_t menu[]; //Array declarado en Bitmap.h
volatile uint8_t gameStarted = 0; // 0 = en menú, 1 = juego activo
uint16_t TOP = 1; // Variable TOP inicializada en 1

// Variables para UART y animación
volatile uint8_t rxData = 0;
volatile char uartCmd   = 0;


uint8_t   spriteVar     = 0;       // va de 0 a 2
uint16_t  spriteX       =  82;    // posición X inicial
const uint16_t INITIAL_SPRITE_Y = 204;  // Posición Y inicial
uint16_t currentSpriteY = INITIAL_SPRITE_Y;  // Posición Y actual

uint8_t  dkFrame     = 1;           // va de 1 a 5
uint32_t lastDkTick  = 0;
const uint32_t dkInterval = 700;    // ms entre frames de DK


// Variables para el salto
uint8_t isJumping = 0;          // 0 = no salta, 1 = en salto
uint8_t jumpFrame = 0;          // Frame actual del salto (0-24)
const uint8_t jumpHeight = 12;  // Altura máxima del salto en píxeles
const uint8_t jumpLength = 12;  // Distancia horizontal del salto
uint16_t preJumpX, preJumpY;    // Posición antes del salto
uint8_t preJumpDirection;       // Dirección antes del salto
uint8_t preJumpAnimFrame;       // Frame de animación antes del salto

volatile char lastMoveCmd = 'R'; // Dirección anterior para salto



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Función para restaurar el fondo en la posición anterior
void RestoreMarioBackground(uint16_t x, uint16_t y, uint8_t direction) {
    // Dimensiones del sprite de Mario (18x18 píxeles)
    const uint16_t spriteW = 18;
    const uint16_t spriteH = 18;
    // Ancho de la pantalla (320 píxeles en modo horizontal)
    const uint16_t screenW = 320;

    // Validación: Si las coordenadas (x,y) están fuera de los límites de la pantalla, salir
    if (x + spriteW > screenW || y + spriteH > 240) return;

    // Buffer temporal para almacenar los datos del fondo (18x18 píxeles, 2 bytes por píxel)
    uint8_t buffer[spriteW * spriteH * 2];

    // Copiar la región del fondo original (donde se dibujará Mario)
    for (uint16_t row = 0; row < spriteH; row++) {
        // Calcular posición de inicio en el array 'fondo' (imagen completa del fondo)
        uint32_t srcOffset = ((y + row) * screenW + x) * 2;
        // Copiar una línea horizontal de 18 píxeles (36 bytes) del fondo al buffer
        memcpy(&buffer[row * spriteW * 2], &fondo[srcOffset], spriteW * 2);
    }

    // Dibujar el fragmento del fondo en la pantalla LCD (sobreescribiendo a Mario)
    LCD_Bitmap(x, y, spriteW, spriteH, buffer);
}

void ShowMenu() {
    LCD_Clear(0x0000); // Limpia la pantalla
    LCD_Bitmap(0, 0, 320, 240, menu); // Muestra el menú

   }

/* USER CODE BEGIN 0 */
void HandleJump() {
    if (!isJumping) return;

    // Calcular posición vertical (forma de parábola)
    uint8_t verticalOffset = 0;
    if (jumpFrame <= jumpHeight) {
        verticalOffset = jumpFrame; // Fase de subida
    } else {
        verticalOffset = 2*jumpHeight - jumpFrame; // Fase de bajada
    }

    // Restaurar fondo en la posición anterior
    uint16_t prevY = currentSpriteY - ((jumpFrame <= jumpHeight) ? jumpFrame : 2 * jumpHeight - jumpFrame);
    uint16_t restoreX = spriteX;
    uint16_t restoreY = prevY;
    const uint16_t spriteW = 18;
    const uint16_t spriteH = 18;

    const uint16_t screenWidth = 320;
    uint32_t offset = (restoreY * screenWidth + restoreX) * 2;
    LCD_Bitmap(restoreX, restoreY, spriteW, spriteH, fondo + offset);


    // Mover horizontalmente
    if (preJumpDirection == 0 && spriteX < 262) { // Derecha
        spriteX++;
    } else if (preJumpDirection == 1 && spriteX > 81) { // Izquierda
        spriteX--;
    }

    // Dibujar sprite de salto en nueva posición
    LCD_Sprite(spriteX, currentSpriteY - verticalOffset, 18, 18, jump, 2, 0, preJumpDirection, 0);

    // Actualizar frame del salto
    jumpFrame++;

    // Finalizar salto
    if (jumpFrame >= 2*jumpHeight) {
        isJumping = 0;
        // Restaurar sprite de caminata normal
        RestoreMarioBackground(spriteX, currentSpriteY - verticalOffset, preJumpDirection);
        LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, preJumpAnimFrame, preJumpDirection, 0);
    }
}
/* USER CODE END 0 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rxData, 1);
  LCD_Init();
  ShowMenu(); // Mostrar menú al inicio

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1) {
  	    uint32_t now = HAL_GetTick();

  	    // Si estamos en el menú
  	    if (!gameStarted) {
  	        if (uartCmd == 'A') { // Iniciar juego
  	            gameStarted = 1;
  	            uartCmd = 0;

  	            // Inicializar juego
  	            LCD_Clear(0x00);
  	            LCD_Bitmap(0, 0, 320, 240, fondo);
  	            FillRect(64, 37, 48, 32, 0x0000);

  	            // Dibujar elementos del juego
  	            LCD_Bitmap(231, 191, 10, 36, E1);
  	            LCD_Bitmap(79, 161, 10, 35, E2);
  	            LCD_Bitmap(159, 128, 10, 41, E3);
  	            LCD_Bitmap(119, 99, 10, 38, E4);
  	            LCD_Bitmap(231, 72, 10, 33, E5);
  	            LCD_Bitmap(175, 44, 10, 36, E6);

  	            // Resetear posición del personaje
  	            spriteX = 82;
  	            currentSpriteY = INITIAL_SPRITE_Y;
  	            spriteVar = 0;

  	            // Dibujar personaje inicial
  	            LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, spriteVar, 0, 0);
  	            lastDkTick = HAL_GetTick();
  	        }
  	        else if (uartCmd == 'B') { // Resetear TOP
  	            TOP = 0;
  	            uartCmd = 0;
  	            // Actualizar visualización del TOP
  	            char topText[20];
  	            sprintf(topText, "TOP: %d", TOP);
  	            LCD_Print(topText, 100, 200, 1, 0xFFFF, 0x0000);
  	        }
  	    }
  	    else { // Si el juego está activo
  	      HandleJump();

  	            // Solo procesar movimiento si no está saltando
  	            if (!isJumping) {
  	                if (uartCmd == 'R') {
  	                    // Movimiento a derecha
  	                    RestoreMarioBackground(spriteX, currentSpriteY, 0);
  	                    if(spriteX < 262) spriteX++;

  	                    LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, spriteVar, 0, 0);
  	                    spriteVar = (spriteVar + 1) % 3;
  	                    uartCmd = 0;
  	                }
  	                else if (uartCmd == 'L') {
  	                    //Movimiento a izquierda
  	                    RestoreMarioBackground(spriteX, currentSpriteY, 1);
  	                    if(spriteX > 81) spriteX--;

  	                    LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, spriteVar, 1, 0);
  	                    spriteVar = (spriteVar + 1) % 3;
  	                    uartCmd = 0;
  	                }
  	            }

  	        // Actualizar DK si ya pasó el intervalo
  	        if (now - lastDkTick >= dkInterval) {
  	            lastDkTick = now;
  	            dkFrame++;
  	            if (dkFrame > 5) dkFrame = 1;
  	            LCD_Sprite(64, 37, 48, 32, DK, 6, dkFrame, 0, 0);
  	        }

  	        // Manejar movimiento del personaje
  	        if (uartCmd == 'R') { // Movimiento a derecha
  	            // 1. Regenerar fondo en el lado izquierdo de la posición anterior
  	            RestoreMarioBackground(spriteX, currentSpriteY, 0);

  	            // 2. Actualizar posición
  	            if(spriteX < 262) spriteX++;

  	            // 3. Lógica de escalones
  	            if(spriteX == 160) currentSpriteY = 203;
  	            else if(spriteX == 185) currentSpriteY = 202;
  	            else if(spriteX == 208) currentSpriteY = 201;
  	            else if(spriteX == 232) {currentSpriteY = 200;
  	            //LCD_Bitmap(231, 191, 10, 36, E1);
  	            }
  	            else if(spriteX == 256) currentSpriteY = 199;


  	            // 4. Dibujar nuevo frame
  	            LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, spriteVar, 0, 0);
  	            spriteVar = (spriteVar + 1) % 3;

  	            uartCmd = 0;
  	        }
  	        else if (uartCmd == 'L') { // Movimiento a izquierda
  	            // 1. Regenerar fondo en el lado derecho de la posición anterior
  	            RestoreMarioBackground(spriteX, currentSpriteY, 1);

  	            // 2. Actualizar posición
  	            if(spriteX > 81) spriteX--;

  	            // 3. Lógica de escalones
  	            if(spriteX == 159) currentSpriteY = 204;
  	            else if(spriteX == 184) currentSpriteY = 203;

  	            // 4. Dibujar nuevo frame
  	            LCD_Sprite(spriteX, currentSpriteY, 17, 16, walk, 3, spriteVar, 1, 0);
  	            spriteVar = (spriteVar + 1) % 3;

  	            uartCmd = 0;
  	        }

  	        if (now - lastFireTick >= fireInterval) {
  	            lastFireTick = now;
  	            fireframe = (fireframe + 1) % 2; // Ajusta según tus frames de fuego
  	            // Actualizar animación de fuego aquí
  	        }
  	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin SD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (!gameStarted && (rxData == 'A' || rxData == 'B')) {
            uartCmd = rxData;
        }
        if (gameStarted) {
            uartCmd = rxData;

            // Actualizar dirección si es movimiento
            if (uartCmd == 'R' || uartCmd == 'L') {
                lastMoveCmd = uartCmd;
            }

            // Salto
            if (uartCmd == 'U' && !isJumping) {
                preJumpX = spriteX;
                preJumpY = currentSpriteY;
                preJumpDirection = (lastMoveCmd == 'R') ? 0 : 1;
                preJumpAnimFrame = spriteVar;

                isJumping = 1;
                jumpFrame = 0;

                RestoreMarioBackground(spriteX, currentSpriteY, preJumpDirection);
            }
        }

        HAL_UART_Receive_IT(&huart2, &rxData, 1);
    }
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
	while (1) {
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
