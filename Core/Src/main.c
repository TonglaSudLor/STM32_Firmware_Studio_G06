/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "motor_controller.h"
#include "modbus_bridge.h"
#include "telemetry_hub.h"
#include "hw_io.h"
#include "kalman.h"
#include "params.h"
#include "config.h"
#include "current_sensor.h"
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
volatile uint8_t rx_byte;
volatile char rx_packet[4];
volatile int rx_ptr = 0;
volatile char rx_debug_log[16]; // Circular log to see command history in Live Expressions
volatile uint8_t debug_idx = 0;
volatile uint8_t modbus_rx_byte;

/* --- USART3 (ESP32) non-blocking TX queue (bug 0-C) -----------------------
 * Both the RX-complete callback and Motor_SendAudioCommand run in ISR context
 * (USART3 IRQ and the 100 Hz TIM6 control loop). A blocking HAL_UART_Transmit
 * there stalls the ISR for milliseconds, delaying the Receive_IT re-arm →
 * RX overrun → error-callback fault loop. Bytes are enqueued here; the main
 * loop drains the queue at thread level where blocking is safe. */
#define U3_TXQ_SIZE 64
static volatile uint8_t  u3_txq[U3_TXQ_SIZE];
static volatile uint16_t u3_txq_head = 0;   /* producer index */
static volatile uint16_t u3_txq_tail = 0;   /* consumer index */

/* Guard flag for Bug 1-D: set true while LPUART1 is being torn down and
 * re-initialised during a baud-rate switch.  _write() checks this flag so
 * that any printf that fires during the reconfiguration window returns
 * immediately instead of writing to a half-torn-down peripheral. */
volatile bool lpuart_reconfiguring = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ISR-safe byte enqueue for USART3 TX (bug 0-C). Drops bytes if full — echo
 * and audio commands are non-critical. PRIMASK guards the head update so a
 * non-ISR caller (e.g. homing audio) cannot be preempted mid-enqueue. */
void USART3_QueueTx(const uint8_t *data, uint16_t len) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((u3_txq_head + 1u) % U3_TXQ_SIZE);
        if (next == u3_txq_tail) break;       /* full — drop remainder */
        u3_txq[u3_txq_head] = data[i];
        u3_txq_head = next;
    }
    __set_PRIMASK(primask);
}

/* Drain the USART3 TX queue from the main loop (thread context only). */
static void USART3_DrainTx(void) {
    while (u3_txq_tail != u3_txq_head) {
        uint8_t b = u3_txq[u3_txq_tail];
        if (HAL_UART_Transmit(&huart3, &b, 1, 5) != HAL_OK) break;
        u3_txq_tail = (uint16_t)((u3_txq_tail + 1u) % U3_TXQ_SIZE);
    }
}

/**
 * @brief Reconfigure LPUART1 between Dashboard (115200 8N1) and Modbus (230400 8E1).
 * @param modbus_mode  true=Modbus/Base System, false=Dashboard/Joystick
 */
void LPUART1_SetMode(bool modbus_mode) {
	/* Block _write() while the peripheral is being reconfigured (Bug 1-D). */
	lpuart_reconfiguring = true;

	/* Abort any ongoing Receive_IT — do NOT DeInit.
	 * HAL_UART_DeInit calls HAL_UART_MspDeInit which disables the LPUART1
	 * RCC clock and reconfigures the GPIO pins to analog.  The subsequent
	 * HAL_UART_MspInit then calls HAL_RCCEx_PeriphCLKConfig which can time
	 * out or return HAL_ERROR on a live re-init → Error_Handler → infinite
	 * loop → IWDG reset.  Calling HAL_UART_Init on a non-RESET-state handle
	 * skips MspInit entirely and just reconfigures the peripheral registers
	 * in-place, which is all we need for a baud-rate change. */
	HAL_UART_Abort(&hlpuart1);
	HAL_Delay(5);   /* let any in-flight TX byte finish (~44 µs at 115200) */

	if (modbus_mode) {
		hlpuart1.Init.BaudRate = 230400;
		hlpuart1.Init.WordLength = UART_WORDLENGTH_9B;
		hlpuart1.Init.Parity = UART_PARITY_EVEN;
	} else {
		hlpuart1.Init.BaudRate = 115200;
		hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
		hlpuart1.Init.Parity = UART_PARITY_NONE;
	}
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
		Error_Handler();
	HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8);
	HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8);
	HAL_UARTEx_DisableFifoMode(&hlpuart1);

	lpuart_reconfiguring = false;
	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &modbus_rx_byte, 1);
}

/* Deferred toggle flag — set from ISR/anywhere, processed in main loop only.
 * Mode_Toggle() uses HAL_Delay + HAL_UART_DeInit which can't run safely in ISR
 * (HAL_Delay depends on SysTick, which has lower priority than TIM6/UART ISRs). */
volatile bool mode_toggle_request = false;

/**
 * @brief Request a mode toggle from anywhere (ISR-safe).
 *        Actual toggle happens in main loop.
 */
void Mode_Toggle(void) {
	mode_toggle_request = true;
}

/**
 * @brief Perform the actual mode switch. ONLY call from main loop.
 *        Toggles control_system_mode and reconfigures LPUART1.
 */
static void Mode_Toggle_Impl(void) {
	extern volatile Control_SystemMode_t control_system_mode;
	extern volatile bool emergency_stop;
	extern volatile bool startup_estop_pending;
	hw.mode_toggle_fired++;   /* visible in Live Expressions — confirms this function ran */

	if (control_system_mode == CONTROL_MODE_JOYSTICK) {
		control_system_mode = CONTROL_MODE_BASE_SYSTEM;
		LPUART1_SetMode(true);   /* switch to 230400 8E1 for Modbus */
		Motor_SendAudioCommand('S');
		printf("[DBG] Mode -> BASE_SYSTEM (fired=%lu)\r\n", (unsigned long)hw.mode_toggle_fired);

		FAULT_CLR(FAULT_STARTUP_ESTOP | FAULT_JOYSTICK_LOST);
		startup_estop_pending = false;
		if (!hw.in_estop && fault_code == FAULT_NONE) {
			emergency_stop = false;
		}
	} else {
		control_system_mode = CONTROL_MODE_JOYSTICK;
		LPUART1_SetMode(false);  /* switch back to 115200 8N1 for dashboard */
		Motor_SendAudioCommand('J');
		printf("[DBG] Mode -> JOYSTICK (fired=%lu)\r\n", (unsigned long)hw.mode_toggle_fired);
	}
}

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
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	/* Read the slide switch at boot so the system starts in the correct mode
	 * without requiring a toggle-cycle. Selected_Mode_Pin is PULLUP active-low:
	 * GPIO_PIN_RESET = switch in BASE position (sw_boot=1). */
	{
		extern volatile Control_SystemMode_t control_system_mode;
		uint8_t sw_boot = (HAL_GPIO_ReadPin(Selected_Mode_GPIO_Port, Selected_Mode_Pin)
		                   == GPIO_PIN_RESET) ? 1u : 0u;
		control_system_mode = sw_boot ? CONTROL_MODE_BASE_SYSTEM : CONTROL_MODE_JOYSTICK;
	/* ModbusBridge_Init MUST run before LPUART1_SetMode arms the RX interrupt.
	 * Any byte arriving on LPUART1 goes straight to ModbusBridge_RxCallback(),
	 * which dereferences hmodbus.htim. If that pointer is still NULL the MCU
	 * faults immediately. */
		ModbusBridge_Init();
		LPUART1_SetMode(sw_boot ? true : false);

		/* Give the base system time to send its first Modbus probe and get a
		 * response before Motor_Init() occupies the CPU for another ~50 ms.
		 * Without this, the base system's cold-start scan arrives during the
		 * MX peripheral init window when LPUART1 RX is not yet armed, gets
		 * no reply, and declares "abnormal heartbeat" — requiring a manual
		 * STM32 reset to re-sync.  300 ms is enough for any base system to
		 * finish booting and fire its first poll. */
		if (sw_boot) {
			uint32_t poll_end = HAL_GetTick() + 300;
			while (HAL_GetTick() < poll_end) {
				ModbusBridge_Process();
			}
		}
	}

	Config_Init(); // Load tuning from Flash
	Motor_Init();
	HW_Init();
	CurrentSensor_Init(&hadc1);
	Motor_SetVoltageLimit(SUPPLY_VOLTAGE, SUPPLY_VOLTAGE); /* 24 V — must match params.h SUPPLY_VOLTAGE used by the FF math */
	Motor_SetMotionProfile(250.0f, 500.0f, 0.1f);
	Telemetry_Init(&hlpuart1);
	Kalman_Init();
	HAL_TIM_Base_Start_IT(&htim6); /* fires HAL_TIM_PeriodElapsedCallback at 1 kHz — control loop */
	/* Bug 1-H: start TIM7 so Kalman_Tick fires at 1 kHz.
	 * PREREQUISITE: TIM7 must be enabled in CubeMX (.ioc) as a Basic Timer,
	 * Prescaler=(170-1), Period=(1000-1), with "TIM7 global interrupt" checked
	 * under NVIC settings.  Without that .ioc entry MX_TIM7_Init() and htim7
	 * are not generated and this line will fail to compile. */
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &modbus_rx_byte, 1);
	HAL_UART_Receive_IT(&huart3, (uint8_t*) &rx_byte, 1);

	/* --- Skip startup menu, use current position as home --- */
	{
		extern volatile bool emergency_stop;
		extern volatile bool startup_estop_pending;
		emergency_stop = false;
		startup_estop_pending = false;
		FAULT_CLR(FAULT_STARTUP_ESTOP);
		printf("\r\n>>> System Ready (skipped startup menu) <<<\r\n");
	}

	/* IWDG — 15-second independent hardware watchdog.
	 * If the firmware hangs, TIM1 keeps generating PWM in hardware (motor runs
	 * uncontrolled). IWDG resets the MCU; Motor_Init() then latches startup ESTOP
	 * which cuts PWM immediately.  15 s is safely longer than the worst-case
	 * gripper sequence (4 × REED_SW_TIMEOUT_MS = 12 s); wait_for_reed() also
	 * kicks the IWDG on each loop iteration. */
	IWDG->KR  = 0x5555U;   /* unlock PR and RLR */
	IWDG->PR  = 6U;         /* prescaler /256 */
	IWDG->RLR = 1875U;      /* 256 × 1875 / 32000 Hz ≈ 15 s nominal */
	/* Wait for register update to propagate through LSI domain (normally < 300 µs).
	 * Cap at 10 ms so a stuck SR bit cannot block the main loop from starting. */
	{ uint32_t _t = HAL_GetTick(); while ((IWDG->SR & 0x7U) && (HAL_GetTick() - _t < 10)) {} }
	IWDG->KR  = 0xCCCCU;   /* start IWDG */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_matlab_tick = HAL_GetTick();
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		hw.dbg_loop_top++;   /* DIAG: proves the main loop is alive and iterating */
		IWDG->KR = 0xAAAAU;  /* kick watchdog — loop is alive */

		// Dummy usage to force linker to keep these symbols for Live Expressions
		if (debug_idx > 100)
			rx_debug_log[0] = 0;

		USART3_DrainTx();   /* flush deferred ESP32 echo/audio bytes (bug 0-C) */

		/* Emit strings deferred by the 100 Hz control ISR (bug 1-G). */
		Motor_DrainControlLog();

		/* Run the blocking gripper Pick/Place sequence at thread level (bug 1-F).
		 * The joystick command arrives in the USART3 RX ISR and only sets
		 * gripper_seq_request; the busy-wait on reed switches must happen here,
		 * never in the ISR. Clear the request before running so a press during
		 * execution is re-latched for the next loop. */
		if (gripper_seq_request != 0) {
			uint8_t req = gripper_seq_request;
			gripper_seq_request = 0;
			if      (req == 1) Gripper_Sequence_Pick();
			else if (req == 2) Gripper_Sequence_Place();
		}

		if (HAL_GetTick() - last_matlab_tick >= 20) {
			Motor_SendDataToMatlab();
			/* HW_RefreshIO() is owned solely by the 100 Hz TIM6 ISR now (bug 1-A);
			 * calling it here too made it reentrant and corrupted its static
			 * debounce/ADC state. The ISR already refreshes hw at 100 Hz. */
			Telemetry_Update();
			last_matlab_tick = HAL_GetTick();
		}

		// Flash Save Check (Triggered from Dashboard via Telemetry CMD)
		extern volatile bool flash_save_requested;
		if (flash_save_requested) {
			flash_save_requested = false;
			Config_Save();
		}

		/* Slide-switch debounce and mode sync — runs entirely in the main loop.
		 * Reads Selected_Mode_Pin directly every 10 ms. Requires 5 consecutive
		 * consistent reads (50 ms) before committing — fast enough for a slide
		 * switch but rejects brief EMI spikes. Mode change is blocked while
		 * emergency_stop is active so relay-arc noise cannot cause a spurious
		 * mode flip during or just after an E-stop event. */
		{
			static uint32_t sw_last_tick    = 0;
			static uint8_t  sw_stable_state = 0xFF; /* 0xFF = uninitialized */
			static uint8_t  sw_candidate    = 0;
			static uint8_t  sw_count        = 0;

			if (HAL_GetTick() - sw_last_tick >= 10) {
				sw_last_tick = HAL_GetTick();
				uint8_t sw_raw = (HAL_GPIO_ReadPin(Selected_Mode_GPIO_Port, Selected_Mode_Pin)
				                  == GPIO_PIN_RESET) ? 1 : 0;

				if (sw_stable_state == 0xFF) {
					sw_stable_state = sw_raw;
					sw_candidate    = sw_raw;
				} else if (sw_raw == sw_candidate) {
					if (sw_count < 5) sw_count++;           /* 5 × 10 ms = 50 ms */
					if (sw_count == 5 && sw_raw != sw_stable_state) {
						sw_stable_state = sw_raw;
						bool sw_wants_base = (sw_raw == 1);
						bool is_base = (control_system_mode == CONTROL_MODE_BASE_SYSTEM);
						if (sw_wants_base != is_base) {
							Mode_Toggle_Impl();
						}
					}
				} else {
					sw_candidate = sw_raw;
					sw_count     = 0;
				}
			}
		}

		hw.dbg_loop_premode++;  /* DIAG: proves the loop reaches the mode-toggle check */

		/* Handle deferred mode-toggle requests from dashboard / joystick button.
		 * Doing the actual switch here (main loop) is safe because HAL_Delay
		 * works correctly outside of higher-priority interrupts. */
		if (mode_toggle_request) {
			mode_toggle_request = false;
			Mode_Toggle_Impl();
		}

		if (trigger_homing_sequence) {
			static uint32_t last_homing_tick = 0;
			if (HAL_GetTick() - last_homing_tick >= 10) {
				if (Motor_RunHomingSequence()) {
					trigger_homing_sequence = false;
				}
				last_homing_tick = HAL_GetTick();
			}
		}

		ModbusBridge_UpdateRegisters();
		ModbusBridge_Process();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 230400;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_9B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_EVEN;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;   /* 170 MHz / 170 / 1000 = 1 kHz — inner speed loop */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 169;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 49;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Gripper_Up_Pin|Gripper_Down_Pin|Motor_Direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay__SysStatus_Pin|Relay_Sysmode_Pin|Relay_MotorPower_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Reed_Close_Pin Reed_Down_Pin Reed_Open_Pin */
  GPIO_InitStruct.Pin = Reed_Close_Pin|Reed_Down_Pin|Reed_Open_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Gripper_Up_Pin Gripper_Down_Pin Motor_Direction_Pin */
  GPIO_InitStruct.Pin = Gripper_Up_Pin|Gripper_Down_Pin|Motor_Direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : E_Stop_Pin */
  GPIO_InitStruct.Pin = E_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Selected_Mode_Pin Reset_Btn_Pin */
  GPIO_InitStruct.Pin = Selected_Mode_Pin|Reset_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Reed_Up_Pin */
  GPIO_InitStruct.Pin = Reed_Up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Reed_Up_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay__SysStatus_Pin Relay_Sysmode_Pin Relay_MotorPower_Pin */
  GPIO_InitStruct.Pin = Relay__SysStatus_Pin|Relay_Sysmode_Pin|Relay_MotorPower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Proximity_Sensor_Pin */
  GPIO_InitStruct.Pin = Proximity_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Proximity_Sensor_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	extern volatile Control_SystemMode_t control_system_mode;
	/* Route printf / debug output to LPUART1 (Dashboard) ONLY.
	 *
	 * USART3 connects to the ESP32 joystick module.  The only data the ESP32
	 * expects to receive from the STM32 is a single-character echo sent by
	 * HAL_UART_RxCpltCallback as an audio-handshake ACK.  Sending debug
	 * strings to USART3 causes the ESP32 to receive unexpected data, which
	 * makes it fire a 'D' (disconnect) status packet → FAULT_JOYSTICK_LOST
	 * → emergency_stop = true, blocking all motor commands.
	 *
	 * Do NOT add huart3 back here.  Debug logs go to the dashboard only.
	 *
	 * Bug 1-D guard: if LPUART1 is mid-reconfiguration (DeInit called but
	 * Init not yet complete), writing to the peripheral can hard-fault the
	 * MCU.  Return immediately — the log message is sacrificed. */
	if (lpuart_reconfiguring) {
		return len;
	}
	if (control_system_mode != CONTROL_MODE_BASE_SYSTEM) {
		/* JOYSTICK / normal mode: LPUART1 is the dashboard serial channel */
		HAL_UART_Transmit(&hlpuart1, (uint8_t*) ptr, len, 10);
	}
	/* In BASE_SYSTEM mode LPUART1 carries Modbus RTU — suppress debug output
	 * entirely so we never corrupt Modbus frames. */
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* E-Stop EXTI on PA5: DELIBERATELY DOES NOTHING.
	 *
	 * Background: PA5 has only the MCU's internal ~40 kΩ pull-up — no external
	 * pull-up is possible on this PCB. While the motor runs, conducted and
	 * radiated noise (brush commutation, relay arc) couples onto the high-
	 * impedance pin and produces brief LOW pulses. A microsecond-window
	 * majority vote inside the ISR cannot distinguish those bursts from a real
	 * press because the burst itself can sustain LOW for the entire vote.
	 *
	 * The polled debounce in HW_RefreshIO (100 Hz, 8 consecutive LOW reads,
	 * counter resets on any HIGH read) is the sole authoritative path now.
	 * A real button press easily sustains LOW for >80 ms; EMI bursts do not.
	 * 80 ms detection latency is well below human perception, so safety is
	 * preserved while eliminating the spurious-trip storm.
	 *
	 * The EXTI line stays enabled (CubeMX configures it on falling edge) so
	 * the falling edge still wakes the MCU from any low-power state — but we
	 * intentionally do not act on it here. */
	(void)GPIO_Pin;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		Motor_ControlLoop();
	} else if (htim->Instance == TIM7) {
		/* 1 kHz Kalman filter tick. u is derived from the last applied PWM
		 * duty cycle; measurement is the encoder angle in radians. */
		extern volatile float current_pwm;
		float u = (current_pwm / 100.0f) * SUPPLY_VOLTAGE;
		float theta_rad = encoder.current_position_deg * (3.14159265f / 180.0f);
		Kalman_SanityTick(u);
		Kalman_Tick(u, theta_rad);
	} else if (htim->Instance == TIM16) {
		/* Modbus RTU T3.5 silence timeout — signal that a frame has ended */
		ModbusBridge_TimerCallback();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		char ch = (char) rx_byte;

		// Debug logging (optional, keeps a history of raw bytes)
		rx_debug_log[debug_idx++ % 16] = ch;

		static bool line_too_long = false;
		if (ch == '\n' || ch == '\r') {
			if (rx_ptr == 3 && !line_too_long) {
				/* Validate: all 3 bytes must be printable ASCII (>= 0x20).
				 * UART framing errors leave the data register as 0x00.
				 * Three null bytes look like a packet with status=0 (not 'C'),
				 * which calls Motor_ProcessPacket → is_joystick_connected=false
				 * → FAULT_JOYSTICK_LOST → emergency_stop=true, permanently
				 * re-latching the e-stop faster than Reset can clear it.
				 * Reject any packet with non-printable bytes to block this. */
				if ((uint8_t)rx_packet[0] >= 0x20 &&
				    (uint8_t)rx_packet[1] >= 0x20 &&
				    (uint8_t)rx_packet[2] >= 0x20)
				{
					// We have a valid 3-char packet: [Action][Safety][Status]
					Motor_ProcessPacket(rx_packet[0], rx_packet[1], rx_packet[2]);

					/* Echo the action character back for the audio handshake.
					 * Enqueued (not blocking) — we are inside the USART3 RX ISR (bug 0-C). */
					USART3_QueueTx((const uint8_t*) &rx_packet[0], 1);
				}
				/* else: silently discard — null/garbage bytes from framing error */
			}
			rx_ptr = 0;
			line_too_long = false;
		} else {
			/* Also reject non-printable mid-packet bytes immediately so a framing
			 * error byte cannot contaminate an otherwise valid packet. */
			if ((uint8_t)ch < 0x20) {
				rx_ptr = 0;           /* flush partial packet */
				line_too_long = false;
			} else if (rx_ptr < 3) {
				rx_packet[rx_ptr++] = ch;
			} else {
				line_too_long = true;
			}
		}

		HAL_UART_Receive_IT(&huart3, (uint8_t*) &rx_byte, 1);
	} else if (huart->Instance == LPUART1) {
		ModbusBridge_RxCallback(modbus_rx_byte);
		/* In BASE_SYSTEM mode the wire carries Modbus RTU binary frames.
		 * Feeding binary bytes into the telemetry text parser would call
		 * Motor_RefreshWatchdog() on garbage data and could misfire commands. */
		if (control_system_mode != CONTROL_MODE_BASE_SYSTEM) {
			Telemetry_ProcessByte(modbus_rx_byte);
		}
		/* HAL_UART_Receive_IT can return HAL_BUSY if the interrupt is still
		 * being processed or a byte arrives exactly during this call.
		 * Loop until it returns HAL_OK to ensure the receiver is re-armed. */
		while (HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &modbus_rx_byte, 1) == HAL_BUSY);
	}
}

/* UART Error recovery.
 *
 * STM32 HAL pitfall: when a UART RX gets a frame error / parity error /
 * overrun (very common during initial ESP32 boot when its TX line floats,
 * or when the system mode flips between BASE_SYSTEM/JOYSTICK and the two
 * sides re-init at slightly different times), HAL sets an error flag and
 * the RX interrupt **stops being serviced** until the flag is cleared and
 * Receive_IT is re-armed. Without this callback the joystick UART (USART3)
 * goes permanently deaf — buzzer still chimes on ESP32-side connect, but
 * STM32 never sees another byte and is_joystick_connected stays false
 * forever.
 *
 * Fix: on any error, clear the error flags and re-arm Receive_IT on the
 * affected UART.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_PEFLAG(huart);
		HAL_UART_Receive_IT(huart, (uint8_t*) &rx_byte, 1);
	} else if (huart->Instance == LPUART1) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_PEFLAG(huart);
		if (control_system_mode == CONTROL_MODE_BASE_SYSTEM) {
			/* Modbus mode: reset protocol state machine + re-arm RX */
			ModbusBridge_UartErrorRecovery();
		} else {
			/* Dashboard mode: re-arm with retry (handles HAL_BUSY) */
			while (HAL_UART_Receive_IT(huart, (uint8_t*) &modbus_rx_byte, 1) == HAL_BUSY);
		}
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
