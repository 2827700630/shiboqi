/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TFTh/TFT_CAD.h"  // 包含绘图函数
#include "TFTh/TFT_init.h" // 包含初始化函数
#include "TFTh/TFT_text.h" // 包含文本显示函数
#include "TFTh/TFT_io.h"   // 包含IO函数
#include <math.h>          // 用于sin函数生成波形
#include <stdio.h>         // 用于sprintf格式化字符串
#include <string.h>        // 用于字符串处理函数
#include <stdbool.h>       // 用于布尔类型定义
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

/* USER CODE BEGIN PV */
TFT_HandleTypeDef htft1;       // 第一个TFT屏幕句柄
#define TFT1_SCREEN_WIDTH 240  // TFT屏幕宽度
#define TFT1_SCREEN_HEIGHT 320 // TFT屏幕高度
TFT_HandleTypeDef htft2;       // 第二个TFT屏幕句柄
#define TFT2_SCREEN_WIDTH 128  // TFT屏幕宽度
#define TFT2_SCREEN_HEIGHT 160 // TFT屏幕高度

// 示波器相关变量
#define WAVEFORM_POINTS TFT1_SCREEN_WIDTH // 波形点数等于屏幕宽度
uint16_t waveform_data1[WAVEFORM_POINTS]; // 存储通道1波形Y坐标
uint16_t waveform_data2[WAVEFORM_POINTS]; // 存储通道2波形Y坐标
float time_base = 0.01f;                  // 时间基准 (默认10ms/div)
float voltage_scale1 = 1.0f;              // 通道1电压刻度 (V/div)
float voltage_scale2 = 1.0f;              // 通道2电压刻度 (V/div)
float trigger_level = 1.5f;               // 触发电平 (V)
char text_buffer[50];                     // 用于显示文本的缓冲区
uint8_t run_state = 1;                    // 运行状态 1:运行 0:停止
uint8_t channel1_enabled = 1;             // 通道1使能状态
uint8_t channel2_enabled = 0;             // 通道2使能状态
char coupling_mode[4] = "DC";             // 通道1耦合方式 (DC/AC)
char trigger_source[6] = "CHAN1";         // 触发源 (CHAN1/CHAN2)
char trigger_slope[4] = "POS";            // 触发斜率 (POS/NEG)
char trigger_mode[5] = "EDGE";            // 触发模式
char trigger_sweep[7] = "AUTO";           // 触发扫描模式 (AUTO/SINGLE)

// 波形测量数据
float signal_frequency = 50.0f; // 信号频率 (Hz)
float signal_amplitude = 3.3f;  // 信号幅值 (V峰峰值)
float signal_offset = 0.0f;     // 信号偏移 (V)
uint8_t show_measurement = 1;   // 是否显示测量值

// 波形参数
float sine_frequency = 1.0f;   // 正弦波频率系数
float square_frequency = 2.0f; // 方波频率系数
float phase = 0.0f;            // 波形相位

// UART接收相关
#define UART_RX_BUFFER_SIZE 128 // 增大缓冲区以容纳多行指令
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_data; // 单字节接收
volatile uint16_t uart_rx_index = 0;
volatile uint8_t uart_rx_complete = 0; // 接收完成标志

// 示波器网格设置
#define GRID_SIZE 30 // 网格大小（像素）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void parse_uart_command(char *command);
void analyze_waveform(uint16_t *wave_data, uint16_t points);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化第一个TFT屏幕 (ST7789)
  TFT_Init_Instance(&htft1, &hspi1, TFT_CS_GPIO_Port, TFT_CS_Pin);
  TFT_Config_Pins(&htft1, TFT_DC_GPIO_Port, TFT_DC_Pin,
                  TFT_RES_GPIO_Port, TFT_RES_Pin,
                  TFT_BL_GPIO_Port, TFT_BL_Pin);
  TFT_Config_Display(&htft1, 0, 0, 0);                                       // 设置方向、X/Y偏移 (方向0: 竖屏)
  TFT_IO_Init(&htft1);                                                       // 初始化IO层
  TFT_Init_ST7789v3(&htft1);                                                 // ST7789 屏幕初始化
  TFT_Fill_Area(&htft1, 0, 0, TFT1_SCREEN_WIDTH, TFT1_SCREEN_HEIGHT, BLACK); // 清屏为黑色

  // 初始化第二个TFT屏幕 (ST7735S)
  TFT_Init_Instance(&htft2, &hspi2, CS2_GPIO_Port, CS2_Pin);
  TFT_Config_Pins(&htft2, DC2_GPIO_Port, DC2_Pin,
                  RES2_GPIO_Port, RES2_Pin,
                  BL2_GPIO_Port, BL2_Pin);
  TFT_Config_Display(&htft2, 2, 2, 1);                                       // 设置方向、X/Y偏移 (方向0: 竖屏)
  TFT_IO_Init(&htft2);                                                       // 初始化IO层
  TFT_Init_ST7735S(&htft2);                                                  // ST7735S 屏幕初始化
  TFT_Fill_Area(&htft2, 0, 0, TFT2_SCREEN_WIDTH, TFT2_SCREEN_HEIGHT, BLACK); // 清屏为深灰色

  // 启动UART1接收中断，每次接收一个字节
  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // --- 1. 模拟数据生成 ---
    if (run_state) // 只有在运行状态下才更新波形
    {
      phase += 0.1f * sine_frequency; // 调整正弦波相位
      if (phase > 2 * 3.14159f)
      {
        phase -= 2 * 3.14159f;
      }

      // 通道1 - 正弦波
      if (channel1_enabled)
      {
        for (int i = 0; i < WAVEFORM_POINTS; i++)
        {
          // 生成正弦波数据，振幅根据电压刻度设置调整
          float value = sinf(phase + (float)i * 2 * 3.14159f / (WAVEFORM_POINTS / sine_frequency));

          // 映射到屏幕Y坐标，考虑电压刻度和触发电平
          float voltage_value = value * voltage_scale1 * 2; // 假设满屏高度为±4个电压刻度
          float screen_height_per_volt = (TFT1_SCREEN_HEIGHT / 8) / voltage_scale1;
          waveform_data1[i] = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - voltage_value * screen_height_per_volt);

          // 确保坐标在屏幕范围内
          if (waveform_data1[i] >= TFT1_SCREEN_HEIGHT)
            waveform_data1[i] = TFT1_SCREEN_HEIGHT - 1;
          if (waveform_data1[i] < 0)
            waveform_data1[i] = 0;
        }
      }

      // 通道2 - 方波
      if (channel2_enabled)
      {
        // 计算一个完整方波周期的点数
        int points_per_cycle = (int)(WAVEFORM_POINTS / square_frequency);
        if (points_per_cycle < 10)
          points_per_cycle = 10;

        for (int i = 0; i < WAVEFORM_POINTS; i++)
        {
          // 方波：高电平或低电平
          bool is_high = ((i % points_per_cycle) < (points_per_cycle / 2));

          // 计算方波在Y轴上的位置
          float voltage_high = 2.0f * voltage_scale2; // 高电平为2个刻度
          float voltage_low = -2.0f * voltage_scale2; // 低电平为-2个刻度
          float screen_height_per_volt = (TFT1_SCREEN_HEIGHT / 8) / voltage_scale2;

          if (is_high)
            waveform_data2[i] = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - voltage_high * screen_height_per_volt);
          else
            waveform_data2[i] = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - voltage_low * screen_height_per_volt);

          // 确保坐标在屏幕范围内
          if (waveform_data2[i] >= TFT1_SCREEN_HEIGHT)
            waveform_data2[i] = TFT1_SCREEN_HEIGHT - 1;
          if (waveform_data2[i] < 0)
            waveform_data2[i] = 0;
        }
      }

      // 分析波形数据并更新测量值
      if (channel1_enabled)
        analyze_waveform(waveform_data1, WAVEFORM_POINTS);
    }

    // --- 2. 绘制TFT1 (示波器波形) ---
    // a. 清除旧波形区域
    TFT_Fill_Area(&htft1, 0, 0, TFT1_SCREEN_WIDTH, TFT1_SCREEN_HEIGHT, BLACK);

    // b. 绘制网格
    // 绘制水平线
    for (int y = 0; y < TFT1_SCREEN_HEIGHT; y += GRID_SIZE) // 每GRID_SIZE像素画一条线
    {
      TFT_Draw_Fast_HLine(&htft1, 0, y, TFT1_SCREEN_WIDTH, GRAY); // 用灰色绘制水平线
    }
    // 绘制垂直线
    for (int x = 0; x < TFT1_SCREEN_WIDTH; x += GRID_SIZE) // 每GRID_SIZE像素画一条线
    {
      TFT_Draw_Fast_VLine(&htft1, x, 0, TFT1_SCREEN_HEIGHT, GRAY); // 用灰色绘制垂直线
    }
    // 绘制中心线
    TFT_Draw_Fast_HLine(&htft1, 0, TFT1_SCREEN_HEIGHT / 2, TFT1_SCREEN_WIDTH, GBLUE);
    TFT_Draw_Fast_VLine(&htft1, TFT1_SCREEN_WIDTH / 2, 0, TFT1_SCREEN_HEIGHT, GBLUE);

    // 在示波器屏幕上绘制触发电平线
    if (channel1_enabled || channel2_enabled)
    {
      // 将触发电平映射到屏幕Y坐标
      uint16_t trigger_y = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - (trigger_level / voltage_scale1) * (TFT1_SCREEN_HEIGHT / 8));
      if (trigger_y >= 0 && trigger_y < TFT1_SCREEN_HEIGHT)
      {
        // 用虚线绘制触发电平
        for (int x = 0; x < TFT1_SCREEN_WIDTH; x += 6)
        {
          TFT_Draw_Fast_HLine(&htft1, x, trigger_y, 3, MAGENTA);
        }

        // 在屏幕右侧绘制触发指示标志
        TFT_Draw_Triangle(&htft1,
                          TFT1_SCREEN_WIDTH - 10, trigger_y,
                          TFT1_SCREEN_WIDTH - 2, trigger_y - 4,
                          TFT1_SCREEN_WIDTH - 2, trigger_y + 4,
                          MAGENTA);
      }
    }

    // c. 绘制波形
    if (channel1_enabled)
    {
      // 绘制通道1信息
      sprintf(text_buffer, "CH1");
      TFT_Show_String(&htft1, 5, 45, text_buffer, YELLOW, BLACK, 16, 0);

      // 绘制通道1波形
      for (int i = 0; i < WAVEFORM_POINTS - 1; i++)
      {
        TFT_Draw_Line(&htft1, i, waveform_data1[i], i + 1, waveform_data1[i + 1], YELLOW); // 用黄色绘制波形
      }
    }

    if (channel2_enabled)
    {
      // 绘制通道2信息
      sprintf(text_buffer, "CH2");
      TFT_Show_String(&htft1, 35, 45, text_buffer, CYAN, BLACK, 16, 0);

      // 绘制通道2波形
      for (int i = 0; i < WAVEFORM_POINTS - 1; i++)
      {
        TFT_Draw_Line(&htft1, i, waveform_data2[i], i + 1, waveform_data2[i + 1], CYAN); // 用青色绘制波形
      }
    }

    // 在停止状态下显示提示
    if (!run_state)
    {
      sprintf(text_buffer, "STOP");
      TFT_Show_String(&htft1, TFT1_SCREEN_WIDTH - 40, 5, text_buffer, RED, BLACK, 16, 0);
    }

    // --- 3. 绘制TFT2 (参数显示) ---
    // a. 清屏
    TFT_Fill_Area(&htft2, 0, 0, TFT2_SCREEN_WIDTH, TFT2_SCREEN_HEIGHT, BLACK);

    // b. 显示参数
    // 显示标题和运行状态
    TFT_Show_String(&htft2, 5, 5, "Oscilloscope", WHITE, BLACK, 16, 0);
    if (run_state)
    {
      TFT_Show_String(&htft2, 90, 5, "RUN", GREEN, BLACK, 16, 0);
    }
    else
    {
      TFT_Show_String(&htft2, 90, 5, "STOP", RED, BLACK, 16, 0);
    }

    // 分隔线
    TFT_Draw_Fast_HLine(&htft2, 0, 25, TFT2_SCREEN_WIDTH, BROWN);

    // 通道状态
    sprintf(text_buffer, "CH1:%s", channel1_enabled ? "ON" : "OFF");
    TFT_Show_String(&htft2, 5, 30, text_buffer, YELLOW, BLACK, 16, 0);

    sprintf(text_buffer, "CH2:%s", channel2_enabled ? "ON" : "OFF");
    TFT_Show_String(&htft2, 70, 30, text_buffer, CYAN, BLACK, 16, 0);

    // 显示时间基准 (更新为秒/格)
    if (time_base >= 1000.0f)
      sprintf(text_buffer, "Time: %.1fs/div", time_base / 1000.0f);
    else
      sprintf(text_buffer, "Time: %.1fms/div", time_base);
    TFT_Show_String(&htft2, 5, 50, text_buffer, BRRED, BLACK, 16, 0);

    // 显示电压刻度 - 分别显示两个通道的电压刻度
    sprintf(text_buffer, "V1: %.1fV/div", voltage_scale1);
    TFT_Show_String(&htft2, 5, 70, text_buffer, YELLOW, BLACK, 16, 0);

    if (channel2_enabled)
    {
      sprintf(text_buffer, "V2: %.1fV/div", voltage_scale2);
      TFT_Show_String(&htft2, 5, 90, text_buffer, CYAN, BLACK, 16, 0);
    }

    // 显示触发信息
    uint16_t trig_y = 90;
    if (channel2_enabled)
      trig_y = 110;

    sprintf(text_buffer, "Trig: %s", trigger_source);
    TFT_Show_String(&htft2, 5, trig_y, text_buffer, MAGENTA, BLACK, 16, 0);

    // 显示触发斜率
    trig_y += 20;
    sprintf(text_buffer, "Slope: %s", strcmp(trigger_slope, "POS") == 0 ? "Rise" : "Fall");
    TFT_Show_String(&htft2, 5, trig_y, text_buffer, MAGENTA, BLACK, 16, 0);

    // 耦合方式
    trig_y += 20;
    sprintf(text_buffer, "Coupl: %s", coupling_mode);
    TFT_Show_String(&htft2, 5, trig_y, text_buffer, LIGHTBLUE, BLACK, 16, 0);

    // 如果有足够空间，显示测量信息
    if (channel1_enabled && trig_y + 20 < TFT2_SCREEN_HEIGHT - 20)
    {
      sprintf(text_buffer, "Freq: %.1f Hz", signal_frequency);
      TFT_Show_String(&htft2, 5, trig_y + 20, text_buffer, GREEN, BLACK, 16, 0);
    }

    // --- 4. 串口指令处理 (占位符) ---
    if (uart_rx_complete)
    {
      parse_uart_command((char *)uart_rx_buffer);
      uart_rx_complete = 0;
    }

    // --- 5. 延时 ---
    HAL_Delay(20); // 控制刷新率，避免闪烁太快
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  分析波形数据并更新测量值
 * @param  wave_data 波形数据数组
 * @param  points 波形点数
 * @retval None
 */
void analyze_waveform(uint16_t *wave_data, uint16_t points)
{
  if (points < 10 || !channel1_enabled || !run_state)
    return;

  // 查找最大值和最小值
  uint16_t max_value = 0;
  uint16_t min_value = TFT1_SCREEN_HEIGHT;

  for (uint16_t i = 0; i < points; i++)
  {
    if (wave_data[i] > max_value)
      max_value = wave_data[i];
    if (wave_data[i] < min_value)
      min_value = wave_data[i];
  }

  // 计算幅值 (转换为电压值)
  uint16_t peak_to_peak = (min_value > max_value) ? 0 : (max_value - min_value);
  float pixels_per_volt = (TFT1_SCREEN_HEIGHT / 8) / voltage_scale1; // 每伏特对应的像素数
  signal_amplitude = (float)peak_to_peak / pixels_per_volt;

  // 计算直流偏移
  uint16_t mid_point = (max_value + min_value) / 2;
  signal_offset = ((float)TFT1_SCREEN_HEIGHT / 2 - mid_point) / pixels_per_volt;

  // 计算频率 (零点交叉法)
  // 假设一个完整周期内有两个零点交叉
  // 为简化计算，我们搜索上升沿
  uint16_t crossings = 0;
  uint16_t first_crossing = 0;
  uint16_t last_crossing = 0;
  uint16_t mid_level = TFT1_SCREEN_HEIGHT / 2;

  for (uint16_t i = 1; i < points; i++)
  {
    // 检测上升沿穿越中点
    if (wave_data[i - 1] > mid_level && wave_data[i] <= mid_level)
    {
      if (crossings == 0)
        first_crossing = i;
      else
        last_crossing = i;

      crossings++;
    }
  }

  // 如果找到至少两个交叉点，计算频率
  if (crossings >= 2 && last_crossing > first_crossing)
  {
    // 计算周期数
    float num_cycles = (float)(crossings - 1) / 2.0f;

    // 计算周期时间 (ms)
    float period_time = (float)(last_crossing - first_crossing) * time_base / GRID_SIZE;

    // 计算频率 (Hz)
    if (period_time > 0)
      signal_frequency = (1000.0f * num_cycles) / period_time;
    else
      signal_frequency = 0;
  }
}

/**
 * @brief  串口接收中断回调
 * @param  huart UART句柄
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 将接收到的字符存入缓冲区
    if (uart_rx_data == '\n' || uart_rx_data == '\r')
    {
      // 收到回车换行符，表示指令结束
      if (uart_rx_index > 0)
      {
        uart_rx_buffer[uart_rx_index] = '\0'; // 添加字符串结束符
        uart_rx_complete = 1;                 // 设置完成标志
        uart_rx_index = 0;                    // 重置索引
      }
    }
    else
    {
      // 存储字符到缓冲区
      if (uart_rx_index < UART_RX_BUFFER_SIZE - 1)
      {
        uart_rx_buffer[uart_rx_index++] = uart_rx_data;
      }
    }

    // 重新启动接收中断
    HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
  }
}

/**
 * @brief  解析串口接收到的指令
 * @param  command 指令字符串
 * @retval None
 */
void parse_uart_command(char *command)
{
  // 回显收到的指令（调试用）
  HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 10);

  // 将指令转换为大写以统一处理
  for (int i = 0; command[i]; i++)
  {
    if (command[i] >= 'a' && command[i] <= 'z')
    {
      command[i] = command[i] - 'a' + 'A';
    }
  }

  // 自动设置
  if (strstr(command, ":AUTOSCALE"))
  {
    time_base = 0.01f;               // 默认时基设置为10ms/div
    voltage_scale1 = 1.0f;           // 默认通道1电压刻度
    voltage_scale2 = 1.0f;           // 默认通道2电压刻度
    trigger_level = 0.0f;            // 默认触发电平
    run_state = 1;                   // 运行状态
    channel1_enabled = 1;            // 启用通道1
    channel2_enabled = 1;            // 启用通道2
    sine_frequency = 1.0f;           // 重置正弦波频率
    square_frequency = 1.0f;         // 重置方波频率
    strcpy(coupling_mode, "DC");     // 默认耦合方式
    strcpy(trigger_slope, "POS");    // 默认上升沿触发
    strcpy(trigger_source, "CHAN1"); // 默认通道1触发

    // 发送确认消息
    HAL_UART_Transmit(&huart1, (uint8_t *)"Autoscale OK\r\n", 13, 100);
  }

  // 通道开关控制
  else if (strstr(command, ":CHAN1 ON"))
  {
    channel1_enabled = 1;
    HAL_UART_Transmit(&huart1, (uint8_t *)"Channel 1 ON\r\n", 14, 100);
  }
  else if (strstr(command, ":CHAN1 OFF"))
  {
    channel1_enabled = 0;
    HAL_UART_Transmit(&huart1, (uint8_t *)"Channel 1 OFF\r\n", 15, 100);
  }
  else if (strstr(command, ":CHAN2 ON"))
  {
    channel2_enabled = 1;
    HAL_UART_Transmit(&huart1, (uint8_t *)"Channel 2 ON\r\n", 14, 100);
  }
  else if (strstr(command, ":CHAN2 OFF"))
  {
    channel2_enabled = 0;
    HAL_UART_Transmit(&huart1, (uint8_t *)"Channel 2 OFF\r\n", 15, 100);
  }

  // 垂直设置 - 电压量程
  else if (strstr(command, ":CHAN1:VOLTS"))
  {
    float new_volts = 0;
    char *volt_pos = strstr(command, "VOLTS") + 5; // 跳过"VOLTS"

    // 跳过空格
    while (*volt_pos == ' ')
      volt_pos++;

    // 提取电压值
    if (sscanf(volt_pos, "%fV", &new_volts) == 1)
    {
      voltage_scale1 = new_volts;
      char resp[30];
      sprintf(resp, "Voltage scale CH1: %.1f V/div\r\n", voltage_scale1);
      HAL_UART_Transmit(&huart1, (uint8_t *)resp, strlen(resp), 100);
    }
  }
  else if (strstr(command, ":CHAN2:VOLTS"))
  {
    float new_volts = 0;
    char *volt_pos = strstr(command, "VOLTS") + 5; // 跳过"VOLTS"

    // 跳过空格
    while (*volt_pos == ' ')
      volt_pos++;

    // 提取电压值
    if (sscanf(volt_pos, "%fV", &new_volts) == 1)
    {
      voltage_scale2 = new_volts;
      char resp[30];
      sprintf(resp, "Voltage scale CH2: %.1f V/div\r\n", voltage_scale2);
      HAL_UART_Transmit(&huart1, (uint8_t *)resp, strlen(resp), 100);
    }
  }

  // 耦合方式设置
  else if (strstr(command, ":CHAN1:COUP"))
  {
    if (strstr(command, "DC"))
    {
      strcpy(coupling_mode, "DC");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Coupling: DC\r\n", 14, 100);
    }
    else if (strstr(command, "AC"))
    {
      strcpy(coupling_mode, "AC");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Coupling: AC\r\n", 14, 100);
    }
  }

  // 新的时基设置格式 - :TIM:MAIN
 // 通过精确匹配处理特定时基值
else if (strstr(command, ":TIM:MAIN 0.001"))
{
  time_base = 1.0f; // 1ms/div
  sine_frequency = 2.0f; // 快速
  square_frequency = 2.0f;
  HAL_UART_Transmit(&huart1, (uint8_t *)"Timebase: 1.0 ms/div\r\n", 24, 100);
}
else if (strstr(command, ":TIM:MAIN 0.01"))
{
  time_base = 10.0f; // 10ms/div
  sine_frequency = 1.0f; // 中速
  square_frequency = 1.5f;
  HAL_UART_Transmit(&huart1, (uint8_t *)"Timebase: 10.0 ms/div\r\n", 25, 100);
}
else if (strstr(command, ":TIM:MAIN 0.1"))
{
  time_base = 100.0f; // 100ms/div
  sine_frequency = 0.2f; // 非常慢
  square_frequency = 0.5f;
  HAL_UART_Transmit(&huart1, (uint8_t *)"Timebase: 100.0 ms/div\r\n", 26, 100);
}

  // 触发源设置
  else if (strstr(command, ":TRIG:EDGE:SOUR"))
  {
    if (strstr(command, "CHAN1"))
    {
      strcpy(trigger_source, "CHAN1");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Trigger source: CHAN1\r\n", 24, 100);
    }
    else if (strstr(command, "CHAN2"))
    {
      strcpy(trigger_source, "CHAN2");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Trigger source: CHAN2\r\n", 24, 100);
    }
  }

  // 触发斜率设置
  else if (strstr(command, ":TRIG:EDGE:SLOP"))
  {
    if (strstr(command, "POS"))
    {
      strcpy(trigger_slope, "POS");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Trigger slope: Rising\r\n", 24, 100);
    }
    else if (strstr(command, "NEG"))
    {
      strcpy(trigger_slope, "NEG");
      HAL_UART_Transmit(&huart1, (uint8_t *)"Trigger slope: Falling\r\n", 25, 100);
    }
  }

  // 运行/停止控制
  else if (strstr(command, ":RUN"))
  {
    run_state = !run_state; // 切换运行/停止状态
    if (run_state)
      HAL_UART_Transmit(&huart1, (uint8_t *)"RUN\r\n", 5, 100);
    else
      HAL_UART_Transmit(&huart1, (uint8_t *)"STOP\r\n", 6, 100);
  }

  // 停止指令
  else if (strstr(command, ":STOP"))
  {
    run_state = 0; // 停止
    HAL_UART_Transmit(&huart1, (uint8_t *)"STOP\r\n", 6, 100);
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
