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
uint16_t waveform_data[WAVEFORM_POINTS];  // 存储波形Y坐标
float time_base = 1.0f;                   // 时间基准 (ms/div)
float voltage_scale = 0.5f;               // 电压刻度 (V/div)
float trigger_level = 1.5f;               // 触发电平 (V)
char text_buffer[50];                     // 用于显示文本的缓冲区
uint8_t run_state = 1;                    // 运行状态 1:运行 0:停止
uint8_t channel1_enabled = 1;             // 通道1使能状态
uint8_t channel2_enabled = 0;             // 通道2使能状态 (暂未绘制)
char coupling_mode[4] = "DC";             // 耦合模式 (DC/AC)
char trigger_mode[5] = "EDGE";            // 触发模式
char trigger_sweep[7] = "AUTO";           // 触发扫描模式 (AUTO/SINGLE)

// 波形测量数据
float signal_frequency = 50.0f;           // 信号频率 (Hz)
float signal_amplitude = 3.3f;            // 信号幅值 (V峰峰值)
float signal_offset = 0.0f;               // 信号偏移 (V)
uint8_t show_measurement = 1;             // 是否显示测量值

// UART接收相关
#define UART_RX_BUFFER_SIZE 64
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_data; // 单字节接收
volatile uint16_t uart_rx_index = 0;
volatile uint8_t uart_rx_complete = 0; // 接收完成标志
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
  TFT_Config_Display(&htft2, 2, 2, 1);                                          // 设置方向、X/Y偏移 (方向0: 竖屏)
  TFT_IO_Init(&htft2);                                                          // 初始化IO层
  TFT_Init_ST7735S(&htft2);                                                     // ST7735S 屏幕初始化
  TFT_Fill_Area(&htft2, 0, 0, TFT2_SCREEN_WIDTH, TFT2_SCREEN_HEIGHT, BLACK); // 清屏为深灰色

  // 启动UART1接收中断，每次接收一个字节
  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);

  float phase = 0.0f; // 用于生成模拟正弦波的相位
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // --- 1. 模拟数据生成 (替换为实际ADC采样和处理) ---
    phase += 0.1f; // 调整这个值改变频率
    if (phase > 2 * 3.14159f)
    {
      phase -= 2 * 3.14159f;
    }
    for (int i = 0; i < WAVEFORM_POINTS; i++)
    {
      // 生成模拟正弦波数据 (-1.0 to 1.0)
      float value = sinf(phase + (float)i * 2 * 3.14159f / WAVEFORM_POINTS * 2); // 乘以2增加周期数
      // 映射到屏幕Y坐标 (0 到 TFT1_SCREEN_HEIGHT - 1)
      // 假设屏幕中心为0V, 幅度占屏幕高度的80%
      waveform_data[i] = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - value * (TFT1_SCREEN_HEIGHT * 0.4f));
      // 确保坐标在屏幕范围内
      if (waveform_data[i] >= TFT1_SCREEN_HEIGHT)
      {
        waveform_data[i] = TFT1_SCREEN_HEIGHT - 1;
      }
    }

    // 分析波形数据并更新测量值
    analyze_waveform(waveform_data, WAVEFORM_POINTS);

    // --- 2. 绘制TFT1 (示波器波形) ---
    // a. 清除旧波形区域 (可以优化为只清除波形线条本身)
    TFT_Fill_Area(&htft1, 0, 0, TFT1_SCREEN_WIDTH, TFT1_SCREEN_HEIGHT, BLACK);

    // b. 绘制网格 (可选)
    // 绘制水平线
    for (int y = 0; y < TFT1_SCREEN_HEIGHT; y += 30) // 每30像素画一条线
    {
      TFT_Draw_Fast_HLine(&htft1, 0, y, TFT1_SCREEN_WIDTH, GRAY); // 用灰色绘制水平线
    }
    // 绘制垂直线
    for (int x = 0; x < TFT1_SCREEN_WIDTH; x += 30) // 每30像素画一条线
    {
      TFT_Draw_Fast_VLine(&htft1, x, 0, TFT1_SCREEN_HEIGHT, GRAY); // 用灰色绘制垂直线
    }
    // 绘制中心线
    TFT_Draw_Fast_HLine(&htft1, 0, TFT1_SCREEN_HEIGHT / 2, TFT1_SCREEN_WIDTH, BROWN);
    TFT_Draw_Fast_VLine(&htft1, TFT1_SCREEN_WIDTH / 2, 0, TFT1_SCREEN_HEIGHT, BROWN);

    // c. 绘制波形
    for (int i = 0; i < WAVEFORM_POINTS - 1; i++)
    {
      TFT_Draw_Line(&htft1, i, waveform_data[i], i + 1, waveform_data[i + 1], YELLOW); // 用黄色绘制波形
    }

    // 在示波器屏幕上绘制触发电平线
    if (channel1_enabled)
    {
      // 将触发电平映射到屏幕Y坐标
      uint16_t trigger_y = (uint16_t)(TFT1_SCREEN_HEIGHT / 2 - (trigger_level / voltage_scale) * (TFT1_SCREEN_HEIGHT / 8));
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

    // --- 3. 绘制TFT2 (参数显示) ---
    if (show_measurement)
    {
      // a. 清屏 (如果参数不经常变动，可以优化为只在变化时重绘)
      TFT_Fill_Area(&htft2, 0, 0, TFT2_SCREEN_WIDTH, TFT2_SCREEN_HEIGHT, BLACK);

      // b. 显示参数 (需要TFT_text库支持)
      // 假设有 TFT_Show_String(htft, x, y, string, fg_color, bg_color, font_size, background_mode)
      // background_mode: 0=不透明背景, 1=透明背景
      // 注意: TFT_Show_String 的参数可能需要根据你的库调整

      // 显示标题
      TFT_Show_String(&htft2, 5, 5, "Oscilloscope", WHITE, BLACK, 16, 0);
      TFT_Draw_Fast_HLine(&htft2, 0, 25, TFT2_SCREEN_WIDTH, BROWN); // 分隔线

      // 显示时间基准
      sprintf(text_buffer, "Time: %.1f ms/div", time_base);
      TFT_Show_String(&htft2, 5, 35, text_buffer, CYAN, BLACK, 16, 0);

      // 显示电压刻度
      sprintf(text_buffer, "Volt: %.1f V/div", voltage_scale);
      TFT_Show_String(&htft2, 5, 55, text_buffer, GREEN, BLACK, 16, 0);

      // 显示触发电平
      sprintf(text_buffer, "Trig: %.1f V", trigger_level);
      TFT_Show_String(&htft2, 5, 75, text_buffer, MAGENTA, BLACK, 16, 0);

      // 显示信号频率
      sprintf(text_buffer, "Freq: %.1f Hz", signal_frequency);
      TFT_Show_String(&htft2, 5, 95, text_buffer, LIGHTBLUE, BLACK, 16, 0);

      // 显示信号幅值
      sprintf(text_buffer, "Amp: %.1f Vpp", signal_amplitude);
      TFT_Show_String(&htft2, 5, 115, text_buffer, ORANGE, BLACK, 16, 0);

      // 显示信号偏移
      sprintf(text_buffer, "Offset: %.1f V", signal_offset);
      TFT_Show_String(&htft2, 5, 135, text_buffer, WHITE, BLACK, 16, 0);
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
  RCC_ClkInitStruct RCC_ClkInitStruct = {0};

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
    if (wave_data[i] > max_value) max_value = wave_data[i];
    if (wave_data[i] < min_value) min_value = wave_data[i];
  }
  
  // 计算幅值 (转换为电压值)
  uint16_t peak_to_peak = (min_value > max_value) ? 0 : (max_value - min_value);
  float pixels_per_volt = (TFT1_SCREEN_HEIGHT / 8) / voltage_scale; // 每伏特对应的像素数
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
    if (wave_data[i-1] > mid_level && wave_data[i] <= mid_level)
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
    float period_time = (float)(last_crossing - first_crossing) * time_base / 30.0f;
    
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
        uart_rx_complete = 1;                  // 设置完成标志
        uart_rx_index = 0;                     // 重置索引
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
  // HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), 100);
  // HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 10);
  
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
    time_base = 1.0f;      // 默认时基设置
    voltage_scale = 1.0f;  // 默认电压刻度
    trigger_level = 0.0f;  // 默认触发电平
    run_state = 1;         // 运行状态
    strcpy(coupling_mode, "DC");    // 默认耦合方式
    strcpy(trigger_mode, "EDGE");   // 默认触发模式
    strcpy(trigger_sweep, "AUTO");  // 默认触发扫描
    
    // 可以添加更多自动设置的参数
  }
  
  // 通道开关控制
  else if (strstr(command, ":CHAN1 ON"))
  {
    channel1_enabled = 1;
  }
  else if (strstr(command, ":CHAN1 OFF"))
  {
    channel1_enabled = 0;
  }
  else if (strstr(command, ":CHAN2 ON"))
  {
    channel2_enabled = 1;
  }
  else if (strstr(command, ":CHAN2 OFF"))
  {
    channel2_enabled = 0;
  }
  
  // 垂直设置 - 电压量程
  else if (strstr(command, ":CHAN1:VOLTS"))
  {
    float new_volts = 0;
    char *volt_pos = strstr(command, "VOLTS") + 5; // 跳过"VOLTS"
    
    // 跳过空格
    while (*volt_pos == ' ') volt_pos++;
    
    // 提取电压值
    if (sscanf(volt_pos, "%fV", &new_volts) == 1)
    {
      voltage_scale = new_volts;
    }
  }
  
  // 耦合方式设置
  else if (strstr(command, ":CHAN1:COUPL"))
  {
    if (strstr(command, "DC"))
    {
      strcpy(coupling_mode, "DC");
    }
    else if (strstr(command, "AC"))
    {
      strcpy(coupling_mode, "AC");
    }
  }
  
  // 水平设置 - 时基
  else if (strstr(command, ":TIMEBASE"))
  {
    float new_time = 0;
    char *time_pos = strstr(command, "TIMEBASE") + 8; // 跳过"TIMEBASE"
    
    // 跳过空格
    while (*time_pos == ' ') time_pos++;
    
    // 提取时间值，支持ms和us单位
    if (strstr(time_pos, "MS") || strstr(time_pos, "ms"))
    {
      if (sscanf(time_pos, "%fMS", &new_time) == 1 || sscanf(time_pos, "%fms", &new_time) == 1)
      {
        time_base = new_time;
      }
    }
    else if (strstr(time_pos, "US") || strstr(time_pos, "us"))
    {
      if (sscanf(time_pos, "%fUS", &new_time) == 1 || sscanf(time_pos, "%fus", &new_time) == 1)
      {
        time_base = new_time / 1000.0f; // 转换为ms
      }
    }
  }
  
  // 触发设置
  else if (strstr(command, ":TRIG:MODE"))
  {
    if (strstr(command, "EDGE"))
    {
      strcpy(trigger_mode, "EDGE");
    }
    // 可以添加更多触发模式
  }
  else if (strstr(command, ":TRIG:LEV"))
  {
    float level_change = 0;
    if (sscanf(command, ":TRIG:LEV %fV", &level_change) == 1)
    {
      trigger_level += level_change; // 相对变化
    }
  }
  else if (strstr(command, ":TRIG:SWEEP"))
  {
    if (strstr(command, "SINGLE"))
    {
      strcpy(trigger_sweep, "SINGLE");
    }
    else if (strstr(command, "AUTO"))
    {
      strcpy(trigger_sweep, "AUTO");
    }
  }
  
  // 运行/停止控制
  else if (strstr(command, ":RUN"))
  {
    run_state = !run_state; // 切换运行/停止状态
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
