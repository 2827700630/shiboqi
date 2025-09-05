/*
  ESP32 串口数据显示到TFT屏幕
  从串口1读取数据并显示在ST7789 TFT屏幕上
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <HardwareSerial.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) // ESP32-S3
#define TFT_CS 3
#define TFT_RST 5
#define TFT_DC 46
#endif

// 使用硬件SPI接口
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// 注意：Serial1在ESP32中已经预定义，无需重新创建

void setup(void)
{
  // 初始化串口0用于调试
  Serial.begin(115200);
  Serial.println("ESP32 TFT显示串口数据");

  // 初始化串口1，ESP32-S3推荐引脚：RX=18, TX=17
  // 对于ESP32-S3，可以使用GPIO 17和18，或者根据您的接线调整
  Serial1.begin(115200, SERIAL_8N1, 18, 17);  // RX=18, TX=17
  Serial.println("串口1已初始化 (RX=18, TX=17)");

  // 初始化TFT显示屏
  tft.init(240, 280);           // 1.69" 280x240 TFT
  tft.setRotation(2);           // 设置屏幕方向
  tft.fillScreen(ST77XX_BLACK); // 清屏

  // 设置文本参数
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(4);
  tft.setCursor(0, 0);

  // 显示标题
  tft.println("----------");

  Serial.println("TFT显示屏已初始化");
}

void loop()
{
  // 检查串口1是否有数据
  if (Serial1.available())
  {
    String receivedData = Serial1.readString();
    // receivedData.trim(); // 去除首尾空格和换行符

    // 在串口0输出接收到的数据（用于调试）
    Serial.println("接收到数据: " + receivedData);

    // 清除屏幕内容区域（保留标题）
    tft.fillRect(0, 50, tft.width(), tft.height() - 50, ST77XX_BLACK);

    // 设置光标位置到内容区域
    tft.setCursor(0, 50);
    tft.setTextColor(ST77XX_GREEN);
    // tft.setTextSize(2);

    // 在TFT屏幕上显示接收到的数据
    tft.println(receivedData);

    // // 可选：显示时间戳
    // tft.setTextColor(ST77XX_YELLOW);
    // tft.println("");
    // tft.print("time: ");
    // tft.print(millis()/1000);
    // tft.println("s");
  }
  delay(100); // 短暂延时避免过于频繁的检查
}