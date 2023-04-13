#define BLINKER_WIFI
#include <Blinker.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

#define DHTPIN 15 // 温湿度传感器引脚
#define LDRPIN 36 // 光敏电阻引脚
#define SOILPIN1 32 // 土壤湿度传感器1引脚
#define DHTPIN_SOIL 4 // 土壤温湿度传感器引脚
#define FLAMEPIN 16 // 火焰传感器引脚
#define SMOKEPIN 34 //烟雾传感器引脚
#define BUZZERPIN 26 // 蜂鸣器引脚
#define LEDPIN 27 // LED引脚
#define LEDPIN1 11//补光灯针脚1
#define LEDPIN2 10//补光灯针脚2

LiquidCrystal_I2C lcd(32,16,2);  // 设置I2C地址和屏幕行列数

#define DHTTYPE DHT11 // 温湿度传感器型号
#define DHTTYPE_SOIL DHT11 // 土壤温湿度传感器型号

char auth[] = "40a629d880e0"; // Blinker授权码
char ssid[] = "JinTai"; // WIFI名称
char pswd[] = "j12345678"; // WIFI密码

BlinkerNumber HUMI("humi"); // 湿度数据流
BlinkerNumber TEMP("temp"); // 温度数据流
BlinkerNumber LIGHT("light"); // 光照数据流
BlinkerNumber SOIL1("soil1"); // 土壤湿度数据流1
BlinkerNumber SOIL_TEMP("soil_temp");// 土壤温度数据流
BlinkerNumber SOIL_HUMI("soil_humi");// 土壤湿度数据流2
BlinkerButton Button1("btn2");//定义侧面灯光的开关
BlinkerButton Button2("btn1");//定义真面灯光的开关

DHT dht(DHTPIN, DHTTYPE); // 温湿度传感器对象
DHT dht_soil(DHTPIN_SOIL, DHTTYPE_SOIL); // 土壤温湿度传感器对象

float humi_read = 0, temp_read = 0, light_read = 0, soil_read1 = 0, soil_read2 = 0;
float soil_temp_read = 0, soil_humi_read = 0;
float V_read = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 2000;

int threshold = 50;   // 烟雾检测阈值设定为50 ppm

unsigned long startTime = 0; // 记录火焰开始时间
unsigned long duration = 0;  // 记录火焰持续时间

// 按下按键即会执行该函数
void Button1_callback(const String & state) {
    BLINKER_LOG("get button state: ", state);
    if (state=="on") {
        digitalWrite(LEDPIN1, HIGH);
        // 反馈开关状态
        Button1.print("on");
    } else if(state=="off"){
        digitalWrite(LEDPIN1, LOW);
        // 反馈开关状态
        Button1.print("off");
    }
}

// 按下按键即会执行该函数
void Button2_callback(const String & state) {
    BLINKER_LOG("get button state: ", state);
    if (state=="on") {
        digitalWrite(LEDPIN2, HIGH);
        // 反馈开关状态
        Button1.print("on");
    } else if(state=="off"){
        digitalWrite(LEDPIN2, LOW);
        // 反馈开关状态
        Button1.print("off");
    }
}
void heartbeat()
{
HUMI.print(humi_read); // 发送湿度数据流
TEMP.print(temp_read); // 发送温度数据流
LIGHT.print(light_read); // 发送光照数据流
SOIL1.print(soil_read1); // 发送土壤湿度数据流1
SOIL_TEMP.print(soil_temp_read);// 发送土壤温度数据流
SOIL_HUMI.print(soil_humi_read);// 发送土壤湿度数据流2
}

void dataStorage()
{
Blinker.dataStorage("temp", temp_read); // 存储温度数据
Blinker.dataStorage("humi", humi_read); // 存储湿度数据
Blinker.dataStorage("light", light_read); // 存储光照数据
Blinker.dataStorage("soil1", soil_read1); // 存储土壤湿度数据1
Blinker.dataStorage("soil_temp", soil_temp_read);// 存储土壤温度数据
Blinker.dataStorage("soil_humi", soil_humi_read);// 存储土壤湿度数据2
}

void setup()
{

Serial.begin(115200);// 初始化串口通信
BLINKER_DEBUG.stream(Serial);// 设置调试信息输出流
BLINKER_DEBUG.debugAll();// 输出所有调试信息

pinMode(FLAMEPIN, INPUT);
pinMode(BUZZERPIN, OUTPUT);
pinMode(LEDPIN, OUTPUT);

Blinker.begin(auth, ssid, pswd);// 启动Blinker库的功能
Blinker.attachHeartbeat(heartbeat);// 设置心跳函数
Blinker.attachDataStorage(dataStorage);// 设置数据存储函数
Button1.attach(Button1_callback);
Button1.attach(Button2_callback);
dht.begin();// 初始化DHT传感器
dht_soil.begin();// 初始化土壤湿度传感器

lcd.begin(); // 初始化液晶屏

// 初始化有LED的IO
pinMode(LEDPIN1, OUTPUT);
digitalWrite(LEDPIN1, LOW);
pinMode(LEDPIN2,OUTPUT);
digitalWrite(LEDPIN2, LOW);
}

//烟雾检测函数
void smoke() {
  static bool smokeDetected = false;         // 状态变量，记录当前是否检测到烟雾
  static unsigned long startTime;            // 记录烟雾检测开始时间
  static unsigned int onDuration = 0;        // 记录烟雾持续时间（单位：毫秒）
  static float maxVolume = 0.5;              // 蜂鸣器最大音量
  static float volumeIncrement = 0.05;       // 蜂鸣器音量增量
  static unsigned int maxBlinkInterval = 500; // LED灯最大闪烁时间间隔（单位：毫秒）
  static unsigned int minBlinkInterval = 50;  // LED灯最小闪烁时间间隔（单位：毫秒）
  static unsigned int blinkDuration = 1000;   // LED灯每次闪烁的持续时间（单位：毫秒）
  
  int sensorValue = analogRead(SMOKEPIN);   // 读取模拟输入信号
  float voltage = sensorValue / 1024.0 * 5.0; // 将模拟信号转换为电压值
  float ppm = voltage / 0.1;                 // 将电压值转换为气体浓度（单位：ppm）
  Serial.print("PPM: ");
  Serial.println(ppm);
  
  if (ppm > threshold) {                     // 如果检测到的浓度超过阈值
    if (!smokeDetected) {                    // 如果之前没有检测到烟雾
      smokeDetected = true;                  // 更新状态变量为true
      startTime = millis();                  // 记录当前时间为烟雾检测开始时间
      digitalWrite(BUZZERPIN, HIGH);         // 触发蜂鸣器报警
      digitalWrite(LEDPIN, HIGH);            // 触发LED灯提示
      onDuration = 0;                        // 烟雾持续时间清零
    }
    else {                                   // 如果之前已经检测到烟雾
      unsigned long currentTime = millis();  // 记录当前时间
      onDuration = currentTime - startTime;  // 计算烟雾持续的时间
      if (onDuration > 5000) {               // 如果烟雾持续时间超过5秒
        maxVolume = 1.0;                     // 将蜂鸣器音量增大至最大值
        volumeIncrement = 0.1;               // 增大音量时的增量也变大
        maxBlinkInterval = 100;              // 闪烁频率增大至最快
        minBlinkInterval = 25;
      }
      if (onDuration % blinkDuration == 0) { // 每隔一段时间进行LED灯的闪烁
        unsigned int blinkInterval = map(onDuration, 0, 5000, maxBlinkInterval, minBlinkInterval);
        digitalWrite(LEDPIN, !digitalRead(LEDPIN));
        delay(blinkInterval);
      }
      float volume = map(onDuration, 0, 5000, 0, maxVolume);
      tone(BUZZERPIN, 500, volume);          // 调整蜂鸣器音量
    }
  }
  else {                                     // 如果检测到的浓度低于阈值
    smokeDetected = false;
    digitalWrite(BUZZERPIN, LOW);
    digitalWrite(LEDPIN,LOW);
  }
}
// 火焰检测函数
void flame() {
  int flameReading = digitalRead(FLAMEPIN); // 读取火焰传感器模块的数字输入信号
  
  if (flameReading == LOW) {                     // 如果数字信号为HIGH，则表示检测到火焰
    if (startTime == 0) {                        // 如果还没有记录火焰开始时间，则记录当前时间
      startTime = millis();
    }
    duration = millis() - startTime;             // 计算火焰持续时间
    
    // 根据火焰持续时间来控制警报器的强度
    if (duration < 2000) {                       // 当火焰持续时间小于2秒时
      digitalWrite(LEDPIN, HIGH);                // 点亮LED灯
      tone(BUZZERPIN, 1000);                     // 发出中频声音
      delay(100);                                 // 延迟100毫秒
      noTone(BUZZERPIN);                          // 停止发声
      delay(100);                                 // 延迟100毫秒
    } else if (duration < 5000) {                // 当火焰持续时间在2秒和5秒之间时
      digitalWrite(LEDPIN, HIGH);                // 点亮LED灯
      tone(BUZZERPIN, 1500);                     // 发出高频声音
      delay(50);                                  // 延迟50毫秒
      noTone(BUZZERPIN);                          // 停止发声
      delay(50);                                  // 延迟50毫秒
    } else {                                      // 当火焰持续时间大于等于5秒时
      digitalWrite(LEDPIN, HIGH);                // 点亮LED灯
      tone(BUZZERPIN, 2000);                     // 发出超高频声音
      delay(10);                                  // 延迟10毫秒
      noTone(BUZZERPIN);                          // 停止发声
      delay(10);                                  // 延迟10毫秒
    }
  } else {
    digitalWrite(LEDPIN, LOW);                   // 关闭LED灯
    noTone(BUZZERPIN);                            // 停止发声
    startTime = 0;                                // 重置火焰开始时间和持续时间
    duration = 0;
  }
}

void loop()
{
smoke();
flame();

Blinker.run();// 运行Blinker库

// 读取DHT传感器、LDR传感器和土壤湿度传感器的数据
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {
previousMillis = currentMillis;

float h = dht.readHumidity();
float t = dht.readTemperature();
int l = analogRead(LDRPIN);
int s1 = analogRead(SOILPIN1);
float h_soil = dht_soil.readHumidity();
float t_soil = dht_soil.readTemperature();

  lcd.setCursor(0,0);              // 将光标移动到第一行第一列
  lcd.print("Temp:");              // 打印"Temp:"
  lcd.print(t);                    // 打印温度值
  lcd.print((char)223);            // 打印温度单位 "°"
  lcd.print("C");                  // 打印温度单位 "C"
  lcd.setCursor(0,1);              // 将光标移动到第二行第一列
  lcd.print("Humidity:");          // 打印"Humidity:"
  lcd.print(h);                    // 打印湿度值
  lcd.print("%");                  // 打印湿度单位 "%"
  
  delay(5000);                     // 延迟5秒
// 如果无法读取DHT传感器的数据
if (isnan(h) || isnan(t)) {
BLINKER_LOG("Failed to read from DHT sensor!");
} else {
// 打印湿度和温度数据到串口
BLINKER_LOG("Humidity: ", h, " %");
BLINKER_LOG("Temperature: ", t, " *C");

humi_read = h;
temp_read = t;
}

// 如果无法读取土壤湿度传感器的数据
if (isnan(h_soil) || isnan(t_soil)) {
BLINKER_LOG("Failed to read from Soil DHT sensor!");
} else {
// 打印湿度和温度数据到串口
BLINKER_LOG("Humidity: ", h, " %");
BLINKER_LOG("Temperature: ", t, " *C");

humi_read = h;
temp_read = t;
}

// 如果无法读取土壤湿度传感器的数据
if (isnan(h_soil) || isnan(t_soil)) {
BLINKER_LOG("Failed to read from Soil DHT sensor!");
} else {
// 打印土壤湿度和温度数据到串口
BLINKER_LOG("Soil Humidity: ", h_soil, " %");
BLINKER_LOG("Soil Temperature: ", t_soil, " *C");

soil_humi_read = h_soil;
soil_temp_read = t_soil;
}

// 将LDR传感器的数据映射到0-100的范围内
light_read = map(l, 0, 4095, 100, 0);
// 将土壤湿度传感器1的数据映射到0-100的范围内
soil_read1 = map(s1, 4095, 0, 0, 100);

// 打印光照和土壤湿度1的数据到串口
BLINKER_LOG("Light: ", light_read, " %");
BLINKER_LOG("Soil Moisture 1: ", soil_read1, " %");

// 将湿度、温度、光照和土壤湿度1的数据发送到点灯APP中
HUMI.print(humi_read);
TEMP.print(temp_read);
LIGHT.print(light_read);
SOIL1.print(soil_read1);
SOIL_TEMP.print(soil_temp_read);
SOIL_HUMI.print(soil_humi_read);
}
}