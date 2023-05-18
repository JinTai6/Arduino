//开发环境是建立在Arduino环境的基础上的
//我们使用的是ESP32开发板
//15号引脚是DHT11温湿度传感器用来测量室内的温度和湿度，36号针脚是管敏电阻传感器来测量光照强度
//32和33号针脚是YL-69土壤湿度传感器，分别来测量土壤的深层和表层的湿度
//16，26,27是火焰传感器，蜂鸣器，红色LED灯用来实现火焰报警器的功能
//34，26,27是烟雾传感器，蜂鸣器，红色LED灯用来实现烟雾报警器的功能
//22和21号针脚代表I2C的SCL和SDA来实现通信
//11和10号针脚是两个LED灯用于植物的补光，在点灯APP中的按键实现开关
//39号针脚用来测量设备电压
//32的I2C的地址是和LCD 1602A的屏幕的显示
//0x67的I2C地址是用于BMP180用来测量大气压强和海拔高度的使用
//25和0是来控制两个继电器来分别控制小风扇和水泵的继电器
#define BLINKER_WIFI
#include <Blinker.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>
#include <TimeLib.h>

#define DHTPIN 15 // 温湿度传感器引脚
#define LDRPIN 36 // 光敏电阻引脚
#define SOILPIN1 32 // 土壤湿度传感器1引脚
#define SOILPIN2 33 // 土壤湿度传感器2引脚
#define FLAMEPIN 16 // 火焰传感器引脚
#define SMOKEPIN 34 //烟雾传感器引脚
#define BUZZERPIN 26 // 蜂鸣器引脚
#define LEDPIN 27 // LED引脚
#define LED 12 //1号LED补光灯针脚
#define LED1 14 //2号LED补光灯针脚
#define V_PIN 39 //测量设备电压要用到的针脚
#define RAIN_SENSOR_PIN 35 //雨滴传感器针脚
#define ONE_WIRE_BUS 4 // 定义DS18B20的引脚为4
#define jidianqi1 25 //定义一个控制小风扇的继电器1号
#define jidianqi2 0 //板载的继电器用于水泵
#define buttonPin 17 // 按钮引脚
#define SMOKE_THRESHOLD 1200 // 设置烟雾传感器阈值

int currentPage = 0; // 当前显示的页面
unsigned long lastButtonPressTime = 0; // 上一次按下按钮的时间

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(32,16,2);  // 设置I2C地址和屏幕行列数

Adafruit_BMP085 bmp; //设置BMP180传感器的型号

#define DHTTYPE DHT11 // 温湿度传感器型号

#define SMOKE_THRESHOLD 1400 // 设置烟雾传感器阈值

bool light1_auto = true; // 补光灯1是否自动控制
bool light2_auto = true; // 补光灯2是否自动控制
bool fan_auto = true; // 小风扇是否自动控制
bool pump_auto = true; // 水泵是否自动控制

char auth[] = "40a629d880e0"; // Blinker授权码
char ssid[] = "JinTai"; // WIFI名称
char pswd[] = "j12345678"; // WIFI密码

BlinkerNumber HUMI("humi"); // 湿度数据流
BlinkerNumber TEMP("temp"); // 温度数据流
BlinkerNumber LIGHT("light"); // 光照数据流
BlinkerNumber SOIL1("soil1"); // 土壤湿度数据流1
BlinkerNumber SOIL2("soil2"); // 土壤湿度数据流2
BlinkerNumber VOLTAGE("voltage"); //测量设备的电压
BlinkerNumber PRE("pressure"); //大气压强
BlinkerNumber ALTITUDE("altitude"); //海拔高度
BlinkerNumber SOILTEMP("soil_temp"); //土壤温度
BlinkerNumber RUNTIME("run_time_min"); //设备运行时间

BlinkerButton Button1("btn1");
BlinkerButton Button2("btn2");
BlinkerButton Button3("btn3");
BlinkerButton Button4("btn4");

DHT dht(DHTPIN, DHTTYPE); // 温湿度传感器对象

float humi_read = 0, temp_read = 0, light_read = 0, soil_read1 = 0, soil_read2 = 0,soil_temp = 0,run_tme = 0,run_time_min = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 2000;


// 按下按键即会执行该函数
void button1_callback(const String & state) {
  BLINKER_LOG("get button state: ", state);
  if (light1_auto) {
    // 如果补光灯1处于自动控制模式，则切换到手动控制模式
    light1_auto = false;
    digitalWrite(LED, !digitalRead(LED));
  } else {
    // 如果补光灯1处于手动控制模式，则切换到自动控制模式
    light1_auto = true;
  }
}

// 按下按键即会执行该函数
void button2_callback(const String & state) {
  BLINKER_LOG("get button state: ", state);
  if (light2_auto) {
    // 如果补光灯2处于自动控制模式，则切换到手动控制模式
    light2_auto = false;
    digitalWrite(LED1, !digitalRead(LED1));
  } else {
    // 如果补光灯2处于手动控制模式，则切换到自动控制模式
    light2_auto = true;
  }
}

// 按下按键即会执行该函数
void button3_callback(const String & state) {
  BLINKER_LOG("get button state: ", state);
  if (fan_auto) {
    // 如果小风扇处于自动控制模式，则切换到手动控制模式
    fan_auto = false;
    digitalWrite(jidianqi1, !digitalRead(jidianqi1));
  } else {
    // 如果小风扇处于手动控制模式，则切换到自动控制模式
    fan_auto = true;
  }
}

// 按下按键即会执行该函数
void button4_callback(const String & state) {
  BLINKER_LOG("get button state: ", state);
  if (pump_auto) {
    // 如果水泵处于自动控制模式，则切换到手动控制模式
    pump_auto = false;
    digitalWrite(jidianqi2, !digitalRead(jidianqi2));
  } else {
    // 如果水泵处于手动控制模式，则切换到自动控制模式
    pump_auto = true;
  }
}

void heartbeat()
{
HUMI.print(humi_read); // 发送湿度数据流
TEMP.print(temp_read); // 发送温度数据流
LIGHT.print(light_read); // 发送光照数据流
SOIL1.print(soil_read1); // 发送土壤湿度数据流1
SOIL2.print(soil_read2); // 发送土壤湿度数据流2
}

void dataStorage()
{
Blinker.dataStorage("temp", temp_read); // 存储温度数据
Blinker.dataStorage("humi", humi_read); // 存储湿度数据
Blinker.dataStorage("light", light_read); // 存储光照数据
Blinker.dataStorage("soil1", soil_read1); // 存储土壤湿度数据1
Blinker.dataStorage("soil2", soil_read2); // 存储土壤湿度数据2
Blinker.dataStorage("soil_temp", soil_temp); // 存储土壤湿度数据2
}

void setup()
{

Serial.begin(115200);// 初始化串口通信
BLINKER_DEBUG.stream(Serial);// 设置调试信息输出流
BLINKER_DEBUG.debugAll();// 输出所有调试信息

pinMode(FLAMEPIN, INPUT);
pinMode(SMOKEPIN, INPUT);
pinMode(RAIN_SENSOR_PIN, INPUT);
pinMode(BUZZERPIN, OUTPUT);
pinMode(LEDPIN, OUTPUT);

Blinker.begin(auth, ssid, pswd);// 启动Blinker库的功能
Blinker.attachHeartbeat(heartbeat);// 设置心跳函数
Blinker.attachDataStorage(dataStorage);// 设置数据存储函数
dht.begin();// 初始化DHT传感器

lcd.begin(); // 初始化液晶屏

if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

sensors.begin();

// 初始化有LED的IO
pinMode(LED, OUTPUT);
digitalWrite(LED, LOW);
Button1.attach(button1_callback);
// 初始化有LED的IO
pinMode(LED1, OUTPUT);
digitalWrite(LED1, LOW);
Button2.attach(button2_callback);

pinMode(jidianqi1,OUTPUT);
digitalWrite(jidianqi1, LOW);
Button3.attach(button3_callback);

pinMode(jidianqi2,OUTPUT);
digitalWrite(jidianqi2, LOW);
Button4.attach(button4_callback);

pinMode(buttonPin, INPUT_PULLUP);
configTime(0, 0, "pool.ntp.org", "time.nist.gov");

}

//--------------------------------------------------------
// 烟雾和火焰报警总函数
void smokeAndFireAlarm() {
  int flameState = digitalRead(FLAMEPIN); // 读取火焰传感器的数字值
  uint32_t adcValue = analogRead(SMOKEPIN); // 读取烟雾传感器的模拟值
  
  // 检测火焰传感器
  if (flameState == LOW) {
    // 火焰报警
    digitalWrite(BUZZERPIN, HIGH); // 打开蜂鸣器
    digitalWrite(LEDPIN, HIGH); // 打开LED灯
    Blinker.vibrate(); // 手机振动
    Blinker.wechat("发现火焰！发现火焰！请观察安全情况！"); // 微信推送消息
    Blinker.notify("发现火焰！发现火焰！请观察安全情况！"); // 手机通知消息
    Serial.println("Fire detected!"); // 输出消息到串口
  } else { // 如果没有火焰，关闭蜂鸣器和LED灯
    digitalWrite(BUZZERPIN, LOW);
    digitalWrite(LEDPIN, LOW);
  }
  
  // 检测烟雾传感器
  if (adcValue > SMOKE_THRESHOLD) { // 判断是否超过阈值
    // 烟雾报警
    digitalWrite(BUZZERPIN, HIGH); // 打开蜂鸣器
    digitalWrite(LEDPIN, HIGH); // 打开LED灯
    Blinker.vibrate(); // 手机振动
    Blinker.wechat("发现烟雾！发现烟雾！请观察安全情况！"); // 微信推送消息
    Blinker.notify("发现烟雾！发现烟雾！请观察安全情况！"); // 手机通知消息
    Serial.println("Smoke detected!"); // 输出消息到串口
  } else { // 如果没有烟雾，关闭蜂鸣器和LED灯
    digitalWrite(BUZZERPIN, LOW);
    digitalWrite(LEDPIN, LOW);
  }
}
//--------------------------------------------------------

//--------------------------------------------------------
// 雨滴报警总函数
void rainAlarm() {
  int rainState = digitalRead(RAIN_SENSOR_PIN);
  
  // 检测雨滴传感器
  if (rainState == LOW) {
    Blinker.vibrate();
    Blinker.wechat("发现雨滴！发现雨滴！请观察安全情况！");
    Blinker.notify("发现雨滴！发现雨滴！请观察安全情况！");
    Serial.println("Rain detected!");  // 输出消息到串口
  }
}
//--------------------------------------------------------

void loop()
{
smokeAndFireAlarm();  // 调用总函数触发烟雾和火焰报警功能

rainAlarm();  // 调用总函数触发雨滴报警功能

Blinker.run();// 运行Blinker库

// 读取DHT传感器、LDR传感器和土壤湿度传感器的数据
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {
previousMillis = currentMillis;

float h = dht.readHumidity();
float t = dht.readTemperature();
int l = analogRead(LDRPIN);
int s1 = analogRead(SOILPIN1);
int s2 = analogRead(SOILPIN2);
float temperature = bmp.readTemperature();
float pressure = bmp.readPressure();
float altitude = bmp.readAltitude();
sensors.requestTemperatures();
float soil_temp = sensors.getTempCByIndex(0);
SOILTEMP.print(soil_temp);

// 如果无法读取DHT传感器的数据
if (isnan(h) || isnan(t)) {
BLINKER_LOG("Failed to read from DHT sensor!");
} else {
humi_read = h;
temp_read = t;
}

// 检查按钮是否被按下
if (digitalRead(buttonPin) == LOW) {
  // 如果按钮被按下，则更新当前显示的页面
  currentPage = (currentPage + 1) % 5;
  lastButtonPressTime = millis();
  delay(200); // 去抖
}

// 如果按钮按下时间超过10秒，则返回时间页面
if (millis() - lastButtonPressTime > 10000) {
  currentPage = 0;
}

// 获取当前时间
time_t now = time(nullptr);
struct tm *timeinfo = localtime(&now);

// 根据当前显示的页面更新LCD屏幕上的内容
lcd.clear();
switch (currentPage) {
  case 0:
    // 在LCD屏幕上显示当前时间
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Date:");
    lcd.print(timeinfo->tm_year + 1900);
    lcd.print("-");
    lcd.print(timeinfo->tm_mon + 1);
    lcd.print("-");
    lcd.print(timeinfo->tm_mday);
    lcd.setCursor(0, 1);
    lcd.print("Time:");
    if (timeinfo->tm_hour < 10) lcd.print("0");
    lcd.print(timeinfo->tm_hour);
    lcd.print(":");
    if (timeinfo->tm_min < 10) lcd.print("0");
    lcd.print(timeinfo->tm_min);
    break;
case 1:
    // 显示温度和湿度
    lcd.clear(); // 清除屏幕上的旧内容
    lcd.setCursor(0,0);              // 将光标移动到第一行第一列
    lcd.print("Temp:");              // 打印"Temp:"
    lcd.print(t);                    // 打印温度值
    lcd.print((char)223);            // 打印温度单位 "°"
    lcd.print("C");                  // 打印温度单位 "C"
    lcd.setCursor(0,1);              // 将光标移动到第二行第一列
    lcd.print("Humidity:");              // 打印"Humidity:"
    lcd.print(h);                    // 打印湿度值
    lcd.print("%");                  // 打印湿度单位 "%"
    break;
  case 2:
    // 显示光照和土壤温度
    lcd.clear(); // 清除屏幕上的旧内容
    lcd.setCursor(0,0);              // 将光标移动到第一行第一列
    lcd.print("Light:");             // 打印"Light:"
    lcd.print(light_read);           // 打印温度值
    lcd.print("%");                  // 打印湿度单位 "%"
    lcd.setCursor(0,1);              // 将光标移动到第二行第一列
    lcd.print("SoilTemp:");           // 打印"soil_temp:"
    lcd.print(soil_temp);            // 打印湿度值
    lcd.print((char)223);            // 打印温度单位 "°"
    lcd.print("C");                  // 打印温度单位 "C"
    break;
  case 3:
    // 显示土壤的表层温度和底层温度
    lcd.clear(); // 清除屏幕上的旧内容
    lcd.setCursor(0,0);              // 将光标移动到第一行第一列
    lcd.print("SurHumi:");        // 打印"SurfaceTemp"
    lcd.print(soil_read1);
    lcd.print((char)223);            // 打印温度单位 "°"
    lcd.print("C");                  // 打印温度单位 "C"
    lcd.setCursor(0,1);              // 将光标移动到第二行第一列
    lcd.print("BotHumi:");        // 打印"BottomTemp:"
    lcd.print(soil_read2);
    lcd.print((char)223);            // 打印温度单位 "°"
    lcd.print("C");                  // 打印温度单位 "C"
    break;
  case 4:
    // 显示大气压强和海拔
    lcd.clear(); // 清除屏幕上的旧内容
    lcd.setCursor(0,0);              // 将光标移动到第一行第一列
    lcd.print("Pre:");           // 打印"Pressure:"
    lcd.print(pressure);             // 
    lcd.print("Pa");                 // 
    lcd.setCursor(0,1);              // 将光标移动到第二行第一列
    lcd.print("Alt:");           //
    lcd.print(altitude);             //
    lcd.print(" M");                  //
    break;
}

// 将LDR传感器的数据映射到0-100的范围内
light_read = map(l, 0, 4095, 100, 0);
// 将土壤湿度传感器1的数据映射到0-100的范围内
soil_read1 = map(s1, 4095, 0, 0, 100);
// 将土壤湿度传感器2的数据映射到0-100的范围内
soil_read2 = map(s2, 4095, 0, 0, 100);

// 读取ESP32的电压值
int sensorValue = analogRead(39);
// 将读取到的ADC值转换成电压值
float voltage = sensorValue / 4095.0 * 5;
VOLTAGE.print(voltage);   //将电压值传输到Blinker云平台

PRE.print(pressure);
ALTITUDE.print(altitude);

// 将湿度、温度、光照和土壤湿度1的数据发送到点灯APP中
HUMI.print(humi_read);
TEMP.print(temp_read);
LIGHT.print(light_read);
SOIL1.print(soil_read1);
SOIL2.print(soil_read2);

//以下代码来自动控制补光灯、水泵和小风扇
if (light1_auto) {
  if (light_read < 28) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}
if (light2_auto) {
  if (light_read < 28) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }
}
if (fan_auto) {
  if (t > 40) {
    digitalWrite(jidianqi1, HIGH);
  } else {
    digitalWrite(jidianqi1, LOW);
  }
}
if (pump_auto) {
  if ((soil_read1 + soil_read2) / 2 < 25) {
    digitalWrite(jidianqi2, HIGH);
  } else {
    digitalWrite(jidianqi2, LOW);
  }
}

time_t run_time = Blinker.runTime();
run_time_min = run_time / 60;
RUNTIME.print(run_time_min);

delay(500);
Blinker.delay(500);
}
}
