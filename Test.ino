//开发环境是建立在Arduino环境的基础上的
//我们使用的是ESP32开发板
//15号引脚是DHT11温湿度传感器用来测量室内的温度和湿度，36号针脚是管敏电阻传感器来测量光照强度
//32和33号针脚是YL-69土壤湿度传感器，分别来测量土壤的深层和表层的湿度
//4号针脚是DHT11传感器用来测量土壤的温度和表层湿度（现在预计想要把这个DHT11缩测得的湿度删除）
//16，26,27是火焰传感器，蜂鸣器，红色LED灯用来实现火焰报警器的功能
//34，26,27是烟雾传感器，蜂鸣器，红色LED灯用来实现烟雾报警器的功能
//22和21号针脚代表I2C的SCL和SDA来实现通信
#define BLINKER_WIFI
#include <Blinker.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 15 // 温湿度传感器引脚
#define LDRPIN 36 // 光敏电阻引脚
#define SOILPIN1 32 // 土壤湿度传感器1引脚
#define SOILPIN2 33 // 土壤湿度传感器2引脚
#define DHTPIN_SOIL 4 // 土壤温湿度传感器引脚
#define FLAMEPIN 16 // 火焰传感器引脚
#define SMOKEPIN 34 //烟雾传感器引脚
#define BUZZERPIN 26 // 蜂鸣器引脚
#define LEDPIN 27 // LED引脚

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
BlinkerNumber SOIL2("soil2"); // 土壤湿度数据流2
BlinkerNumber SOIL_TEMP("soil_temp");// 土壤温度数据流
BlinkerNumber SOIL_HUMI("soil_humi");// 土壤湿度数据流2

DHT dht(DHTPIN, DHTTYPE); // 温湿度传感器对象
DHT dht_soil(DHTPIN_SOIL, DHTTYPE_SOIL); // 土壤温湿度传感器对象

float humi_read = 0, temp_read = 0, light_read = 0, soil_read1 = 0, soil_read2 = 0;
float soil_temp_read = 0, soil_humi_read = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 2000;
const int SMOKE_THRESHOLD = 2000;

void heartbeat()
{
HUMI.print(humi_read); // 发送湿度数据流
TEMP.print(temp_read); // 发送温度数据流
LIGHT.print(light_read); // 发送光照数据流
SOIL1.print(soil_read1); // 发送土壤湿度数据流1
SOIL2.print(soil_read2); // 发送土壤湿度数据流2
SOIL_TEMP.print(soil_temp_read);// 发送土壤温度数据流
SOIL_HUMI.print(soil_humi_read);// 发送土壤湿度数据流2
}

void dataStorage()
{
Blinker.dataStorage("temp", temp_read); // 存储温度数据
Blinker.dataStorage("humi", humi_read); // 存储湿度数据
Blinker.dataStorage("light", light_read); // 存储光照数据
Blinker.dataStorage("soil1", soil_read1); // 存储土壤湿度数据1
Blinker.dataStorage("soil2", soil_read2); // 存储土壤湿度数据2
Blinker.dataStorage("soil_temp", soil_temp_read);// 存储土壤温度数据
Blinker.dataStorage("soil_humi", soil_humi_read);// 存储土壤湿度数据2
}

void setup()
{

Serial.begin(115200);// 初始化串口通信
BLINKER_DEBUG.stream(Serial);// 设置调试信息输出流
BLINKER_DEBUG.debugAll();// 输出所有调试信息

pinMode(FLAMEPIN, INPUT_PULLUP);
pinMode(BUZZERPIN, OUTPUT);
pinMode(LEDPIN, OUTPUT);

Blinker.begin(auth, ssid, pswd);// 启动Blinker库的功能
Blinker.attachHeartbeat(heartbeat);// 设置心跳函数
Blinker.attachDataStorage(dataStorage);// 设置数据存储函数
dht.begin();// 初始化DHT传感器
dht_soil.begin();// 初始化土壤湿度传感器

lcd.begin(); // 初始化液晶屏
}

void flame() {
  static unsigned long flame_start_time = 0; // 火焰开始检测时间
  static int flame_alert_level = 0; // 火焰报警等级，从0-5
  static int flame_detected = 0; // 是否检测到火焰

  int flame_reading = digitalRead(FLAMEPIN); // 读取火焰传感器数值

  if (flame_reading == LOW) { // 如果检测到火焰
    if (flame_detected == 0) { // 如果之前没有检测到火焰
      flame_start_time = millis(); // 记录检测到火焰的时间
    }
    flame_detected = 1; // 设置已检测到火焰
    unsigned long flame_duration = millis() - flame_start_time; // 计算检测到火焰的持续时间
    if (flame_duration < 5000) { // 如果持续时间小于5秒
      flame_alert_level = map(flame_duration, 0, 5000, 0, 5); // 根据持续时间计算火焰报警等级
    } else { // 如果持续时间大于等于5秒
      flame_alert_level = 5; // 设置火焰报警等级为5
    }
  } else { // 如果没有检测到火焰
    flame_detected = 0; // 设置未检测到火焰
    flame_alert_level = 0; // 设置火焰报警等级为0
  }

  if (flame_alert_level > 0) { // 如果火焰报警等级大于0
    digitalWrite(LEDPIN, HIGH); // 点亮红色LED灯
    for (int i = 0; i < flame_alert_level; i++) { // 按照火焰报警等级，循环控制蜂鸣器响应的次数和间隔
      digitalWrite(BUZZERPIN, HIGH);
      delay(100);
      digitalWrite(BUZZERPIN, LOW);
      delay(100);
    }
  } else { // 如果火焰报警等级为0
    digitalWrite(LEDPIN, LOW); // 关闭红色LED灯
    digitalWrite(BUZZERPIN, LOW); // 关闭蜂鸣器
  }
}

void loop()
{

//检测是否存在火焰
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
int s2 = analogRead(SOILPIN2);
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
// 将土壤湿度传感器2的数据映射到0-100的范围内
soil_read2 = map(s2, 4095, 0, 0, 100);

// 打印光照和土壤湿度1的数据到串口
BLINKER_LOG("Light: ", light_read, " %");
BLINKER_LOG("Soil Moisture 1: ", soil_read1, " %");
BLINKER_LOG("Soil Moisture 2: ", soil_read2, " %");

// 将湿度、温度、光照和土壤湿度1的数据发送到点灯APP中
HUMI.print(humi_read);
TEMP.print(temp_read);
LIGHT.print(light_read);
SOIL1.print(soil_read1);
SOIL2.print(soil_read2);
SOIL_TEMP.print(soil_temp_read);
SOIL_HUMI.print(soil_humi_read);
}
}
