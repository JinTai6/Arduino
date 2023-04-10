// 我们使用的是ESP32的开发板，用的开发软件为：Arduino
// 测量温度和湿度的是DHT11，测量土壤温度的是DS18B20传感器，测量底层土壤湿度的的：YL-69，测量表层土壤湿度的不知道是什么型号
#define BLINKER_WIFI
#include <Blinker.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTPIN 15      
#define LDRPIN 36      
#define SOILPIN1 32  
#define SOILPIN2 33
#define DS18B20_PIN 34 

#define DHTTYPE DHT11   

char auth[] = "40a629d880e0";
char ssid[] = "JinTai";
char pswd[] = "j12345678";

BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");
BlinkerNumber LIGHT("light");
BlinkerNumber SOIL1("soil1");
BlinkerNumber SOIL2("soil2");
BlinkerNumber soil_temperature("soil_temperature"); // 创建一个数据端点用于存储土壤温度

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

float humi_read = 0, temp_read = 0, light_read = 0, soil_read1 = 0, soil_read2 = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 2000; // 2 seconds
 
void heartbeat()
{
    HUMI.print(humi_read);
    TEMP.print(temp_read);
    LIGHT.print(light_read);
    SOIL1.print(soil_read1);
    SOIL2.print(soil_read2);
}

void dataStorage()
{
    Blinker.dataStorage("temp", temp_read); //数据组件名，数据值
    Blinker.dataStorage("humi", humi_read);
    Blinker.dataStorage("light",light_read);
    Blinker.dataStorage("soil1", soil_read1);
    Blinker.dataStorage("soil2", soil_read2);
    Blinker.dataStorage("soil_temperature", sensors.getTempCByIndex(0)); // 存储土壤温度数据    
}

void setup()
{
    Serial.begin(115200);
    BLINKER_DEBUG.stream(Serial);
    BLINKER_DEBUG.debugAll();
 
    Blinker.begin(auth, ssid, pswd);
    Blinker.attachHeartbeat(heartbeat);
    Blinker.attachDataStorage(dataStorage);

    dht.begin();
    sensors.begin(); //初始化DS18B20传感器

    pinMode(LDRPIN, INPUT);
    pinMode(SOILPIN1, INPUT);
    pinMode(SOILPIN2, INPUT);
}

void loop()
{
    Blinker.run();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      float h = dht.readHumidity();
      float t = dht.readTemperature();
      int l = analogRead(LDRPIN);
      int s1 = analogRead(SOILPIN1);
      int s2 = analogRead(SOILPIN2);

      if (isnan(h) || isnan(t))
      {
          BLINKER_LOG("Failed to read from DHT sensor!");
      }
      else
      {   
          BLINKER_LOG("Humidity: ", h, " %");
          BLINKER_LOG("Temperature: ", t, " *C");
          
          humi_read = h;
          temp_read = t;
      }

      // 处理土壤湿度数据
      soil_read1 = map(s1, 0, 4095, 100, 0);
      soil_read2 = map(s2, 0, 4095, 100, 0);
      float soil_avg = (soil_read1 + soil_read2) / 2;
      BLINKER_LOG("Soil Moisture: ", soil_avg, " %");

      // 处理光照强度数据
      light_read = map(l, 0, 4095, 100, 0);
      BLINKER_LOG("Light Intensity: ", light_read);

      // 处理土壤温度数据
      sensors.requestTemperatures(); // 请求传感器进行测量
      float soil_temp = sensors.getTempCByIndex(0); // 获取土壤温度

      if (soil_temp == -127.00) {
        BLINKER_LOG("Failed to read from DS18B20 sensor!");
      } else {
        BLINKER_LOG("Soil Temperature: ", soil_temp, " C");
        soil_temperature.print(soil_temp); // 将数据发送到数据端点
        Blinker.dataStorage("soil_temperature", soil_temp); // 将数据存储到Blinker平台上的数据存储区域
}
}

      // 根据光照强度的大小动态调整读取传感器数据的时间间隔
      if (light_read > 70)
      {
      Blinker.delay(500);
      }
      else
      {
      Blinker.delay(2000);
      }
      }
