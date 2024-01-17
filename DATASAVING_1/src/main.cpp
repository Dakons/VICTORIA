#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SD.h>

Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x44);

const int chipSelect = 5; // Пин CS (Chip Select) для SD-карты
File dataFile;            // Объект для работы с файлом на SD-карте
float dist, dist_filtered, dist_filtered_1, dist_filtered_2; // Для фильтрации
unsigned long sensTimer = 0;
//float dist_filtered_1, dist_filtered_2; // Для фильтрации
float k;       // Коэффициент для бегущего среднего
byte i, delta; // счётчики

float ReadAndFilterUS(float dist, byte ina219_NUM);
float convertToMillimeters(float sensorValue);

void setup()
{
  Serial.begin(115200);

  // Инициализация SD-карты
  if (!SD.begin(chipSelect))
  {
    Serial.println("Ошибка инициализации SD-карты.");
    while (1)
    {
    delay(10);
    }
    
  }
  // Открываем файл на добавление данных
  dataFile = SD.open("/data.txt", FILE_WRITE);

  // Записываем заголовок таблицы
  dataFile.println("Time,Sensor_1,Sensor_2,Sensor_1_F,Sensor_2_F");

  // Закрываем файл
  dataFile.close();

  // Инициализация датчиков тока
  Wire.begin(21, 22);
  ina219_1.setCalibration_16V_400mA();
  ina219_2.setCalibration_16V_400mA();
 
  if (!ina219_1.begin())
  {
    Serial.println("Failed to find INA219_1 chip");
    while (1)
    {
      delay(10);
    }
  }
  
  if (!ina219_2.begin())
  {
    Serial.println("Failed to find INA219_2 chip");
    while (1)
    {
      delay(10);
    }
  }
}

void loop()
{
  // Чтение и фильтрация данных с УЗ-датчиков

  float dist_1 = ina219_1.getCurrent_mA();
  float dist_2 = ina219_2.getCurrent_mA();

  
  float ddist_filtered_1 = ReadAndFilterUS(dist_1, 1);
  delay(1);

  float ddist_filtered_2 = ReadAndFilterUS(dist_2, 2);
  delay(1);


Serial.println(dist_1);
Serial.println(dist_2);

Serial.println(ddist_filtered_1);
Serial.println(ddist_filtered_2);
Serial.println("______");

  // Открываем файл на добавление данных

  dataFile = SD.open("/data.txt", FILE_APPEND);

  // Записываем данные в файл
  
  String dataRow = String(millis()) + "," + String(dist_1) + "," + String(dist_2) + "," + String(ddist_filtered_1) + "," + String(ddist_filtered_2);
  dataFile.println(dataRow);
  // Закрываем файл
  dataFile.close();

  delay(1000);
  
}


float ReadAndFilterUS(float dist, byte ina219_NUM) // ina219_1.getCurrent_mA();
{

  switch (ina219_NUM)
  {
  case 1:
    dist_filtered = dist_filtered_1;
    break;
  case 2:
    dist_filtered = dist_filtered_2;
    break;
  }

  if (millis() - sensTimer > 1)
  {
    // dist = ina219_1.getCurrent_mA();
    delta = abs(dist_filtered - dist); // расчёт изменения с предыдущим

    if (delta > 250) // если большое - резкий коэффициент
    {
      k =0.95; 
    }
    else if (delta <= 250 && delta > 75)
    {
      k = 0.8;
    }
    else if (delta <= 75 && delta > 25)
    {
      k = 0.4;//0.1;
    }
    else if (delta <= 25 && delta > 10) 
    {
      k = 0.2;
    }
        else if (delta <= 10 && delta > 5) 
    {
      k = 0.1;
    }
    else if (delta <= 5 && delta > 2)
    {
      k = 0.05;
    }
    else if (delta <= 2 && delta > 0)
    {
      k = 0.025;
    }

    // если маленькое - плавный коэффициент

    dist_filtered += (dist - dist_filtered) * k; // фильтр "бегущее среднее"

    sensTimer = millis(); // сбросить таймер
  }

  switch (ina219_NUM)
  {
  case 1:
    dist_filtered_1 = dist_filtered;
    break;
  case 2:
    dist_filtered_2 = dist_filtered;
    break;
  default:

    break;
  }
  return dist_filtered; //+0.43
}

float convertToMillimeters(float sensorValue)
{
  if (sensorValue < 380.0)
  {
    return 0.0;
  }
  else if (sensorValue >= 380.0 && sensorValue < 4.0)
  {
    sensorValue = 4.0;
  }

  if (sensorValue > 2200.0)
  {
    return 1.0;
  }
  else if (sensorValue >= 1900.0 && sensorValue <= 2200.0)
  {
    sensorValue = 1900.0;
  }

  float minSensorValue = 400.0;    // Минимальное значение с токового датчика (в мА)
  float minMillimeters = 100.0;  // Минимальное значение в миллиметрах
  float maxSensorValue = 2000.0;   // Максимальное значение с токового датчика (в мА)
  float maxMillimeters = 2000.0; // Максимальное значение в миллиметрах

  // Выполняем линейную интерполяцию
  float millimeters = ((sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxMillimeters - minMillimeters) + minMillimeters;

  millimeters = ceil(millimeters);
  
  return millimeters;
}

