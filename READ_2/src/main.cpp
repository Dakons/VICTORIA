#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x44);

unsigned long sensTimer = 0;

float dist, dist_filtered, dist_filtered_1, dist_filtered_2; // Для фильтрации

float k;       // Коэффициент для бегущего среднего
byte i, delta; // счётчики


float ReadAndFilterUS(float dist, byte ina219_NUM);

float convertToMillimeters(float sensorValue);

void setup()
{
  Wire.begin(21, 22);
  Serial.begin(115200);
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
  Serial.println("US_1, US_2");
}

void loop()

{
  float dist_1 = convertToMillimeters(ReadAndFilterUS(ina219_1.getCurrent_mA(), 1));
  float dist_2 = convertToMillimeters(ReadAndFilterUS(ina219_2.getCurrent_mA(), 2));

  Serial.print(dist_1);
  Serial.print(',');
  Serial.print(dist_2);
  Serial.println();
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

    if (delta > 1) // если большое - резкий коэффициент
    {
      k = 0.95;
    }
    else if (delta <= 1 && delta > 0.3)
    {
      k = 0.8;
    }
    else if (delta <= 0.3 && delta >= 0.1)
    {
      k = 0.1;
    }
    else if (delta <= 0.1)
    {
      k = 0.05;
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
  return dist_filtered + 0.35; //+0.43
}

float convertToMillimeters(float sensorValue)
{
  if (sensorValue < 3.7)
  {
    return 0.0;
  }
  else if (sensorValue >= 3.7 && sensorValue < 4.0)
  {
    sensorValue = 4.0;
  }

  if (sensorValue > 22.0)
  {
    return 1.0;
  }
  else if (sensorValue >= 20.0 && sensorValue <= 22.0)
  {
    sensorValue = 20.0;
  }

  float minSensorValue = 4.0;    // Минимальное значение с токового датчика (в мА)
  float minMillimeters = 100.0;  // Минимальное значение в миллиметрах
  float maxSensorValue = 20.0;   // Максимальное значение с токового датчика (в мА)
  float maxMillimeters = 2000.0; // Максимальное значение в миллиметрах

  // Выполняем линейную интерполяцию
  float millimeters = ((sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxMillimeters - minMillimeters) + minMillimeters;

  return millimeters;
}