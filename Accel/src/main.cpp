#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include <SD.h>
const int chipSelect = 5; // Пин CS (Chip Select) для SD-карты
File dataFile;            // Объект для работы с файлом на SD-карте


#define MPU6050_ADDRESS 0x68 // Адрес MPU6050 по умолчанию

Adafruit_MPU6050 mpu;

float i;


void teleplot(const char *label, float value);

void setup() {
  Serial.begin(115200);

  if (!mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("Не удалось инициализировать MPU6050. Проверьте подключение.");
    while (1);
  }
  Serial.println("MPU6050 инициализирован успешно!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);



if (!SD.begin(chipSelect))
  {
    Serial.println("Ошибка инициализации SD-карты.");
    while (1)
    {
    delay(10);
    }
  }
Serial.println("SD-карта инициализирована успешно!");
 // Открываем файл на добавление данных
  dataFile = SD.open("/data_Accel.txt", FILE_WRITE);

  // Записываем заголовок таблицы
  dataFile.println("Time,Accel.X,Accel.Y,Gyro.X,Gyro.Y,Gyro.Z,Temp");

  // Закрываем файл
  dataFile.close();


  //Serial.println("setup");
}


void loop() 
{
  //Serial.print("loop");
  //i += 0.1;
  //Serial.println(i);
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

    // Открываем файл на добавление данных

  dataFile = SD.open("/data_Accel.txt", FILE_APPEND);

  // Записываем данные в файл

  String dataRow = String(millis()) + "," + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z) + "," + String(g.gyro.x) + "," + String(g.gyro.y) + "," + String(g.gyro.z)+ "," + String(temp.temperature);
  dataFile.println(dataRow);
  // Закрываем файл
  dataFile.close();

  delay(10);
/*
Serial.print(">Ускорение по X:");
Serial.println(a.acceleration.x);
Serial.print(">Ускорение по Y:");
Serial.println(a.acceleration.y);
Serial.print(">Ускорение по Z:");
Serial.println(a.acceleration.z);

Serial.print(">Гироскоп по X:");
Serial.println(g.gyro.x);
Serial.print(">Гироскоп по Y:");
Serial.println(g.gyro.y);
Serial.print(">Гироскоп по Z:");
Serial.println(g.gyro.z);

Serial.print(">Температура:");
Serial.println(temp.temperature);



  delay(10); // Задержка между измерениями

  */
}

void teleplot(const char *label, float value) 
{
  Serial.print(">");
  Serial.print(label);
  Serial.print(":");
  Serial.println(value);
}