#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include <SD.h>
const int chipSelect = 5; // Пин CS (Chip Select) для SD-карты
File dataFile;            // Объект для работы с файлом на SD-карте


#define MPU6050_ADDRESS 0x68 // Адрес MPU6050 по умолчанию

Adafruit_MPU6050 mpu;



void setup() {
  Serial.begin(115200);

  if (!mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("Не удалось инициализировать MPU6050. Проверьте подключение.");
    while (1);
  }
  Serial.println("MPU6050 инициализирован успешно!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);




  int sdAttempts = 0;
  while (!SD.begin(chipSelect) && sdAttempts < 20) {
    Serial.println("Ошибка инициализации SD-карты. Повторная попытка...");
    delay(1000);
    sdAttempts++;
  }

  if (sdAttempts >= 20) {
    Serial.println("Не удалось инициализировать SD-карту. Проверьте подключение.");
    while (1);
  }
  
Serial.println("SD-карта инициализирована успешно!");
 // Открываем файл на добавление данных
  dataFile = SD.open("/data_Accel.txt", FILE_WRITE);

  // Записываем заголовок таблицы
  dataFile.println("Time,Accel.X,Accel.Y,Gyro.X,Gyro.Y,Gyro.Z,Temp");

  // Закрываем файл
  dataFile.close();


}


void loop() 
{
  
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
}
