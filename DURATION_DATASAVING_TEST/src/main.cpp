#include <SPI.h>
#include <SD.h>

const int chipSelect = 5; // Пин CS (Chip Select) для SD-карты
File dataFile;            // Объект для работы с файлом на SD-карте

float getData();

void setup()
{
  // Инициализация SD-карты
  Serial.begin(115200);
  if (!SD.begin(chipSelect))
  {
    Serial.println("Ошибка инициализации SD-карты.");
    while (1)
      ;
  }
  
  // Открываем файл на добавление данных
  dataFile = SD.open("/data.txt", FILE_APPEND);

  // Записываем начало сессии в файл
  dataFile.println("----- Начало сессии -----");

  // Закрываем файл
  dataFile.close();
}

void loop()
{
    // Получение данных с датчика (замените это на вашу функцию получения данных)
  float sensorData = getData();
  
  

  
  // Открываем файл на добавление данных
  dataFile = SD.open("/data.txt", FILE_APPEND);
  unsigned long startTime = micros();  // Запоминаем время начала сохранения
  // Получение текущего времени в миллисекундах
  //unsigned long currentTime = millis();
  
  // Записываем данные и время в файл
  dataFile.print("Time: ");
  dataFile.print(startTime);
  dataFile.print(", Data: ");
  dataFile.println(sensorData, 4);  // Округляем до 4 знаков после запятой
  unsigned long endTime = micros();  // Запоминаем время окончания сохранения
  // Закрываем файл
  dataFile.close();
  
 // unsigned long endTime = millis();  // Запоминаем время окончания сохранения
  
  // Вычисляем длительность сохранения
  unsigned long saveDuration = endTime - startTime;
  
  Serial.print("Сохранение заняло: ");
  Serial.print(saveDuration);
  Serial.println(" мkс");
  
  // Задержка между сохранениями (например, каждую секунду)
  delay(1000);
}

// Функция для получения данных с датчика (замените её на вашу реализацию)
float getData() {
  // Ваш код для получения данных с датчика
  // Возвращайте полученные данные
  return 0.0048828125;  // Пример: чтение аналогового пина A0 и преобразование в напряжение (0-5V)
}
