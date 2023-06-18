#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define IN1 2
#define IN2 15
#define ENA 5

Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x44);

unsigned long sensTimer = 0;
float dist, dist_filtered, dist_filtered_1, dist_filtered_2; // Для фильтрации
float dist_1, dist_2;
float k;       // Коэффициент для бегущего среднего
byte i, delta; // счётчики

int total_direction;                                             // направление
float HIGH_VAL, LOW_VAL, Err_H, prevErr_H, P_H, I_H, D_H, PID_H; // Для пида

float ReadAndFilterUS(float dist, byte ina219_NUM);
float convertToMillimeters(float sensorValue);
void configurePWM(int dutyCycle);
void serialPWM();
void PID_HEIGHT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL);

void setup()
{
  Serial.begin(115200);
  ////Подключение датчиков тока
  
    Wire.begin(21, 22);
    //Serial.begin(115200);
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
  
  //////////

  // Настраиваем пины для драйвера
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // pinMode(IN3, OUTPUT);
  // pinMode(IN4, OUTPUT);
  // pinMode(ENB, OUTPUT);

  // настраиваем шим
  ledcSetup(0, 10, 8);
  ledcAttachPin(ENA, 0); // Привязка пина к каналу ШИМ
  ledcWrite(0, 0);

  // Ставим в тормоз
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, HIGH);

  // обнуляем коэффициенты
  P_H = 0.0;
  I_H = 0.0;
  D_H = 0.0;
  PID_H = 0.0;
}

void loop()

{

  /*
  CRIT_CHECK();
  PID_HEIGHT();
  PID_TILT();
  */
  /*Проверка работы ПИД
  PID_HEIGHT(600, 400, 1, 0, 0, 600, 400);
  Serial.println("---");
  delay(5000);

  PID_HEIGHT(1000, 400, 1, 0, 0, 600, 400);
  Serial.println("---");
  delay(5000);

  PID_HEIGHT(2000, 400, 1, 0, 0, 600, 400);
  Serial.println("---");
  delay(5000);

  PID_HEIGHT(-2000, -1000, 1, 0, 0, 600, 400);
  Serial.println("---");
  delay(5000);
  */

  // Вывод инфы с датчиков тока
  /*
    float dist_1 = convertToMillimeters(ReadAndFilterUS(ina219_1.getCurrent_mA(), 1));
    float dist_2 = convertToMillimeters(ReadAndFilterUS(ina219_2.getCurrent_mA(), 2));

    Serial.print(dist_1);
    Serial.print(',');
    Serial.print(dist_2);
    Serial.println();
  */

  dist_1 = convertToMillimeters(ReadAndFilterUS(ina219_1.getCurrent_mA(), 1));
  delay(1);
  dist_2 = convertToMillimeters(ReadAndFilterUS(ina219_2.getCurrent_mA(), 2));
  delay(1);

  Serial.print(dist_1);
  Serial.print(',');
  Serial.print(dist_2);
  Serial.println();
  PID_HEIGHT(dist_1, dist_2, 1, 0, 0, 700, 500);
  Serial.println("---");
  delay(1000);

//Serial.println("---");

  //delay(2000);
/* 
  PID_HEIGHT(dist_1, dist_2, 1, 0, 0, 500, 300);
  
 
  Serial.print(dist_1);
  Serial.print(',');
  Serial.print(dist_2);
  Serial.println();

*/
}

float ReadAndFilterUS(float dist, byte ina219_NUM) // ina219_1.getCurrent_mA();
{

  switch (ina219_NUM)
  {
  case 1:
    dist_filtered = dist_filtered_1;
    //Serial.println();
    //Serial.print("Case 1, dist_filtered = ");
    //Serial.println(dist_filtered);
    //delay(1000);
    break;
  case 2:
    dist_filtered = dist_filtered_2;
    //Serial.print("Case 2, dist_filtered = ");
    //Serial.println(dist_filtered);
    //delay(1000);
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
    //Serial.print("Case 1, dist_filtered_1 = ");
    //Serial.println(dist_filtered_1);
    //delay(1000);

    break;
  case 2:
    dist_filtered_2 = dist_filtered;
    //Serial.print("Case 2, dist_filtered_2 = ");
    //Serial.println(dist_filtered_2);
    //delay(1000);
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

void PID_HEIGHT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL)

{

  Err_H = VAL_LEFT + VAL_RIGHT;
  Err_H /= 2;

  if (Err_H > HIGH_VAL)
  {
    Err_H = Err_H - HIGH_VAL;
  }
  else if (Err_H < LOW_VAL)
  {
    Err_H = Err_H - LOW_VAL;
  }
  else
  {
    Err_H = 0;
  }

  P_H = Err_H * Kp;
  I_H = I_H + Err_H * Ki;
  D_H = (Err_H - prevErr_H) * Kd;

  if (I_H > 2000.0 || I_H < -2000.0)
  {
    I_H = 0;
  }

  PID_H = P_H + I_H + D_H;

  Serial.print(" 1 PID_H - ");
  Serial.println(PID_H);

  if (PID_H < 0)
  {
    total_direction = 1;
    PID_H = map(PID_H, 0, -2500, 0, 100);

    PID_H = constrain(PID_H, 0, 100);
  }
  else if (PID_H > 0)
  {
    total_direction = 2;
    PID_H = map(PID_H, 0, 2500, 0, 100);

    PID_H = constrain(PID_H, 0, 100);
  }
  else
  {
    total_direction = 3;
    // Serial.println(PID);
  }

  // PID = map(PID, -1000, 2000, -100, 100);
  // PID = map(PID, -100, 100, 0, 100);

  Serial.print(" 2 PID_H - ");
  Serial.println(PID_H);

  configurePWM(PID_H);
}

void configurePWM(int dutyCycle)
{

  switch (total_direction)
  {
  case 1:

    // Motor_Drive(1);

    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));

    break;

  case 2:
    // Motor_Drive(2);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
    ledcWrite(0, map(dutyCycle, 0, 100, 0, 255)); // реверснул

    break;
  case 3:

    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, HIGH);
    ledcWrite(0, 0);

    break;
  }

  Serial.print("DC -");
  Serial.println(dutyCycle);
  Serial.print("dir -");
  Serial.println(total_direction);
}
