#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define IN1 2
#define IN2 15

int onTime, offTime, period, total_direction;



Adafruit_INA219 ina219_1;

unsigned long sensTimer = 0;




float dist_3[3] = {0.0, 0.0, 0.0};


void configurePWM(int frequency, int dutyCycle);
void generatePWM(int pin);

void serialPWM();

void Motor_Drive(int direction);


float HIGH_VAL, LOW_VAL, Err, prevErr, P_H, I_H, D_H, PID_H; //Для пида
float Curr_1, Curr_2, Curr_3;
float middle, dist, dist_filtered; // Для фильтрации

float k; //Коэффициент для бегущего среднего
byte i, delta; //счётчики


//Adafruit_INA219 ina219_2;
float middle_of_3(float a, float b, float c);
float ReadAndFilterUS();

float convertToMillimeters(float sensorValue);

void PID_HEIGHT(float VAL_LEFT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL);

void setup() 
{

/* 
Wire.begin(21, 22);

//Serial.println("clean, median");  
//ina219_1.setCalibration_32V_2A();
ina219_1.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    //Serial.println("Failed to find INA219 chip");  
    while (1) { delay(10); }
  }
//ina219_2.begin();
*/
Serial.begin(115200);


pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
Motor_Drive(3);

//configurePWM(4, 90);


ledcSetup(0, 10, 8);
ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ

ledcWrite(0, 0);
//ledcDetachPin(IN1);

P_H= 0.0;
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

}

float ReadAndFilterUS()
{
  if (millis() - sensTimer > 1)
  {                          // измерение и вывод каждые 50 мс
    dist = ina219_1.getCurrent_mA();
    
    delta = abs(dist_filtered - dist);                      // расчёт изменения с предыдущим
    if (delta > 1)  // если большое - резкий коэффициент
    {
      k = 0.95;
    } 
    else if (delta <= 1 && delta > 0.3)
    {
      k = 0.8;
    }     
    else if (delta <= 0.3 && delta >=0.1)
    {
      k = 0.1;
    }
      else if (delta <= 0.1)
    {
      k = 0.05;
    }
                                   // если маленькое - плавный коэффициент

    dist_filtered += ( dist -dist_filtered) * k;     // фильтр "бегущее среднее"
    
    sensTimer = millis();                                   // сбросить таймер

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


  float minSensorValue = 4.0;  // Минимальное значение с токового датчика (в мА)
  float minMillimeters = 100.0;  // Минимальное значение в миллиметрах
  float maxSensorValue = 20.0;  // Максимальное значение с токового датчика (в мА)
  float maxMillimeters = 2000.0;  // Максимальное значение в миллиметрах

  // Выполняем линейную интерполяцию
  float millimeters = ((sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxMillimeters - minMillimeters) + minMillimeters;

  return millimeters;
}


void PID_HEIGHT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL)

{

Err = VAL_LEFT + VAL_RIGHT;
Err /= 2;

if (Err > HIGH_VAL)
{
  Err = Err - HIGH_VAL;
  
}
else if (Err < LOW_VAL)
{
  Err = Err - LOW_VAL;
}
else
{
Err = 0;
} 

P_H = Err * Kp;
I_H = I_H + Err * Ki;
D_H = (Err - prevErr) * Kd;

if (I_H>2000.0 || I_H<-2000.0) 
{
  I_H = 0;
}

PID_H = P_H + I_H + D_H;

Serial.print(" 1 PID_H - ");
Serial.println(PID_H);

if (PID_H < 0)
{
Motor_Drive(2);
PID_H = map(PID_H, 0, -2500, 0, 100);

PID_H = constrain(PID_H, 0, 100);
}
else if (PID_H > 0)
{
Motor_Drive(1);
PID_H = map(PID_H, 0, 2500, 0, 100);

PID_H = constrain(PID_H, 0, 100);

}
else
{
Motor_Drive(3);
//Serial.println(PID);
} 

//PID = map(PID, -1000, 2000, -100, 100);
//PID = map(PID, -100, 100, 0, 100);

Serial.print(" 2 PID_H - ");
Serial.println(PID_H);

configurePWM(10, PID_H);

}


void configurePWM(int frequency, int dutyCycle)
{

ledcDetachPin(IN1);
ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ
ledcSetup(0, frequency, 8);
Serial.print("DC -");
Serial.println(dutyCycle);
Serial.print("dir -");
Serial.println(total_direction);

switch (total_direction)
{
case 1:
ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));
  break;

case 2:
ledcWrite(0, map(dutyCycle, 0, 100, 255, 0));
  break;
case 3:
//ledcDetachPin(IN1);
break;
}
  
}


void Motor_Drive(int direction)
{
total_direction = direction;
//Serial.println(direction);
 switch (direction)
 {
 case 1: //Если вперёд
  digitalWrite(IN2, LOW);
  //digitalWrite(IN1, HIGH);
  //Настройка pwm
  break;
 
 case 2://Если назад
 digitalWrite(IN2, HIGH);
 //digitalWrite(IN1, LOW); 
 //Настройка pwm
  break;

case 3: //Тормоз
ledcDetachPin(IN1);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
 }
}