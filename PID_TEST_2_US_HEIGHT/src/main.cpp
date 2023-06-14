#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define IN1 2
#define IN2 15
#define ENA 5

int onTime, offTime, period, total_direction;



Adafruit_INA219 ina219_1;

unsigned long sensTimer = 0;




float dist_3[3] = {0.0, 0.0, 0.0};


void configurePWM(int dutyCycle);
void generatePWM(int pin);

void serialPWM();


float HIGH_VAL, LOW_VAL, Err_H, prevErr_H, P_H, I_H, D_H, PID_H; //Для пида
float Curr_1, Curr_2, Curr_3;
float middle, dist, dist_filtered; // Для фильтрации

float k; //Коэффициент для бегущего среднего
byte i, delta; //счётчики


//Adafruit_INA219 ina219_2;
float middle_of_3(float a, float b, float c);
float ReadAndFilterUS();

float convertToMillimeters(float sensorValue);

void PID_HEIGHT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL);

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
pinMode(ENA, OUTPUT);

pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);


ledcSetup(0, 10, 8);
ledcAttachPin(ENA, 0);  // Привязка пина к каналу ШИМ
ledcWrite(0, 0);

digitalWrite(IN2, HIGH);
digitalWrite(IN1, HIGH);

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

if (I_H>2000.0 || I_H<-2000.0) 
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
//Serial.println(PID);
} 

//PID = map(PID, -1000, 2000, -100, 100);
//PID = map(PID, -100, 100, 0, 100);

Serial.print(" 2 PID_H - ");
Serial.println(PID_H);

configurePWM(PID_H);

}


void configurePWM(int dutyCycle)
{

switch (total_direction)
{
case 1:

//Motor_Drive(1);


digitalWrite(IN2, LOW);
digitalWrite(IN1, HIGH);
ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));

break;

case 2:
//Motor_Drive(2);
digitalWrite(IN2, HIGH);
digitalWrite(IN1, LOW);
ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));//реверснул


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

