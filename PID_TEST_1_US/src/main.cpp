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


float HIGH_VAL, LOW_VAL, Err, prevErr, P, I, D, PID;
float Curr_1, Curr_2, Curr_3;
float middle, dist, dist_filtered; // Для фильтрации

float k; //Коэффициент для бегущего среднего
byte i, delta; //счётчики


//Adafruit_INA219 ina219_2;
float middle_of_3(float a, float b, float c);
float ReadAndFilterUS();

float convertToMillimeters(float sensorValue);

void PID_MOVE(float value, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL);

void setup() 
{
Wire.begin(21, 22);
Serial.begin(115200);
//Serial.println("clean, median");  
//ina219_1.setCalibration_32V_2A();
ina219_1.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    //Serial.println("Failed to find INA219 chip");  
    while (1) { delay(10); }
  }
//ina219_2.begin();


pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
Motor_Drive(1);

//configurePWM(4, 90);


ledcSetup(0, 10, 8);
ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ

ledcWrite(0, 0);
//ledcDetachPin(IN1);
}



void loop() 

{
//Serial.println(convertToMillimeters(ReadAndFilterUS()));
//delay(80);
//Curr_1 = ReadAndFilterUS();
//Serial.println(Curr_1);
//Serial.println(ina219_1.getCurrent_mA());

/*
Serial.print(ina219_1.getCurrent_mA());
Serial.print(',');
Curr_1 = ReadAndFilterUS();
Serial.print(Curr_1);
Serial.println();
*/
PID_MOVE(convertToMillimeters(ReadAndFilterUS()), 1, 0, 0.5, 600, 400);

}

float middle_of_3(float a, float b, float c) 
{
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}


float ReadAndFilterUS()
{
  if (millis() - sensTimer > 1)
  {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    //if (i > 1) i = 0;
    //else i++;

    //dist_3[i] = ina219_1.getCurrent_mA();                 // получить расстояние в текущую ячейку массива
    //dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений
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


void PID_MOVE(float value, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL)

{



if (value > HIGH_VAL)
{
  Err = value - HIGH_VAL;
  
}
else if (value < LOW_VAL)
{
  Err = value - LOW_VAL;
}
else Err = 0;

P = Err * Kp;
I = I + Err * Ki;
D = (Err - prevErr) * Kd;


PID = P + I + D;

if (I>2000.0 || I<-2000.0) I = 0;


if (PID < 0)
{
Motor_Drive(2);
PID = map(PID, -100, 0, 0, 100);

PID = constrain(PID, 0, 100);
//Serial.println(-PID);
}
else if (PID > 0)
{
Motor_Drive(1);
PID = map(PID, 0, 2500, 0, 100);

PID = constrain(PID, 0, 100);
//Serial.println(PID);
}
else
{
Motor_Drive(3);
//Serial.println(PID);
} 

//PID = map(PID, -1000, 2000, -100, 100);
//PID = map(PID, -100, 100, 0, 100);


//configurePWM(10, PID);

}


void configurePWM(int frequency, int dutyCycle)
{

//ledcDetachPin(IN1);
//ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ
//ledcSetup(0, frequency, 8);

Serial.println(dutyCycle);

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


/*
void serialPWM()
{
 
  if (Serial.available() >= 3)
   {
    int frequency = Serial.parseInt();
    if (Serial.read() == ' ') 
    {
      int dutyCycle = Serial.parseInt();
      if (frequency && dutyCycle >= 0 && dutyCycle <= 100)
       {
        configurePWM(frequency, dutyCycle);
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.print(" Hz, Duty Cycle: ");
        Serial.print(dutyCycle);
        Serial.println("%");
      }
    }
  }
}
*/
void serialPWM() 
{
  if (Serial.available() >= 5)
  {
    int mode = Serial.parseInt();
    char delimiter = Serial.read();
    int freq = Serial.parseInt();
    delimiter = Serial.read();
    int dc = Serial.parseInt();

    if (delimiter == ' ' && mode >= 1 && mode <= 3 && freq >= 1 && freq <= 10000 && dc >= 0 && dc <= 100)
    {
      Motor_Drive(mode);
      configurePWM(freq, dc);
      Motor_Drive(mode);
     
      Serial.println();
      Serial.print("Mode: ");
      Serial.print(mode);
      Serial.print(", Frequency: ");
      Serial.print(freq);
      Serial.print(" Hz, Duty Cycle: ");
      Serial.print(dc);
      Serial.println("%");
    }
  }
}



void Motor_Drive(int direction)
{
total_direction = direction;
Serial.println(direction);
 switch (direction)
 {
 case 1: //Если вперёд
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  //Настройка pwm
  break;
 
 case 2://Если назад
 digitalWrite(IN2, HIGH);
 digitalWrite(IN1, LOW); 
 //Настройка pwm
  break;

case 3: //Тормоз
ledcDetachPin(IN1);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
 }
}