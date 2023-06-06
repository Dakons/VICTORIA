#include <Arduino.h>



#define IN1 2
#define IN2 15

int onTime, offTime, period, total_direction;



void configurePWM(int frequency, int dutyCycle);
void generatePWM(int pin);

void serialPWM();

void Motor_Drive(int direction);

void setup() 

{
Serial.begin(115200);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
Motor_Drive(1);
//configurePWM(4, 90);

ledcSetup(0, 10, 8);
ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ

ledcWrite(0, map(50, 0, 100, 255, 100));


}

void loop() 

{
 serialPWM();
 //generatePWM(IN1);



}

/*
void configurePWM(int frequency, int dutyCycle) 
{
int period = 1000 / frequency; // Период ШИМ (мс)
onTime = period * dutyCycle / 100; // Время активного уровня ШИМ (мс)
offTime = period - onTime; // Время неактивного уровня ШИМ (мс)


switch (total_direction)
{
case 1:
  break;

case 2:
int c = onTime;
onTime = offTime;
offTime = c;
  break;
}
*/

void configurePWM(int frequency, int dutyCycle)
{

ledcDetachPin(IN1);
ledcSetup(0, frequency, 8);
ledcAttachPin(IN1, 0);  // Привязка пина к каналу ШИМ

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


void generatePWM(int pin)
{
  digitalWrite(pin, HIGH); // Включение ШИМ-сигнала
  delay(onTime); // Задержка для активного уровня ШИМ

  digitalWrite(pin, LOW); // Выключение ШИМ-сигнала
  delay(offTime); // Задержка для неактивного уровня ШИМ
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
 switch (direction)
 {
 case 1: //Если вперёд
  digitalWrite(IN2, LOW);
  //Настройка pwm
  break;
 
 case 2://Если назад
 digitalWrite(IN2, HIGH); 
 //Настройка pwm
  break;

case 3: //Тормоз
ledcDetachPin(IN1);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
 }
}