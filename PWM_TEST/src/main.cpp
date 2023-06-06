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


}

void loop() 

{
 serialPWM();
 generatePWM(IN1);



}


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

}

void generatePWM(int pin)
{
  digitalWrite(pin, HIGH); // Включение ШИМ-сигнала
  delay(onTime); // Задержка для активного уровня ШИМ

  digitalWrite(pin, LOW); // Выключение ШИМ-сигнала
  delay(offTime); // Задержка для неактивного уровня ШИМ
}

void serialPWM() 
{
  if (Serial.available() >= 7) { // Изменено условие для чтения 3-х цифр и 2-х пробелов
    int mode = Serial.parseInt();
    char delimiter = Serial.read(); // Читаем первый пробел
    int frequency = Serial.parseInt();
    delimiter = Serial.read(); // Читаем второй пробел
    int dutyCycle = Serial.parseInt();
    if (delimiter == ' ' && mode >= 1 && mode <= 3 && frequency && dutyCycle >= 0 && dutyCycle <= 100) {
      Motor_Drive(mode); // Устанавливаем режим работы мотора
      configurePWM(frequency, dutyCycle);
      Serial.print("Mode: ");
      Serial.print(mode);
      Serial.print(", Frequency: ");
      Serial.print(frequency);
      Serial.print(" Hz, Duty Cycle: ");
      Serial.print(dutyCycle);
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
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
 }
}