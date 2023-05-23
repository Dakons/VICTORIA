#include <Arduino.h>
#include "NewPing.h"

#define ECHO 6
#define TRIG 7

#define IN1 10
#define IN2 10
#define IN3 10
#define IN4 10

NewPing sonar(TRIG, ECHO, 300);

float dist_3[3] = {0.0, 0.0, 0.0};   // массив для хранения трёх последних измерений
float middle, dist, dist_filtered; // Для фильтрации
float k; //Коэффициент для бегущего среднего
byte i, delta; //счётчики
unsigned long sensTimer;

float ReadAndFilterUS ();
float middle_of_3(float a, float b, float c);
void DriveCyl(float distantion);

void setup() 
{

//pinMode(ECHO, INPUT); Обозначение работы
//pinMode(TRIG, OUTPUT);
//Пины управления драйвером
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);

 Serial.begin(9600);

}

void loop() 
{

Serial.println(ReadAndFilterUS());

DriveCyl(ReadAndFilterUS());



}
float ReadAndFilterUS ()
{
    if (millis() - sensTimer > 50) {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    if (i > 1) i = 0;
    else i++;

    dist_3[i] = (float)sonar.ping() / 57.5;                 // получить расстояние в текущую ячейку массива
    dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений

    delta = abs(dist_filtered - dist);                      // расчёт изменения с предыдущим
    if (delta > 1) k = 0.7;                                 // если большое - резкий коэффициент
    else k = 0.1;                                           // если маленькое - плавный коэффициент

    dist_filtered += (dist_filtered - dist) * k;     // фильтр "бегущее среднее"

    sensTimer = millis();                                   // сбросить таймер

    
    return dist_filtered;
  }
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

void DriveCyl(float distantion)
{
if (distantion < 100)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //Двигаем привод вперёд
}

if (ReadAndFilterUS() > 100)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //Двигаем привод назад
}
else
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  //Тормоз - стоим на месте
}
}