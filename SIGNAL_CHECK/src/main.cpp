#include <Arduino.h>
#include <esp32-hal-timer.h>

#define ENA 2
#define ENB 15

bool timer1_flag = 0;
bool timer2_flag = 0;


volatile bool pulseState1 = HIGH;             // Состояние пина 1 (HIGH/LOW)
volatile unsigned int pulseHighDuration1 = 0; // Длительность импульса HIGH для пина 1 в миллисекундах
volatile unsigned int pulseLowDuration1 = 0;  // Длительность паузы LOW для пина 1 в миллисекундах

volatile bool pulseState2 = HIGH;             // Состояние пина 2 (HIGH/LOW)
volatile unsigned int pulseHighDuration2 = 0; // Длительность импульса HIGH для пина 2 в миллисекундах
volatile unsigned int pulseLowDuration2 = 0;  // Длительность паузы LOW для пина 2 в миллисекундах

hw_timer_t *timer1 = NULL; // Указатель на таймер 1
hw_timer_t *timer2 = NULL; // Указатель на таймер 2

void IRAM_ATTR pulseISR1()
{
  digitalWrite(ENA, pulseState1); // Устанавливаем состояние пина 1

  if (pulseState1 == HIGH)
    timerAlarmWrite(timer1, pulseHighDuration1 * 1000, true); // Запускаем таймер для паузы LOW пина 1
  else
    timerAlarmWrite(timer1, pulseLowDuration1 * 1000, true); // Запускаем таймер для импульса HIGH пина 1

  pulseState1 = !pulseState1; // Инвертируем состояние пина 1
}

void IRAM_ATTR pulseISR2()
{
  digitalWrite(ENB, pulseState2); // Устанавливаем состояние пина 2

  if (pulseState2 == HIGH)
    timerAlarmWrite(timer2, pulseHighDuration2 * 1000, true); // Запускаем таймер для паузы LOW пина 2
  else
    timerAlarmWrite(timer2, pulseLowDuration2 * 1000, true); // Запускаем таймер для импульса HIGH пина 2

  pulseState2 = !pulseState2; // Инвертируем состояние пина 2
}

void configurePulsee(unsigned int highTime, unsigned int lowTime, int Motor_Num, byte TOTAL_MODE);

void setup()
{
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  timer1 = timerBegin(0, 80, true); // Используем таймер 0 (80 предделитель)
  timer2 = timerBegin(1, 80, true); // Используем таймер 1 (80 предделитель)

  //configurePulse(10, 50, ENA, 10, 50, ENB);
  timerAlarmEnable(timer1); // Включаем таймер 1
  timerAlarmEnable(timer2); // Включаем таймер 2

  configurePulsee(100, 50, 1, 0);

  configurePulsee(1000, 50, 2, 1);
}

void loop()
{
  

  
}

void configurePulsee(unsigned int highTime, unsigned int lowTime, int Motor_Num, byte TOTAL_MODE)
{
  if (Motor_Num == 1 && TOTAL_MODE == 0)
  {
    timerDetachInterrupt(timer1);
    timer1_flag = 1;

    return;
  }
  else if (Motor_Num == 2 && TOTAL_MODE == 0)
  {
    timerDetachInterrupt(timer2);
     timer2_flag = 1;
    return;
  }

  switch (Motor_Num)
  {
  case 1:
    // Перенастройка таймера 1
    //timerAttachInterrupt(timer1, pulseISR1, true);
    
    if (timer1_flag == 0)
    {
    timerDetachInterrupt(timer1); // Отключаем прерывание от текущего таймера 1
    }
    timer1_flag = 0;
    pulseHighDuration1 = highTime;
    pulseLowDuration1 = lowTime;
    timerAlarmWrite(timer1, pulseHighDuration1 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
    //pulseState1 = 0;
    timerAttachInterrupt(timer1, pulseISR1, true);            // Привязываем прерывание к таймеру 1
    break;

  case 2:
    // Перенастройка таймера 2
    //timerAttachInterrupt(timer2, pulseISR2, true);
 if (timer2_flag == 0)
    {
    timerDetachInterrupt(timer2); // Отключаем прерывание от текущего таймера 1
    }
timer2_flag = 0;
    pulseHighDuration2 = highTime;
    pulseLowDuration2 = lowTime;
    timerAlarmWrite(timer2, pulseHighDuration2 * 1000, true); // Запускаем таймер для импульса HIGH пина 2
    //pulseState2 = 0;
    timerAttachInterrupt(timer2, pulseISR2, true);            // Привязываем прерывание к таймеру 2
    break;
  }
}