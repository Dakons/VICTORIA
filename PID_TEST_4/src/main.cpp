#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define IN1 2
#define IN2 4
#define ENA 15

#define IN3 5
#define IN4 18
#define ENB 19

#define ENC 23

float j;

Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x44);

unsigned long TimeMoment_1, TimeMoment_2, TimeMoment_3, TimeMoment_4;

unsigned long sensTimer = 0;
float dist, dist_filtered, dist_filtered_1, dist_filtered_2; // Для фильтрации

float k;       // Коэффициент для бегущего среднего
byte i, delta; // счётчики

int total_direction;                          // направление
float Err_T, prevErr_T, P_T, I_T, D_T, PID_T, Speed_T, prevSpeed_T; // Для пида наклона
float Err_H, prevErr_H, P_H, I_H, D_H, PID_H; // Для пида высоты
float dist_1, dist_2;
/////// прерывания

volatile bool FLAG_WORK_1 = 0;
volatile bool FLAG_WORK_2 = 0;

bool timer1_flag = 0;
bool timer2_flag = 0;

volatile bool pulseState1 = HIGH;                // Состояние пина 1 (HIGH/LOW)
volatile unsigned int pulseHighDuration1 = 0;    // Длительность импульса HIGH для пина 1 в миллисекундах
volatile unsigned int pulseLowDuration1 = 0;     // Длительность паузы LOW для пина 1 в миллисекундах
volatile unsigned int NEWpulseHighDuration1 = 0; // Длительность импульса HIGH для пина 1 в миллисекундах
volatile unsigned int NEWpulseLowDuration1 = 0;  // Длительность паузы LOW для пина 1 в миллисекундах

volatile bool pulseState2 = HIGH;                // Состояние пина 2 (HIGH/LOW)
volatile unsigned int pulseHighDuration2 = 0;    // Длительность импульса HIGH для пина 2 в миллисекундах
volatile unsigned int pulseLowDuration2 = 0;     // Длительность паузы LOW для пина 2 в миллисекундах
volatile unsigned int NEWpulseHighDuration2 = 0; // Длительность импульса HIGH для пина 2 в миллисекундах
volatile unsigned int NEWpulseLowDuration2 = 0;  // Длительность паузы LOW для пина 2 в миллисекундах

hw_timer_t *timer1 = NULL; // Указатель на таймер 1
hw_timer_t *timer2 = NULL; // Указатель на таймер 2

void CheckFlag()
{
  bool enaState = digitalRead(ENA);
  bool enbState = digitalRead(ENB);

  if (enaState == HIGH || enbState == HIGH)
  {
    digitalWrite(ENC, HIGH);
  }
  else
  {
    digitalWrite(ENC, LOW);
  }
}

void IRAM_ATTR pulseISR1()
{
  // Устанавливаем состояние пина 1
  // Serial.print("COME");
  if (pulseState1 == HIGH)
  {
    digitalWrite(ENA, pulseState1);
    // Serial.println(" UP");
    timerAlarmWrite(timer1, pulseHighDuration1 * 1000, true); // Запускаем таймер для паузы LOW пина 1
    pulseState1 = !pulseState1;
  }
  else
  {
    digitalWrite(ENA, pulseState1);
    // Serial.println(" LOW");
    timerAlarmWrite(timer1, pulseLowDuration1 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
    pulseState1 = !pulseState1;
  }
  if (FLAG_WORK_1)
  {
    FLAG_WORK_1 = 0;
    pulseHighDuration1 = NEWpulseHighDuration1;
    pulseLowDuration1 = NEWpulseLowDuration1;
  }

  // pulseState1 = !pulseState1; // Инвертируем состояние пина 1
  CheckFlag();
}

void IRAM_ATTR pulseISR2()
{

  if (pulseState2 == HIGH)
  {
    digitalWrite(ENB, pulseState2);
    timerAlarmWrite(timer2, pulseHighDuration2 * 1000, true); // Запускаем таймер для паузы LOW пина 2
    pulseState2 = !pulseState2;
  }
  else
  {
    digitalWrite(ENB, pulseState2);
    timerAlarmWrite(timer2, pulseLowDuration2 * 1000, true); // Запускаем таймер для импульса HIGH пина 2
    pulseState2 = !pulseState2;
  }
  if (FLAG_WORK_2)
  {
    FLAG_WORK_2 = 0;
    pulseHighDuration2 = NEWpulseHighDuration2;
    pulseLowDuration2 = NEWpulseLowDuration2;
  }

  // pulseState2 = !pulseState2; // Инвертируем состояние пина 2

  CheckFlag();
}

void configurePulsee(unsigned int highTime, unsigned int lowTime, int Motor_Num, byte TOTAL_MODE);
// void CheckFlag();

///////
float ReadAndFilterUS(float dist, byte ina219_NUM);
float convertToMillimeters(float sensorValue);
void configurePWM(int dutyCycle, byte Motor_NUM, int high_impulse);
void serialPWM();
void PID_TILT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float Ks, float HIGH_VAL, float LOW_VAL);
void PID_HEIGHT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float HIGH_VAL, float LOW_VAL);
void TOTAL_CHECK();

unsigned int counter = 0;

bool TOTAL_FLAG_MOTOR1 = 0;

bool TOTAL_FLAG_MOTOR2 = 0;

void setup()
{
  Serial.begin(115200);
  // timer1 = timerBegin(0, 80, true); // Используем таймер 0 (80 предделитель) !!!!!
  // timer2 = timerBegin(2, 80, true); // Используем таймер 1 (80 предделитель)

  // configurePulse(10, 50, ENA, 10, 50, ENB);
  // timerAlarmEnable(timer1); // Включаем таймер 1 !!!!!!!11

  // timerAlarmEnable(timer2); // Включаем таймер 2
  // timerAlarmWrite(timer1, 1000 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
  // timerAlarmWrite(timer1, 100 * 1000, true);
  // timerAttachInterrupt(timer1, pulseISR1, true); // Привязываем прерывание к таймеру 1

  ////Подключение датчиков тока
  ///*
  Wire.begin(21, 22);
  // Serial.begin(115200);
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
  //Serial.println("US_1, US_2");
  //*/
  //////////

  // Настраиваем пины для драйвера
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ENC, OUTPUT);

  // настраиваем шим
  /*
    ledcSetup(0, 1000, 8);
    ledcAttachPin(ENA, 0); // Привязка пина к каналу ШИМ
    ledcWrite(0, 0);

    ledcSetup(1, 1000, 8);
    ledcAttachPin(ENB, 1); // Привязка пина к каналу ШИМ
    ledcWrite(1, 0);
  */

  // Ставим в тормоз
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  // обнуляем коэффициенты
  P_T = 0.0;
  I_T = 0.0;
  D_T = 0.0;
  PID_T = 0.0;

  // PID_TILT(1000, 500, 1, 0, 0, 200, -200);
  // PID_HEIGHT(2500, 500, 1, 0, 0, 700, 500);

  pulseHighDuration1 = 50;
  pulseLowDuration1 = 50;
  pulseHighDuration2 = 50;
  pulseLowDuration2 = 50;

  timer1 = timerBegin(0, 80, true);
  timerAlarmWrite(timer1, 1000 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
  timerAttachInterrupt(timer1, pulseISR1, true);
  timerAlarmEnable(timer1);
  configurePulsee(50, 100, 1, 0);

  timer2 = timerBegin(1, 80, true);
  timerAlarmWrite(timer2, 1000 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
  timerAttachInterrupt(timer2, pulseISR2, true);
  timerAlarmEnable(timer2);
  configurePulsee(50, 100, 2, 0);
  Serial.println("setup");

}

void loop()

{
 j+=0.01;


  Serial.print("loop");
  Serial.println(j);

float dist_1 = convertToMillimeters(ReadAndFilterUS(ina219_1.getCurrent_mA(), 1));
  delay(1);
float dist_2 = convertToMillimeters(ReadAndFilterUS(ina219_2.getCurrent_mA(), 2));
  delay(1);


  Serial.print(">dist_1:");
  Serial.println(dist_1);

  Serial.print(">dist_2:");
  Serial.println(dist_2);

Serial.print(">Current_1:");
  Serial.println(ina219_1.getCurrent_mA());

Serial.print(">Current_2:");
  Serial.println(ina219_2.getCurrent_mA());



  //Serial.print(">cos:");
  //Serial.println(cos(j));
/*
  float dist_1 = convertToMillimeters(ReadAndFilterUS(ina219_1.getCurrent_mA(), 1));
  delay(1);
  float dist_2 = convertToMillimeters(ReadAndFilterUS(ina219_2.getCurrent_mA(), 2));
  delay(1);

  PID_TILT(dist_1, dist_2, 0.1, 0, 1.75, -150, 100, -100); // 2.85 d 1 75 //2 P I -01   0.25, -0.01, 1.75, 100, -100

  PID_HEIGHT(dist_1, dist_2, 40, 0, 0, 1200, 1000);
*/
}

float ReadAndFilterUS(float dist, byte ina219_NUM) // ina219_1.getCurrent_mA();
{

  switch (ina219_NUM)
  {
  case 1:
    dist_filtered = dist_filtered_1;
    break;
  case 2:
    dist_filtered = dist_filtered_2;
    break;
  }

  if (millis() - sensTimer > 1)
  {
    // dist = ina219_1.getCurrent_mA();
    delta = abs(dist_filtered - dist); // расчёт изменения с предыдущим

    if (delta > 250) // если большое - резкий коэффициент
    {
      k =0.95; 
    }
    else if (delta <= 250 && delta > 75)
    {
      k = 0.8;
    }
    else if (delta <= 75 && delta > 25)
    {
      k = 0.4;//0.1;
    }
    else if (delta <= 25 && delta > 10) 
    {
      k = 0.2;
    }
        else if (delta <= 10 && delta > 5) 
    {
      k = 0.1;
    }
    else if (delta <= 5 && delta > 2)
    {
      k = 0.05;
    }
    else if (delta <= 2 && delta > 0)
    {
      k = 0.025;
    }

    // если маленькое - плавный коэффициент

    dist_filtered += (dist - dist_filtered) * k; // фильтр "бегущее среднее"

    sensTimer = millis(); // сбросить таймер
  }

  switch (ina219_NUM)
  {
  case 1:
    dist_filtered_1 = dist_filtered;
    break;
  case 2:
    dist_filtered_2 = dist_filtered;
    break;
  default:

    break;
  }
  return dist_filtered; //+0.43
}

float convertToMillimeters(float sensorValue)
{
  if (sensorValue < 380.0)
  {
    return 0.0;
  }
  else if (sensorValue >= 380.0 && sensorValue < 4.0)
  {
    sensorValue = 4.0;
  }

  if (sensorValue > 2200.0)
  {
    return 1.0;
  }
  else if (sensorValue >= 1900.0 && sensorValue <= 2200.0)
  {
    sensorValue = 1900.0;
  }

  float minSensorValue = 400.0;    // Минимальное значение с токового датчика (в мА)
  float minMillimeters = 100.0;  // Минимальное значение в миллиметрах
  float maxSensorValue = 2000.0;   // Максимальное значение с токового датчика (в мА)
  float maxMillimeters = 2000.0; // Максимальное значение в миллиметрах

  // Выполняем линейную интерполяцию
  float millimeters = ((sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxMillimeters - minMillimeters) + minMillimeters;

  millimeters = ceil(millimeters);
  
  return millimeters;
}

void PID_TILT(float VAL_LEFT, float VAL_RIGHT, float Kp, float Ki, float Kd, float Ks, float HIGH_VAL, float LOW_VAL)

{

  Err_T = VAL_LEFT - VAL_RIGHT;

  if (Err_T > HIGH_VAL)
  {
    Err_T = Err_T - HIGH_VAL;
  }
  else if (Err_T < LOW_VAL)
  {
    Err_T = Err_T - LOW_VAL; //!!!
  }
  else
  {
    Err_T = 0;
  }
  //Serial.print(Err_T);
  //Serial.print(",");
// адаптация

  // Serial.print("Err_T =");
  // Serial.println(Err_T);

  Speed_T = (Err_T-prevErr_T)-(Err_T*Ks);

  P_T = Speed_T * Kp;
  I_T = I_T + Speed_T * Ki;
  D_T = (Speed_T - prevSpeed_T) * Kd;
prevSpeed_T = Speed_T;
prevErr_T = Err_T;
  I_T = constrain(I_T, -1000.0, 1000.0);

  if (I_T >= 999.0 || I_T <= -999.0)
  {

   // I_T *= 0.1;
  }

  if (Err_T == 0)
  {
    counter++;
  }
  else
    counter = 0;

  if (counter >= 15)
  {
    I_T = 0;
    counter = 0;
  }


  PID_T = P_T + I_T + D_T;

  // Serial.print(" 1 PID_T - ");
  // Serial.println(I_T);

  if (PID_T < 0)
  {
    total_direction = 1;
    PID_T = map(PID_T, 0, -2500, 0, 100);

    PID_T = constrain(PID_T, 0, 100);
  }
  else if (PID_T > 0)
  {
    total_direction = 2;
    PID_T = map(PID_T, 0, 2500, 0, 100);

    PID_T = constrain(PID_T, 0, 100);
  }
  else
  {
    total_direction = 3;
    // Serial.println(PID);
  }


  if (total_direction == 1)
  {
    configurePWM(PID_T, 1, 50); //60
  
  }

  if (total_direction == 2)
  {
  
    configurePWM(PID_T, 1, 50); //Вправо //45
  }

  if (total_direction == 3)
  {
    configurePWM(PID_T, 1, 200); //!!!
  }

  if (total_direction == 2)
  {
  PID_T = -1 * PID_T;
  }

  Serial.print(PID_T * 10);
  Serial.print(",");
  Serial.println();

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

  // Serial.print("Err_H =");
  // Serial.println(Err_H);

  P_H = Err_H * Kp;
  I_H = I_H + Err_H * Ki;
  D_H = (Err_H - prevErr_H) * Kd;

  if (I_H > 1000.0 || I_H < -1000.0)
  {
    I_H = 0;
  }

  PID_H = P_H + I_H + D_H;

  // Serial.println(PID_H);
  // Serial.print(" 1 PID_H - ");

  if (PID_H < 0)
  {
    total_direction = 1;
    PID_H = map(PID_H, 0, -2500, 0, 100);

    PID_H = constrain(PID_H, 0, 100);
  }
  else if (PID_H > 0)
  {
    total_direction = 2; // вниз
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

  // Serial.print(" 2 PID_H - ");
  // Serial.println(PID_H);

  if (total_direction == 2)
  {
    PID_H *= 1;
  }

  configurePWM(PID_H, 2, 100);
}

void configurePWM(int dutyCycle, byte Motor_NUM, int high_impulse)
{
  if (Motor_NUM == 1)
  {
    switch (total_direction)
    {
    case 1:
      // Motor_Drive(1);
      digitalWrite(IN2, LOW);
      digitalWrite(IN1, HIGH);
      // ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 100), 1, 1);

      break;

    case 2:
      // Motor_Drive(2);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN1, LOW);
      // ledcWrite(0, map(dutyCycle, 0, 100, 0, 255));
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 100), 1, 1);

      break;
    case 3:
      digitalWrite(IN2, HIGH);
      digitalWrite(IN1, HIGH);
      // ledcWrite(0, 0);
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 100), 1, 0);
      break;
    }
  }
  else if (Motor_NUM == 2)
  {
    switch (total_direction)
    {
    case 1:
      // Motor_Drive(1);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      // ledcWrite(1, map(dutyCycle, 0, 100, 0, 255));
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 50), 2, 1);
      break;

    case 2:
      // Motor_Drive(2);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      // ledcWrite(1, map(dutyCycle, 0, 100, 0, 255));
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 50), 2, 1);

      break;
    case 3:
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      configurePulsee(high_impulse, map(dutyCycle, 0, 100, 1000, 50), 2, 0);
      // ledcWrite(1, 0);

      break;
    }
    // Serial.print("DC: ");
    // Serial.println(dutyCycle);
  }
  /*
    Serial.print("Motor_NUM -");
    Serial.println(Motor_NUM);
    Serial.print("DC -");
    Serial.println(dutyCycle);
    Serial.print("dir -");
    Serial.println(total_direction);  h61m-e 2 0 2 ///  p8h77-m le /// 

    */
}


void configurePulsee(unsigned int highTime, unsigned int lowTime, int Motor_Num, byte TOTAL_MODE)
{

  if (Motor_Num == 1 && TOTAL_MODE == 0)
  {
    if (timer1_flag == 0)
    {
      timerDetachInterrupt(timer1);
      timer1_flag = 1;
      digitalWrite(ENA, LOW);
     //Serial.println("STOP_INTERRUPTS MOTOR 1");
    }
    // TOTAL_FLAG_MOTOR1 = 1;
    // TOTAL_CHECK();
    CheckFlag();
    return;
  }

  if (Motor_Num == 2 && TOTAL_MODE == 0)
  {
    if (timer2_flag == 0)
    {
      timerDetachInterrupt(timer2);
      timer2_flag = 1;
      digitalWrite(ENB, LOW);
      //Serial.println("STOP_INTERRUPTS MOTOR 2");
    }
    // TOTAL_FLAG_MOTOR2 = 1;
    // TOTAL_CHECK();
    CheckFlag();
    return;
  }

  switch (Motor_Num)
  {
  case 1:
    if (timer1_flag == 1)
    {
      pulseHighDuration1 = highTime;
      pulseLowDuration1 = lowTime;
      //Serial.println("STEP 1");
      timerAlarmWrite(timer1, pulseHighDuration1 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
      timerAttachInterrupt(timer1, pulseISR1, true);
      timer1_flag = 0;
    }
    else
    {
      FLAG_WORK_1 = 1;
      NEWpulseHighDuration1 = highTime;
      NEWpulseLowDuration1 = lowTime;
    }
    // TOTAL_FLAG_MOTOR1 = 0;
    break;

  case 2:

    if (timer2_flag == 1)
    {
      pulseHighDuration2 = highTime;
      pulseLowDuration2 = lowTime;
      timerAlarmWrite(timer2, pulseHighDuration2 * 1000, true); // Запускаем таймер для импульса HIGH пина 1
      timerAttachInterrupt(timer2, pulseISR2, true);
      timer2_flag = 0;
    }
    else
    {
      FLAG_WORK_2 = 1;
      NEWpulseHighDuration2 = highTime;
      NEWpulseLowDuration2 = lowTime;
    }
    // TOTAL_FLAG_MOTOR2 = 0;
    break;
  }

  // CheckFlag();
  /*
    Serial.print("lowTime - ");
    Serial.println(lowTime);

    */

}
//*/

void TOTAL_CHECK()
{
  if (TOTAL_FLAG_MOTOR1 && TOTAL_FLAG_MOTOR2)
  {
    digitalWrite(ENC, LOW);
  }
}