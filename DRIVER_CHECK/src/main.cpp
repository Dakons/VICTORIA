#include <Arduino.h>

#define IN1 12
#define IN2 14
#define ENA 13

#define IN3 27
#define IN4 26
#define ENB 25

#define ENC 33

void setup()
{
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ENC, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(ENC, LOW);
  digitalWrite(ENA, LOW);
}

void setMotorState(int enPin, int in1Pin, int in2Pin, int state1, int state2, String description)
{
  digitalWrite(enPin, HIGH);  // Включаем драйвер
  digitalWrite(in1Pin, state1);
  digitalWrite(in2Pin, state2);
  Serial.println(description);
}

void loop()
{


  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
Serial.println("Первый режим");
delay(1000);

  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
Serial.println("Второй режим");
delay(1000);
/*
  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
Serial.println("Второй режим");
delay(1000);

  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
Serial.println("Третий режим");
delay(1000);

  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
Serial.println("Четвертый режим");
delay(1000);



  
digitalWrite(ENC, LOW);
digitalWrite(ENA, LOW);
digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("нулевой режим");
delay(1000);

  digitalWrite(ENC, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
Serial.println("Первый режим");
delay(1000);
digitalWrite(ENC, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Второй режим");
  delay(1000);
digitalWrite(ENC, LOW);
  digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Третий режим");
delay(1000);


  digitalWrite(ENC, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Четвертый режим");

delay(1000);
digitalWrite(ENC, HIGH);
digitalWrite(ENA, LOW);
digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Пятый режим");

delay(1000);
*/
}
