#include <ESP8266WiFi.h>
#include <espnow.h>
#include <AccelStepper.h>
#include <cmath>

float pAcc = 0;
bool grow = false;
bool pGrow = grow;
bool detect = false;
int timespamp = 0;
float halfPeriod = 0;

const int filterSize = 10;
float filter[filterSize];
int fi = 0;
const int stepPin = D1;
const int dirPin = D2;

AccelStepper stepper(1, stepPin, dirPin);

float getFilteredAcc() {
  float result = 0;
  for (int i = 0; i < filterSize; ++i) {
    result += filter[i];
  }
  return result / filterSize;
}

void ReceiveCallback(u8 *mac_addr, u8 *data, u8 len) {
  float acc;
  memcpy(&acc, data, sizeof(acc));
  filter[(fi++) % filterSize] = acc;
  // Serial.print(acc);
  // Serial.print(", ");
  // Serial.print(halfPeriod / 100);
  // Serial.print(", ");
  float fAcc = getFilteredAcc();
  // Serial.println(fAcc);
  if (fAcc - pAcc > 0) {
    grow = true;
  } else {
    grow = false;
  }

  if (!pGrow && grow) {
    halfPeriod = millis() - timespamp;
    detect = true;
    digitalWrite(LED_BUILTIN, LOW);
    timespamp = millis();
  } else {
    detect = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  pAcc = fAcc;
  pGrow = grow;
}

const int buttonPin0 = D6;
const int buttonPin1 = D5;

/**
 * Расчет максимального ускорения для треугольного профиля скорости
 * @param steps - количество шагов для перемещения
 * @param totalTime - общее время перемещения в секундах
 * @return максимальное ускорение (шаги/секунда²)
 */
float calculateMaxAccelerationForTriangularProfile(long steps, float totalTime) {
  if (totalTime <= 0) {
    return 0;  // Защита от деления на ноль
  }
  return (4.0 * steps) / (totalTime * totalTime);
}

/**
 * Расчет максимальной скорости при треугольном профиле
 * @param steps - количество шагов для перемещения  
 * @param totalTime - общее время перемещения в секундах
 * @return максимальная скорость (шаги/секунда)
 */
float calculatePeakSpeedForTriangularProfile(long steps, float totalTime) {
  if (totalTime <= 0) {
    return 0;
  }
  return (2.0 * steps) / totalTime;
}
/*
 * Расчет ускорения с учетом ограничения по максимальной скорости
 * @param steps - количество шагов для перемещения
 * @param totalTime - общее время перемещения в секундах
 * @param maxMotorSpeed - максимальная техническая скорость двигателя (шаги/секунда)
 * @return ускорение (шаги/секунда²)
 */
float calculateSafeAcceleration(long steps, float totalTime, float maxMotorSpeed) {
  // Сначала рассчитываем для идеального треугольного профиля
  float idealAcceleration = calculateMaxAccelerationForTriangularProfile(steps, totalTime);
  float peakSpeed = calculatePeakSpeedForTriangularProfile(steps, totalTime);

  // Если пиковая скорость превышает возможности двигателя
  if (peakSpeed > maxMotorSpeed) {
    Serial.println("Внимание: требуется трапецеидальный профиль!");
    Serial.print("Пиковая скорость: ");
    Serial.print(peakSpeed);
    Serial.print(" > Макс. скорость двигателя: ");
    Serial.println(maxMotorSpeed);

    // В этом случае нужно использовать трапецеидальный профиль
    // Возвращаем ускорение, основанное на максимальной скорости двигателя
    return calculateAccelerationForTrapezoidalAtMaxSpeed(steps, totalTime, maxMotorSpeed);
  }

  return idealAcceleration;
}

/**
 * Расчет ускорения когда двигатель работает на максимальной скорости
 * (для трапецеидального профиля)
 */
float calculateAccelerationForTrapezoidalAtMaxSpeed(long steps, float totalTime, float maxSpeed) {
  // Время движения на постоянной скорости
  float timeAtConstantSpeed = totalTime - (2 * maxSpeed / (maxSpeed * maxSpeed / steps));

  if (timeAtConstantSpeed < 0) {
    // Если даже при мгновенном разгоне/замедлении не укладываемся во время
    // Используем максимально возможное ускорение
    Serial.println("Ошибка: Невозможно достичь целевого времени с данным двигателем!");
    return 10000;  // Возвращаем очень большое ускорение
  }

  // Ускорение для достижения максимальной скорости
  return maxSpeed * maxSpeed / steps;
}

bool isMovingForward = true; // Флаг направления

void makeHalfMove(bool up, float desiredTime) {
  // Параметры движения
  long targetSteps = 500;
  float maxMotorSpeed = 2000;  // Максимальная скорость двигателя

  // Расчет для треугольного профиля
  float acceleration = calculateMaxAccelerationForTriangularProfile(targetSteps, desiredTime);
  float peakSpeed = calculatePeakSpeedForTriangularProfile(targetSteps, desiredTime);

  Serial.println("\nДля треугольного профиля:");
  Serial.print("Ускорение: ");
  Serial.print(acceleration);
  Serial.println(" шагов/сек²");
  Serial.print("Пиковая скорость: ");
  Serial.print(peakSpeed);
  Serial.println(" шагов/сек");

  // Проверка ограничений двигателя
  float safeAcceleration = calculateSafeAcceleration(targetSteps, desiredTime, maxMotorSpeed);

  Serial.println("\nС учетом ограничений двигателя:");
  Serial.print("Безопасное ускорение: ");
  Serial.print(safeAcceleration);
  Serial.println(" шагов/сек²");

  // Настройка двигателя
  stepper.setMaxSpeed(maxMotorSpeed);
  stepper.setAcceleration(safeAcceleration);
  stepper.moveTo(targetSteps * (up ? 1 : -1));
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin0, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  delay(500);

  Serial.begin(19200);

  delay(500);

  Serial.println("Receiver started");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Can't init esp now!");
    while (1) {
      delay(100);
    }
  }

  esp_now_register_recv_cb(ReceiveCallback);

  makeHalfMove(isMovingForward, 1);
}



int i = 0;
// the loop function runs over and over again forever
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

  //Serial.println(i++);
  //Serial.println(WiFi.macAddress());
  if (!digitalRead(buttonPin0)) {
    Serial.println("FWD");
    stepper.move(1500);
  }

  if (!digitalRead(buttonPin1)) {
    stepper.move(-1500);
    Serial.println("FWD");
  }

  if (stepper.distanceToGo() == 0) {
    makeHalfMove(!isMovingForward, 0.5);
    isMovingForward = !isMovingForward;  // Меняем флаг
  }

  //Serial.println(".");
  stepper.run();
}