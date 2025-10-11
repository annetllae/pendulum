#include <ESP8266WiFi.h>
#include <espnow.h>
#include <AccelStepper.h>
#include <cmath>

float pAcc = 0;  // предыдущее значение ускорения с маятника
bool grow = false;
bool pGrow = grow;
bool detect = false;
int timespamp = 0;
int halfPeriod = 0;
bool isMovingForward = true;  // Флаг направления
bool runMotor = false;


const int filterSize = 10;
float filter[filterSize];
int fi = 0;  // индекс в фильтре


const int halfPeriodsSize = 20;
int halfPeriods[halfPeriodsSize];
int pi = 0;  // индекс в массиве периодов

const int stepPin = D1;
const int dirPin = D2;

AccelStepper stepper(1, stepPin, dirPin);

float getFilteredAcc() {
  float result = 0;
  for (int i = 0; i < filterSize; ++i) {
    result = result + filter[i];
  }
  return result / filterSize;
}

int getGoodHalfPeriod() {
  int result = 0;
  for (int i = 0; i < halfPeriodsSize; ++i) {
    result = result + halfPeriods[i];
  }
  result = result / halfPeriodsSize;
  for (int i = 0; i < halfPeriodsSize; ++i) {
    if (abs(halfPeriods[i] - result) > (result/50)) {
      Serial.print("Calc period error: ");
      Serial.print(abs(halfPeriods[i] - result));
      Serial.print(" vs ");
      Serial.println(result/50);
      return 0;
    }
  }
  
  return result > 300 ? result : 0;
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
  //Serial.println(fAcc);
  if (fAcc - pAcc > 0) {
    grow = true;
  } else {
    grow = false;
  }

  if (!pGrow && grow && !runMotor) {
    // начало колебания
    detect = true;
    digitalWrite(LED_BUILTIN, LOW);
    halfPeriods[(pi++) % halfPeriodsSize] = millis() - timespamp;
    timespamp = millis();
    halfPeriod = getGoodHalfPeriod();
    if (halfPeriod > 0) {
      runMotor = true;
      isMovingForward = true;
      makeHalfMove(isMovingForward, halfPeriod/2000.0);
      Serial.print("========= period is: ");
      Serial.println(halfPeriod);
      WiFi.mode(WIFI_OFF);
    }
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



void makeHalfMove(bool up, float desiredTime) {
  static bool firstRun = true;
  // Параметры движения
  long targetSteps = 600;
  float maxMotorSpeed = 5000;  // Максимальная скорость двигателя
  desiredTime *= 1.035;

  // Расчет для треугольного профиля
  float acceleration = calculateMaxAccelerationForTriangularProfile(targetSteps, desiredTime);
  float peakSpeed = calculatePeakSpeedForTriangularProfile(targetSteps, desiredTime);

  // Serial.println("\nДля треугольного профиля с временем ");
  // Serial.println(desiredTime);
  // Serial.print("Ускорение: ");
  // Serial.print(acceleration);
  // Serial.println(" шагов/сек²");
  // Serial.print("Пиковая скорость: ");
  // Serial.print(peakSpeed);
  // Serial.println(" шагов/сек");

  // Проверка ограничений двигателя
  float safeAcceleration = calculateSafeAcceleration(targetSteps, desiredTime, maxMotorSpeed);

  // Serial.println("\nС учетом ограничений двигателя:");
  // Serial.print("Безопасное ускорение: ");
  // Serial.print(safeAcceleration);
  // Serial.print("Шагов: ");
  // Serial.print(targetSteps);
  // Serial.println(" шагов/сек²");

  // Настройка двигателя
  stepper.setMaxSpeed(maxMotorSpeed);
  stepper.setAcceleration(safeAcceleration);
  if (firstRun) {
    stepper.setCurrentPosition(targetSteps/2);
    firstRun = false;
  }
  
  stepper.moveTo(targetSteps * (up ? 1 : 0));
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

  //makeHalfMove(isMovingForward, hp);
}


int prevMotorTime = millis();

// the loop function runs over and over again forever
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

  //Serial.println(i++);
  //Serial.println(WiFi.macAddress());
  // if (!digitalRead(buttonPin0)) {
  //   hp += 0.001;
  //   Serial.println(hp * 1000);
  //   delay(100);
  // }

  // if (!digitalRead(buttonPin1)) {
  //   hp -= 0.001;
  //   Serial.println(hp * 1000);
  //   delay(100);
  // }

  if (runMotor) {
    if (stepper.distanceToGo() == 0) {
      Serial.print("Change direction: ");
      Serial.print(millis() - prevMotorTime);
      Serial.print(", period ");
      Serial.println(halfPeriod/2);
      prevMotorTime = millis();

      //makeHalfMove(!isMovingForward, halfPeriod/2000.0);
      makeHalfMove(!isMovingForward, halfPeriod/2000.0);
      isMovingForward = !isMovingForward;  // Меняем флаг
    }
  }

  //Serial.println(".");
  stepper.run();
}