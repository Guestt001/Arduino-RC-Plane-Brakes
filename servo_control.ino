/*
 * ПРОГРАММА УПРАВЛЕНИЯ ДВУМЯ СЕРВОПРИВОДАМИ
 * Версия: 2.0
 * Автор: AI Assistant
 * 
 * ЛОГИКА РАБОТЫ:
 * - Режим ожидания: Input < 1500 → сервы на 1500
 * - Рабочий режим: Input ≥ 1500 → этап 1 (2 сек) → этап 2
 * - Обратная экспонента для плавного изменения частот
 */

#include <Servo.h>
#include <math.h>

// ========== НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ ==========

// 1. ПИНЫ ПОДКЛЮЧЕНИЯ
const int INPUT_PIN = 2;      // Пин для входящего PWM
const int SERVO1_PIN = 9;     // Левая серва
const int SERVO2_PIN = 10;    // Правая серва

// 2. ДИАПАЗОНЫ PWM
const int MID_POSITION = 1750;    // Точка переключения логики
const int MIN_PWM = 1500;         // Минимум для работы
const int MAX_PWM = 2000;         // Максимум PWM

// 3. ВРЕМЕННЫЕ ПАРАМЕТРЫ
const unsigned long PHASE1_DURATION = 2000;  // Длительность этапа 1 (мс)
const unsigned long DEBOUNCE_TIME = 50;      // Антидребезг (мс)

// 4. ЧАСТОТЫ И ЗНАЧЕНИЯ
const int CYCLE_DURATION = 100;   // Базовая длительность цикла
const int DUTY_CYCLE_MIN = 1500;  // Минимум для колебаний
const int DUTY_CYCLE_MAX = 2000;  // Максимум для колебаний
const float PHASE2_BASE_FREQ = 15.0; // Макс. частота этапа 2

// 5. ПОРОГИ И ГИСТЕРЕЗИС
const int WAIT_THRESHOLD = 1500;    // Переход в ожидание
const int WORK_THRESHOLD = 1500;    // Активация работы
const int HYSTERESIS = 10;          // Стабильность переключений

// 6. ЭКСПОНЕНТА И СГЛАЖИВАНИЕ
const float EXP_BASE = 2.71828;
const float EXP_POWER_MIN = -2.0;
const float EXP_POWER_MAX = 0.0;
const float SMOOTHING_FACTOR = 0.1; // 0.1-плавно, 0.5-быстро
const int FILTER_SIZE = 5;          // Размер медианного фильтра

// ========== КОНЕЦ НАСТРАИВАЕМЫХ ПАРАМЕТРОВ ==========

// Объекты сервоприводов
Servo servoLeft;
Servo servoRight;

// Таймеры
unsigned long previousMillisLeft = 0;
unsigned long previousMillisRight = 0;
unsigned long programStartTime = 0;
unsigned long phaseStartTime = 0;

// Состояния
int filteredInputPWM = 1750;
bool leftHighPhase = true;
bool rightHighPhase = true;
int currentPhase = 1;
bool phaseTransition = false;

// Сглаживание частот
float smoothedFreqLeft = 10.0;
float smoothedFreqRight = 10.0;

// Режимы работы
bool isWaitingMode = true;
bool systemActive = false;
bool wasInWaitMode = false;

// Буфер фильтра
int inputBuffer[FILTER_SIZE];
int bufferIndex = 0;

// ========== ФУНКЦИИ ==========

// Медианная фильтрация
int medianFilter(int newValue) {
  inputBuffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  
  int tempBuffer[FILTER_SIZE];
  for (int i = 0; i < FILTER_SIZE; i++) {
    tempBuffer[i] = inputBuffer[i];
  }
  
  // Сортировка
  for (int i = 0; i < FILTER_SIZE - 1; i++) {
    for (int j = i + 1; j < FILTER_SIZE; j++) {
      if (tempBuffer[i] > tempBuffer[j]) {
        int temp = tempBuffer[i];
        tempBuffer[i] = tempBuffer[j];
        tempBuffer[j] = temp;
      }
    }
  }
  
  return tempBuffer[FILTER_SIZE / 2];
}

// Расчет частоты для этапа 1
float calculateExponentialFrequencyPhase1(int pwmValue, bool isLeftServo) {
  float normalizedInput;
  
  if (pwmValue <= MID_POSITION) {
    if (isLeftServo) {
      return 10.0; // Левая: постоянная 10 Гц
    } else {
      // Правая: 1-10 Гц (обратная экспонента)
      normalizedInput = float(pwmValue - WORK_THRESHOLD) / 
                       float(MID_POSITION - WORK_THRESHOLD);
      normalizedInput = constrain(normalizedInput, 0.0, 1.0);
      return 1.0 + normalizedInput * normalizedInput * 9.0;
    }
  } else {
    if (isLeftServo) {
      // Левая: 10-1 Гц (обратная экспонента)
      normalizedInput = float(pwmValue - MID_POSITION) / 
                       float(MAX_PWM - MID_POSITION);
      normalizedInput = constrain(normalizedInput, 0.0, 1.0);
      return 10.0 - normalizedInput * normalizedInput * 9.0;
    } else {
      return 10.0; // Правая: постоянная 10 Гц
    }
  }
}

// Расчет частоты для этапа 2
float calculateFrequencyPhase2(int pwmValue, bool isLeftServo) {
  float normalizedInput;
  
  if (pwmValue <= MID_POSITION) {
    if (isLeftServo) {
      return 0; // Левая: 2000 (не колеблется)
    } else {
      // Правая: 1.5-15 Гц
      normalizedInput = float(pwmValue - WORK_THRESHOLD) / 
                       float(MID_POSITION - WORK_THRESHOLD);
      normalizedInput = constrain(normalizedInput, 0.0, 1.0);
      return 1.5 + normalizedInput * normalizedInput * 13.5;
    }
  } else {
    if (isLeftServo) {
      // Левая: 15-1.5 Гц
      normalizedInput = float(pwmValue - MID_POSITION) / 
                       float(MAX_PWM - MID_POSITION);
      normalizedInput = constrain(normalizedInput, 0.0, 1.0);
      return 15.0 - normalizedInput * normalizedInput * 13.5;
    } else {
      return 0; // Правая: 2000 (не колеблется)
    }
  }
}

// Сглаживание значений
float smoothValue(float currentValue, float previousValue, float smoothingFactor) {
  return previousValue * (1.0 - smoothingFactor) + currentValue * smoothingFactor;
}

// Управление режимами работы
void manageWorkModes() {
  unsigned long currentTime = millis();
  static unsigned long lastModeChangeTime = 0;
  
  // Режим ожидания (сигнал < порога)
  if (filteredInputPWM < WAIT_THRESHOLD) {
    if (!isWaitingMode) {
      isWaitingMode = true;
      systemActive = false;
      wasInWaitMode = true;
      lastModeChangeTime = currentTime;
      Serial.println("=== РЕЖИМ ОЖИДАНИЯ ===");
    }
  } 
  // Рабочий режим (сигнал ≥ порога + гистерезис)
  else if (filteredInputPWM >= (WORK_THRESHOLD + HYSTERESIS)) {
    if (isWaitingMode && (currentTime - lastModeChangeTime >= DEBOUNCE_TIME)) {
      isWaitingMode = false;
      systemActive = true;
      lastModeChangeTime = currentTime;
      
      if (wasInWaitMode) {
        programStartTime = currentTime;
        phaseStartTime = currentTime;
        wasInWaitMode = false;
        Serial.println("=== РАБОЧИЙ РЕЖИМ ===");
      }
    }
  }
  
  // Управление этапами в рабочем режиме
  if (systemActive && !isWaitingMode) {
    unsigned long elapsedTime = currentTime - programStartTime;
    
    if (elapsedTime < PHASE1_DURATION) {
      currentPhase = 1;
    } else {
      currentPhase = 2;
      if (!phaseTransition) {
        phaseTransition = true;
        phaseStartTime = currentTime;
        Serial.println("=== ЭТАП 2 АКТИВИРОВАН ===");
      }
    }
    
    if (phaseTransition && (currentTime - phaseStartTime >= DEBOUNCE_TIME)) {
      phaseTransition = false;
    }
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("=== СИСТЕМА УПРАВЛЕНИЯ СЕРВАМИ ===");
  Serial.println("Ожидание входного сигнала...");
  
  pinMode(INPUT_PIN, INPUT);
  
  servoLeft.attach(SERVO1_PIN);
  servoRight.attach(SERVO2_PIN);
  
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  
  // Инициализация фильтра
  for (int i = 0; i < FILTER_SIZE; i++) {
    inputBuffer[i] = 1500;
  }
  
  isWaitingMode = true;
  systemActive = false;
}

// ========== LOOP ==========
void loop() {
  // Чтение и фильтрация входного сигнала
  int rawInput = pulseIn(INPUT_PIN, HIGH, 25000);
  if (rawInput < 1400 || rawInput > 2100) {
    rawInput = filteredInputPWM;
  }
  filteredInputPWM = medianFilter(rawInput);
  filteredInputPWM = constrain(filteredInputPWM, 1400, 2100);
  
  // Управление режимами
  manageWorkModes();
  
  // РЕЖИМ ОЖИДАНИЯ
  if (isWaitingMode) {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
    leftHighPhase = true;
    rightHighPhase = true;
    smoothedFreqLeft = 10.0;
    smoothedFreqRight = 10.0;
    
    static unsigned long lastWaitDebug = 0;
    if (millis() - lastWaitDebug >= 1000) {
      lastWaitDebug = millis();
      Serial.print("Ожидание | Signal: ");
      Serial.println(filteredInputPWM);
    }
    
    delay(10);
    return;
  }
  
  // РАБОЧИЙ РЕЖИМ
  float targetFreqLeft, targetFreqRight;
  
  if (currentPhase == 1) {
    targetFreqLeft = calculateExponentialFrequencyPhase1(filteredInputPWM, true);
    targetFreqRight = calculateExponentialFrequencyPhase1(filteredInputPWM, false);
  } else {
    targetFreqLeft = calculateFrequencyPhase2(filteredInputPWM, true);
    targetFreqRight = calculateFrequencyPhase2(filteredInputPWM, false);
  }
  
  // Сглаживание частот
  smoothedFreqLeft = smoothValue(targetFreqLeft, smoothedFreqLeft, SMOOTHING_FACTOR);
  smoothedFreqRight = smoothValue(targetFreqRight, smoothedFreqRight, SMOOTHING_FACTOR);
  
  // Расчет интервалов переключения
  unsigned long switchIntervalLeft = (smoothedFreqLeft > 0.1) ? (500 / smoothedFreqLeft) : 0;
  unsigned long switchIntervalRight = (smoothedFreqRight > 0.1) ? (500 / smoothedFreqRight) : 0;
  
  unsigned long currentMillis = millis();
  
  // Управление левой сервой
  if (smoothedFreqLeft <= 0) {
    servoLeft.writeMicroseconds(DUTY_CYCLE_MAX);
  } else if (switchIntervalLeft > 0 && currentMillis - previousMillisLeft >= switchIntervalLeft) {
    previousMillisLeft = currentMillis;
    leftHighPhase = !leftHighPhase;
    servoLeft.writeMicroseconds(leftHighPhase ? DUTY_CYCLE_MAX : DUTY_CYCLE_MIN);
  }
  
  // Управление правой сервой
  if (smoothedFreqRight <= 0) {
    servoRight.writeMicroseconds(DUTY_CYCLE_MAX);
  } else if (switchIntervalRight > 0 && currentMillis - previousMillisRight >= switchIntervalRight) {
    previousMillisRight = currentMillis;
    rightHighPhase = !rightHighPhase;
    servoRight.writeMicroseconds(rightHighPhase ? DUTY_CYCLE_MAX : DUTY_CYCLE_MIN);
  }
  
  // Отладка
  static unsigned long lastDebugTime = 0;
  if (currentMillis - lastDebugTime >= 100) {
    lastDebugTime = currentMillis;
    
    Serial.print("Этап:");
    Serial.print(currentPhase);
    Serial.print(" | T:");
    Serial.print((millis() - programStartTime) / 1000.0, 1);
    Serial.print("с | In:");
    Serial.print(filteredInputPWM);
    Serial.print(" | L:");
    
    if (currentPhase == 1) {
      Serial.print(smoothedFreqLeft, 1);
      Serial.print("Hz | R:");
      Serial.print(smoothedFreqRight, 1);
      Serial.print("Hz");
    } else {
      if (smoothedFreqLeft <= 0) Serial.print("2000");
      else Serial.print(smoothedFreqLeft, 1);
      Serial.print("Hz | R:");
      if (smoothedFreqRight <= 0) Serial.print("2000");
      else Serial.print(smoothedFreqRight, 1);
      Serial.print("Hz");
    }
    
    Serial.print(" | L->");
    Serial.print(leftHighPhase ? "H" : "L");
    Serial.print(" | R->");
    Serial.print(rightHighPhase ? "H" : "L");
    Serial.println();
  }
  
  delay(1);
}
