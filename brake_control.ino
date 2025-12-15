/*
 * СИСТЕМА УПРАВЛЕНИЯ ЭЛЕКТРОМАГНИТАМИ С PWM-ПЕРЕКЛЮЧЕНИЕМ
 * Версия: 7.1 (Обновленные параметры)
 * Изменения:
 * - Максимальное напряжение: 10В вместо 12В
 * - Минимальное напряжение: 3В вместо 2В
 * - 4 уровня вместо 5
 * - 2 переключения вместо 4 для смены уровня
 * - Питание: 10В
 */

// ==================== КОНФИГУРАЦИЯ ====================

// 1. ПИНЫ ПОДКЛЮЧЕНИЯ
const byte INPUT_PIN = 2;         // PWM сигнал с ресивера
const byte LEFT_MAGNET_PIN = 5;   // Левый магнит (PWM управление)
const byte RIGHT_MAGNET_PIN = 9;  // Правый магнит (PWM управление)
const byte INDICATOR_PIN = 3;     // Индикаторный светодиод

// 2. ДИАПАЗОНЫ ВХОДНОГО PWM
const int BRAKE_MIN = 1200;      // Начало тормозного режима
const int BRAKE_MAX = 1400;      // Конец тормозного режима
const int WORK_MIN = 1500;       // Начало рабочего режима
const int NEUTRAL = 1750;        // Нейтральное положение
const int DEAD_ZONE = 50;        // Мертвая зона вокруг нейтрали

// 3. ПАРАМЕТРЫ ПЕРЕКЛЮЧЕНИЯ УРОВНЕЙ (ИЗМЕНЕНО)
const byte PATTERN_CYCLES = 2;           // Циклов для смены уровня (было 4)
const unsigned int PATTERN_TIME = 2000;  // Окно детектирования (мс)
const unsigned int CYCLE_MAX = 800;      // Макс. длительность цикла (мс)
const unsigned int CYCLE_MIN_PAUSE = 100; // Мин. пауза между циклами (мс)
const unsigned int CHANGE_COOLDOWN = 3000; // Задержка после смены (мс)

// 4. РАБОЧИЕ ПАРАМЕТРЫ
const byte HYSTERESIS = 10;      // Гистерезис для стабильности
const unsigned int PHASE1_TIME = 2000;   // Длительность фазы 1 (мс)
const unsigned long MAX_OP_TIME = 12000; // Макс. время работы (мс)
const int PULSE_INTERVAL = 100;  // Интервал пульсации (мс)

// 5. УРОВНИ НАПРЯЖЕНИЯ (ИЗМЕНЕНО: 4 уровня, макс 10В, мин 3В)
const byte VOL_LEVELS = 4;  // Было 5, теперь 4
const float VOLTAGES[VOL_LEVELS][3] = {
  // Фаза1_мин, Фаза1_макс, Фаза2
  {3.0, 10.0, 10.0}, // Уровень 0 (максимум: 3В→10В→10В)
  {3.0, 9.0, 9.0},   // Уровень 1 (3В→9В→9В)
  {3.0, 8.0, 8.0},   // Уровень 2 (3В→8В→8В)
  {3.0, 7.0, 7.0}    // Уровень 3 (минимум: 3В→7В→7В)
};

// 6. СИСТЕМНЫЕ КОНСТАНТЫ (ИЗМЕНЕНО)
const float BRAKE_VOLTAGE = 6.0;     // Напряжение в тормозном режиме (можно оставить 6В)
const float SUPPLY_VOLTAGE = 10.0;   // Питание системы (было 12В)
const byte LED_BRIGHTNESS = 128;     // Яркость светодиода

// ==================== СИСТЕМНЫЕ ПЕРЕМЕННЫЕ ====================

// Состояния системы
struct SystemState {
  byte phase = 1;
  byte voltageLevel = 0;
  bool inMainMode = false;
  bool inBrakeMode = false;
  bool isBlocked = false;
  bool showingLevel = false;
  
  unsigned long phaseStart = 0;
  unsigned long opStart = 0;
  unsigned long levelShowStart = 0;
} state;

// Таймеры пульсации
unsigned long leftPulseTime = 0;
unsigned long rightPulseTime = 0;
bool leftPulseOn = false;
bool rightPulseOn = false;

// Индикатор
unsigned long indicatorTime = 0;
bool indicatorOn = false;

// PWM обработка
int lastPWM = NEUTRAL;
bool wasBrake = false;
bool wasMain = false;

// Детектор паттерна
struct PatternDetector {
  unsigned long windowStart = 0;
  unsigned long lastOn = 0;
  unsigned long lastOff = 0;
  byte cycleCount = 0;
  bool active = false;
  bool wasOn = false;
  bool cooling = false;
  unsigned long coolStart = 0;
} pattern;

// ==================== ОПТИМИЗИРОВАННЫЕ ФУНКЦИИ ====================

// Быстрое чтение PWM с защитой
inline int readPWM() {
  int val = pulseIn(INPUT_PIN, HIGH, 25000);
  return (val < 1000) ? NEUTRAL : val;
}

// Получение текущих напряжений
inline float getMinVoltage() { return VOLTAGES[state.voltageLevel][0]; }
inline float getMaxVoltage() { return VOLTAGES[state.voltageLevel][1]; }
inline float getPhase2Voltage() { return VOLTAGES[state.voltageLevel][2]; }

// Конвертация напряжения в ШИМ (0-255) с учетом 10В питания
inline byte voltToPWM(float volt) {
  volt = constrain(volt, 0.0, SUPPLY_VOLTAGE);
  return (byte)((volt / SUPPLY_VOLTAGE) * 255.0);
}

// Управление магнитами
inline void setMagnet(byte pin, float voltage) {
  analogWrite(pin, voltToPWM(voltage));
}

inline void offMagnet(byte pin) {
  analogWrite(pin, 0);
}

inline void offAll() {
  offMagnet(LEFT_MAGNET_PIN);
  offMagnet(RIGHT_MAGNET_PIN);
}

// Детектирование паттерна для смены уровня (2 цикла вместо 4)
void checkPattern(bool currentlyOn) {
  unsigned long now = millis();
  
  // Проверка задержки
  if (pattern.cooling) {
    if (now - pattern.coolStart >= CHANGE_COOLDOWN) {
      pattern.cooling = false;
    }
    return;
  }
  
  // Если режим активен - сброс детектирования
  if (state.inBrakeMode || state.inMainMode) {
    pattern.active = false;
    pattern.cycleCount = 0;
    pattern.wasOn = currentlyOn;
    return;
  }
  
  // Определение переходов
  bool turnedOn = (!pattern.wasOn && currentlyOn);
  bool turnedOff = (pattern.wasOn && !currentlyOn);
  pattern.wasOn = currentlyOn;
  
  // Обработка включения
  if (turnedOn) {
    pattern.lastOn = now;
    
    // Проверка минимальной паузы
    if (pattern.lastOff > 0 && (now - pattern.lastOff) < CYCLE_MIN_PAUSE) {
      pattern.active = false;
      pattern.cycleCount = 0;
      return;
    }
    
    // Активация детектирования
    if (!pattern.active) {
      pattern.active = true;
      pattern.windowStart = now;
      pattern.cycleCount = 0;
    }
  }
  
  // Обработка выключения
  if (turnedOff && pattern.lastOn > 0) {
    pattern.lastOff = now;
    
    // Проверка длительности включения
    if ((now - pattern.lastOn) <= CYCLE_MAX && pattern.active) {
      pattern.cycleCount++;
      
      // Проверка окна детектирования
      if (now - pattern.windowStart > PATTERN_TIME) {
        pattern.active = false;
        pattern.cycleCount = 0;
      } else if (pattern.cycleCount >= PATTERN_CYCLES) {
        // Паттерн обнаружен - меняем уровень (2 цикла достаточно)
        changeLevel();
        pattern.cooling = true;
        pattern.coolStart = now;
        pattern.active = false;
        pattern.cycleCount = 0;
      }
    } else {
      pattern.active = false;
      pattern.cycleCount = 0;
    }
  }
  
  // Сброс при истечении окна
  if (pattern.active && (now - pattern.windowStart > PATTERN_TIME)) {
    pattern.active = false;
    pattern.cycleCount = 0;
  }
}

// Смена уровня напряжения
void changeLevel() {
  state.voltageLevel = (state.voltageLevel + 1) % VOL_LEVELS; // Цикл 0-3
  state.showingLevel = true;
  state.levelShowStart = millis();
  state.isBlocked = true;
  offAll();
}

// Индикация уровня напряжения (адаптирована для 4 уровней)
void showLevel() {
  if (!state.showingLevel) return;
  
  unsigned long elapsed = millis() - state.levelShowStart;
  
  // Длительность индикации - 3 секунды
  if (elapsed > 3000) {
    state.showingLevel = false;
    if (state.isBlocked && elapsed > 500) state.isBlocked = false;
    analogWrite(INDICATOR_PIN, 0);
    return;
  }
  
  // Первая секунда - мигания (1-4 для уровней 0-3)
  if (elapsed < 1000) {
    byte blinks = state.voltageLevel + 1; // 1-4 миганий
    byte phase = (elapsed / 200) % (blinks * 2);
    analogWrite(INDICATOR_PIN, (phase < blinks) ? LED_BRIGHTNESS : 0);
  } 
  // Следующие 2 секунды - постоянный свет с яркостью уровня
  else {
    // 4 уровня: 100%, 75%, 50%, 25% яркости
    byte brightness = map(state.voltageLevel, 0, VOL_LEVELS-1, 
                         LED_BRIGHTNESS, LED_BRIGHTNESS/4);
    analogWrite(INDICATOR_PIN, brightness);
  }
}

// Рабочая индикация
void updateIndicator(int pwm, bool leftPulsing, bool rightPulsing) {
  // Приоритет: индикация уровня
  if (state.showingLevel) {
    showLevel();
    return;
  }
  
  // Блокировка - мигание
  if (state.isBlocked) {
    if (millis() - indicatorTime > 500) {
      indicatorTime = millis();
      indicatorOn = !indicatorOn;
      analogWrite(INDICATOR_PIN, indicatorOn ? LED_BRIGHTNESS : 0);
    }
    return;
  }
  
  // Режим ожидания - выключен
  if (!state.inMainMode && !state.inBrakeMode) {
    analogWrite(INDICATOR_PIN, 0);
    return;
  }
  
  // Определение необходимости пульсации
  bool shouldPulse = false;
  
  if (state.inBrakeMode) {
    shouldPulse = (pwm < 1300 && rightPulsing) || (pwm > 1300 && leftPulsing);
  } else if (state.inMainMode) {
    if (state.phase == 1) {
      shouldPulse = true;
    } else if (state.phase == 2) {
      shouldPulse = (abs(pwm - NEUTRAL) > DEAD_ZONE) && 
                   ((pwm < NEUTRAL && rightPulsing) || (pwm > NEUTRAL && leftPulsing));
    }
  }
  
  // Пульсация или постоянный свет
  if (shouldPulse) {
    if (millis() - indicatorTime > PULSE_INTERVAL) {
      indicatorTime = millis();
      indicatorOn = !indicatorOn;
      analogWrite(INDICATOR_PIN, indicatorOn ? LED_BRIGHTNESS : 0);
    }
  } else {
    analogWrite(INDICATOR_PIN, LED_BRIGHTNESS);
  }
}

// Проверка и установка режимов
void updateModes(int pwm) {
  // Проверка блокировки
  if (state.isBlocked && !state.showingLevel) {
    bool inBrake = (pwm >= BRAKE_MIN - HYSTERESIS && pwm <= BRAKE_MAX + HYSTERESIS);
    bool inMain = (pwm >= WORK_MIN - HYSTERESIS);
    if (!inBrake && !inMain) {
      state.isBlocked = false;
      state.inMainMode = false;
      state.inBrakeMode = false;
      state.phase = 1;
      lastPWM = pwm;
    }
    return;
  }
  
  // Проверка максимального времени работы
  if ((state.inMainMode || state.inBrakeMode) && 
      (millis() - state.opStart >= MAX_OP_TIME)) {
    state.isBlocked = true;
    state.inMainMode = false;
    state.inBrakeMode = false;
    state.phase = 1;
    offAll();
  }
  
  // Определение режимов с гистерезисом
  int brakeMin = BRAKE_MIN - (state.inBrakeMode ? HYSTERESIS : 0);
  int brakeMax = BRAKE_MAX + (state.inBrakeMode ? HYSTERESIS : 0);
  int workMin = WORK_MIN - (state.inMainMode ? HYSTERESIS : 0);
  
  bool isBrake = (pwm >= brakeMin && pwm <= brakeMax);
  bool isMain = (pwm >= workMin && !isBrake);
  
  if (!state.isBlocked) {
    if (isMain && !state.inMainMode && (isMain != wasMain)) {
      state.inMainMode = true;
      state.inBrakeMode = false;
      state.phaseStart = millis();
      state.opStart = millis();
      state.phase = 1;
    } else if (isBrake && !state.inBrakeMode && (isBrake != wasBrake)) {
      state.inBrakeMode = true;
      state.inMainMode = false;
      state.opStart = millis();
      state.phase = 1;
    } else if (!isMain && !isBrake && (state.inMainMode || state.inBrakeMode)) {
      state.inMainMode = false;
      state.inBrakeMode = false;
      state.phase = 1;
    }
  }
  
  wasMain = isMain;
  wasBrake = isBrake;
  lastPWM = pwm;
}

// Расчет напряжения для магнита
float calcVoltage(int pwm, bool isLeft) {
  if (state.inBrakeMode) return BRAKE_VOLTAGE;
  
  if (state.phase == 1) {
    unsigned long elapsed = millis() - state.phaseStart;
    if (elapsed > PHASE1_TIME) elapsed = PHASE1_TIME;
    float progress = (float)elapsed / PHASE1_TIME;
    return getMinVoltage() + progress * (getMaxVoltage() - getMinVoltage());
  } else {
    return getPhase2Voltage();
  }
}

// ==================== SETUP И LOOP ====================

void setup() {
  // Настройка пинов
  pinMode(INPUT_PIN, INPUT);
  pinMode(LEFT_MAGNET_PIN, OUTPUT);
  pinMode(RIGHT_MAGNET_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  
  // Инициализация
  offAll();
  analogWrite(INDICATOR_PIN, 0);
  
  // Сброс состояний
  pattern.active = false;
  pattern.cycleCount = 0;
  pattern.wasOn = false;
  pattern.cooling = false;
  
  // Показать текущий уровень при старте
  state.showingLevel = true;
  state.levelShowStart = millis();
}

void loop() {
  // Чтение входного сигнала
  int pwm = readPWM();
  
  // Обновление режимов
  updateModes(pwm);
  checkPattern(state.inMainMode);
  
  // Выход если заблокировано
  if (state.isBlocked && !state.showingLevel) {
    delay(10);
    return;
  }
  
  // Переход на фазу 2
  if (state.inMainMode && state.phase == 1) {
    if (millis() - state.phaseStart > PHASE1_TIME) {
      state.phase = 2;
    }
  }
  
  // Расчет напряжений
  float leftVolt = calcVoltage(pwm, true);
  float rightVolt = calcVoltage(pwm, false);
  
  // Управление магнитами
  bool leftActive = false, rightActive = false;
  bool leftPulsing = false, rightPulsing = false;
  
  if (state.inBrakeMode) {
    leftActive = rightActive = true;
    
    if (pwm < 1300) {
      rightPulsing = true;
      setMagnet(LEFT_MAGNET_PIN, leftVolt);
      
      if (millis() - rightPulseTime > PULSE_INTERVAL) {
        rightPulseTime = millis();
        rightPulseOn = !rightPulseOn;
        setMagnet(RIGHT_MAGNET_PIN, rightPulseOn ? rightVolt : 0);
      }
    } else if (pwm > 1300) {
      leftPulsing = true;
      setMagnet(RIGHT_MAGNET_PIN, rightVolt);
      
      if (millis() - leftPulseTime > PULSE_INTERVAL) {
        leftPulseTime = millis();
        leftPulseOn = !leftPulseOn;
        setMagnet(LEFT_MAGNET_PIN, leftPulseOn ? leftVolt : 0);
      }
    } else {
      setMagnet(LEFT_MAGNET_PIN, leftVolt);
      setMagnet(RIGHT_MAGNET_PIN, rightVolt);
    }
    
  } else if (state.inMainMode) {
    leftActive = rightActive = true;
    
    // Определение необходимости пульсации
    bool pulseLeft = (state.phase == 1) ? true : (pwm > NEUTRAL + DEAD_ZONE);
    bool pulseRight = (state.phase == 1) ? true : (pwm < NEUTRAL - DEAD_ZONE);
    
    leftPulsing = pulseLeft;
    rightPulsing = pulseRight;
    
    // Управление левым магнитом
    if (pulseLeft) {
      if (millis() - leftPulseTime > PULSE_INTERVAL) {
        leftPulseTime = millis();
        leftPulseOn = !leftPulseOn;
        setMagnet(LEFT_MAGNET_PIN, leftPulseOn ? leftVolt : 0);
      }
    } else {
      setMagnet(LEFT_MAGNET_PIN, leftVolt);
    }
    
    // Управление правым магнитом
    if (pulseRight) {
      if (millis() - rightPulseTime > PULSE_INTERVAL) {
        rightPulseTime = millis();
        rightPulseOn = !rightPulseOn;
        setMagnet(RIGHT_MAGNET_PIN, rightPulseOn ? rightVolt : 0);
      }
    } else {
      setMagnet(RIGHT_MAGNET_PIN, rightVolt);
    }
    
  } else {
    offAll();
  }
  
  // Обновление индикации
  updateIndicator(pwm, leftPulsing, rightPulsing);
  
  delay(10);
}
