/*
 * СИСТЕМА УПРАВЛЕНИЯ ЭЛЕКТРОМАГНИТАМИ С PWM-ПЕРЕКЛЮЧЕНИЕМ
 * Версия: 8.1 (Оптимизированная)
 */

// ==================== КОНФИГУРАЦИЯ ====================
const byte INPUT_PIN = 2;         // PWM вход
const byte LEFT_MAGNET_PIN = 5;   // Левый магнит
const byte RIGHT_MAGNET_PIN = 9;  // Правый магнит
const byte INDICATOR_PIN = 3;     // Индикатор

const int BRAKE_MIN = 1200;       // Начало тормозов
const int BRAKE_MAX = 1400;       // Конец тормозов
const int WORK_MIN = 1500;        // Начало работы
const int NEUTRAL = 1750;         // Нейтраль
const int DEAD_ZONE = 10;         // Мертвая зона

const byte MAIN_PATTERN_CYCLES = 2;     // Основной режим: 2 цикла
const byte BRAKE_PATTERN_CYCLES = 3;    // Тормоза: 3 цикла
const unsigned int PATTERN_TIME = 2000; // Окно детектирования
const unsigned int CYCLE_MAX = 800;     // Макс. цикл
const unsigned int CYCLE_MIN_PAUSE = 100;// Мин. пауза
const unsigned int CHANGE_COOLDOWN = 3000;// Задержка

const byte HYSTERESIS = 10;             // Гистерезис
const unsigned int PHASE1_TIME = 2000;  // Фаза 1
const unsigned long MAX_OP_TIME = 12000;// Макс. работа
const int PULSE_INTERVAL = 100;         // Пульсация

const byte MAIN_LEVELS = 4;             // Уровни основного
const byte BRAKE_LEVELS = 4;            // Уровни тормозов

const float MAIN_VOLTAGES[MAIN_LEVELS][3] = {
  {3.0, 10.0, 10.0}, {3.0, 9.0, 9.0}, {3.0, 8.0, 8.0}, {3.0, 7.0, 7.0}
};

const float BRAKE_VOLTAGES[BRAKE_LEVELS] = {8.0, 7.0, 6.0, 5.0};
const float SUPPLY_VOLTAGE = 10.0;      // Питание
const byte LED_BRIGHTNESS = 128;        // Яркость

// ==================== СИСТЕМНЫЕ ПЕРЕМЕННЫЕ ====================
struct SystemState {
  byte phase = 1;
  byte mainLevel = 0;
  byte brakeLevel = 0;
  bool inMainMode = false;
  bool inBrakeMode = false;
  bool isBlocked = false;
  bool showingLevel = false;
  byte showingWhich = 0; // 0-нет, 1-основной, 2-тормоз
  
  unsigned long phaseStart = 0;
  unsigned long opStart = 0;
  unsigned long levelStart = 0;
} state;

struct Timer {
  unsigned long leftPulse = 0;
  unsigned long rightPulse = 0;
  unsigned long indicator = 0;
  bool leftOn = false;
  bool rightOn = false;
  bool indicatorOn = false;
} timer;

struct PatternDet {
  unsigned long windowStart = 0;
  unsigned long lastOn = 0;
  unsigned long lastOff = 0;
  byte cycleCount = 0;
  bool active = false;
  bool wasOn = false;
  bool cooling = false;
  unsigned long coolStart = 0;
};

PatternDet mainPattern, brakePattern;

int lastPWM = NEUTRAL;
bool wasBrake = false, wasMain = false;

// ==================== ОПТИМИЗИРОВАННЫЕ ФУНКЦИИ ====================
inline int readPWM() {
  int val = pulseIn(INPUT_PIN, HIGH, 25000);
  return val < 1000 ? NEUTRAL : val;
}

inline float getMainMin() { return MAIN_VOLTAGES[state.mainLevel][0]; }
inline float getMainMax() { return MAIN_VOLTAGES[state.mainLevel][1]; }
inline float getPhase2() { return MAIN_VOLTAGES[state.mainLevel][2]; }
inline float getBrakeVolt() { return BRAKE_VOLTAGES[state.brakeLevel]; }

inline byte voltToPWM(float volt) {
  volt = volt < 0.0 ? 0.0 : (volt > SUPPLY_VOLTAGE ? SUPPLY_VOLTAGE : volt);
  return (byte)((volt / SUPPLY_VOLTAGE) * 255.0);
}

inline void setMag(byte pin, float volt) {
  analogWrite(pin, voltToPWM(volt));
}
inline void offMag(byte pin) { analogWrite(pin, 0); }
inline void offAll() { offMag(LEFT_MAGNET_PIN); offMag(RIGHT_MAGNET_PIN); }

void handlePattern(PatternDet* pat, bool activeNow, byte neededCycles, bool isBrake) {
  unsigned long now = millis();
  
  if (pat->cooling && (now - pat->coolStart >= CHANGE_COOLDOWN)) {
    pat->cooling = false;
  }
  if (pat->cooling || state.inBrakeMode || state.inMainMode) {
    pat->active = false;
    pat->cycleCount = 0;
    pat->wasOn = activeNow;
    return;
  }
  
  bool turnedOn = !pat->wasOn && activeNow;
  bool turnedOff = pat->wasOn && !activeNow;
  pat->wasOn = activeNow;
  
  if (turnedOn) {
    pat->lastOn = now;
    if (pat->lastOff && (now - pat->lastOff) < CYCLE_MIN_PAUSE) {
      pat->active = false;
      pat->cycleCount = 0;
      return;
    }
    if (!pat->active) {
      pat->active = true;
      pat->windowStart = now;
      pat->cycleCount = 0;
    }
  }
  
  if (turnedOff && pat->lastOn) {
    pat->lastOff = now;
    
    if ((now - pat->lastOn) <= CYCLE_MAX && pat->active) {
      pat->cycleCount++;
      
      if (now - pat->windowStart > PATTERN_TIME) {
        pat->active = false;
        pat->cycleCount = 0;
      } else if (pat->cycleCount >= neededCycles) {
        if (isBrake) changeBrake();
        else changeMain();
        pat->cooling = true;
        pat->coolStart = now;
        pat->active = false;
        pat->cycleCount = 0;
      }
    } else {
      pat->active = false;
      pat->cycleCount = 0;
    }
  }
  
  if (pat->active && (now - pat->windowStart > PATTERN_TIME)) {
    pat->active = false;
    pat->cycleCount = 0;
  }
}

void changeMain() {
  state.mainLevel = (state.mainLevel + 1) % MAIN_LEVELS;
  state.showingLevel = true;
  state.showingWhich = 1;
  state.levelStart = millis();
  state.isBlocked = true;
  offAll();
}

void changeBrake() {
  state.brakeLevel = (state.brakeLevel + 1) % BRAKE_LEVELS;
  state.showingLevel = true;
  state.showingWhich = 2;
  state.levelStart = millis();
  state.isBlocked = true;
  offAll();
}

void showLevel() {
  if (!state.showingLevel) return;
  
  unsigned long elapsed = millis() - state.levelStart;
  if (elapsed > 3000) {
    state.showingLevel = false;
    state.showingWhich = 0;
    if (state.isBlocked && elapsed > 500) state.isBlocked = false;
    analogWrite(INDICATOR_PIN, 0);
    return;
  }
  
  if (elapsed < 1000) {
    byte blinks = (state.showingWhich == 1 ? state.mainLevel : state.brakeLevel) + 1;
    byte interval = state.showingWhich == 2 ? 150 : 200;
    byte phase = (elapsed / interval) % (blinks * 2);
    analogWrite(INDICATOR_PIN, phase < blinks ? LED_BRIGHTNESS : 0);
  } else {
    byte level = state.showingWhich == 1 ? state.mainLevel : state.brakeLevel;
    byte brightness = map(level, 0, 3, LED_BRIGHTNESS, LED_BRIGHTNESS/4);
    analogWrite(INDICATOR_PIN, brightness);
  }
}

void updateIndicator(int pwm, bool leftPulse, bool rightPulse) {
  if (state.showingLevel) { showLevel(); return; }
  
  if (state.isBlocked) {
    if (millis() - timer.indicator > 500) {
      timer.indicator = millis();
      timer.indicatorOn = !timer.indicatorOn;
      analogWrite(INDICATOR_PIN, timer.indicatorOn ? LED_BRIGHTNESS : 0);
    }
    return;
  }
  
  if (!state.inMainMode && !state.inBrakeMode) {
    analogWrite(INDICATOR_PIN, 0);
    return;
  }
  
  bool shouldPulse = false;
  if (state.inBrakeMode) {
    shouldPulse = (pwm < 1300 && rightPulse) || (pwm > 1300 && leftPulse);
  } else if (state.inMainMode) {
    if (state.phase == 1) {
      shouldPulse = true;
    } else if (state.phase == 2) {
      shouldPulse = (abs(pwm - NEUTRAL) > DEAD_ZONE) && 
                   ((pwm < NEUTRAL && rightPulse) || (pwm > NEUTRAL && leftPulse));
    }
  }
  
  if (shouldPulse) {
    if (millis() - timer.indicator > PULSE_INTERVAL) {
      timer.indicator = millis();
      timer.indicatorOn = !timer.indicatorOn;
      analogWrite(INDICATOR_PIN, timer.indicatorOn ? LED_BRIGHTNESS : 0);
    }
  } else {
    analogWrite(INDICATOR_PIN, LED_BRIGHTNESS);
  }
}

void updateModes(int pwm) {
  if (state.isBlocked && !state.showingLevel) {
    bool inBrake = pwm >= BRAKE_MIN - HYSTERESIS && pwm <= BRAKE_MAX + HYSTERESIS;
    bool inMain = pwm >= WORK_MIN - HYSTERESIS;
    if (!inBrake && !inMain) {
      state.isBlocked = false;
      state.inMainMode = false;
      state.inBrakeMode = false;
      state.phase = 1;
      lastPWM = pwm;
    }
    return;
  }
  
  if ((state.inMainMode || state.inBrakeMode) && 
      (millis() - state.opStart >= MAX_OP_TIME)) {
    state.isBlocked = true;
    state.inMainMode = false;
    state.inBrakeMode = false;
    state.phase = 1;
    offAll();
  }
  
  int brakeMin = BRAKE_MIN - (state.inBrakeMode ? HYSTERESIS : 0);
  int brakeMax = BRAKE_MAX + (state.inBrakeMode ? HYSTERESIS : 0);
  int workMin = WORK_MIN - (state.inMainMode ? HYSTERESIS : 0);
  
  bool isBrake = pwm >= brakeMin && pwm <= brakeMax;
  bool isMain = pwm >= workMin && !isBrake;
  
  if (!state.isBlocked) {
    if (isMain && !state.inMainMode && isMain != wasMain) {
      state.inMainMode = true;
      state.inBrakeMode = false;
      state.phaseStart = millis();
      state.opStart = millis();
      state.phase = 1;
    } else if (isBrake && !state.inBrakeMode && isBrake != wasBrake) {
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

float calcVoltage(int pwm, bool isLeft) {
  if (state.inBrakeMode) return getBrakeVolt();
  
  if (state.phase == 1) {
    unsigned long elapsed = millis() - state.phaseStart;
    if (elapsed > PHASE1_TIME) elapsed = PHASE1_TIME;
    float progress = (float)elapsed / PHASE1_TIME;
    return getMainMin() + progress * (getMainMax() - getMainMin());
  }
  return getPhase2();
}

// ==================== SETUP И LOOP ====================
void setup() {
  pinMode(INPUT_PIN, INPUT);
  pinMode(LEFT_MAGNET_PIN, OUTPUT);
  pinMode(RIGHT_MAGNET_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  
  offAll();
  analogWrite(INDICATOR_PIN, 0);
  
  mainPattern.active = false;
  brakePattern.active = false;
  
  state.showingLevel = true;
  state.showingWhich = 1;
  state.levelStart = millis();
}

void loop() {
  int pwm = readPWM();
  
  updateModes(pwm);
  
  bool inBrakeRange = pwm >= BRAKE_MIN - HYSTERESIS && pwm <= BRAKE_MAX + HYSTERESIS;
  handlePattern(&mainPattern, state.inMainMode, MAIN_PATTERN_CYCLES, false);
  handlePattern(&brakePattern, inBrakeRange, BRAKE_PATTERN_CYCLES, true);
  
  if (state.isBlocked && !state.showingLevel) {
    delay(10);
    return;
  }
  
  if (state.inMainMode && state.phase == 1) {
    if (millis() - state.phaseStart > PHASE1_TIME) {
      state.phase = 2;
    }
  }
  
  float leftVolt = calcVoltage(pwm, true);
  float rightVolt = calcVoltage(pwm, false);
  
  bool leftActive = false, rightActive = false;
  bool leftPulse = false, rightPulse = false;
  
  if (state.inBrakeMode) {
    leftActive = rightActive = true;
    
    if (pwm < 1300) {
      rightPulse = true;
      setMag(LEFT_MAGNET_PIN, leftVolt);
      
      if (millis() - timer.rightPulse > PULSE_INTERVAL) {
        timer.rightPulse = millis();
        timer.rightOn = !timer.rightOn;
        setMag(RIGHT_MAGNET_PIN, timer.rightOn ? rightVolt : 0);
      }
    } else if (pwm > 1300) {
      leftPulse = true;
      setMag(RIGHT_MAGNET_PIN, rightVolt);
      
      if (millis() - timer.leftPulse > PULSE_INTERVAL) {
        timer.leftPulse = millis();
        timer.leftOn = !timer.leftOn;
        setMag(LEFT_MAGNET_PIN, timer.leftOn ? leftVolt : 0);
      }
    } else {
      setMag(LEFT_MAGNET_PIN, leftVolt);
      setMag(RIGHT_MAGNET_PIN, rightVolt);
    }
    
  } else if (state.inMainMode) {
    leftActive = rightActive = true;
    
    bool pulseLeft = state.phase == 1 || pwm > NEUTRAL + DEAD_ZONE;
    bool pulseRight = state.phase == 1 || pwm < NEUTRAL - DEAD_ZONE;
    
    leftPulse = pulseLeft;
    rightPulse = pulseRight;
    
    if (pulseLeft) {
      if (millis() - timer.leftPulse > PULSE_INTERVAL) {
        timer.leftPulse = millis();
        timer.leftOn = !timer.leftOn;
        setMag(LEFT_MAGNET_PIN, timer.leftOn ? leftVolt : 0);
      }
    } else {
      setMag(LEFT_MAGNET_PIN, leftVolt);
    }
    
    if (pulseRight) {
      if (millis() - timer.rightPulse > PULSE_INTERVAL) {
        timer.rightPulse = millis();
        timer.rightOn = !timer.rightOn;
        setMag(RIGHT_MAGNET_PIN, timer.rightOn ? rightVolt : 0);
      }
    } else {
      setMag(RIGHT_MAGNET_PIN, rightVolt);
    }
    
  } else {
    offAll();
  }
  
  updateIndicator(pwm, leftPulse, rightPulse);
  delay(10);
}
