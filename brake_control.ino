#include <Arduino.h>
#include <mavlink.h>

// ---------------- ПИНЫ ----------------
#define LEFT_PIN 25
#define RIGHT_PIN 27

// ---------------- PWM ----------------
#define PWM_FREQ 1000
#define PWM_RES 8

#define CH_LEFT 0
#define CH_RIGHT 1

// ---------------- MAVLINK ----------------
HardwareSerial mavSerial(2);

// ---------------- ДАННЫЕ ----------------
float yaw_rc = 0.0f;       // команда пилота (-1..1)
float yaw_actual = 0.0f;   // реальный yaw (рад)
int rc10_raw = 1000;

// автокурс
float target_yaw = 0.0f;
bool heading_locked = false;

// режимы
enum BrakeMode {
  MODE_OFF,
  MODE_MAIN,
  MODE_BRAKE
};

BrakeMode currentMode = MODE_OFF;

// таймеры
unsigned long brakeStartTime = 0;
unsigned long lastMsgTime = 0;

// ---------------- НАСТРОЙКИ ----------------
const float MAIN_BRAKE_FORCE = 0.5f;
const float BRAKE_MIN = 0.4f;
const float BRAKE_MAX = 1.0f;
const uint32_t RAMP_TIME = 1500;

// автокурс
const float AUTO_KP = 1.2f;
const float AUTO_LIMIT = 0.4f;
const float PILOT_OVERRIDE = 0.25f;

// ---------------- ФУНКЦИИ ----------------
void setupPWM() {
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PIN, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_PIN, CH_RIGHT);
}

int toPWM(float val) {
  val = constrain(val, 0.0f, 1.0f);
  return (int)(val * 255.0f);
}

// нормализация угла (-PI..PI)
float wrapAngle(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

void handleMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mavSerial.available()) {
    uint8_t c = mavSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      switch (msg.msgid) {

        case MAVLINK_MSG_ID_RC_CHANNELS: {
          mavlink_rc_channels_t rc;
          mavlink_msg_rc_channels_decode(&msg, &rc);

          yaw_rc = (rc.chan4_raw - 1500) / 500.0f;
          rc10_raw = rc.chan10_raw;

          lastMsgTime = millis();
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_attitude_t att;
          mavlink_msg_attitude_decode(&msg, &att);

          yaw_actual = att.yaw; // радианы
          break;
        }
      }
    }
  }
}

BrakeMode getMode() {
  if (rc10_raw < 1400) return MODE_OFF;
  if (rc10_raw < 1700) return MODE_MAIN;
  return MODE_BRAKE;
}

float getBrakeForce() {
  if (currentMode == MODE_OFF) return 0.0f;

  if (currentMode == MODE_MAIN) return MAIN_BRAKE_FORCE;

  if (currentMode == MODE_BRAKE) {
    uint32_t t = millis() - brakeStartTime;
    float k = constrain((float)t / (float)RAMP_TIME, 0.0f, 1.0f);
    return BRAKE_MIN + (BRAKE_MAX - BRAKE_MIN) * k;
  }

  return 0.0f;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  mavSerial.begin(57600, SERIAL_8N1, 16, 17);

  setupPWM();
}

// ---------------- LOOP ----------------
void loop() {

  handleMavlink();

  // -------- FAILSAFE --------
  if (millis() - lastMsgTime > 200) {
    currentMode = MODE_OFF;
    heading_locked = false;
  }

  // -------- РЕЖИМ --------
  BrakeMode newMode = getMode();

  if (newMode == MODE_BRAKE && currentMode != MODE_BRAKE) {
    brakeStartTime = millis();
  }

  currentMode = newMode;

  float base_brake = getBrakeForce();

  // -------- АВТОКУРС --------
  float auto_corr = 0.0f;

  if (base_brake > 0.2f) {

    if (!heading_locked) {
      target_yaw = yaw_actual;
      heading_locked = true;
    }

    float error = wrapAngle(target_yaw - yaw_actual);

    if (fabs(yaw_rc) < PILOT_OVERRIDE) {
      auto_corr = error * AUTO_KP;
      auto_corr = constrain(auto_corr, -AUTO_LIMIT, AUTO_LIMIT);
    }

  } else {
    heading_locked = false;
  }

  float final_yaw = yaw_rc + auto_corr;

  // -------- ДИФФЕРЕНЦИАЛЬНОЕ ТОРМОЖЕНИЕ --------
  float left = base_brake + final_yaw * 0.5f;
  float right = base_brake - final_yaw * 0.5f;

  left = constrain(left, 0.0f, 1.0f);
  right = constrain(right, 0.0f, 1.0f);

  ledcWrite(CH_LEFT, toPWM(left));
  ledcWrite(CH_RIGHT, toPWM(right));

  delay(10);
}