#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE8E8F0F0E1LL;
RF24 radio(7, 8);

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte AUX3;
  byte AUX4;
};

MyData data;

// ================= BOTONES =================
// Derecha (ROLL + PITCH)
const int RbtnLeft = 3;
const int RbtnRight = 9;
const int RbtnForward = 10;
const int RbtnBackward = 6;

// Izquierda (YAW)
const int LbtnLeft = A1;
const int LbtnRight = A4;
const int LbtnUp = A5;
const int LbtnDown = A0;

// estados actuales
int RbtnRightS, RbtnLeftS, RbtnForwardS, RbtnBackwardS;
int LbtnRightS, LbtnLeftS;

// estados anteriores
int prevRbtnRightS = HIGH;
int prevRbtnLeftS  = HIGH;
int prevRbtnForwardS = HIGH;
int prevRbtnBackwardS = HIGH;

int prevLbtnRightS = HIGH;
int prevLbtnLeftS  = HIGH;

// ================= TRIM =================
int trimRoll = 0;
int trimPitch = 0;
int trimYaw = 0;

const int trimStep = 1;   // fino
const int trimMax  = 30;
unsigned long ledTimer = 0;
const int ledDuration = 100;

// ================= INIT =================
void resetData()
{
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
  //  data.AUX3 = 0;
  //  data.AUX4 = 0;
}

void setup()
{
  //Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // botones con pullup
  pinMode(RbtnRight, INPUT_PULLUP);
  pinMode(RbtnLeft, INPUT_PULLUP);
  pinMode(RbtnForward, INPUT_PULLUP);
  pinMode(RbtnBackward, INPUT_PULLUP);

  pinMode(LbtnRight, INPUT_PULLUP);
  pinMode(LbtnLeft, INPUT_PULLUP);
  //  pinMode(LbtnUp, INPUT_PULLUP);
  //  pinMode(LbtnDown, INPUT_PULLUP);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);

  resetData();
}

// ================= MAP =================
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);

  // DEADZONE
  if (abs(val - middle) < 10) {
    val = middle;
  }

  if (val < middle)
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);

  return (reverse ? 255 - val : val);
}

// ================= LOOP =================
void loop()
{
  // ===== LEER BOTONES =====
  RbtnRightS    = digitalRead(RbtnRight);
  RbtnLeftS     = digitalRead(RbtnLeft);
  RbtnForwardS  = digitalRead(RbtnForward);
  RbtnBackwardS = digitalRead(RbtnBackward);

  LbtnRightS = digitalRead(LbtnRight);
  LbtnLeftS  = digitalRead(LbtnLeft);

  // ===== TRIM POR CLICK =====

  // ROLL
  if (RbtnRightS == LOW && prevRbtnRightS == HIGH) {
    trimRoll += trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  if (RbtnLeftS == LOW && prevRbtnLeftS == HIGH) {
    trimRoll -= trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  // PITCH
  if (RbtnForwardS == LOW && prevRbtnForwardS == HIGH) {
    trimPitch += trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  if (RbtnBackwardS == LOW && prevRbtnBackwardS == HIGH) {
    trimPitch -= trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  // YAW (lado izquierdo)
  if (LbtnRightS == LOW && prevLbtnRightS == HIGH) {
    trimYaw += trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  if (LbtnLeftS == LOW && prevLbtnLeftS == HIGH) {
    trimYaw -= trimStep;
    digitalWrite(2, HIGH);
    ledTimer = millis();
  }

  // limitar
  trimRoll  = constrain(trimRoll, -trimMax, trimMax);
  trimPitch = constrain(trimPitch, -trimMax, trimMax);
  trimYaw   = constrain(trimYaw, -trimMax, trimMax);

  // guardar estados
  prevRbtnRightS    = RbtnRightS;
  prevRbtnLeftS     = RbtnLeftS;
  prevRbtnForwardS  = RbtnForwardS;
  prevRbtnBackwardS = RbtnBackwardS;

  prevLbtnRightS = LbtnRightS;
  prevLbtnLeftS  = LbtnLeftS;

  // ===== JOYSTICKS =====
  data.throttle = mapJoystickValues(analogRead(A6), 0, 508, 1023, false);

  data.yaw = constrain(
               mapJoystickValues(analogRead(A7), 0, 525, 1023, true) + trimYaw,
               0, 255
             );

  data.pitch = constrain(
                 mapJoystickValues(analogRead(A3), 0, 508, 1023, false) + trimPitch,
                 0, 255
               );

  data.roll = constrain(
                mapJoystickValues(analogRead(A2), 0, 528, 1023, false) + trimRoll,
                0, 255
              );

  data.AUX1 = digitalRead(4);
  data.AUX2 = digitalRead(5);
  //  data.AUX3 = !digitalRead(A5);
  //  data.AUX4 = !digitalRead(A0);

  // ===== DEBUG SERIAL =====
  //  Serial.print("THR: "); Serial.print(data.throttle);
  //  Serial.print(" | YAW: "); Serial.print(data.yaw);
  //  Serial.print(" | PITCH: "); Serial.print(data.pitch);
  //  Serial.print(" | ROLL: "); Serial.print(data.roll);
  //  Serial.print(" | AUX1: "); Serial.print(data.AUX1);
  //  Serial.print(" | AUX2: "); Serial.print(data.AUX2);
  //  Serial.print(" | AUX3: "); Serial.print(data.AUX3);
  //  Serial.print(" | AUX4: "); Serial.print(data.AUX4);
  //  Serial.println();

  // ===== ENVIAR =====
  if (millis() - ledTimer > ledDuration) {
    digitalWrite(2, LOW);
  }

  radio.write(&data, sizeof(MyData));
}
