// 3RPS Ball Balancer - Direct Servo Control
// Speed optimized version

#include <Servo.h>
#include <TouchScreen.h>
#include <math.h>

// ───────────────────────────────────────────────
// TOUCHSCREEN
// ───────────────────────────────────────────────
#define XP 9
#define YP A2
#define XM A3
#define YM 8
#define TS_OHMS 765
#define MIN_PRESSURE 0

TouchScreen ts = TouchScreen(XP, YP, XM, YM, TS_OHMS);

// ───────────────────────────────────────────────
// SERVOS
// ───────────────────────────────────────────────
Servo servoA;
Servo servoB;
Servo servoC;

#define PIN_A 3
#define PIN_B 5
#define PIN_C 6

#define HOME_ANGLE 105
#define SERVO_MIN  60
#define SERVO_MAX  160
int basec = 144;
int basea = 72;
int baseb = 103;
// ───────────────────────────────────────────────
// TOUCHSCREEN CENTER
// ────────

double Xoffset = 515;
double Yoffset = 500;

// ───────────────────────────────────────────────
// PID CONSTANTS
// Tune kp first, then add kd, then ki last
// ───────────────────────────────────────────────
double kp_y = 0.0375;   // increase if reacting too slow
double ki_y = 0.0015;    // add last, prevents drift
double kd_y = 1.45;    // add after kp works, prevents overshoot

double kp_x = 0.061;   // increase if reacting too slow
double ki_x = 0.0009;    // add last, prevents drift
double kd_x = 1.5; 

double error[2]     = {0, 0};
double errorPrev[2] = {0, 0};
double integr[2]    = {0, 0};
double deriv[2]     = {0, 0};
double out[2]       = {0, 0};

bool detected = false;
long timeI;

// ───────────────────────────────────────────────
// MOVE PLATFORM
// ───────────────────────────────────────────────
void moveTo(double tiltX, double tiltY) {
  double angleA = basea - tiltY;
  double angleB = baseb + 0.5 * tiltY + 0.866 * tiltX;
  double angleC = basec + 0.5 * tiltY - 0.866 * tiltX;

  int a = constrain((int)round(angleA), SERVO_MIN, SERVO_MAX);
  int b = constrain((int)round(angleB), SERVO_MIN, SERVO_MAX);
  int c = constrain((int)round(angleC), SERVO_MIN, SERVO_MAX);

  servoA.write(a);
  servoB.write(b);
  servoC.write(c);
}

// ───────────────────────────────────────────────
// PID LOOP
// ───────────────────────────────────────────────
void PID(double setpointX, double setpointY) {
  TSPoint p = ts.getPoint();

  if (p.z > MIN_PRESSURE && p.x > 100 && p.y > 100) {
    detected = true;

    for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];
      error[i] = (i == 0) * (Xoffset - p.x - setpointX)
               + (i == 1) * (Yoffset - p.y - setpointY);

      integr[i] += error[i] + errorPrev[i];

      deriv[i] = error[i] - errorPrev[i];
      deriv[i] = (isnan(deriv[i]) || isinf(deriv[i])) ? 0 : deriv[i];

      out[0] = kp_x * error[0] + ki_x * integr[0] + kd_x * deriv[0];
      out[1] = kp_y * error[1] + ki_y * integr[1] + kd_y * deriv[1];
      out[0] = constrain(out[0], -12.0, 12.0);  // ±20 degrees max tilt
      out[1] = constrain(out[1], -12.0, 12.0);  // ±20 degrees max tilt

    }

    // Serial debug — comment out after tuning for faster loop
    Serial.print("X OUT="); Serial.print(out[0]);
    Serial.print(" Y OUT="); Serial.print(out[1]);
    Serial.print(" Ball X="); Serial.print(p.x);
    Serial.print(" Ball Y="); Serial.println(p.y);

  } else {
    delay(10);
    TSPoint p2 = ts.getPoint();
    if (p2.z <= MIN_PRESSURE || p2.x <= 100 || p2.y <= 100) {
      detected = false;
      integr[0] = 0;
      integr[1] = 0;
    }
  }

  // Faster 10ms loop
  timeI = millis();
  while (millis() - timeI < 10) {
    if (detected) {
      moveTo(-out[0], out[1]);
    } else {


      
      moveTo(0, 0);
    }
  }
}

// ───────────────────────────────────────────────
// SETUP
// ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  servoA.attach(PIN_A);
  servoB.attach(PIN_B);
  servoC.attach(PIN_C);

  servoA.write(basea);
  servoB.write(baseb);
  servoC.write(basec);

  delay(1000);
  Serial.println("Platform ready!");
}

// ───────────────────────────────────────────────
// MAIN LOOP
// ───────────────────────────────────────────────
void loop() {
  PID(0, 0);
}