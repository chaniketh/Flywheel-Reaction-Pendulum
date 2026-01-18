// ===============================
// Reaction Wheel Inverted Pendulum
// PD Controller (Encoder Based)
// ===============================

// -------- Encoder pins ----------
#define ENC_A 2
#define ENC_B 3

// -------- Motor driver pins -----
#define R_PWM 6
#define L_PWM 5

// -------- Encoder variables -----
volatile long encoderCount = 0;
long lastCount = 0;

// CHANGE THIS after measuring
const float COUNTS_PER_REV = 600.0;

// -------- Control parameters ----
float Kp = 100025.0;
float Kd = 1.2;

// -------- Timing ---------------
unsigned long lastTime = 0;

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  Serial.println("PD Controller Started");
}

// MAIN LOOP

void loop() {

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;
  lastTime = now;

  // ---- Read encoder safely ----
  noInterrupts();
  long count = encoderCount;
  interrupts();

  // ---- Compute angle (rad) ----
  float theta = (count / COUNTS_PER_REV) * 2.0 * PI ;

  // ---- Angular velocity ----
  float theta_dot = ((count - lastCount) / COUNTS_PER_REV) * 2.0 * PI / dt;
  lastCount = count;

  // ---- PD control law ----
  float u = (Kp * theta + Kd * theta_dot);

  // ---- Saturate command ----
  u = constrain(u, -255, 255);

  // ---- Drive motor ----
  driveMotor(u);

  // ---- Debug output ----
  Serial.print("theta (deg): ");
  Serial.print(theta * 180.0 / PI);
  Serial.print(" | theta_dot: ");
  Serial.print(theta_dot);
  Serial.print(" | PWM: ");
  Serial.println(u);

  delay(5); // ~200 Hz loop
}

// ===============================
// MOTOR DRIVER FUNCTION
// ===============================
void driveMotor(float u) {
  int pwm = abs((int)u);

  if (u > 0) {
    analogWrite(R_PWM, pwm);
    analogWrite(L_PWM, 0);
  } else {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, pwm);
  }
}

// ===============================
// ENCODER ISR
// ===============================
void encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}