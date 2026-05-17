// ─────────────────────────────────────────────
//               PINS CONFIG
// ─────────────────────────────────────────────

#define S0  A4
#define S1  A3
#define S2  A2
#define S3  A1
#define S4  A0

const int ENA = 5,  IN1 = 8,  IN2 = 9;
const int ENB = 6,  IN3 = 10, IN4 = 11;

const int buzzer = 13;
const int led = 7;


// ─────────────────────────────────────────────
//               TUNING PARAMETERS
// ─────────────────────────────────────────────

int BASE_SPEED = 130;
int MAX_SPEED  = 200;
int TURN_SPEED = 150;

float Kp = 35.0;
float Kd = 20.0;


// ─────────────────────────────────────────────
//               STATE VARIABLES
// ─────────────────────────────────────────────

int s[5];
float lastError = 0;

const int W[5] = { -2, -1, 0, 1, 2 };


// ─────────────────────────────────────────────
//               MOTOR CONTROL
// ─────────────────────────────────────────────

void motor(int ena, int a, int b, int spd) {
  digitalWrite(a, spd >= 0 ? LOW  : HIGH);
  digitalWrite(b, spd >= 0 ? HIGH : LOW);
  analogWrite(ena, constrain(abs(spd), 0, 255));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


// ─────────────────────────────────────────────
//               SENSOR READING
// ─────────────────────────────────────────────

void readSensors() {
  for (int i = 0; i < 5; i++)
    s[i] = (digitalRead(A0 + i) == LOW) ? 1 : 0;
}

int activeSensors() {
  return s[0] + s[1] + s[2] + s[3] + s[4];
}

float computeError() {
  float sum = 0;
  int total = 0;

  for (int i = 0; i < 5; i++) {
    sum += W[i] * s[i];
    total += s[i];
  }

  return (total == 0) ? lastError : sum / total;
}


// ─────────────────────────────────────────────
//               TURN DETECTION
// ─────────────────────────────────────────────

bool isUTurn()   { return activeSensors() == 5; }
bool is90Right() { return s[4] == 1 && s[0] == 0; }
bool is90Left()  { return s[0] == 1 && s[4] == 0; }
bool isLost()    { return activeSensors() == 0; }


// ─────────────────────────────────────────────
//        SPIN UNTIL LINE IS FOUND
// ─────────────────────────────────────────────

void spinUntilLine(int leftSpd, int rightSpd) {

  motor(ENA, IN1, IN2, leftSpd);
  motor(ENB, IN3, IN4, rightSpd);
  delay(150);

  unsigned long timeout = millis() + 1500;

  while (millis() < timeout) {
    motor(ENA, IN1, IN2, leftSpd);
    motor(ENB, IN3, IN4, rightSpd);

    if (digitalRead(S3) == LOW) break;
  }

  stopMotors();
  delay(20);
  lastError = 0;
}


// ─────────────────────────────────────────────
//                   SETUP
// ─────────────────────────────────────────────

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  // startup signal
  digitalWrite(buzzer, HIGH); delay(100);
  digitalWrite(buzzer, LOW);  delay(100);
  digitalWrite(buzzer, HIGH); delay(100);
  digitalWrite(buzzer, LOW);

  digitalWrite(led, HIGH); delay(300);
  digitalWrite(led, LOW);

  delay(500);
}


// ─────────────────────────────────────────────
//                   MAIN LOOP
// ─────────────────────────────────────────────

void loop() {

  readSensors();

  // ── U TURN ─────────────────────────────
  if (isUTurn()) {
    digitalWrite(led, HIGH);
    stopMotors(); delay(80);
    spinUntilLine(TURN_SPEED, -TURN_SPEED);
    digitalWrite(led, LOW);
    return;
  }

  // ── 90 RIGHT ───────────────────────────
  if (is90Right()) {
    digitalWrite(led, HIGH);
    stopMotors(); delay(20);
    spinUntilLine(TURN_SPEED, -TURN_SPEED);
    digitalWrite(led, LOW);
    return;
  }

  // ── 90 LEFT ────────────────────────────
  if (is90Left()) {
    digitalWrite(led, HIGH);
    stopMotors(); delay(20);
    spinUntilLine(-TURN_SPEED, TURN_SPEED);
    digitalWrite(led, LOW);
    return;
  }

  // ── LOST LINE ──────────────────────────
  if (isLost()) {
    digitalWrite(led, HIGH);
    motor(ENA, IN1, IN2, -80);
    motor(ENB, IN3, IN4, -80);
    return;
  }

  // ── NORMAL FOLLOW (PD CONTROL) ────────
  digitalWrite(led, LOW);

  float error = computeError();
  float correction = (Kp * error) + (Kd * (error - lastError));

  lastError = error;

  int L = constrain(BASE_SPEED + (int)correction, 0, MAX_SPEED);
  int R = constrain(BASE_SPEED - (int)correction, 0, MAX_SPEED);

  motor(ENA, IN1, IN2, L);
  motor(ENB, IN3, IN4, R);
}