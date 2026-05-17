# Wheeled Robot Project Code Documentation

## Files Summary

1. **car1.ino**
   - Advanced line-following robot controller using 5 IR sensors and PD control.
   - Includes turn detection (90 degree left/right and U-turn), lost-line handling, and motor speed correction.

2. **cardirectiontest1.ino**
   - Basic motor direction test sketch.
   - Runs the robot through forward, backward, right, left, and stop in timed sequence.

3. **LineFollower2IRtest2.ino**
   - Simple 2-IR line follower.
   - Reads left/right sensors and chooses movement: forward, left, right, or stop.

4. **line_follower_kinematics.m.txt**
   - MATLAB simulation of differential-drive robot kinematics on a window-shaped track.
   - Computes wheel angular velocities, integrates pose, and animates robot motion for two laps.

---

## car1.ino

```cpp
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
```

---

## cardirectiontest1.ino

```cpp
#define speedL 10
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define speedR 5
//adel ehab adel
void setup()
{
  Serial.begin(9600);

  for(int i=5; i<=10; i++)
  {
    pinMode(i, OUTPUT);
  }
}

void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 150);
  analogWrite(speedR, 150);
}

void backward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(speedL, 150);
  analogWrite(speedR, 150);
}

void left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 0);
  analogWrite(speedR, 150);
}

void right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 150);
  analogWrite(speedR, 0);
}

void stopMotor()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(speedL, 0);
  analogWrite(speedR, 0);
}

void loop()
{
  forward();
  delay(2000);

  backward();
  delay(2000);

  right();
  delay(2000);

  left();
  delay(2000);

  stopMotor();
  delay(2000);
}
```

---

## LineFollower2IRtest2.ino

```cpp
#define speedL 10
 #define IN1 9
 #define IN2 8
 #define IN3 7
 #define IN4 6
 #define speedR 5
 #define sensorL 4
 #define sensorR 3
 int sl=0;
 int sr=0;
 //adel ehab adel
 void setup() {  
for(int i=5;i<=10;i++)
 {
 pinMode(i, OUTPUT); 
}  
pinMode(sensorR, INPUT);
 pinMode(sensorL, INPUT);
 }
 void forword()
 {
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW); 
digitalWrite(IN3, HIGH); 
digitalWrite(IN4, LOW);
 analogWrite(speedL,100);
 analogWrite(speedR,100); 
}
 void backword()
 {
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH); 
digitalWrite(IN3, LOW); 
digitalWrite(IN4, HIGH);
 analogWrite(speedL,100);
 analogWrite(speedR,100);  
}
 void left()
 {
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW); 
digitalWrite(IN3, HIGH); 
digitalWrite(IN4, LOW); 
analogWrite(speedL,0);
 analogWrite(speedR,100); 
}
 void right()
 {
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW); 
digitalWrite(IN3, LOW); 
digitalWrite(IN4, LOW);
 analogWrite(speedL,100);
 analogWrite(speedR,0); 
}
 void stopp(){
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW); 
digitalWrite(IN3, LOW); 
digitalWrite(IN4, LOW); 
analogWrite(speedL,0);
 analogWrite(speedR,0); 
}
 void loop(){
 sl=digitalRead(sensorL);
 sr=digitalRead(sensorR);
 if (sl==0&&sr==0)
 forword();
 else if (sl==0&&sr==1)
 right();
 else if (sl==1&&sr==0)
 left();
 else if (sl==1&&sr==1)
 stopp();  
}
```

---

## line_follower_kinematics.m.txt

```matlab
clear; clc; close all;
% =========================================================================
% 1. ROBOT CAR AND TRACK PARAMETERS
% =========================================================================
rR = 0.0325;         % Radius of Right wheel (meters)
rL = 0.0325;         % Radius of Left wheel (meters)
b  = 0.1;            % Half-width of the robot (meters)
v  = 0.3;            % Target forward velocity (m/s)

% Track dimensions based on the provided image
L_side   = 1.0;      % Length of vertical sides (100cm)
L_bottom = 1.0;      % Length of bottom side (100cm)
R_turn   = 0.5;      % Radius of the top semi-circle (50cm)

% Starting point (Middle of the bottom line)
start_x     = 0.5;   % x-coordinate (exactly in the middle of the 1m line)
start_y     = 0.0;   % y-coordinate
start_theta = 0;     % Initial heading (0 rad = facing right)

% Spin velocity for sharp 90-degree corners (rad/s)
omega_spin = pi/4;   

% =========================================================================
% 2. TIMING AND COMMAND SIGNALS
% =========================================================================
% The track consists of 7 segments for one complete lap:
t_seg = zeros(1, 7);
t_seg(1) = (1.0 - start_x) / v;       % 1. Straight right to bottom-right corner
t_seg(2) = (pi/2) / omega_spin;       % 2. Point turn 90 deg CCW
t_seg(3) = L_side / v;                % 3. Straight up the right side
t_seg(4) = (pi * R_turn) / v;         % 4. Top semi-circle U-turn
t_seg(5) = L_side / v;                % 5. Straight down the left side
t_seg(6) = (pi/2) / omega_spin;       % 6. Point turn 90 deg CCW
t_seg(7) = start_x / v;               % 7. Straight right back to the start mark

t_end = cumsum(t_seg);                % End times for each segment
t_lap = t_end(end);                   % Total time for one complete lap

dt = 0.003;
t = 0:dt:(2 * t_lap);                 % Run for exactly 2 full laps

% Pre-allocate wheel velocities
phidotR = zeros(1, length(t));
phidotL = zeros(1, length(t));

% Fill the arrays with commands based on the track shape
for i = 1:length(t)
    time_in_lap = mod(t(i), t_lap);
    
    if time_in_lap <= t_end(1)
        % Segment 1: Straight right
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(2)
        % Segment 2: Spin 90 deg CCW in place
        phidotR(i) =  omega_spin * b / rR; 
        phidotL(i) = -omega_spin * b / rL;
    elseif time_in_lap <= t_end(3)
        % Segment 3: Straight up
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(4)
        % Segment 4: Top Semi-circle
        omega_curve = v / R_turn;
        phidotR(i) = (v + omega_curve * b) / rR; 
        phidotL(i) = (v - omega_curve * b) / rL;
    elseif time_in_lap <= t_end(5)
        % Segment 5: Straight down
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(6)
        % Segment 6: Spin 90 deg CCW in place
        phidotR(i) =  omega_spin * b / rR; 
        phidotL(i) = -omega_spin * b / rL;
    else
        % Segment 7: Straight right to complete lap
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    end
end

% =========================================================================
% 3. KINEMATICS INTEGRATION
% =========================================================================
x     = zeros(1, length(t));
y     = zeros(1, length(t));
theta = zeros(1, length(t));

% Apply the starting positions
x(1)     = start_x;
y(1)     = start_y;
theta(1) = start_theta;

for ii = 2:length(t)
    J = [rR*cos(theta(ii-1))/2,  rL*cos(theta(ii-1))/2; ...
         rR*sin(theta(ii-1))/2,  rL*sin(theta(ii-1))/2; ...
         0.5*rR/b,              -0.5*rL/b];
    deltapose = dt * J * [phidotR(ii-1); phidotL(ii-1)];
    x(ii)     = x(ii-1)     + deltapose(1);
    y(ii)     = y(ii-1)     + deltapose(2);
    theta(ii) = theta(ii-1) + deltapose(3);
end

% =========================================================================
% 4. VISUALIZATION & ANIMATION
% =========================================================================
figure('Name', 'Window Shaped Track Animation', 'Position', [100, 100, 600, 700]);
hold on; axis equal; grid on;
title('Differential Drive Robot on Window Track');
xlabel('X (meters)'); ylabel('Y (meters)');
xlim([-0.4, 1.4]); ylim([-0.4, 1.8]);

% Draw the new track
plot([0, 1], [0, 0], 'k--', 'LineWidth', 2); % Bottom straight
plot([1, 1], [0, 1], 'k--', 'LineWidth', 2); % Right straight
plot([0, 0], [1, 0], 'k--', 'LineWidth', 2); % Left straight
ang = linspace(0, pi, 50);
plot(0.5 + 0.5*cos(ang), 1 + 0.5*sin(ang), 'k--', 'LineWidth', 2); % Top semi-circle

% Draw the start mark from the middle of the bottom line
plot([start_x start_x], [-0.05 0.05], 'r-', 'LineWidth', 2);
text(start_x, -0.1, 'Start (Middle)', 'Color', 'r', 'HorizontalAlignment', 'center');

% Define robot shape
chassis_x = [-0.1,  0.1,  0.1, -0.1];
chassis_y = [-0.1, -0.1,  0.1,  0.1];
wheel_x   = [-0.08, 0.08, 0.08, -0.08];
wheelR_y  = [-0.17, -0.17, -0.11, -0.11];
wheelL_y  = [ 0.11,  0.11,  0.17,  0.17];

h_chassis = patch('XData', [], 'YData', [], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k');
h_wheelR  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_wheelL  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_nose    = plot(0, 0, 'r-', 'LineWidth', 3);

% Animation loop 
for ii = 1:10:length(t)
    th    = theta(ii);
    R_mat = [cos(th), -sin(th); sin(th), cos(th)];
    
    rot_chassis = R_mat * [chassis_x; chassis_y];
    rot_wheelR  = R_mat * [wheel_x;   wheelR_y];
    rot_wheelL  = R_mat * [wheel_x;   wheelL_y];
    
    set(h_chassis, 'XData', rot_chassis(1,:) + x(ii), 'YData', rot_chassis(2,:) + y(ii));
    set(h_wheelR,  'XData', rot_wheelR(1,:)  + x(ii), 'YData', rot_wheelR(2,:)  + y(ii));
    set(h_wheelL,  'XData', rot_wheelL(1,:)  + x(ii), 'YData', rot_wheelL(2,:)  + y(ii));
    set(h_nose, 'XData', [x(ii), x(ii) + 0.15*cos(th)], 'YData', [y(ii), y(ii) + 0.15*sin(th)]);
    
    drawnow;
end
fprintf('Finished simulating exactly %d points.\n', length(t));
```
