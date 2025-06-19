const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;
const int speedSensorPin = 2;

volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;

float targetRPM = 30; // Setpoint

void rpmCounter() {
  pulseCount++;
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(speedSensorPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(speedSensorPin), rpmCounter, RISING);
  Serial.begin(9600);

  // Start motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void loop() {
  if (millis() - lastTime >= 1000) { // 1 second interval
    noInterrupts();
    rpm = (pulseCount * 60.0) / 20.0; // 20 slots per rev
    pulseCount = 0;
    interrupts();

    float error = targetRPM - rpm;
    int pwm = fuzzyControl(error);

    analogWrite(ENA, pwm);

    Serial.print("RPM: "); Serial.print(rpm);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | PWM: "); Serial.println(pwm);

    lastTime = millis();
  }
}

int fuzzyControl(float error) {
  int pwm;

  // Fuzzy logic based on error magnitude
  if (error > 20) {
    pwm = 255;  // Very low speed, need full power
  }
  else if (error > 10) {
    pwm = 200;
  }
  else if (error > 5) {
    pwm = 150;
  }
  else if (error > 2) {
    pwm = 120;
  }
  else if (error > -2) {
    pwm = 100;  // Near target, hold steady
  }
  else if (error > -5) {
    pwm = 80;
  }
  else if (error > -10) {
    pwm = 60;
  }
  else {
    pwm = 40;  // Too fast, reduce speed a lot
  }

  pwm = constrain(pwm, 0, 255);
  return pwm;
}
