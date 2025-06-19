// PID Control for DC Motor using LM393 Speed Sensor + L298N

const int ENA = 9;         // PWM to ENA pin on L298N
const int IN1 = 8;         // Motor input 1
const int IN2 = 7;         // Motor input 2
const int speedSensorPin = 2;

volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;

// PID parameters
float kp = 2, ki = 0.4, kd = 0.05;
float error, last_error = 0, integral = 0, derivative = 0;
float targetRPM = 200; // Desired RPM

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

  // Start motor in one direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void loop() {
  if (millis() - lastTime >= 1000) { // every 1000ms (1 second)
    noInterrupts();
    rpm = (pulseCount * 60.0) / 20.0; // 20 slots per revolution
    pulseCount = 0;
    interrupts();

    // PID Control
    error = targetRPM - rpm;
    integral += error;
    derivative = error - last_error;
    float output = kp * error + ki * integral + kd * derivative;
    last_error = error;

    output = constrain(output, 0, 255);
    analogWrite(ENA, output); // Send PWM to ENA pin

    Serial.print("Pulse: "); Serial.print(pulseCount);
    Serial.print(" | RPM: "); Serial.print(rpm);
    Serial.print(" | PWM: "); Serial.println(output);

    lastTime = millis();
  }
}
