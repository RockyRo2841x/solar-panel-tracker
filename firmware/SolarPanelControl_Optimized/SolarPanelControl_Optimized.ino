#include <Arduino.h>
#include <avr/io.h>
#include <math.h>

// ======================================================
// TASK 1 - AUTOMATIC SOLAR PANEL CONTROL SYSTEM
// Requirement-to-code map (useful for the report / video):
//   - Sun-angle formula from LDRs ..... updateLDRAngle()
//   - PID feedback loop ............... runPID()
//   - DC motor H-bridge drive ......... driveMotor()
//   - 4 LEDs on only 2 I/O pins ....... updateLEDDecoder()
//   - Serial telemetry ................ printData()
//   - Non-blocking scheduling .......... loop() + millis()
//   - Required loop frequencies:
//       LDR   -> 20 Hz  (50 ms)
//       PID   -> 100 Hz (10 ms)
//       LEDs  -> 4 Hz   (250 ms)
//       Print -> 1 Hz   (1000 ms)
// ======================================================

// ======================================================
// CONFIGURATION & CONSTANTS
// ======================================================

constexpr float KP = 2.5f;
constexpr float KI = 0.01f;
constexpr float KD = 0.5f;
constexpr float DT = 0.01f;   // must match PID_INTERVAL_MS (100 Hz)

constexpr unsigned long LDR_INTERVAL_MS   = 50;   // 20 Hz
constexpr unsigned long PID_INTERVAL_MS   = 10;   // 100 Hz
constexpr unsigned long LED_INTERVAL_MS   = 250;  // 4 Hz
constexpr unsigned long PRINT_INTERVAL_MS = 1000; // 1 Hz

constexpr float MAX_ANGLE = 70.0f;
constexpr float PULLDOWN_RESISTOR = 10000.0f;

// If the panel drives itself INTO a travel limit and stalls there
// (current angle locked at +/-70 with a large error that never shrinks,
// PWM saturated the same direction every cycle), the H-bridge polarity
// is mirrored relative to the feedback potentiometer. Flip this single
// constant to -1 to correct it instead of rewiring anything.
constexpr int MOTOR_DIR_SIGN = 1;

// ======================================================
// SYSTEM STATE & DATA STRUCTURES
// ======================================================

struct PIDController {
  float error = 0.0f;
  float integral = 0.0f;
  float previousError = 0.0f;
  float output = 0.0f;
} pid;

float targetAngle = 0.0f;
float currentAngle = 0.0f;

unsigned long prevLDRMillis   = 0;
unsigned long prevPIDMillis   = 0;
unsigned long prevLEDMillis   = 0;
unsigned long prevPrintMillis = 0;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================
uint16_t readADC(uint8_t channel);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void setupHardware();
void driveMotor(float output);
void updateLDRAngle();
void runPID();
void updateLEDDecoder();
void printData();

// ======================================================
// MAIN LOOP
// ======================================================

void setup() {
  Serial.begin(9600);
  setupHardware();
  Serial.println("Automatic Solar Panel Control System Started");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - prevLDRMillis >= LDR_INTERVAL_MS) {
    prevLDRMillis = currentMillis;
    updateLDRAngle();
  }

  if (currentMillis - prevPIDMillis >= PID_INTERVAL_MS) {
    prevPIDMillis = currentMillis;
    runPID();
  }

  if (currentMillis - prevLEDMillis >= LED_INTERVAL_MS) {
    prevLEDMillis = currentMillis;
    updateLEDDecoder();
  }

  if (currentMillis - prevPrintMillis >= PRINT_INTERVAL_MS) {
    prevPrintMillis = currentMillis;
    printData();
  }
}

// ======================================================
// HARDWARE INITIALIZATION
// ======================================================

void setupHardware() {
  // Motor (L293D H-bridge) pins:
  // D9 (PB1) = ENA (PWM speed), D8 (PB0) = IN1, D7 (PD7) = IN2
  DDRB |= (1 << DDB1) | (1 << DDB0);
  DDRD |= (1 << DDD7);

  // LED decoder pins - only 2 I/O pins drive 4 LEDs via external OR/AND logic
  DDRB |= (1 << DDB5) | (1 << DDB4);

  PORTB &= ~((1 << PORTB5) | (1 << PORTB4) | (1 << PORTB0));
  PORTD &= ~(1 << PORTD7);

  // ADC: AVCC reference, /128 prescaler -> 125 kHz ADC clock
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Timer1: Fast PWM 8-bit, non-inverting, prescaler /8 -> ~7.8 kHz on D9 (ENA)
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 0;
}

// ======================================================
// SENSOR & MATH LOGIC
// ======================================================

uint16_t readADC(uint8_t channel) {
  ADMUX = (1 << REFS0) | (channel & 0x07);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateLDRAngle() {
  int rawLDR1 = constrain(readADC(0), 1, 1022); // A0 = LDR Left
  int rawLDR2 = constrain(readADC(1), 1, 1022); // A1 = LDR Right

  float R1 = PULLDOWN_RESISTOR * ((1023.0f / (float)rawLDR1) - 1.0f);
  float R2 = PULLDOWN_RESISTOR * ((1023.0f / (float)rawLDR2) - 1.0f);

  // theta = 70*(R1-R2)*1e4 / (2*R1*R2 + (R1+R2)*1e4)
  float numerator   = 70.0f * (R1 - R2) * 10000.0f;
  float denominator = (2.0f * R1 * R2) + ((R1 + R2) * 10000.0f);

  if (denominator != 0.0f) {
    targetAngle = constrain((numerator / denominator), -MAX_ANGLE, MAX_ANGLE);
  }
}

// ======================================================
// CONTROL SYSTEM
// ======================================================

void runPID() {
  int rawAngle = readADC(2); // A2 = panel position feedback pot
  currentAngle = mapFloat((float)rawAngle, 0.0f, 1023.0f, -MAX_ANGLE, MAX_ANGLE);

  pid.error = targetAngle - currentAngle;

  float P = KP * pid.error;

  // Anti-windup: stop accumulating the integral once last cycle's output was
  // already saturated and still pushing in the same direction as the error.
  bool saturated = (pid.output >= 255.0f && pid.error > 0.0f) ||
                   (pid.output <= -255.0f && pid.error < 0.0f);
  if (!saturated) {
    pid.integral += pid.error * DT;
    pid.integral = constrain(pid.integral, -100.0f, 100.0f);
  }
  float I = KI * pid.integral;

  float derivative = (pid.error - pid.previousError) / DT;
  float D = KD * derivative;

  pid.output = constrain((P + I + D), -255.0f, 255.0f);
  pid.previousError = pid.error;

  driveMotor(pid.output);
}

// ======================================================
// MOTOR DRIVER CONTROL (L293D / L298N)
// ======================================================

void driveMotor(float output) {
  output *= MOTOR_DIR_SIGN; // correct H-bridge polarity vs. feedback pot if needed

  if (fabsf(pid.error) < 0.8f) { // Deadband
    OCR1A = 0;
    PORTB &= ~(1 << PORTB0);
    PORTD &= ~(1 << PORTD7);
    return;
  }

  int pwmValue = constrain((int)fabsf(output), 25, 255);

  if (output > 0) {
    PORTB |= (1 << PORTB0);  // IN1 HIGH
    PORTD &= ~(1 << PORTD7); // IN2 LOW
    OCR1A = pwmValue;
  } else {
    PORTB &= ~(1 << PORTB0); // IN1 LOW
    PORTD |= (1 << PORTD7);  // IN2 HIGH
    OCR1A = pwmValue;
  }
}

// ======================================================
// PERIPHERALS & DEBUGGING
// ======================================================

void updateLEDDecoder() {
  // Thermometer code on 2 pins -> 4 LEDs via external OR (U1) / AND (U2) gates
  // err<5   -> A=1,B=1 (4 LEDs)   err<10  -> A=1,B=0 (3 LEDs)
  // err<15  -> A=0,B=1 (2 LEDs)   err>=15 -> A=0,B=0 (1 LED, the always-on baseline)
  float absError = fabsf(pid.error);
  bool A = false;
  bool B = false;

  if (absError < 5.0f)       { A = true;  B = true;  }
  else if (absError < 10.0f) { A = true;  B = false; }
  else if (absError < 15.0f) { A = false; B = true;  }

  PORTB &= ~((1 << PORTB5) | (1 << PORTB4));
  if (A) PORTB |= (1 << PORTB5);
  if (B) PORTB |= (1 << PORTB4);
}

void printData() {
  Serial.print("Pan: ");    Serial.print(currentAngle, 2);
  Serial.print(" | Sun: ");  Serial.print(targetAngle, 2);
  Serial.print(" | Err: ");  Serial.print(pid.error, 2);
  Serial.print(" | PWM: ");  Serial.println(pid.output, 2);
}
