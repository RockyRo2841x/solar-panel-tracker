#include <Arduino.h>
#include <avr/io.h>
#include <math.h>

constexpr float KP = 2.5f;
constexpr float KI = 0.01f;
constexpr float KD = 0.5f;
constexpr float DT = 0.01f; 

constexpr unsigned long LDR_INTERVAL_MS   = 50;
constexpr unsigned long PID_INTERVAL_MS   = 10;
constexpr unsigned long LED_INTERVAL_MS   = 250;
constexpr unsigned long PRINT_INTERVAL_MS = 1000;

constexpr float MAX_ANGLE = 70.0f;
constexpr float PULLDOWN_RESISTOR = 10000.0f;

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

uint16_t readADC(uint8_t channel);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void setupHardware();
void driveMotor(float output);
void updateLDRAngle();
void runPID();
void updateLEDDecoder();
void printData();

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

void setupHardware() {
  DDRB |= (1 << DDB1) | (1 << DDB0); 
  DDRD |= (1 << DDD7);               


  DDRB |= (1 << DDB5) | (1 << DDB4);

  PORTB &= ~((1 << PORTB5) | (1 << PORTB4) | (1 << PORTB0));
  PORTD &= ~(1 << PORTD7);

  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 0; // Start with 0 speed
}

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
  int rawLDR1 = constrain(readADC(0), 1, 1022); 
  int rawLDR2 = constrain(readADC(1), 1, 1022); 

  float R1 = PULLDOWN_RESISTOR * ((1023.0f / (float)rawLDR1) - 1.0f);
  float R2 = PULLDOWN_RESISTOR * ((1023.0f / (float)rawLDR2) - 1.0f);

  float R_sum_scaled = (R1 + R2) * 10000.0f;
  float numerator = 70.0f * (R1 - R2) * 10000.0f;
  float denominator = (2.0f * R1 * R2) + R_sum_scaled;

  if (denominator != 0.0f) {
    targetAngle = constrain((numerator / denominator), -MAX_ANGLE, MAX_ANGLE);
  }
}

void runPID() {
  int rawAngle = readADC(2); 
  currentAngle = mapFloat((float)rawAngle, 0.0f, 1023.0f, -MAX_ANGLE, MAX_ANGLE);

  pid.error = targetAngle - currentAngle;

  float P = KP * pid.error;
  
  pid.integral += pid.error * DT;
  pid.integral = constrain(pid.integral, -100.0f, 100.0f); 
  float I = KI * pid.integral;

  float derivative = (pid.error - pid.previousError) / DT;
  float D = KD * derivative;

  pid.output = constrain((P + I + D), -255.0f, 255.0f);
  pid.previousError = pid.error;

  driveMotor(pid.output);
}

void driveMotor(float output) {
  if (abs(pid.error) < 0.8f) {
    OCR1A = 0;
    PORTB &= ~(1 << PORTB0);
    PORTD &= ~(1 << PORTD7);
    return;
  }

  int pwmValue = constrain(abs((int)output), 25, 255); 

  if (output > 0) {

    PORTB |= (1 << PORTB0);
    PORTD &= ~(1 << PORTD7);
    OCR1A = pwmValue;
  } else {
    PORTB &= ~(1 << PORTB0);
    PORTD |= (1 << PORTD7);
    OCR1A = pwmValue;
  }
}

void updateLEDDecoder() {
  float absError = abs(pid.error);
  bool A = false;
  bool B = false;

  if (absError < 5.0f)      { A = true;  B = true;  } 
  else if (absError < 10.0f){ A = true;  B = false; } 
  else if (absError < 15.0f){ A = false; B = true;  } 

  PORTB &= ~((1 << PORTB5) | (1 << PORTB4)); 
  
  if (A) PORTB |= (1 << PORTB5);
  if (B) PORTB |= (1 << PORTB4);
}

void printData() {
  Serial.print("Sun: ");   Serial.print(targetAngle, 2);
  Serial.print(" | Pan: "); Serial.print(currentAngle, 2);
  Serial.print(" | Err: "); Serial.print(pid.error, 2);
  Serial.print(" | PWM: "); Serial.println(pid.output, 2);
}