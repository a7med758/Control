#include <PID_v1.h>

// Define PID variables
double setpoint = 100;    // Desired speed (RPM or any unit)
double input;            // Current speed
double output;           // PWM output to motor

// PID tuning parameters
double Kp = 2.0;  // Proportional gain
double Ki = 5.0;  // Integral gain
double Kd = 1.0;  // Derivative gain

// Create PID instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor control pin
const int motorPin = 9;

// Encoder readings
const int encoderPin = 2; // Example pin for encoder

// Variables for encoder
volatile int encoderCount = 0;
int previousEncoderCount = 0;

// Timer for speed calculation
unsigned long lastTime = 0;
const unsigned long sampleTime = 100; // Time in milliseconds

void setup() {
  Serial.begin(9600);
  
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT);
  
  // Attach interrupt to encoder pin
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  
  // Initialize PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calculate speed every sampleTime milliseconds
  if (currentTime - lastTime >= sampleTime) {
    // Compute the speed
    int encoderDifference = encoderCount - previousEncoderCount;
    double speed = (encoderDifference / (float)sampleTime) * 60000; // Speed in RPM (adjust calculation as per encoder resolution)
    
    // Update PID input
    input = speed;
    
    // Compute PID output
    myPID.Compute();
    
    // Set motor speed (PWM)
    analogWrite(motorPin, output);
    
    // Print values for debugging
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" RPM, Current Speed: ");
    Serial.print(speed);
    Serial.print(" RPM, Output: ");
    Serial.println(output);
    
    // Update for next iteration
    previousEncoderCount = encoderCount;
    lastTime = currentTime;
  }
}

// Encoder ISR to count pulses
void encoderISR() {
  encoderCount++;
}
