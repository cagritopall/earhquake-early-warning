# earhquake-early-warning
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo servoMotor;
int xpin = A1;
int ypin = A2;
int zpin = A3;
int xvalue;
int yvalue;
int zvalue;
// Variables for calibration
float xOffset = 0.0;
float yOffset = 0.0;
float zOffset = 0.0;
// Buzzer setup
int buzzerPin = 8;
// Servo setup
int servoPin = 10;
int servoStartPosition = 90;
int servoRotation = 90;
// Relay setup
const int relayPin = 4;
// Voice Recognition setup
VR myVR(2, 3); // 2:RX, 3:TX
#define onRecord (0)
#define offRecord (1)
// Threshold for acceleration
float threshold = 1.5;
void setup()
{
Serial.begin(9600);
// Initialize ADXL345
if (!accel.begin())
{
Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
while (1);
}
// Calibrate the accelerometer
calibrate();
// Set up pins
pinMode(buzzerPin, OUTPUT);
pinMode(relayPin, OUTPUT);
digitalWrite(relayPin, LOW); // Initial state: Relay is off
// Attach servo to its pin
servoMotor.attach(servoPin);
servoMotor.write(servoStartPosition);
// Initialize Voice Recognition
myVR.begin(9600);
if (myVR.clear() == 0)
{
Serial.println("Recognizer cleared.");
}
else
{
Serial.println("Not find VoiceRecognitionModule.");
Serial.println("Please check connection and restart Arduino.");
while (1);
}
if (myVR.load((uint8_t)onRecord) >= 0)
{
Serial.println("onRecord loaded");
}
if (myVR.load((uint8_t)offRecord) >= 0)
{
Serial.println("offRecord loaded");
59
}
}
void loop()
{
uint8_t buf[64]; // Declare the buf variable here
// Read and process ADXL335 values
xvalue = analogRead(xpin);
int x = map(xvalue, 253, 382, -100, 100);
float xg = (float)x / (-100.00);
Serial.print(xg);
Serial.print("g ");
yvalue = analogRead(ypin);
int y = map(yvalue, 250, 380, -100, 100);
float yg = (float)y / (-100.00);
Serial.print("\t");
Serial.print(yg);
Serial.print("g ");
zvalue = analogRead(zpin);
int z = map(zvalue, 266, 393, -100, 100);
float zg = (float)z / 100.00;
Serial.print("\t");
Serial.print(zg);
Serial.print("g ");
// Read and process ADXL345 values
sensors_event_t event;
accel.getEvent(&event);
// Apply calibration offsets
event.acceleration.x -= xOffset;
event.acceleration.y -= yOffset;
event.acceleration.z -= zOffset;
// Print the calibrated acceleration values
Serial.print("Calibrated Acceleration (m/s^2): X = ");
Serial.print(event.acceleration.x);
61
Serial.print(", Y = ");
Serial.print(event.acceleration.y);
Serial.print(", Z = ");
Serial.println(event.acceleration.z);
// Check if both X and Y accelerations exceed the threshold
if (abs(event.acceleration.x) > threshold && abs(event.acceleration.y) > threshold)
{
// Activate the buzzer and relay
digitalWrite(buzzerPin, HIGH);
digitalWrite(relayPin, HIGH); // Open the relay circuit
delay(5000); // Wait for 5 seconds
digitalWrite(buzzerPin, LOW);
// Rotate the servo motor 180 degrees anticlockwise
servoMotor.write(0); // Rotate to 0 degrees (180 degrees anticlockwise)
delay(1000); // Wait for the servo to move
// Rotate the servo back to the initial position
servoMotor.write(servoStartPosition);
}
// Voice Recognition
int ret;
ret = myVR.recognize(buf, 50);
if (ret > 0)
{
switch (buf[1])
{
case onRecord:
// Turn on the buzzer
digitalWrite(buzzerPin, HIGH);
delay(5000); // Add a delay for the buzzer sound
digitalWrite(buzzerPin, LOW);
break;
case offRecord:
// Turn off the buzzer
digitalWrite(buzzerPin, HIGH);
delay(200); // Add a different delay for the off sound
digitalWrite(buzzerPin, LOW);
break;
default:
Serial.println("Record function undefined");
break;
}
}
delay(100);
}
void calibrate()
{
int numSamples = 100;
float xSum = 0.0;
float ySum = 0.0;
float zSum = 0.0;
// Collect samples for calibration
64
for (int i = 0; i < numSamples; i++)
{
sensors_event_t event;
accel.getEvent(&event);
xSum += event.acceleration.x;
ySum += event.acceleration.y;
zSum += event.acceleration.z;
delay(10);
}
// Calculate the average offsets
xOffset = xSum / numSamples;
yOffset = ySum / numSamples;
zOffset = zSum / numSamples;
