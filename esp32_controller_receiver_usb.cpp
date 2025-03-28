#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

#define SERVO_MIN 700
#define SERVO_MID 900
#define SERVO_MAX 1500  // Max PWM for 180 degrees (adjust if needed)
#define SWEEP_DELAY 10

int prevXPosition = SERVO_MID;  // Previous position for X-axis servo (Servo 5)
int prevYPosition = SERVO_MID;  // Previous position for Y-axis servo (Servo 0)

// Scaling factors to increase sensitivity
#define X_SCALE_FACTOR 15  // Decreased value to increase movement per change
#define Y_SCALE_FACTOR 18  // Decreased value to increase movement per change

void setup() {
    Wire.begin(6, 7); // SDA = GPIO6, SCL = GPIO7
    Wire.setClock(400000);
    pwm.begin();
    pwm.setPWMFreq(100); // 50Hz for servos

    // Move Servo 0 from 90 to 0 and back to 90
    
    pwm.setPWM(2, 0, 1100);
    delay(1000);

    // Initialize servos to a neutral position (middle)
    pwm.setPWM(2, 0, 1100); // Servo 0 (Y-axis)
    pwm.setPWM(5, 0, SERVO_MID); // Servo 5 (X-axis)

    // Start serial communication to receive data from Python
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');  // Read the incoming string

        if (command.startsWith("X")) {
            // Read X-axis value and map to servo position
            int xValue = command.substring(1).toInt();  // Get the X value from the command
            int servoXPosition = map(xValue, 0, 640, 500, SERVO_MAX);  // Map X value to servo PWM range
            servoXPosition = (servoXPosition - prevXPosition) / X_SCALE_FACTOR + prevXPosition; // Apply scaling factor to increase sensitivity
            servoXPosition = constrain(servoXPosition, SERVO_MIN, SERVO_MAX);  // Constrain the value to the servo range

            pwm.setPWM(5, 0, servoXPosition);  // Set position of Servo 5 (X-axis)
            prevXPosition = servoXPosition;  // Update previous X position
        }

        if (command.startsWith("Y")) {
            // Read Y-axis value and map to servo position
            int yValue = command.substring(1).toInt();  // Get the Y value from the command
            int servoYPosition = map(yValue, 480, 0, SERVO_MIN, SERVO_MAX);  // Map Y value to servo PWM range
            servoYPosition = (servoYPosition - prevYPosition) / Y_SCALE_FACTOR + prevYPosition; // Apply scaling factor to increase sensitivity
            servoYPosition = constrain(servoYPosition, SERVO_MIN, SERVO_MAX);  // Constrain the value to the servo range

            pwm.setPWM(2, 0, servoYPosition);  // Set position of Servo 0 (Y-axis)
            prevYPosition = servoYPosition;  // Update previous Y position
        }
    }
}
