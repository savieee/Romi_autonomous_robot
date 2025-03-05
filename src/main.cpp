#include <Arduino.h>
#include <Romi32U4Motors.h>
#include <Romi32U4Encoders.h>

// Motor and encoder objects
Romi32U4Motors motors;
Romi32U4Encoders encoders;

// PID Controller Parameters
float Kp = 0.;   // Proportional Gain
float Ki = 0.005;  // Integral Gain
float Kd = 0.2;   // Derivative Gain

// Distance Constants
const float WHEEL_CIRCUMFERENCE = 220.0;  // mm (full wheel rotation)
const int COUNTS_PER_WHEEL_ROTATION = 1440;  // Encoder counts per wheel rotation
const float MM_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_ROTATION; // ~0.153 mm per count

// Function to move the robot using PID control
void moveDistancePID(float targetDistance) {
    int targetCounts = targetDistance / MM_PER_COUNT;  // Convert mm to encoder counts

    // Reset encoder counts
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    // PID Variables
    float error, lastError = 0;
    float integral = 0;
    float derivative;
    int baseSpeed = 50;  // Base motor effort
    int maxSpeed = 100;  // Maximum motor effort

    while (true) {
        // Get encoder counts
        int leftCounts = encoders.getCountsLeft();
        int rightCounts = encoders.getCountsRight();
        int avgCounts = (leftCounts + rightCounts) / 2;  // Average count for accuracy

        // Calculate distance traveled
        float currentDistance = avgCounts * MM_PER_COUNT;

        // Compute PID Error
        error = targetDistance - currentDistance;  // How far we are from target
        integral += error;  // Accumulate error (integral term)
        derivative = error - lastError;  // Compute rate of change
        lastError = error;  // Store error for next iteration

        // PID Control Output
        float controlSignal = (Kp * error) + (Ki * integral) + (Kd * derivative);
        int motorEffort = baseSpeed + controlSignal;  // Adjust speed dynamically

        // Constrain motor effort within safe range
        motorEffort = constrain(motorEffort, 0, maxSpeed);

        // Apply motor effort
        motors.setEfforts(motorEffort, motorEffort);

        // Print real-time distance
        Serial.print("Distance Traveled: ");
        Serial.print(currentDistance);
        Serial.println(" mm");

        // Stop when reaching the target distance
        if (currentDistance >= targetDistance) {
            motors.setEfforts(0, 0);  // Stop motors
            Serial.println("Target reached. Stopping.");
            break;
        }

        delay(50);  // Small delay to avoid spamming
    }
}

void setup() {
    Serial.begin(9600);
    delay(1000);
}

void loop() {
    Serial.println("Moving precisely 10 cm...");
    moveDistancePID(100.0);  // Move exactly 100 mm (10 cm)
    delay(5000);  // Wait before repeating
}
