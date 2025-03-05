#include <Arduino.h>
#include <Romi32U4Motors.h>
#include <Romi32U4Encoders.h>

Romi32U4Motors motors;  // Motor control object
Romi32U4Encoders encoders;  // Encoder object

void setup() {
    Serial.begin(9600);  // Start Serial Monitor for debugging
    delay(1000);  // Allow time for Serial Monitor to start
}

void loop() {
    Serial.println("Moving forward...");
    motors.setEfforts(100, 100);  // Move forward at full speed
    delay(2000);  // Run for 2 seconds

    Serial.println("Stopping...");
    motors.setEfforts(0, 0);  // Stop motors
    delay(1000);

    // Read and print encoder values
    Serial.print("Left Encoder: ");
    Serial.print(encoders.getCountsLeft());
    Serial.print(" | Right Encoder: ");
    Serial.println(encoders.getCountsRight());

    delay(2000);  // Wait before repeating loop
}

