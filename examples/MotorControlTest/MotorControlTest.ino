#include <Arduino.h>
#include "MotorControl.h"

// Motor control instance (ENA, IN1, IN2, ENB, IN3, IN4)
MotorControl motors(10, 22, 24, 11, 26, 28);

void setup() {
  Serial.begin(9600);
  motors.begin();
  
  Serial.println("Motor Control Test Started");
  Serial.println("================================");
  delay(1000);
}

void loop() {
  // Test 1: Forward movement
  Serial.println("\n[TEST 1] Moving FORWARD at speed 150");
  motors.forward(150);
  delay(3000);
  
  // Test 2: Stop
  Serial.println("[TEST 2] STOP");
  motors.stop();
  delay(1000);
  
  // Test 3: Backward movement
  Serial.println("[TEST 3] Moving BACKWARD at speed 120");
  motors.backward(120);
  delay(3000);
  
  // Test 4: Stop
  Serial.println("[TEST 4] STOP");
  motors.stop();
  delay(1000);
  
  // Test 5: Turn right
  Serial.println("[TEST 5] Turning RIGHT at speed 100");
  motors.turnRight(100);
  delay(2000);
  
  // Test 6: Stop
  Serial.println("[TEST 6] STOP");
  motors.stop();
  delay(1000);
  
  // Test 7: Turn left
  Serial.println("[TEST 7] Turning LEFT at speed 100");
  motors.turnLeft(100);
  delay(2000);
  
  // Test 8: Stop
  Serial.println("[TEST 8] STOP");
  motors.stop();
  delay(1000);
  
  // Test 9: Individual motor control - left only
  Serial.println("[TEST 9] Left motor FORWARD at speed 150");
  motors.setLeftMotor(150, true);
  motors.stopRight();
  delay(2000);
  
  // Test 10: Right motor only
  Serial.println("[TEST 10] Right motor FORWARD at speed 150");
  motors.stopLeft();
  motors.setRightMotor(150, true);
  delay(2000);
  
  // Test 11: Both motors at different speeds (curved path)
  Serial.println("[TEST 11] Curved path - Left 150, Right 80");
  motors.setLeftMotor(150, true);
  motors.setRightMotor(80, true);
  delay(2000);
  
  // Final stop
  Serial.println("[TEST COMPLETE] Stopping all motors");
  motors.stop();
  delay(3000);
  
  Serial.println("\nTest cycle complete. Restarting...");
  delay(2000);
}
