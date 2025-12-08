// Clean main.cpp: instantiate modules and run tasks selected by DIP switch.

#include <Arduino.h>
#include "TurnController.h"
#include "LineSensors.h"
#include "LineFollower.h"
#include "ColorSensor.h"
#include "RGBLED.h"
#include "MotorControl.h"
#include "MoveController.h"
#include "ToFSensor.h"
#include "CircleFollower.h"
#include "Task2_1.h"
#include "Task2_2.h"
#include "RampTask.h"
#include "UltrasonicSensor.h"
#include "GetTaskNumber.h"
#include "Task4_1.h"
#include <Wire.h>

// Motor pins and control (L298N)
const int ENA = 10, IN1 = 22, IN2 = 24;  // Left motor
const int ENB = 11, IN3 = 26, IN4 = 28;  // Right motor
MotorControl motors(ENA, IN1, IN2, ENB, IN3, IN4);

int maxSpeed = 150;  // Maximum motor speed (0-255)
int minSpeed = 50;   // Slow speed near target

// ---------------- Encoder Pins ----------------
#define R_ENC_A 18
#define R_ENC_B 19
#define L_ENC_A 2
#define L_ENC_B 3

volatile long rightPulse = 0;
volatile long leftPulse = 0;

const float wheelDiameter = 6.5; // cm
const int encoderPPR = 1650;     // measured pulses per wheel rotation
const float wheelBase = 14.25;    // distance between wheels in cm (measure your robot)

// MoveController: provides precise distance moves using encoders
MoveController mover(ENA, IN1, IN2, ENB, IN3, IN4, wheelDiameter, encoderPPR, wheelBase, 150, 50);

// Forward declarations for ISRs so attachInterrupt can reference them here
void rightEncoderISR();
void leftEncoderISR();

// Line sensor pins (update if your wiring differs)
const uint8_t sensorPins[] = {52, 50, 48, 46, 44, 42, 40, 38};
const uint8_t sensorCount = 8;

// Line sensor and follower objects
LineSensors sensors(sensorPins, sensorCount);
// LineFollower(ena,in1,in2,enb,in3,in4, baseSpeed, Kp, Ki, Kd)
LineFollower follower(ENA, IN1, IN2, ENB, IN3, IN4, 120, 0.20f, 0.00f, 0.05f);

uint16_t sensorValuesArr[8];

// -------- Color Sensor Setup --------
// RGB LED 1 (left indicator) - pins: R=6, G=4, B=5
RGBLED ledLeft(6, 4, 5);
// RGB LED 2 (right indicator) - pins: R=A6, G=A7, B=A8
RGBLED ledRight(A6, A7, A8);

// Color Sensor 1 (left) - pins: S0=39, S1=41, S2=37, S3=35, OUT=33
ColorSensor colorSensorLeft(39, 41, 37, 35, 33, &ledLeft);
// Color Sensor 2 (right) - pins: S0=A1, S1=A2, S2=A3, S3=A4, OUT=A5
ColorSensor colorSensorRight(A1, A2, A3, A4, A5, &ledRight);

// ToF sensors (XSHUT pins)
ToFSensor tof1(23);
ToFSensor tof2(25);

// Circle follower: follows outer circle, detects opening, transitions to inner circle
CircleFollower circle(&tof1, &tof2, &motors);

// Task 2.1: Line following with IR sensors, encoder moves, 180° rotation
const int LEFT_IR = 36;
const int RIGHT_IR = 34;
Task2_1 task2_1(&sensors, &motors, &mover, LEFT_IR, RIGHT_IR);
// Task 2.2: follow-up task to run after Task2_1
Task2_2 task2_2(&sensors, &motors, &mover, LEFT_IR, RIGHT_IR);
// Ramp task: run after Task2_2 completes
RampTask rampTask(&sensors, &motors, &mover, LEFT_IR, RIGHT_IR);

// Ultrasonic sensor for front distance
#define US_TRIG 32
#define US_ECHO 30
UltrasonicSensor usensor(US_TRIG, US_ECHO);

// DIP switch pins (task selector)
#define DIP1 27
#define DIP2 29
#define DIP3 31
GetTaskNumber taskSelector(DIP1, DIP2, DIP3);

// Task4_1 instance
Task4_1 task4(&sensors, &follower, &usensor, &circle, wheelDiameter, encoderPPR, wheelBase, maxSpeed, minSpeed, 20);

// Start task number read from DIP (0..7)
int startTaskNumber = 0;

void setup() {
	Serial.begin(9600);

	// Start I2C for ToF sensors
	Wire.begin();

	// Initialize motors
	motors.begin();

	// Encoder pins
	pinMode(R_ENC_A, INPUT_PULLUP);
	pinMode(R_ENC_B, INPUT_PULLUP);
	pinMode(L_ENC_A, INPUT_PULLUP);
	pinMode(L_ENC_B, INPUT_PULLUP);

	// Attach interrupts
	attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);

	// Initialize and calibrate line sensors
	Serial.println("Initializing line sensors...");
	sensors.begin();
	Serial.println("Calibrating sensors... Move the robot across the line for ~3 seconds");
	sensors.autoCalibrate(3000);
	Serial.println("Sensor calibration complete!");

	// Initialize RGB LEDs
	ledLeft.begin();
	ledRight.begin();

	// Initialize color sensors
	Serial.println("Initializing color sensors...");
	colorSensorLeft.begin();
	colorSensorRight.begin();
	Serial.println("Color sensors ready!");

	// Initialize ToF sensors: ensure XSHUT pins are held low first
	Serial.println("Initializing ToF sensors...");
	pinMode(23, OUTPUT); pinMode(25, OUTPUT);
	digitalWrite(23, LOW); digitalWrite(25, LOW);
	delay(10);
	if (!tof1.begin(0x30)) {
		Serial.println("Failed to initialize ToF sensor 1");
	} else {
		Serial.println("ToF sensor 1 ready at 0x30");
	}
	if (!tof2.begin(0x31)) {
		Serial.println("Failed to initialize ToF sensor 2");
	} else {
		Serial.println("ToF sensor 2 ready at 0x31");
	}

	// Start circle follower
	circle.begin();
	circle.setOuterTargetDistance(200);   // Follow outer circle at 200 mm
	circle.setInnerTargetDistance(150);   // Follow inner circle at 150 mm
	circle.setOpeningThreshold(300);      // Opening is when both sensors read > 300 mm
	circle.setPID(1.0, 0.0, 0.6);        // PID tuning
	Serial.println("Circle follower ready!");

	// Initialize Task 2.1 (line following with IR sensors)
	task2_1.begin();
	task2_1.setLinePID(0.035, 0.0, 0.015);  // Line-following PID
	task2_1.setBaseSpeed(90);                // Base speed for line following

	// Initialize Task 2.2
	task2_2.begin();
	// Initialize Ramp task
	rampTask.begin();

	// Initialize ultrasonic and DIP selector
	usensor.begin();
	taskSelector.begin();
	startTaskNumber = taskSelector.readTask();
	Serial.print("Selected start task: "); Serial.println(startTaskNumber);

	Serial.println("Initialization complete");
}

// Helper: run task2 sequence (Task2_1 -> move 5 cm -> Task2_2 -> return true when done)
static bool runTask2Sequence() {
	static bool task21Handled = false;
	static bool task22Started = false;

	bool taskDone = task2_1.update();
	if (taskDone && !task21Handled) {
		Serial.println("Task 2.1 COMPLETE! Moving 5cm and starting Task 2.2...");
		mover.moveForwardCm(5);
		task21Handled = true;
	}

	if (task21Handled) {
		if (!task22Started) { task2_2.begin(); task22Started = true; }
		bool done2 = task2_2.update();
		if (done2) {
			Serial.println("Task 2.2 COMPLETE!");
			return true;
		}
	}
	return false;
}

// Placeholder run for Task1 (grid solving). Replace the body when you provide Task1 code.
static bool runTask1() {
	Serial.println("Task1 (grid) not implemented yet. Returning complete.");
	delay(500);
	return true;
}

void loop() {
	static int currentTask = -1;
	static bool sequentialMode = true; // if true, run tasks from startTaskNumber upwards

	if (currentTask == -1) {
		currentTask = startTaskNumber;
		Serial.print("Starting at task: "); Serial.println(currentTask);
	}

	if (currentTask > 7) {
		// All tasks done
		motors.stop();
		while (true) { delay(1000); }
	}

	bool done = false;
	switch (currentTask) {
		case 0:
			done = runTask1();
			break;
		case 1:
			done = runTask2Sequence();
			break;
		case 2:
			// Task 3 = Ramp standalone
			done = rampTask.update();
			break;
		case 3:
			// Task 4 = Task4_1
			{
				static bool task4Started = false;
				if (!task4Started) { task4.begin(); task4Started = true; }
				if (!task4.update()) done = false;
				else done = true;
			}
			break;
		default:
			Serial.print("Task "); Serial.print(currentTask); Serial.println(" not implemented.");
			done = true;
			break;
	}

	if (done) {
		Serial.print("Task "); Serial.print(currentTask); Serial.println(" complete.");
		if (sequentialMode) currentTask++;
		else { // if not sequential, stay on same task
			while (true) { delay(1000); }
		}
	}
}

// ---------------- Encoder ISRs ----------------
void rightEncoderISR() {
	if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) rightPulse++;
	else rightPulse--;
}

void leftEncoderISR() {
	if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) leftPulse--;  // inverted direction
	else leftPulse++;
}

