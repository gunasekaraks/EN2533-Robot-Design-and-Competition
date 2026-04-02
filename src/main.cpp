// ESP32 Built-in LED Blink Program

#include <Arduino.h>

// Built-in LED pin for ESP32 (typically GPIO2)
#define LED_BUILTIN 2

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

