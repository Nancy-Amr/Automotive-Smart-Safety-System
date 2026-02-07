# Automotive Safety & Rear Parking Assistance System

This project is a real-time embedded safety system. It integrates intelligent door locking and ultrasonic-based parking assistance into a unified platform using the **TM4C123GH6PM microcontroller** and **FreeRTOS**.

## 🚗 Project Overview

The system provides two core automotive safety functionalities:

1. **Intelligent Door Locking:** Automatically manages vehicle locks based on speed and ignition state.


2. **Rear Parking Assistance:** Detects obstacles while reversing and provides visual and auditory alerts.



The project demonstrates embedded design principles such as task synchronization, interrupt management, and resource sharing.

## 🛠 Hardware Components

The system interfaces several peripherals with the ARM Cortex-M4-based MCU:

* **TM4C123GH6PM Microcontroller:** The central processing unit operating at 80 MHz.


* **Ultrasonic Sensor (HC-SR04):** Measures rear obstacle distance in centimeters.


* **Potentiometer:** Simulates vehicle speed via ADC (scaled 0–30 km/h).


* **Gear Selection Switches:** Simulates Park (P), Drive (D), and Reverse (R) positions.


* **User Interface:**
* **LCD Display ( ):** Shows speed, lock status, gear, and distance.


* **RGB LED:** Visual proximity indicator (Green/Yellow/Red).


* **Buzzer:** Auditory alert with variable beeping frequency.





---

## 💻 Software Architecture

The software is built on **FreeRTOS**, employing a modular task-based architecture to ensure real-time responsiveness.

### Task Breakdown & Prioritization

Tasks are prioritized based on safety and timing urgency:

1. **Priority 4 (Highest) - SwitchTask:** Monitors gear switches and ensures valid transitions for safety.


2. **Priority 3 - Ignition & Door Lock Tasks:** Handles ignition transitions and manual lock overrides.


3. **Priority 2 - Sensing & Alerts:** Includes `SensorTask` (ultrasonic), `PotentiometerTask` (speed), and `BuzzerTask` (audio alerts).


4. **Priority 1 (Lowest) - LCDTask:** Manages display updates from a serialized message queue.



### Inter-Task Communication & Synchronization

* **Message Queues:** `lcdQueue` for display updates and `distanceQueue` for transmitting readings to the buzzer.


* **Mutexes & Semaphores:** Used to protect shared variables like `xSpeedMutex`, `doorStateSemaphore`, and `lcdSemaphore` to prevent race conditions.



---

## 🚦 Functional Behavior

* **Auto-Locking:** Doors lock automatically when speed exceeds **10 km/h**. All doors unlock when the ignition is turned **OFF**.


* **Safety Interlocks:** If the door is open while in Drive or Reverse, the system forces speed to **0 km/h**, prevents gear engagement, and displays "Close Door First".


* **Parking Proximity Alerts:**
* **Safe (>100 cm):** Green LED.


* **Caution (30–100 cm):** Yellow LED and moderate beeping.


* **Danger (<30 cm):** Red LED and rapid beeping.


* **Emergency Stop ( cm):** Vehicle is forced to a "Stopped" state.

