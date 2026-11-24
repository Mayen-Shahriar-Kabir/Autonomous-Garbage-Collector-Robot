# 🤖 Autonomous Garbage-Collecting Robot with Claw

## Overview
The **Autonomous Garbage-Collecting Robot** is a low-cost, smart robotic system designed to automate the collection of small to medium-sized litter in indoor and semi-indoor environments (such as classrooms, corridors, and campuses). Built around an Arduino Uno and an ESP32-CAM, this robot integrates autonomous navigation, computer vision-based object detection, and mechanical collection into a single platform.

By automating repetitive cleaning tasks, this project aims to improve the cleanliness of shared spaces and minimize manual human contact with unhygienic waste.

## 🎥 Video Demonstration
**Watch the robot in action:** [YouTube Shorts Demo](https://youtube.com/shorts/0xuDSTMGyCg?feature=share)

## ✨ Key Features
* **Autonomous Serpentine Navigation:** Uses dual HC-SR04 ultrasonic sensors to detect walls and obstacles, automatically executing a serpentine lane-change maneuver to systematically sweep an area.
* **Vision-Based Garbage Detection:** Utilizes an ESP32-CAM module to visually detect garbage in the robot's path and send serial commands to the Arduino to trigger the pickup sequence.
* **Mechanical Claw Collection:** Employs two SG90 servo motors to operate a robotic arm and gripper, accurately picking up detected litter and depositing it into an onboard container.
* **Smart Capacity Monitoring:** A 1kg load cell paired with an HX711 amplifier continuously weighs the collected garbage. A buzzer alert is triggered automatically when the bin reaches maximum capacity.
* **Regulated Power System:** Features an LM2596S DC-DC step-down buck converter to isolate and regulate the 5V power supply to sensitive components and servos, preventing voltage drops caused by the DC drive motors.

## 🛠️ Hardware Stack
* **Main Controller:** Arduino Uno R3
* **Vision & Wireless:** ESP32-CAM Wi-Fi + Bluetooth Module
* **Navigation Sensors:** 2x HC-SR04 Ultrasonic Sensors
* **Actuators (Collection):** 2x SG90 Servo Motors (180°)
* **Actuators (Drive):** 4x DC Gear Motors + 4WD Chassis
* **Motor Driver:** L298N Module
* **Weight Monitoring:** 1kg Load Cell + HX711 Amplifier Module
* **Power Management:** LM2596S Buck Converter, 4x 3.7V Batteries (14.8V nominal)
* **Alerts:** Active Buzzer Module

## ⚡ System Architecture
The system uses a master-slave communication architecture. The **Arduino Uno** serves as the master controller, handling all real-time motor control, ultrasonic distance measurements, and weight monitoring. The **ESP32-CAM** acts as a dedicated vision processor, sending encoded serial commands (e.g., `<C>` for centered target, `<G>` for grip) to the Arduino when garbage is spotted. 

To ensure stability, the 14.8V main battery directly feeds the L298N motor driver for high-torque wheel movement, while the LM2596S buck converter steps this voltage down to a stable 5V rail for the Arduino, ESP32-CAM, servos, and sensors.

## 🚀 How It Works (Workflow)
1. **Search Phase:** The robot moves forward, actively scanning the floor with the ESP32-CAM while pinging the ultrasonic sensors to avoid walls.
2. **Detection & Alignment:** When the ESP32-CAM detects garbage, the robot halts and stabilizes. 
3. **Pickup Sequence:** The arm lowers, the claw grips the object, lifts it over the chassis, and drops it into the rear container. The camera feed is temporarily ignored during this sequence to prevent the claw from triggering a false detection.
4. **Weight Check:** The load cell checks if the bin is full. If full, the buzzer sounds; if not, the robot resumes its search pattern.

## 🔮 Future Improvements
* Upgrading to a more powerful object classification model on the ESP32-CAM to separate biodegradable and non-biodegradable waste.
* Adding an automatic return-to-base feature when the bin is full or the battery is low.
* Implementing a mobile application for live video streaming and manual override controls via Wi-Fi.
