README – Landmine Detection Robot
Project Overview
This project involves a semi-autonomous robot capable of detecting metallic landmines, capturing GPS coordinates of detection events, and streaming live video for remote inspection. It uses WiFi for real-time control and communication, and is operated wirelessly using a smartphone app (Blynk).
How to Work with This Prototype
Hardware Components Required - 
•	Arduino Uno (for motor and sensor control)
•	ESP32 (for Blynk communication and live data transfer)
•	ESP32-CAM (for live video streaming)
•	L298N Motor Driver
•	4 × DC Motors
•	Robocraze Metal Detector Module
•	KY-038 Sound Sensor
•	NEO-6M GPS Module
•	Power supply (Li-ion battery or 9V battery pack)
•	Robot chassis + wheels
Control Interface – Smartphone (Blynk App)
•	App Required: Blynk IoT (iOS/Android)
•	Authentication: Enter your Blynk Auth Token from the Arduino code
•	Virtual Pins Used:
o	V4 – Joystick X (Left/Right)
o	V5 – Joystick Y (Forward/Backward)
o	V6 – Slider to control motor speed (0–255)
o	V7/V8 – GPS coordinates displayed on screen when metal is detected
Setup Instructions
1.	Upload the Arduino Code
o	Open the provided .ino file in Arduino IDE.
o	Select the correct board (ESP32 Dev Module).
o	Connect the ESP32 via USB and upload the code.
o	Replace the WiFi credentials and Blynk token with your own.
2.	Hardware Connections
o	Wire the DC motors to the L298N Motor Driver, then connect control pins to Arduino (pins 14, 27, 26, 25, 32, 33, 18, 19).
o	Connect the metal detector module and KY-038 sound sensor to analog pin A0 or GPIO 34 on ESP32.
o	Attach GPS module (TX → GPIO16, RX → GPIO17).
o	Connect ESP32-CAM with separate power (recommended 5V with 2A).
o	Mount everything securely to the robot chassis.
3.	Power On
o	Power both Arduino Uno and ESP32 modules using a rechargeable battery pack.
o	Wait for ESP32 to connect to WiFi (check serial monitor output for confirmation).
Operating the Robot
•	Launch Blynk App on your phone.
•	Use the joystick to navigate the robot:
o	Forward, backward, left, and right control via V4 and V5.
•	Adjust speed using the slider (V6).
•	Live GPS location of metal detection appears in real-time (V7, V8).
•	ESP32-CAM provides live video feed (access via your local IP address – shown in serial monitor).
Metal Detection Behavior
•	The metal detector module senses metallic objects.
•	The KY-038 sound sensor detects beeps from the detector.
•	Upon confirmation, GPS coordinates are automatically captured and sent to the phone.
Live Video Streaming
•	Once ESP32-CAM is powered, it connects to your WiFi.
•	Open the serial monitor in Arduino IDE to get the IP address.
•	Paste the IP into your browser to view the live video stream.
Code Repository
https://docs.google.com/document/d/1B7x8Dk-LP09qeOe1vgF6MmF2n9cfjUkpsGFtlFINjVo/edit?usp=sharing
👤 Team Members and Contributions

Name	Enrollment No.	Contact Info
Shrut Shah	AU2240157	+91 63526 62919
Tanush Mudaliar	AU2240088	+91 74900 56081
Aryan Patel	AU2240248	+91 73837 05055
Aaditya Patel	AU2240209	+91 94294 39851
Adit Srivastav	AU2240111	+91 87448 80303

