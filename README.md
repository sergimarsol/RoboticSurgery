# Robotic Surgery

## Overview

Welcome to the Robotic Surgery Project repository! This project aims to show the implementation of the integration of advanced robotic technologies into the operating room. The use of robotics in surgery brings precision, flexibility, and enhanced control, ultimately improving patient outcomes. This readme provides an overview of the project, including how it works and the ideal surgical procedure. You will find the main code in the file [robotic_surgery.py](./robotic_surgery.py)

## How It Works

The robotic surgery system uses three microprocessors ESP32: a slave, a master, and one for the servomotors. The master microprocessor receives input from two push buttons (s1 and s2) and from a IMU sensor for 3D orientation. It sends the RPW (3D orientation) and the input from the s1 and s2 buttons to the servomotors. The microprocessor from the servomotors performs the movements and sends back the filtered torque to the master. Then, the master sends the RPW, the buttons' inputs and the torque to the slave, which shows the simulation of the movements in RoboDK.

The robotic surgery system operates in multiple modes, each designed to address specific surgical needs. Below you can find the inputs with which the idfferent modes of operation can be controlled.

### Mode 1 – Gripper orientation

- Keyboard "g" or push s1 and s2
- Orientation controlled by Master

### Mode 2 – Endowrist orientation

- Keyboard "e"
- Orientation controlled by master

### Mode 3 – Gripper open/close

- Keyboard "c" or push s1 to close gripper
- Keyboard "o" or push s2 to open gripper

### Mode 4 – Endowrist up/down

- Keyboard "u" to move up
- Keyboard "d" to move down

## Surgical procedure

The ideal surgical procedure using the robotic surgery system involves a seamless integration of technology and surgical expertise. Here are key aspects of an optimal surgical procedure:

### 1. Preoperative Planning

Utilize the system's advanced planning tools to meticulously plan the surgery. This includes selecting the appropriate mode of operation, defining key points for robotic assistance, and optimizing the surgical approach.

### 2. Collaborative Execution

During the procedure, work collaboratively with the robotic system, leveraging its precision and capabilities. Maintain control over critical decision-making aspects while allowing the system to handle routine and repetitive tasks, optimizing efficiency and reducing surgical fatigue.

### 3. Real-time Adaptability

Take advantage of real-time feedback and adaptability offered by the robotic system. Adjust the surgical plan as needed based on intraoperative findings, ensuring flexibility and responsiveness to unforeseen challenges.

### 4. Postoperative Analysis

After the surgery, analyze data collected during the procedure to assess outcomes and refine future surgical approaches. Continuous improvement based on real-world feedback ensures ongoing optimization of the robotic surgery system.

## Getting Started

Thank you for joining me on this journey to see how robotic technologies can redefine the future of surgery! If you have any questions or feedback, feel free to reach out to me.

Happy operating!
