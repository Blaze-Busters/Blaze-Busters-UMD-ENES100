## UMD ENES 100: Autonomous Fire Suppression Vehicle

This project implements the control code and systems for the **Autonomous Off-Terrain Vehicle (OTV)** tasked with completing the **Fire Suppression Mission** as part of the UMD ENES 100 course requirements.


## Mission Objectives

The primary goal of the mission is to successfully navigate the arena, identify the fire's setup, and selectively suppress the flames while meeting all navigational and reporting requirements.

### **Zone 1: Navigation and Identification**

* **Approach:** Navigate the OTV to a position within **150 mm** of the designated fire site.
* **Topography Report:** Identify and transmit the **topography direction** (A, B, or C) that is pointed towards the OTV's starting location.
* **Flame Count:** Measure and correctly transmit the **total number of active flames** present on the pedestal.

### **Zone 2: Fire Suppression**

* **Containment:** Contain the fire by **extinguishing all flames** except for the **middle flame**.

### **Zone 3: Final Navigation**

* **Obstacle Clearance:** Navigate **completely past** the three designated obstacles.
* **Destination:** Navigate **completely into** the final destination zone.


## Fire Suppression Randomization Procedures

To ensure robustness, the mission setup includes several randomized elements that the OTV's code must handle autonomously.

### **1. Fire Pedestal Placement**

* The **center of the pedestal base** is placed on the ground within **50 mm** of the destination, relative to the vision system’s Oxy reference frame.
* The fire pedestal is secured to the arena mats using **Velcro**.

### **2. Fire Topography Orientation**

* The fire pedestal's topography is randomly selected to be oriented in direction **A, B, or C**.

### **3. Active Flame Selection**

* **Middle Flame:** The **middle flame is always lit** and must be included in the total count of active flames.
* **Remaining Flames:** Each of the remaining **four flames** is randomly selected to be lit or unlit.
