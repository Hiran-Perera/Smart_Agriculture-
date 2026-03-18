# Smart Agriculture LoRa Network-

A secure, low power smart agriculture system built for long range field monitoring and irrigation control.

This project uses LoRa communication to connect remote sensor nodes with a central controller. It collects environmental and soil data, protects communication with encryption, reduces node power usage through sleep features, and supports both automatic and manual pump control.

Key strengths of the project:

- Encrypted LoRa communication for safer data transfer
- Sleep based node operation for lower power consumption
- Smart irrigation control using soil moisture data
- Manual and automatic pump control modes
- Reliable communication with acknowledgements and retry handling
- Local real time interface for direct monitoring and control
- Modular software structure for easier upgrades and scaling

## Features

### Secure communication
The system includes encryption support for LoRa packets. This helps protect sensor readings and control commands during transmission. It adds a practical security layer to the network and makes the system stronger than a basic LoRa demo.

### Low power sleep operation
Remote sensor nodes are designed to use sleep based working cycles. The nodes wake up, collect data, transmit it, and return to low power mode. This reduces energy use and makes the design more suitable for battery powered agricultural deployment.

### Smart irrigation control
The project can control a water pump based on soil conditions. This turns the system into an active automation platform instead of only a monitoring system.

### Manual and automatic modes
The pump can be switched manually by the user or controlled automatically by the system logic. This gives flexibility during testing, maintenance, and real use.

### Reliable LoRa network handling
The network logic includes acknowledgement support, retries, and duplicate packet filtering. This improves communication reliability between field nodes and the central controller.

### Real time monitoring
The system can display and process important field values in real time, including sensor data, relay state, battery condition, and control status.

### Local user interface
A TFT based interface allows direct interaction with the system. Users can view live values, navigate screens, and control settings without needing a separate computer.

### Multi parameter sensing
The platform is designed to handle multiple types of data such as:

- Soil moisture
- Temperature
- Humidity
- Light level
- Battery voltage
- Pump or relay state

### Modular code design
The software is split into separate modules for communication, payload handling, automation, data storage, and user interface logic. This makes the project easier to maintain and extend.

## System overview

The project is built around three main parts.

### 1. Sensor nodes
The remote nodes collect agricultural data from the field. These nodes are designed for periodic sensing and transmission using LoRa. Sleep features help reduce power consumption between readings.

### 2. Central node
The central node receives data from remote nodes, validates messages, manages acknowledgements, applies automation rules, and sends commands such as pump control or configuration updates.

### 3. User interface
The interface provides local monitoring and control. It allows the user to check live readings, observe system status, and interact with irrigation settings.

## Main project goals

- Monitor environmental and soil conditions over long range wireless communication
- Reduce power use in remote sensing nodes
- Secure transmitted data with encryption
- Automate irrigation based on sensor input
- Provide a complete end to end smart agriculture solution

## Hardware used

- ESP32 based boards
- LoRa transceiver modules
- Soil moisture sensor
- Temperature and humidity sensor
- Light sensor
- Relay module for pump control
- TFT display
- Rotary encoder or local input controls
- Battery or external power source

## Software structure

The project is organized into separate files for different tasks such as:

- Main central node logic
- LoRa packet definitions
- Communication manager
- Automation logic
- UI state and screen drawing
- Input handling
- Data model management
- Encryption support

This modular structure helps future development and testing.

## How it works

1. Remote nodes wake from sleep mode
2. Sensor values are collected
3. Data is packed and transmitted using LoRa
4. Encrypted packets are received by the central node
5. The central node validates and processes the data
6. The system checks irrigation logic and control conditions
7. Pump commands can be triggered automatically or manually
8. Live information is shown on the local interface

## Applications

This system is suitable for:

- Smart irrigation research
- Precision agriculture prototypes
- Low power wireless monitoring
- Secure IoT field deployments
- Student and academic embedded systems projects

## Future improvements

- Cloud dashboard integration
- Firebase or web monitoring
- Historical data logging
- Solar powered node support
- Mobile alerts
- AI based irrigation prediction
- More sensor nodes and larger deployment coverage
