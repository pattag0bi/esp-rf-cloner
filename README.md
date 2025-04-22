# ESP32 RF Cloner

A sophisticated RF signal capture and replay system using ESP32 and web interface.

## Features

- Real-time RF signal capture at 433MHz
- Signal storage and management
- Signal replay functionality
- Raw timing analysis
- Web-based control interface
- Signal export capability

## Hardware Requirements

- ESP32 Development Board
- 433MHz RF Transmitter and Receiver
- USB cable for programming and power

## Software Requirements

- Arduino IDE
- Required Libraries:
  - RCSwitch
  - RadioHead
  - BluetoothSerial
  - EEPROM
  - ArduinoJson

## Installation

1. Clone this repository
2. Open `6_esp32_433mhz_cloner.ino` in Arduino IDE
3. Install required libraries through Arduino Library Manager
4. Upload the code to your ESP32
5. Open `index.html` in a modern web browser (Chrome or Edge recommended)

## Usage

1. Connect to the ESP32 through the web interface
2. Use the control panel to:
   - Start/Stop signal capture
   - List captured signals
   - Replay signals
   - Delete signals
   - Export signals
3. View signal details and analysis in the signal library

## Project Structure

- `6_esp32_433mhz_cloner.ino` - ESP32 firmware
- `index.html` - Web interface
- `README.md` - Project documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
# esp-rf-cloner
