# BMKG Earthquake Scrapper with ESP32/ Arduino

## Overview
This project integrates BMKG's real-time earthquake data with an ESP32 to notify users of seismic activity. The system scrapes earthquake data from the BMKG website and sends alerts via ESP32 using LEDs, buzzers, or serial communication. The web scrapping process is by microcontroller like ESP32 or any Arduino that have wifi module to connect to internet or hotspot. 

## Features
- **Real-time Data Scraping:** Fetches earthquake data from BMKG's website.
- **ESP32 Notifications:** Alerts users of seismic events through audio signals or IoT.
- **Customizable Triggers:** Configure thresholds (e.g., magnitude, location) for alerts.

## Requirements
### Hardware
- ESP32 microcontroller
- LED/Buzzer/Other output devices
- Wi-Fi connection

### Software
- Python 3.x
- Libraries: `requests`, `beautifulsoup4`
- ESP32 Arduino IDE or MicroPython firmware

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/bmkg-quake-esp32.git
