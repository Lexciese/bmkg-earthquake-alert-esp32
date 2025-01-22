# BMKG Earthquake Alert with ESP32/ Arduino

## Overview
This project integrates BMKG's real-time earthquake data with an ESP32 to notify users of seismic activity. The system scrapes earthquake data from the BMKG website and sends alerts via ESP32 using LEDs, buzzers, or serial communication. The API request process is by microcontroller like ESP32 or any Arduino that have wifi module to connect to internet or hotspot. You can customize the threshold of earthquake to get notified by distance and magnitude. 

## Features
- **Real-time Data Scraping:** Fetches earthquake data from BMKG's website.
- **ESP32 Notifications:** Alerts users of seismic events through audio signals or IoT.
- **Customizable Triggers:** Configure thresholds (e.g., magnitude, location) for alerts.

## Requirements
### Hardware
- ESP32 microcontroller
- LED/Buzzer/Other output devices
- Wi-Fi connection

### Dependencies
- ESP32 Arduino IDE
- ArduinoJson.h by Benoit BLANCHON
- HttpClient.h by Adrian McEwen
- Open Weather Map to get current location

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/bmkg-quake-esp32.git
2. Open the file in Arduino IDE, make sure to have already installed the boards:  "Arduino ESP32 Boards" by Arduino or "esp32" by Espressif Systems
3. Change the SSID, password, location, and your own OpenWeatherMap API key

> #### How to get OpenWeatherMap API key
> Official Documentation: [How to call OpenWeather APIs with a freemium plan](https://openweathermap.org/appid#:~:text=good%20yet%20free-,How%20to%20call%20OpenWeather%20APIs%20with%20a%20freemium%20plan,-The%20API%20key)
> 1. Sign up for an account on the OpenWeatherMap website
> 2. Log in to your account
> 3. Go to your account dashboard
> 4. Click on “My API” in the drop-down menu
> 5. Find your API key (APPID) in the confirmation email
> Your API keys can always be found on your account page
   
6. Install the dependencies: "ArduinoJson" by Benoit Blanchon and "HttpClient" by Adrian McEwen
7. Connect the microcontroller, and click upload
8. Done.

Note:
- By default, the alert notification methods is only by blinking the built-in LED. You can customize anything you want, like adding additional alert method with buzzer.
- Because this program requests API every 2 second interval, if turned on 24 hours, it will consumes ~12 megabyte per day. You can change the interval in the code.
