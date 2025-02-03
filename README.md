ESP-IDF Library for ENS160 Gas Sensor and ATH21 Temperature/Humidity Sensor

This library provides a C implementation for interfacing with the ENS160 gas sensor and ATH21 sensor using the ESP-IDF framework. Both sensors communicate via I2C bus.

Key Features:
- Full ESP-IDF framework compatibility
- Simple API for sensor readings
- Error handling and data validation
- Power management support

Known Issues:
If the ENS160 sensor starts returning zero values consistently, adding a 10µF/16V decoupling capacitor near the sensor's power pins typically resolves the issue.

![alt text](image.png)

