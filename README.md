# ESP32 AWS IoT sensor
ESP32 draft project to report data from a light sensor to AWS IoT cloud topic.

1. Connects to WiFi network 
2. Reads data from GY-302 (BH1750) ambient light sensor via I2C bus
3. Publishes sensor raw data to MQTT topic in JSON format
