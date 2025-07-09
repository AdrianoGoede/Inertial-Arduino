📘 Overview
ArduinoInertialTracker is a simple yet functional inertial navigation system that leverages sensor fusion algorithms (Mahony/Madgwick) to estimate orientation and position using data from an MPU6050 IMU. The project is developed on an Arduino Uno R4 WiFi and uses a GY-87 sensor module, which includes the MPU6050 and additional sensors.

  ⚠️ This is a learning-oriented project. The tracking may suffer from drift and inaccuracy over time due to sensor limitations and integration errors, but it's a great entry point to understand IMU data processing, sensor fusion, and inertial navigation concepts.

🛠 Hardware Used
Arduino Uno R4 WiFi (or compatible board)

- GY-87 Sensor Module
- MPU6050 (Accelerometer + Gyroscope)
- Breadboard and jumper wires
- USB cable
- (Optional) Battery/power supply for mobile testing


🚀 Features
Sensor fusion using Mahony or Madgwick filters

- Accelerometer and gyroscope calibration
- Real-time orientation (pitch, roll, yaw)
- World-frame acceleration transformation
- Velocity and position estimation with Zero-Velocity Updates (ZUPT)
- Smoothing using exponential filtering
- Planned: WiFi-based transmission of tracking data
- Planned: Real-time web-based visualization


🤝 Contributions
Pull requests are welcome! If you’ve made improvements or want to experiment with new filters or visualization methods, feel free to fork and submit a PR.


📜 License
MIT License – free to use, modify, and distribute.
