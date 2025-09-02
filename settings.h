#define SERIAL_BAUD_RATE                  115200
#define FILTER_SAMPLING_RATE              100.0
#define MPU_CALIBRATION_SAMPLES           50
#define STATIONARY_THRESHOLD              0.1f

#define INERTIAL_TRACKING_TASK_STACK_SIZE 512
#define INERTIAL_TRACKING_TASK_PRIORITY   3
#define COMMUNICATION_TASK_STACK_SIZE     512
#define COMMUNICATION_TASK_PRIORITY       1
#define STRING_BUFFER_SIZE                150

#define MQTT_BROKER_ADDRESS               ""
#define MQTT_BROKER_PORT                  1883
#define MQTT_PUBLISH_INTERVAL             1000

#define MQTT_VELOCITY_POSITION_TOPIC      "Arduinertial/PositionVelocity"
#define MQTT_SENSOR_STATE_TOPIC           "Arduinertial/SensorState"
#define MQTT_EULER_ANGLES_TOPIC           "Arduinertial/EulerAngles"

#define WIFI_SSID                         ""
#define WIFI_PASSWORD                     ""