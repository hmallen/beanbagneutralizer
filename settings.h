// Comment out to log only to Serial
#define MQTT_LOGGING
#define MQTT_LOGGER_DELAY 500

#define MQTT_OUT_TOPIC "beanbag/neutralizer"
#define MQTT_LOGGER_TOPIC "beanbag/logger"
#define MQTT_PUB_MESSAGE "connected"
#define MQTT_IN_TOPIC "beanbag/command/#"
#define MQTT_WILL_TOPIC "beanbag/neutralizer"
#define MQTT_WILL_QOS 0
#define MQTT_WILL_RETAIN 0
#define MQTT_WILL_MESSAGE "disconnected"
