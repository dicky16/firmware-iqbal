#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <Servo.h>

/* Servo */
Servo servo;
int servoON = 90;
int servoOFF = 0;
int servoStatus = false;

/* WiFi */
const char* ssid = "kandang";
const char* password = "1234567890";

/* DHT Sensor */
#define DHTPIN 5
#define DHTTYPE    DHT11
DHT dht(DHTPIN, DHTTYPE);
float t = 0.0;
float h = 0.0;
unsigned long mSensor = 0;

/* Soil Moisture Sensor */
#define SOIL A0
float soilValue;
#define SERVO D0

/* PH Sensor */
#define PH_PIN D4
float PH;

/* MQTT */
String MQTT_HOST = "mqtt://devmyeco.my.id";
int MQTT_PORT = 1884;
String MQTT_USERNAME = "client";
String MQTT_PASSWORD = "client";

String topicServoValue = "sensor/1/sv";
String topicServoStatus = "sensor/1/ss";
String topicTemp = "sensor/1/temp";
String topicHum = "sensor/1/hum";
String topicSoil = "sensor/1/soil";
String topicPH = "sensor/1/PH";

unsigned long mPublish = 0;

WiFiClient client;
PubSubClient mqtt(client);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived");
  Serial.println(topic);
}

void reconnect() {
  if(WiFi.status() == WL_CONNECTED) {
    if(!mqtt.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (mqtt.connect("esp8266_client", MQTT_USERNAME.c_str(), MQTT_PASSWORD.c_str())) {
        Serial.println("MQTT Connected");
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(" try again in 5 seconds");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  /* DHT Setup */
  dht.begin();

  /* Servo Setup */
  servo.attach(SERVO);

  /* PH Setup */
  pinMode(PH_PIN, INPUT);

  /* Connect WiFi */
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  int check;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(".");
    delay(500);
    check++;
    if(check >= 20) {
      Serial.println("Cannot connect to WiFI.");
      return;
    }
  }

  mqtt.setServer(MQTT_HOST.c_str(), MQTT_PORT);
  mqtt.setCallback(callback);
  if(!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect("esp8266_client", MQTT_USERNAME.c_str(), MQTT_PASSWORD.c_str())) {
      Serial.println("MQTT Connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
  Serial.print("WiFi Connected IP : ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if(millis() - mSensor > 2000) {
    t = dht.readTemperature();
    h = dht.readHumidity();
    if(isnan(t) || isnan(h)) {
      Serial.println("Failed to read DHT Sensor");
    }
    int readPH = analogRead(PH_PIN);
    PH = (-0.0139*readPH)+7.7851;
    int soil = analogRead(SOIL);
    soilValue = (100 - ((soil/1023.00)*100));
    Serial.print("Temp : ");
    Serial.println(t);
    Serial.print("Humd : ");
    Serial.println(h);
    Serial.print("Soil Moisture : ");
    Serial.println(soilValue);
    mSensor = millis();
  }

  if(millis() - mPublish > 700) {
    mqtt.publish(topicServoStatus.c_str(), String(servoStatus).c_str());
    mqtt.publish(topicTemp.c_str(), String(t).c_str());
    mqtt.publish(topicHum.c_str(), String(h).c_str());
    mqtt.publish(topicSoil.c_str(), String(soilValue).c_str());
    mqtt.publish(topicPH.c_str(), String(PH).c_str());
    mPublish = millis();
  }

  if(soilValue <= 70) {
    servo.write(servoON);
    servoStatus = true;
  } else {
    servo.write(servoOFF);
    servoStatus = false;
  }

  mqtt.loop();
  reconnect();
}
