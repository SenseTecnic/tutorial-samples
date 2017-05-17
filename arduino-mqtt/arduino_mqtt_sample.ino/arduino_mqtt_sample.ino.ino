#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"
#include <PubSubClient.h>
#include "DHT.h"

#define ESP_SSID                                "Your SSID"
#define ESP_PASS                                "Your Wifi Password"

#define mqtt_server                             "mqtt.sensetecnic.com"
#define mqtt_port                               1883
#define mqtt_client_id                          "Your MQTT Client ID"
#define mqtt_client_user                        "Your STS Username"
#define mqtt_client_pw                          "Your MQTT Client Password"
#define mqtt_client_topic_send_status           "users/<Your Username>/arduino/status"
#define mqtt_client_topic_send_temperature      "users/<Your Username>/arduino/send/temperature"
#define mqtt_client_topic_send_humidity         "users/<Your Username>/arduino/send/humidity"
#define mqtt_client_topic_receive_led           "users/<Your Username>/L"
#define birthMsg                                "Arduino Client connected"
#define willMsg                                 "Arduino Client disconnected"

#define LEDpin 8
#define DHTPIN 9
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

String temp_str; 
String hum_str;
char temp[50];
char hum[50];
float humidityReading;  //Stores humidity value
float temperatureReading; //Stores temperature value
long lastMsg = 0;
long lastLoop = 0;

int status = WL_IDLE_STATUS;   // the Wifi radio's status

WiFiEspClient espClient;

//print any message received for subscribed topic
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if((char)payload[0] == '1') {
    Serial.println("LED is on");
    digitalWrite(LEDpin, HIGH);
  } else if ((char)payload[0] == '0'){
    Serial.println("LED is off");
    digitalWrite(LEDpin, LOW);
  }
}

PubSubClient client(mqtt_server, mqtt_port, callback, espClient);

SoftwareSerial soft(6, 7); // RX, TX

void setup() {
  // initialize serial for debugging
  Serial.begin(9600);
  // initialize serial for ESP module
  soft.begin(9600);
  // initialize ESP module
  WiFi.init(&soft);

  dht.begin();
  pinMode(LEDpin, OUTPUT);
  
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ESP_SSID);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ESP_SSID, ESP_PASS);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");  
}

void sendData() {

  humidityReading = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temperatureReading = dht.readTemperature();

  Serial.print("Humidity reading: ");
  Serial.println(humidityReading);
  Serial.print("Temperature reading: ");
  Serial.println(temperatureReading);

  if (isnan(humidityReading) || isnan(temperatureReading)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  temp_str = String(temperatureReading); 
  temp_str.toCharArray(temp, temp_str.length() + 1); 

  hum_str = String(humidityReading); 
  hum_str.toCharArray(hum, hum_str.length() + 1);   

  client.publish(mqtt_client_topic_send_temperature, temp);
  client.publish(mqtt_client_topic_send_humidity, hum);

}

void loop() {

  long now = millis(); 

  if (now - lastMsg > 15000) {
    lastMsg = now;
    sendData();
  }  
  
  if (now - lastLoop > 200) {
    lastLoop = now;
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect(mqtt_client_id, mqtt_client_user, mqtt_client_pw, mqtt_client_topic_send_status, 1, 1, willMsg)) {
      Serial.println("connected");
      client.publish(mqtt_client_topic_send_status, birthMsg);
      Serial.println(client.subscribe(mqtt_client_topic_receive_led));
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

