
#include "Particle.h"
#include "math.h"
#include "Air_Quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT\Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT\Adafruit_MQTT.h"
#include "credentials.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonOnOff"); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/randomNumber");

/************Declare Variables*************/
unsigned int last, lastTime;
float pubValue;
int subValue;
float num;
const int OnboardLED = D7;
bool buttonOnOff;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

int DUST_SENSOR_PIN = A0;
int SENSOR_READING_INTERVAL = 3000;
int AQS_PIN = A1; // aqs Air Quality Sensor
//int BMEPIN = ; 
int temp;
int pressure;
int humidity;
int getBMEValues;
int moisture;
const int WTRPUMP = D19;
const int MOISTPIN = A2; 
const int OLED_RESET=-1;


AirQualitySensor aqSensor(AQS_PIN);
Adafruit_BME280 bme;
Adafruit_SSD1306 display(OLED_RESET);

//const int DUSTPIN = A1;
unsigned long lastInterval;
unsigned long lowPulseOccupancy;
unsigned long last_lpo;
unsigned long duration;

float ratio = 0;
float concentration = 0;

SYSTEM_MODE(SEMI_AUTOMATIC);

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);
String getAirQuality();
void BMEValues(int temp, int pressure, int humidity);

void getDustSensorReadings();


void setup() {
  Serial.begin(9600);

pinMode(OnboardLED,OUTPUT);

  // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  // Setup MQTT subscription
  mqtt.subscribe(&subFeed);

pinMode(DUST_SENSOR_PIN, INPUT);
pinMode(MOISTPIN, INPUT);
pinMode(WTRPUMP, OUTPUT);
lastInterval = millis();
aqSensor.init();
bme.begin(0x76);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);


  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();

}  

void loop() {

MQTT_connect();
  MQTT_ping();

  pubValue = random(10000);

  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10000))) {
    if (subscription == &subFeed) {
      subValue = atoi((char *)subFeed.lastread);
      Serial.printf("subValue is: %i \n", subValue);
    }
  }

    if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
      pubFeed.publish(pubValue);
      Serial.printf("Publishing %0.2f \n",pubValue); 
       //Serial.printf("Current random number is %i \n", num);
      } 
    lastTime = millis();
  }

buttonOnOff = subValue;
digitalWrite(OnboardLED, buttonOnOff);

moisture = analogRead(A2);

display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.printf("Temp: %i \n", temp);
display.printf("Pressure: %i \n", pressure);
display.printf("Humidity: %i \n", humidity);
display.printlnf("LPO: %d \n", lowPulseOccupancy);
display.printlnf("Ratio: %f%% \n", ratio);
display.printlnf("Concentration: %f \n", concentration);

display.display();
delay(1000);
display.clearDisplay();

duration = pulseIn(DUST_SENSOR_PIN, INPUT);

lowPulseOccupancy = lowPulseOccupancy + duration;
  
  temp= bme.readTemperature();
  pressure= bme.readPressure();
  humidity= bme.readHumidity();

if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
{

  getDustSensorReadings();

  String quality = getAirQuality();
  Serial.printlnf("Air quality is: %s", quality.c_str());

  //BMEValues(temp, pressure, humidity);
  Serial.printlnf("Current temperature %d",temp);
  Serial.printlnf("Current pressure %d", pressure);
  Serial.printlnf("Current humidity %d", humidity);

  Serial.printf("Moisture levels are: %i \n", moisture);
 
  if(moisture > 2000)
  digitalWrite(WTRPUMP, HIGH);
  
  lowPulseOccupancy = 0;

  lastInterval = millis();

}

}

void getDustSensorReadings(){

 if (lowPulseOccupancy == 0){

    lowPulseOccupancy = last_lpo;
  } else {

      last_lpo = lowPulseOccupancy;

  }

  ratio = lowPulseOccupancy / (SENSOR_READING_INTERVAL * 10.0);
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;

  Serial.printlnf("LPO: %d", lowPulseOccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f", concentration);

  // This needs more commentary ________________ this is the divider betwwn dust and air quality

  //String quality = getAirQuality();
  //Serial.printlnf("Air quality is: %s", quality.c_str());
  Serial.printlnf("Air quality is: %i", aqSensor.slope());

}

  String getAirQuality(){
    int quality;
    quality = aqSensor.slope();
    String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
    {
      qual = "Danger";
    } else if (quality == AirQualitySensor::HIGH_POLLUTION)
    {
      qual = "High Pollution";
    } else if (quality == AirQualitySensor::LOW_POLLUTION)
    {
      qual = "Low Pollution";
    } else if (quality == AirQualitySensor::FRESH_AIR) 
    {
      qual = "Fresh Air";
    }

    //Serial.printf("Air Quality is: \n", qual);
  
    return qual;
  };

  void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

  //void BMEValues(int temp, int pressure, int humidity){

    //temp= (int)bme.readTemperature();
    //pressure= (int)(bme.readPressure() / 100.0F);
    //humidity= (int)bme.readHumidity();
  //}