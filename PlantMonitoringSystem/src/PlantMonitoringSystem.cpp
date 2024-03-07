
#include "Particle.h"
#include "math.h"
#include "Air_Quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"


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
  //void BMEValues(int temp, int pressure, int humidity){

    //temp= (int)bme.readTemperature();
    //pressure= (int)(bme.readPressure() / 100.0F);
    //humidity= (int)bme.readHumidity();
  //}