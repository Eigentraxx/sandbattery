// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>

// This #include statement was automatically added by the Particle IDE.
#include <spark-dallas-temperature.h>


 
//02.01.24
/*
notes
sparkx address is 0x60
sparkx 2 address is 0x61
ina address Default address is 0x40. removed from project
tsl address is I2C address is 0x29 - not changeable
////
added temp/humidity sensor for inside container
si7021 The default I2C address is 0x40. - not changeable

Measure power in to battery from solar panels.
measure power out by automower in summer. Heater elements for sand battery in winter.
*/

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Si7021.h>

// This #include statement was automatically added by the Particle IDE.
#include <ArduinoJson.h>
// this is needed for json
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1

// This #include statement was automatically added by the Particle IDE.
#include <TinyGPS++.h>

// This #include statement was automatically added by the Particle IDE.
#include "sparkxacs37800.h"

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_TSL2591.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_INA219_.h>
//Adafruit_INA219 ina219; changed to 2nd sparkx

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

ACS37800 mySensor;

ACS37800 mySecondSensor;

TinyGPSPlus gps;


/*
int relayOne = D8;
int relayTwo = D7;
int relayThree = D6;
int relayFour = D5;
*/
bool openRelays = true; // State variable to control relay opening
const int relayPins[] = {5, 6, 7, 8}; // each realy controls one heating element
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
int currentRelayIndex = 0;

const long heaterInterval = 35000;  // how long the heating elements are powered
unsigned long lastAExecution = 0;  //heater elements
unsigned long lastBExecution = 0;
unsigned long programStartTime = 0;

Adafruit_Si7021 sensor = Adafruit_Si7021();
 
char szInfo[96];
char dbLoc[40];
char GUID[40];
char *dbTemp ="eigen_one";
int t = 0;
char *szTemp = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";
char *szHex = "0123456789abcdef-";
int nLen = strlen (szTemp);

// Data wire is plugged into port 4
#define ONE_WIRE_BUS D4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

float celsius[3] = {NAN, NAN, NAN};




void setup() {
     Particle.function("setHeater", setHeaterState);
     Particle.function("heaterInterval", setHeaterInterval);
     programStartTime = millis(); // Record the start time of the program
  
    // relay setup for heating elements
  // Set relay pins as OUTPUT
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
  }
//
 Wire.begin(); // for i2c
 Serial1.begin(9600); // gps uses Serial 1
 Serial.begin(9600);  // debugging

 // adafruit library
 tsl.setGain(TSL2591_GAIN_LOW);  // for outdoors
 tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
 if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
   // while (true)
 };
 
 // Spark X   voltage meters
 mySecondSensor.begin(0x61);
    mySensor.begin();
    mySensor.setNumberOfSamples(1023, true); // Set the number of samples in shadow memory and eeprom
    mySensor.setBypassNenable(true, true);
   // create the uuid for the session
    guidFunc();

 sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
     // printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }

}

void loop() {
 unsigned long currentMillis = millis();
 if (currentMillis - lastAExecution >= 300000) {
   // runHeaters(); turn off for mower operation
    lastAExecution = currentMillis;
  }

//getTemperature(); turned off for mower operation
 
    
   bool isValidGPS = false;
   float lat, lon;
   int sats;  
    for (unsigned long start = millis(); millis() - start < 1500;){
        // Check GPS data is available
        while (Serial1.available()){
            char c = Serial1.read();
            
            // parse GPS data
            if (gps.encode(c))
                isValidGPS = true;
        }
    }
     if (isValidGPS){
        
        lat = gps.location.lat(); // Latitude in degrees 
        lon = gps.location.lng(); // Longitude in degrees  
        sats = gps.satellites.value();
        Serial.println(lat, ' lat');
    }
    else{
        lat = 0.0;
        lon = 0.0;
        sats = 0.0;
        Serial.println('no gps');
    }
    

  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
  
  //INA 219 was removed

  float voltage =0.0;
  float ampheres = 0;
  float wattage = 0;
 // float loadvoltage = 0;
 // float power_mW = 0;
  mySecondSensor.readInstantaneous(&voltage,&ampheres,&wattage);
/*
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
*/
// sparkx meter number one

    float volts = 0.0;
    float amps = 0.0;
    float watts = 0.0;
    mySensor.readInstantaneous(&volts, &amps, &watts);
  Serial.print(F("Volts: "));
  Serial.print(volts, 2);
  Serial.print(F(" Amps: "));
  Serial.print(amps, 2);
  Serial.print(F(" Watts: "));
  Serial.println(watts, 2);  
 

// get particle system variables
CellularSignal sig = Cellular.RSSI();
int rat = sig.getAccessTechnology();
float strength = sig.getStrength();
float quality = sig.getQuality();
FuelGauge fuel;

// put the data in Json format. This makes it easier to push this data to firebase using the particle web hook.
 DynamicJsonDocument doc(1024);
    doc["latitude"] = lat;
    doc["longitude"] = lon;
    doc["satellites"] = sats;
    doc["gpsstat"] = isValidGPS;
    doc["IR"] = ir;
    doc["full"] = full;
    doc["visible"] = full - ir;
    doc["lux"] = tsl.calculateLux(full, ir);
    doc["battery"]= fuel.getVCell();
    doc["soc"]= fuel.getSoC();
    doc["rat"]= rat;
    doc["strength"] = strength;
    doc["quality"] = quality; 
    doc["sparkxTwoVoltage"] = voltage;
    doc["sparkxTwoWatts"] = wattage;
   // doc["inaload"] = loadvoltage;
    doc["sparkxTwoAmps"] = ampheres; 
   // doc["inapower"] = power_mW; 
    doc["sparkxvolts"] = volts;
    doc["sparkxamps"] = amps; 
    doc["sparkxwatts"] = watts; 
    doc["si7201temp"] = sensor.readTemperature(); 
    doc["si7201humidity"] = sensor.readHumidity(); 
     doc["relayState"] = openRelays; 
    doc["guid"]=GUID;
    
    
         char output[500];  // string for publish
         serializeJson(doc, output);
       Particle.publish("eigen_one", output, PRIVATE);
         serializeJsonPretty(doc, Serial);
  
     delay(360000); 

}


void runHeaters(){
    
  openRelays = true; // Set to true to start opening relays
  if(openRelays){
  // Open relays sequentially for 20 seconds each
  for (int i = 0; i < numRelays; i++) {
    openNextRelayxSeconds();
  }
  
  
  }
  
  openRelays = false;
}

void openNextRelayxSeconds() {
  // Open current relay
  digitalWrite(relayPins[currentRelayIndex], HIGH);
  Serial.print("Relay ");
  Serial.print(currentRelayIndex);
  Serial.println(" opened.");
  delay(300);
    float voltageHeater =0.0;
  float amphHeater = 0;
  float wattageHeater = 0;
 mySecondSensor.readInstantaneous(&voltageHeater,&amphHeater,&wattageHeater);
  DynamicJsonDocument docLoad(1024);
      docLoad["volt"] = voltageHeater;
      docLoad["amps"] = amphHeater;
      docLoad["watts"] = wattageHeater;
    char elemsLoad[400];
    serializeJson(docLoad, elemsLoad);
    Particle.publish("load", elemsLoad, PRIVATE);

  // Wait for delay
  delay(heaterInterval);
getTemperature();
  // Close current relay
  digitalWrite(relayPins[currentRelayIndex], LOW);
  Serial.print("Relay ");
  Serial.print(currentRelayIndex);
  Serial.println(" closed.");

  // Move to the next relay
  currentRelayIndex = (currentRelayIndex + 1) % numRelays;
}

void getTemperature(){
   
   sensors.requestTemperatures(); // Send the command to get temperatures
  
  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
    
    // Output the device ID
    Serial.print("Temperature for device: ");
    Serial.println(i,DEC);

    // Print the data
    float tempC = sensors.getTempC(tempDeviceAddress);
    celsius[i]= tempC;
    Serial.print("Temp C: ");
    Serial.print(tempC);
    Serial.print(" Temp F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
    }   
  }
    
   
     DynamicJsonDocument docTemp(1024);
      docTemp["temp1"] = celsius[0];
      docTemp["temp2"] = celsius[1];
      docTemp["temp3"] = celsius[2];
    char sensorTemps[400];
    serializeJson(docTemp, sensorTemps);
    Particle.publish("temperature", sensorTemps, PRIVATE);

 
}



int setHeaterState(String command){
 if(command == "off" ){
     openRelays = false;
     return 1;
     
     }
   else return -1; 
}

int setHeaterInterval(String timeM){
    
 return -10; 
}

// uuid function.
 void guidFunc(){
   srand((unsigned int)millis()); 
 
    for (t=0; t<nLen+1; t++)
{
    int r = rand () % 16;
    char c = ' ';   

    switch (szTemp[t])
    {
        case 'x' : { c = szHex [r]; } break;
        case 'y' : { c = szHex [r & 0x03 | 0x08]; } break;
        case '-' : { c = '-'; } break;
        case '4' : { c = '4'; } break;
    }

    GUID[t] = ( t < nLen ) ? c : 0x00;
}
}
