#include "DHT.h"
#include <Ethernet.h>
#include <SPI.h>
#include <Shys_Sensor.h>


/* *********************************** */
/*             CONFIGURE               */
/* *********************************** */
// PIN Settings
#define DHTPIN 2
#define MHQPIN_ANALOG A0  
#define MHQPIN_DIGITAL 5
#define LIGHTPIN A1
#define PIRPIN 7

// DHT Settings
#define DHTTYPE DHT22   // DHT11, DHT21 or DHT22

// Sensor IDs
long tempSensorId = 47634;
long humidSensorId = 47635;
long airSensorId = 47636;
long lightSensorId = 47637;
long pirSensorId = 47638;

// PIR Settings
int calibrationTime = 10;        
long unsigned int pirPause = 5000;  
long unsigned int dhtSignalIntervall   = 10000;  
long unsigned int airSignalIntervall   = 10000;  
long unsigned int lightSignalIntervall = 10000;  

// PI Settings
byte _piAddress[] =  {192, 168, 1, 100};


// LAN Settings
byte _mac[]  = {0xBA, 0x6C, 0x4A, 0x36, 0x44, 0xAD  };
byte _ip[]   = { 192, 168, 1, 124 };
byte _dns[]  = { 192, 168, 1, 15  };
byte _gate[] = { 192, 168, 1, 15 };
byte _mask[] = { 255, 255, 255, 0  };

boolean serialOut = false; 
boolean waitForSerialMonitor = false;
/* *********************************** */
/*           END CONFIGURE             */
/* *********************************** */


long temp    = -999;
long humid   = -999;
long air     = -999;
long light   = -999;

long unsigned int lastDHTSignal;  
long unsigned int lastAirSignal;  
long unsigned int lastLightSignal;  

long unsigned int pirLowIn;  
boolean pirLockLow = true;
boolean pirTakeLowTime; 
boolean pirMotionActive = false;
boolean pirMotionSignalSend = false;


DHT dht(DHTPIN, DHTTYPE);

Shys_Sensor sensor  = Shys_Sensor(_mac, _ip, _dns, _gate, _mask, _piAddress);

/**
 *  Basic setup Method
 */
void setup() {
  Serial.begin(9600); 
  if(waitForSerialMonitor){
    while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
    }
  } else {
    delay(1000);
  }
  
  Serial.println(F("HomeControl - Motion, Temp, Humidity, CO2 and Lightlevel Sensor"));
  Serial.println();

  dht.begin();
  
  //MQ2 LED Pin
  pinMode(MHQPIN_DIGITAL, OUTPUT);  
  pinMode(MHQPIN_ANALOG, INPUT);
  pinMode(PIRPIN, INPUT);
  pinMode(LIGHTPIN, INPUT);
 
  
  Serial.print(F("calibrating sensor "));
  for(int i = 0; i < calibrationTime; i++){
     Serial.print(".");
     delay(1000);
  }

  sensor.init();
  
  Serial.println(F(" done"));
  Serial.println(F("SENSOR ACTIVE"));
  delay(50);
}

/**
 *  Basic loop Method
 */
void loop() {
  refreshDHTSensorValues();
  refreshMQ2SensorValues();
  refreshLightSensorValue();
  refreshPIRSensorValue();
  
  
  if(lastDHTSignal+dhtSignalIntervall<millis()){
    sensor.setSensorValue(tempSensorId, temp);
    sensor.setSensorValue(humidSensorId, humid);
    lastDHTSignal = millis();
    if(serialOut){
      Serial.print(F("Humidity: ")); 
      Serial.print(humid);
      Serial.print(F(" %\t"));
      Serial.print(F("Temp: ")); 
      Serial.print(temp);
      Serial.print(F(" *C "));
      Serial.println(F(" "));
    }
  }
  
  if(lastAirSignal+airSignalIntervall<millis()){
    sensor.setSensorValue(airSensorId, air);
    lastAirSignal = millis();

    if(serialOut){
      Serial.print(F("Air-Quality: "));      
      Serial.println(air);
    }
  }  
  
  if(lastLightSignal+lightSignalIntervall<millis()){
    sensor.setSensorValue(lightSensorId, light);
    lastLightSignal = millis();

    if(serialOut){
      Serial.print(F("Light-Level: "));      
      Serial.println(light);
    }
  }
  
  if(!pirMotionSignalSend){
    sensor.setSensorValue(pirSensorId, pirMotionActive?1:0);
    pirMotionSignalSend=true;

    if(serialOut){
      Serial.println(F("Motion Detect - Send Signal "));      
    }
  }
  delay(300);
}


/**
 *  Read values from DHT-Sensor
 */
void refreshDHTSensorValues(){
  humid = dht.readHumidity();
  temp = dht.readTemperature();
  if (isnan(humid) || isnan(temp) ) {
    if(serialOut){
      Serial.println(F("Failed to read from DHT sensor!"));
    }
    return;
  }
}



/**
 *  Read values from Light-Sensor
 */
void refreshLightSensorValue(){
  int sensorValue = analogRead(LIGHTPIN);            
  light = map(sensorValue, 0, 1023, 100, 0);  
  if (isnan(light)) { 
    if(serialOut){
      Serial.println(F("Failed to read from Light Sensor!"));
    }
    return;
  }
}

/**
 *  Read values from MQ2-Sensor
 */
void refreshMQ2SensorValues(){
  int sensorValue = analogRead(MHQPIN_ANALOG);            
  air = map(sensorValue, 0, 1023, 0, 100);  

  digitalWrite(MHQPIN_DIGITAL, LOW);           
}

/**
 *  Check PIR Sensor
 */
void refreshPIRSensorValue(){
  if(digitalRead(PIRPIN) == HIGH){
    if(pirLockLow){  
      //makes sure we wait for a transition to LOW before any further output is made:
      pirLockLow = false; 

      if(serialOut){
        Serial.print(F("motion detected at "));
        Serial.print(millis()/1000);
        Serial.println(F(" sec")); 
      }
      pirMotionActive=true;
      pirMotionSignalSend=false;
      delay(50);
    }         
    pirTakeLowTime = true;
  }
  if(digitalRead(PIRPIN) == LOW){       
    if(pirTakeLowTime){
      pirLowIn = millis();          //save the time of the transition from high to LOW
      pirTakeLowTime = false;       //make sure this is only done at the start of a LOW phase
    }
    
    //if the sensor is low for more than the given pause, 
    //we assume that no more motion is going to happen
    if(!pirLockLow && millis() - pirLowIn > pirPause){  
      //makes sure this block of code is only executed again after 
      //a new motion sequence has been detected
      pirLockLow = true;     
  
      if(serialOut){
        Serial.print(F("motion ended at "));      //output
        Serial.print((millis() - pirPause)/1000);
        Serial.println(F(" sec"));
      }
      
      pirMotionActive=false;
      pirMotionSignalSend=false;
      delay(50);
    }
  }  
}

