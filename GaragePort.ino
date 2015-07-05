/**************************************************************************
 * 
 * Mysensored garageport opener sketch
 * 
 * This sketch is used in an arduino board to tap into an original garageport
 * motor control system. It detects direction of the port, which is sent to 
 * the controller, and also light is sent to controller.
 * 
 * In adition to control port, and light, it can also lock down the port system,
 * which is done my removing supply to the motor.
 * 
 * It presents 3 different subsensors:
 * 1) Port
 * 2) Light
 * 3) Lock
 * 
 * Author    : Thomas Mørch
 * copyright : 2015
 * 
 **************************************************************************/


#include <MySensor.h>
#include <SPI.h>

MySensor gw;

MyMessage msgPort(1, V_STOP);
MyMessage msgLight(2, V_LIGHT);
MyMessage msgLock(3, V_LIGHT);

#define PIN_CLOSE    3
#define PIN_OPEN     4
#define PIN_LIGHT_IN 7
#define PIN_REMOTE   5
#define PIN_ACTIVATE 6
#define POUT_LIGHT    A1
#define POUT_POWER_UP A0

#define ALIVE_INTERVAL 1800000 // Make a ping every half hour

bool last_light_state = false;
int direction = V_TEMP;
int lastDirection = V_DOWN;
bool lockdown = false;
unsigned long remoteActivationMillis = 0;
unsigned long limitTransmission = 0;
unsigned long lastPing = 0;

/**
 * Setup routine
 */
void setup()  
{   
  Serial.begin(115200);
  // Initialize library and add callback for incoming messages
  gw.begin(incomingMessage, 8, true);
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Garage", "1.0");

  gw.present(1, S_COVER); // GaragePort
  gw.present(2, S_LIGHT); // Light
  gw.present(3, S_LIGHT); // LockDown
  // Set pin directions
  pinMode(PIN_CLOSE, INPUT);
  pinMode(PIN_OPEN, INPUT);
  pinMode(POUT_LIGHT, OUTPUT);
  pinMode(PIN_ACTIVATE, INPUT);
  pinMode(PIN_LIGHT_IN, INPUT);
  pinMode(POUT_POWER_UP, OUTPUT);
  digitalWrite(POUT_POWER_UP,LOW);
  last_light_state = !digitalRead(PIN_LIGHT_IN); // Force update of light status, when starting up
}

/**
 * Main processing loop
 */
void loop() 
{
  int incomming = 0;

  // Whenever the remote is detected, power up the rest of the circuit, if it's not powered up already
  if (!lockdown & !digitalRead(POUT_POWER_UP)) {
    if (digitalRead(PIN_REMOTE)) {
      digitalWrite(POUT_POWER_UP, HIGH);  
      unsigned long timer = millis();
      while (1) {
        if (digitalRead(PIN_REMOTE)) timer = millis();  // Wait until pin go low again (user has released the button)
        if (millis() - timer > 1500) break; //must have been low for at least 1500 mSeconds
      }
      if (!digitalRead(PIN_OPEN) & !digitalRead(PIN_CLOSE)) activatePort(); // Only activate port, if it's not moving by now.
      remoteActivationMillis = millis();
    }
      
    if (!digitalRead(PIN_ACTIVATE)) {
      digitalWrite(POUT_POWER_UP, HIGH);
      unsigned timer = millis();
      while(1) {
        if (!digitalRead(PIN_ACTIVATE)) timer = millis();
        if (millis() - timer > 500) break;
      }
      if (!digitalRead(PIN_OPEN) & !digitalRead(PIN_CLOSE)) activatePort(); // Only activate port, if it's not moving by now.
      remoteActivationMillis = millis();
    }
  }
  
  // If 30 seconds have passed since we turned on 24V, and garageport light is not on, we turn of the 24V again.
  // this means that the garageport hasn't been opened / closed..
  if ((millis() - remoteActivationMillis > 30000) & !digitalRead(PIN_LIGHT_IN)) {
    digitalWrite(POUT_POWER_UP, LOW);
  } 

  // Detect if port is opening / Closing, by checking relay states
  if (digitalRead(PIN_OPEN)) {
    sendPortState(V_UP);
  }
  if (digitalRead(PIN_CLOSE)) {
    sendPortState(V_DOWN);
  }
  if (!(digitalRead(PIN_OPEN) || digitalRead(PIN_CLOSE))) {
    sendPortState(V_STOP);
  }

  // Check if we have to turn on/off light (compare to last state of the light)
  if ((digitalRead(PIN_LIGHT_IN) != last_light_state) ) {
    processLight();
  }

  // Check if we should send an alive ping to the controller
  if ((millis() - lastPing) > ALIVE_INTERVAL) {
    gw.sendBatteryLevel(100);
    lastPing = millis();
  }

  // Alway process incoming messages whenever possible
  gw.process();
}

/**
 * Send light status to controller, and turn on/off light locally at the same time.
 */
void processLight()
{
  Serial.print("Turning light ");
  bool lightState = digitalRead(PIN_LIGHT_IN);
  Serial.println(lightState?"ON":"OFF");
  digitalWrite(POUT_LIGHT, lightState);
  gw.send(msgLight.set(lightState));
  last_light_state = lightState;
  digitalWrite(POUT_POWER_UP, digitalRead(PIN_LIGHT_IN));
}

/**
 * Sends the port state to mysensors GW.
 */
void sendPortState(int pState) {
  if ((direction != pState) & limitTransmission + 500 < millis()) { // 500 milli seconds must have passed since last transmission
    Serial.print(F("Port is "));
    switch (pState) {
      case V_UP:
        Serial.println(F("opening"));
        break;
      case V_DOWN:
        Serial.println(F("closing"));
        break;
      case V_STOP:
        lastDirection = direction;
        Serial.println(F("stopped"));
        break;
    }
    gw.send(msgLight.set(digitalRead(POUT_LIGHT)));
    gw.send(msgPort.setType(pState));
    limitTransmission = millis();
    direction = pState;      
  }
}

/**
 * Process incomming messages from the controller
 */
void incomingMessage(const MyMessage &message) {
  Serial.print(F("Remote command : "));

  if (message.sensor == 1 & (message.type == V_UP | message.type == V_DOWN | message.type == V_STOP)) {
    if (message.type != direction & message.type != lastDirection) {
      activatePort();
      direction = V_TEMP;
      Serial.println(F("Activate port!"));
    }
  }

  if (message.sensor == 2 & message.type == V_LIGHT ) {
    digitalWrite(POUT_LIGHT, message.getBool()?1:0);
    Serial.print (F("Lights "));
    Serial.println(message.getBool()?"ON":"OFF");    
  }
  
  if (message.sensor == 3 & message.type == V_LIGHT ) {
    lockdown = !message.getBool(); // Invert lockdown signal from domoticz.
    if (lockdown) digitalWrite(POUT_POWER_UP, LOW); // If lockdown, then be sure to remove 24V
  }
}

/** 
 * Send activation signal to the port, making it either open, or close (opposite of the last action performed by the port itself) 
 */
void activatePort() {
  pinMode(PIN_ACTIVATE, OUTPUT);
  digitalWrite(POUT_POWER_UP, HIGH);
  delay(1000); // Let it power up, before we activate port
  digitalWrite(PIN_ACTIVATE, LOW);
  delay(500);
  digitalWrite(PIN_ACTIVATE, HIGH);
  pinMode(PIN_ACTIVATE, INPUT);
}

