#include <MySensor.h>
#include <SPI.h>

MySensor gw;

MyMessage msgPort(1, V_STOP);
MyMessage msgLight(2, V_LIGHT);


#define PIN_CLOSE    3
#define PIN_OPEN     4
#define PIN_LIGHT    A1
#define PIN_ACTIVATE 6
#define PIN_LIGHT_IN  7
#define POWER_UP     A0
//#define TIME_TO_ACTIVATE_PORT 30000

#define ALIVE_INTERVAL 1800000 // Make a ping every half hour

bool last_light_state = false;
int direction = V_TEMP;
int last_direction = V_DOWN;
unsigned long last_update = 0;

void setup()  
{   
  Serial.begin(115200);
  // Initialize library and add callback for incoming messages
  gw.begin(incomingMessage, 8, true);
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Garage", "1.0");

  gw.present(1, S_COVER); // GaragePort
  gw.present(2, S_LIGHT); // Light
  
  // Set pin directions
  pinMode(PIN_CLOSE, INPUT);
  pinMode(PIN_OPEN, INPUT);
  pinMode(PIN_LIGHT, OUTPUT);
  pinMode(PIN_ACTIVATE, INPUT);
  pinMode(PIN_LIGHT_IN, INPUT);
  pinMode(POWER_UP, OUTPUT);
  digitalWrite(POWER_UP,LOW);
  last_light_state = !digitalRead(PIN_LIGHT_IN); // Force update of light status, when starting up
}


void loop() 
{
  int incomming = 0;
  // Alway process incoming messages whenever possible
  if (digitalRead(PIN_OPEN)) {
    sendPortState(V_UP);
  }
  if (digitalRead(PIN_CLOSE)) {
    sendPortState(V_DOWN);
  }
  if (!(digitalRead(PIN_OPEN) || digitalRead(PIN_CLOSE))) {
    sendPortState(V_STOP);
  }
  
  if ((digitalRead(PIN_LIGHT_IN) != last_light_state) ) {
    Serial.print("Turning light ");
    bool lightState = digitalRead(PIN_LIGHT_IN);
    Serial.println(lightState?"ON":"OFF");
    digitalWrite(PIN_LIGHT, lightState);
    gw.send(msgLight.set(lightState));
    last_light_state = lightState;
  }
  if ((last_update + ALIVE_INTERVAL) < millis()) {
    gw.sendBatteryLevel(100);
    last_update = millis();
  }
  if (Serial.available()) processCmd();
  gw.process();
}


void sendPortState(int pState) {
  if (direction != pState) {
    Serial.print(F("Port is "));
    switch (pState) {
      case V_UP:
        digitalWrite(POWER_UP,HIGH);
        Serial.println(F("opening"));
        break;
      case V_DOWN:
        digitalWrite(POWER_UP,HIGH);
        Serial.println(F("closing"));
        break;
      case V_STOP:
        last_direction = direction;
        digitalWrite(POWER_UP,LOW);
        Serial.println(F("stopped"));
//        delay(500);
        break;
    }
    gw.send(msgLight.set(digitalRead(PIN_LIGHT)));
    gw.send(msgPort.setType(pState));
//    delay(500);
    direction = pState;      
  }
}

void incomingMessage(const MyMessage &message) {
  Serial.print(F("Remote command : "));

  if (message.sensor == 1 & (message.type == V_UP | message.type == V_DOWN | message.type == V_STOP)) {
    if (message.type != direction & message.type != last_direction) {
      activatePort();
      direction = V_TEMP;
      Serial.println(F("Activate port!"));
    }
  }

  if (message.sensor == 2 & message.type == V_LIGHT ) {
    digitalWrite(PIN_LIGHT, message.getBool()?1:0);
    Serial.print (F("Lights "));
    Serial.println(message.getBool()?"ON":"OFF");
    
  }
}

void activatePort() {
  pinMode(PIN_ACTIVATE, OUTPUT);
  digitalWrite(PIN_ACTIVATE, LOW);
  digitalWrite(POWER_UP, HIGH);
  delay(500);
  digitalWrite(PIN_ACTIVATE, HIGH);
  pinMode(PIN_ACTIVATE, INPUT);
}

void processCmd()
{
  char x = Serial.read();
  if (x == '3') activatePort();
  if (x == '1') {
    digitalWrite(PIN_LIGHT, HIGH);
  }
  if (x == '2') {
    digitalWrite(PIN_LIGHT, LOW);
  } 
}

