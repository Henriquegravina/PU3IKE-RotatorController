/*

  Simple Antena Rotator Controller with web interface.
  -> Jquery/Websocket interface: WIP
  -> Http requests: TBD

  Author: Henrique B. Gravina - PU3IKE

 
*/

//Basic func
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebResponseImpl.h>
#include <WebAuthentication.h>
#include <AsyncJson.h>
#include <StringArray.h>
#include <SPIFFSEditor.h>
#include <AsyncWebSocket.h>
#include <AsyncWebSynchronization.h>
#include <WebHandlerImpl.h>
#include <SPIFFSEditor.h>

#include <Preferences.h>
Preferences preferences;

#include <WString.h>  
#include <ArduinoJson.h>

DynamicJsonDocument command_json_msg(1024);
DynamicJsonDocument config_json_msg(1024);
DynamicJsonDocument compass_json_msg(1024);



float heading =0 ;
int analog_in =0 ;

unsigned long lastTime;
unsigned long isTime = 100000;
unsigned long itTimeHigh = 700000;
unsigned long itTimeLow  = 50000;

float max_az_angle = 360.0;
float min_az_angle = -40.0;
int max_az_analog = 1024;
int min_az_analog = 0;

const int AzAdcTableSize = 6;
int Az[AzAdcTableSize] {-40,0,90,180,270,360};
int ADCValue[AzAdcTableSize] = {0,100,200,300,400,500};



long rssi;

// Motor configuration

int motor_type = 0; // 0 = AC // 1 = DC

int motor_ac_cw = 19;
int motor_ac_ccw = 23;

int motor_ac_speed = 18;

int motor_dc_ib = 33;
int motor_dc_eb = 32;

int pwm_speed = 512;

String command_data;
int motor = 0;
int goToAngle = 0;
int goTarget = 0;

// Sensor Config:

int analogPin = 34;


AsyncWebServer server(80);
AsyncWebSocket ws("/compass");

const char* ssid = "SSID";
const char* password = "PASSWD";

//const char* PARAM_MESSAGE = "message";


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT){
    Serial.println("Websocket client connection received");
    client->text(String(int(heading)));

  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
  }
  else if(type == WS_EVT_DATA){
    // Serial.println("Client send some data");
    for(int i=0; i < len; i++) {
      command_data += (char)data[i];
      //Serial.print((char) data[i]);
    }

    Serial.print("Commando recebido: ");
    Serial.println(command_data);

    // Parse Json
    deserializeJson(command_json_msg, command_data);
    if(command_json_msg["type"] == "0"){

      
      if(command_json_msg["cmd"] == "go"){ // Command GO
      goTarget = command_json_msg["go_data"];
      goTarget = constrain(goTarget, min_az_angle, max_az_angle);
      goToAngle = 1; 
      Serial.print("GoTarget: "); 
      Serial.println(goTarget);

      }else if(command_json_msg["cmd"] == "ccw"){ // Command CCW
        Serial.println("<<<<<");
        motor = 1;
        goToAngle = 0;
        pwm_speed = 512;
      
      }else if(command_json_msg["cmd"] == "cw"){ // Command CW
        Serial.println(">>>>"); 
        motor = 2; 
        goToAngle = 0; 
        pwm_speed = 512;

      }else if(command_json_msg["cmd"] == "off"){ // Command OFF/STOP
        Serial.println("||||"); 
        motor = 0; 
        goToAngle = 0;  
        pwm_speed = 512;

      }

    }else if(command_json_msg["type"] == "1"){
       
      
      config_json_msg["type"] = 1; // send back config
      
      config_json_msg["ADCValue0"] = ADCValue[0];
      config_json_msg["ADCValue1"] = ADCValue[1];
      config_json_msg["ADCValue2"] = ADCValue[2];
      config_json_msg["ADCValue3"] = ADCValue[3];
      config_json_msg["ADCValue4"] = ADCValue[4];
      config_json_msg["ADCValue5"] = ADCValue[5];
      
      config_json_msg["Az0"] = Az[0];
      config_json_msg["Az1"] = Az[1];
      config_json_msg["Az2"] = Az[2];
      config_json_msg["Az3"] = Az[3];
      config_json_msg["Az4"] = Az[4];
      config_json_msg["Az5"] = Az[5];
    
      String json_output;
      serializeJson(config_json_msg, json_output);
      ws.textAll(json_output);


    }else if(command_json_msg["type"] == "2"){

      Serial.println("Config json:");  
      ADCValue[0] = command_json_msg["ADCValue0"];
      ADCValue[1] = command_json_msg["ADCValue1"];
      ADCValue[2] = command_json_msg["ADCValue2"];
      ADCValue[3] = command_json_msg["ADCValue3"];
      ADCValue[4] = command_json_msg["ADCValue4"];
      ADCValue[5] = command_json_msg["ADCValue5"];
      
      Az[0] = command_json_msg["Az0"];
      Az[1] = command_json_msg["Az1"];
      Az[2] = command_json_msg["Az2"];
      Az[3] = command_json_msg["Az3"];
      Az[4] = command_json_msg["Az4"];
      Az[5] = command_json_msg["Az5"];

      preferences.putInt("ADCValue0",ADCValue[0]);
      preferences.putInt("ADCValue1",ADCValue[1]);
      preferences.putInt("ADCValue2",ADCValue[2]);
      preferences.putInt("ADCValue3",ADCValue[3]);
      preferences.putInt("ADCValue4",ADCValue[4]);
      preferences.putInt("ADCValue5",ADCValue[5]);

      preferences.putInt("Az0",Az[0]);
      preferences.putInt("Az1",Az[1]);
      preferences.putInt("Az2",Az[2]);
      preferences.putInt("Az3",Az[3]);
      preferences.putInt("Az4",Az[4]);
      preferences.putInt("Az5",Az[5]);
      
      //Set max and min constrain
      max_az_analog = ADCValue[5];
      max_az_angle  = Az[5];
      min_az_analog = ADCValue[0];
      min_az_angle  = Az[0];

      Serial.println("Data received: ");
      Serial.print("ADC: ");
      Serial.print(ADCValue[0]);Serial.print(", ");Serial.print(ADCValue[1]);Serial.print(", ");
      Serial.print(ADCValue[2]);Serial.print(", ");Serial.print(ADCValue[3]);Serial.print(", ");
      Serial.print(ADCValue[4]);Serial.print(", ");Serial.println(ADCValue[5]);

      Serial.print("Az: ");
      Serial.print(Az[0]);Serial.print(", ");Serial.print(Az[1]);Serial.print(", ");
      Serial.print(Az[2]);Serial.print(", ");Serial.print(Az[3]);Serial.print(", ");
      Serial.print(Az[4]);Serial.print(", ");Serial.println(Az[5]);
      


    }
    
    command_data = "";
  }
}

// MAP ADC to AZ
int mapADC(int adcValue) {
  // Find the appropriate index in the table
  int index = 0;
  while (index < AzAdcTableSize && adcValue >= ADCValue[index]) {
    index++;
  }

  // Handle values outside the table range
  if (index == 0) {
    return ADCValue[0];
  } else if (index == AzAdcTableSize) {
    return Az[AzAdcTableSize - 1];
  }

  // Interpolate between two nearest values in the table
  int lowerValue = ADCValue[index - 1];
  int upperValue = ADCValue[index];
  int lowerMappedValue = Az[index - 1];
  int upperMappedValue = Az[index];

  return map(adcValue, lowerValue, upperValue, lowerMappedValue, upperMappedValue);
}

void setup() {
  
  analogReadResolution(10);

  pinMode(motor_ac_ccw, OUTPUT);
  pinMode(motor_ac_cw, OUTPUT);
 
  pinMode(motor_dc_ib, OUTPUT);
  pinMode(motor_dc_eb, OUTPUT);
  
  
  pinMode(LED_BUILTIN,OUTPUT);
  
  Serial.begin(115200);

  preferences.begin("Rotor", false);
  // Get az params
  Az[0] = preferences.getInt("Az0",Az[0]);
  Az[1] = preferences.getInt("Az1",Az[1]);
  Az[2] = preferences.getInt("Az2",Az[2]);
  Az[3] = preferences.getInt("Az3",Az[3]);
  Az[4] = preferences.getInt("Az4",Az[4]);
  Az[5] = preferences.getInt("Az5",Az[5]);
  // Get ADC params
  ADCValue[0] = preferences.getInt("ADCValue0",Az[0]);
  ADCValue[1] = preferences.getInt("ADCValue1",Az[1]);
  ADCValue[2] = preferences.getInt("ADCValue2",Az[2]);
  ADCValue[3] = preferences.getInt("ADCValue3",Az[3]);
  ADCValue[4] = preferences.getInt("ADCValue4",Az[4]);
  ADCValue[5] = preferences.getInt("ADCValue5",Az[5]);

  // Set max a min constrain
  max_az_analog = Az[5];
  max_az_angle  = ADCValue[5];
  min_az_analog = Az[0];
  min_az_angle  = ADCValue[0];

 // ssid = preferences.getString("ssid", "");

     // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }


  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // MDNS-SD
  if (!MDNS.begin("RotorDXS2800")) {
    Serial.println("Error setting up MDNS responder!");
    while(1) {
    delay(1000);
   }
  }
  
  Serial.println("mDNS responder started");
  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);


  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

    
    // Se a go direction via url 
    server.on("/go", HTTP_GET, [] ( AsyncWebServerRequest *request){
      String direction;
      if(request->hasParam("direction")){
        direction = request->getParam("direction")->value();
        request->send(200,"text/plain", direction);
        
        goTarget = direction.toInt();
        goTarget = constrain(goTarget, min_az_angle, max_az_angle);
        goToAngle = 1; 
        Serial.print("URL Go Target: "); 
        Serial.println(goTarget);   
      }
      
    });
    
    server.serveStatic("/", SPIFFS, "/www/");
    server.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.html");
    server.serveStatic("/favicon.ico", SPIFFS, "/www/favicon.ico");
    server.onNotFound(notFound);
    server.begin();
   
}

void loop() {

  if(motor == 0) { // if motor is off the refresh time is slower 
    isTime = itTimeHigh;
  }else{
    isTime = itTimeLow;
  }
  
  if(micros()-lastTime >= isTime){ // Time to refresh data
    
    analog_in =  ( 0.25 * (float)analogRead(analogPin) ) + (analog_in * 0.75);
    //Serial.print("AD: ");
    //Serial.println(analog_in);

    //heading = map(analog_in, min_az_analog,max_az_analog, min_az_angle,max_az_angle); // Convert analogRead to Angle
    //heading = map(heading, 229.0,902.0, min_az_angle,max_az_angle); // Convert analogRead to Angle
    //heading = constrain(heading, min_az_angle, max_az_angle); // Limit Angle values
    heading = mapADC(analog_in); 
    
    //Serial.print("Compass Heading: "); Serial.println(new_head);
  
    rssi = WiFi.RSSI();

    // Send Json status data to client: Type 0 = status
    String json_output;
    compass_json_msg["type"] = 0;
    compass_json_msg["adc_value"] = int(analog_in);
    compass_json_msg["az_angle"] = int(heading);
    compass_json_msg["rssi"] = int(rssi);
    
    serializeJson(compass_json_msg, json_output);
    ws.textAll(json_output);
    
    
    //Serial.print("Compass Heading: "); Serial.println(heading);
    lastTime = micros();

    //Serial.print("IP Address: ");
    //Serial.println(WiFi.localIP());
    
  }

  // Motor Controller

  if(goToAngle == 1){ // Turn motor on until reach the disered angle

    if(abs(goTarget - (int)heading) <= 1){
      motor = 0;
      goToAngle = 0;
    }else if ((int)heading > goTarget) {
      motor = 1;
    }else if ((int)heading < goTarget) {
      motor = 2;
    }

    // If motor type = DC and Angle diference is less then 5 speed down the motor
    if( motor_type == 1) {
      if(abs(goTarget - heading) < 5){ 
        pwm_speed = 300;
      }else{
        pwm_speed = 712;
      }
    }
    
  }

  if(motor == 0) { // Set motor off
    digitalWrite(motor_ac_ccw,LOW);
    digitalWrite(motor_ac_cw,LOW);
    digitalWrite(LED_BUILTIN,LOW);
  }
    
  if(motor == 1) { // Set motor to turn CW
    digitalWrite(LED_BUILTIN,HIGH);
    if( motor_type == 1 ){
      analogWrite(motor_dc_eb,pwm_speed); // DC MOTOR PWM
      digitalWrite(motor_dc_ib,LOW); // DC MOTOR PWM
    }else {
      digitalWrite(motor_ac_ccw,HIGH); // AC MOTOR
      digitalWrite(motor_ac_cw,LOW); // AC MOTOR
    }

  }
  
  if(motor == 2) { // Set motor to turn CCW
    digitalWrite(LED_BUILTIN,HIGH);

    if( motor_type == 1 ){
      analogWrite(motor_dc_eb,pwm_speed); // DC MOTOR PWM
      digitalWrite(motor_dc_ib,HIGH); // DC MOTOR PWM
    } else {
      digitalWrite(motor_ac_ccw,LOW); // AC MOTOR
      digitalWrite(motor_ac_cw,HIGH); // AC MOTOR
    }
  }
  
}
