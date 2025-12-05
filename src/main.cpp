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
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <AsyncWebSocket.h>

#include <Preferences.h>
Preferences preferences;

#include <WString.h>  
#include <ArduinoJson.h>

JsonDocument command_json_msg;
JsonDocument config_json_msg;
JsonDocument compass_json_msg;

// Constants for commands and types
const char* CMD_TYPE_COMMAND = "0";
const char* CMD_TYPE_GET_CONFIG = "1";
const char* CMD_TYPE_SET_CONFIG = "2";

const char* CMD_GO = "go";
const char* CMD_CCW = "ccw";
const char* CMD_CW = "cw";
const char* CMD_OFF = "off";

const int MOTOR_OFF = 0;
const int MOTOR_CCW = 1; // Counter-Clockwise
const int MOTOR_CW = 2;  // Clockwise


float heading =0 ;
int analog_in =0 ;

unsigned long lastAdcTime = 0;
unsigned long adcReadInterval = 100000;
unsigned long last_heading = -1;

unsigned long lastRssiTime = 0;
unsigned long rssiReadInterval = 1000000; // Check RSSI every 5 seconds
long last_rssi = 0;

// Default values, will be overwritten by preferences
float max_az_angle = 210.0;
float min_az_angle = -150.0;
int max_az_analog = 1024;
int min_az_analog = 0;

const int AzAdcTableSize = 6;
int Az[AzAdcTableSize] {-90,0,90,180,270,360};
int ADCValue[AzAdcTableSize] = {694,576,456,336,218,101};

long rssi;

// Motor configuration

int motor_ac_cw = 19;
int motor_ac_ccw = 23;

int motor_ac_speed = 18;

int motor_dc_ib = 33;
int motor_dc_eb = 32;

int pwm_speed = 250;

String command_data;
int motor = MOTOR_OFF;
int goToAngle = 0;
int goTarget = 0;

// Sensor Config:

// Configurable values with defaults
int motor_type = 0; // 0 = AC, 1 = DC
int analogPin = 34;
float alphaFilter = 0.1;
String hostname = "rotor-dxs2800";

AsyncWebServer server(80);
AsyncWebSocket ws("/compass");

// WiFi credentials
String ssid;
String password;
DNSServer dnsServer;

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void handleGoCommand(const JsonDocument& doc) {
    if (!doc["go_data"].isNull()) {
        goTarget = doc["go_data"];
        goTarget = constrain(goTarget, min_az_angle, max_az_angle);
        Serial.print("GoTarget: ");
        Serial.println(goTarget);
        goToAngle = 1;
        pwm_speed = 250;
    }
}

void handleMotorCommand(const JsonDocument& doc) {
    const char* cmd = doc["cmd"];
    if (strcmp(cmd, CMD_GO) == 0) {
        handleGoCommand(doc);
    } else if (strcmp(cmd, CMD_CCW) == 0) {
        Serial.println("<<<<<");
        motor = MOTOR_CCW;
        goToAngle = 0;
        pwm_speed = 250;
    } else if (strcmp(cmd, CMD_CW) == 0) {
        Serial.println(">>>>");
        motor = MOTOR_CW;
        goToAngle = 0;
        pwm_speed = 250;
    } else if (strcmp(cmd, CMD_OFF) == 0) {
        Serial.println("||||");
        motor = MOTOR_OFF;
        goToAngle = 0;
        pwm_speed = 250;
    }
}

void sendConfig(AsyncWebSocketClient* client) {
    config_json_msg["type"] = 1; // send back config
    for (int i = 0; i < AzAdcTableSize; i++) {
        config_json_msg["ADCValue" + String(i)] = ADCValue[i];
        config_json_msg["Az" + String(i)] = Az[i];
    }
    // Send new config values
    config_json_msg["motor_type"] = motor_type;
    config_json_msg["analogPin"] = analogPin;
    config_json_msg["adcReadInterval"] = adcReadInterval;
    config_json_msg["alphaFilter"] = alphaFilter;
    config_json_msg["hostname"] = hostname;

    String json_output;
    serializeJson(config_json_msg, json_output);
    client->text(json_output);
}

void saveConfig(const JsonDocument& doc) {
    Serial.println("Config json received:");
    for (int i = 0; i < AzAdcTableSize; i++) {
        String adcKey = "ADCValue" + String(i);
        String azKey = "Az" + String(i);
        if (!doc[adcKey].isNull()) {
            ADCValue[i] = doc[adcKey];
            preferences.putInt(adcKey.c_str(), ADCValue[i]);
        }
        if (!doc[azKey].isNull()) {
            Az[i] = doc[azKey];
            preferences.putInt(azKey.c_str(), Az[i]);
        }
    }

    // Save new config values
    if (!doc["motor_type"].isNull()) {
        motor_type = doc["motor_type"];
        preferences.putInt("motor_type", motor_type);
    }
    if (!doc["analogPin"].isNull()) {
        analogPin = doc["analogPin"];
        preferences.putInt("analogPin", analogPin);
    }
    if (!doc["adcReadInterval"].isNull()) {
        adcReadInterval = doc["adcReadInterval"];
        preferences.putULong("adcReadInterval", adcReadInterval);
    }
    if (!doc["alphaFilter"].isNull()) {
        alphaFilter = doc["alphaFilter"];
        preferences.putFloat("alphaFilter", alphaFilter);
    }
    if (!doc["hostname"].isNull()) {
        hostname = doc["hostname"].as<String>();
        preferences.putString("hostname", hostname);
    }

    //Set max and min constrain
    max_az_analog = ADCValue[AzAdcTableSize - 1];
    max_az_angle  = Az[AzAdcTableSize - 1];
    min_az_analog = ADCValue[0];
    min_az_angle  = Az[0];

    Serial.println("Data saved: ");
    Serial.print("ADC: ");
    for(int i=0; i<AzAdcTableSize; i++) { Serial.print(ADCValue[i]); if(i < AzAdcTableSize-1) Serial.print(", ");}
    Serial.println();

    Serial.print("Az: ");
    for(int i=0; i<AzAdcTableSize; i++) { Serial.print(Az[i]); if(i < AzAdcTableSize-1) Serial.print(", ");}
    Serial.println();
}
 
void handleWebSocketMessage(AsyncWebSocket * server, AsyncWebSocketClient * client, void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        command_data = "";
        for(size_t i=0; i < len; i++) {
            command_data += (char)data[i];
        }

        Serial.print("Commando recebido: ");
        Serial.println(command_data);

        command_json_msg.clear();
        DeserializationError error = deserializeJson(command_json_msg, command_data);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
        }

        const char* type = command_json_msg["type"];

        if (strcmp(type, CMD_TYPE_COMMAND) == 0 && !command_json_msg["cmd"].isNull()) {
            handleMotorCommand(command_json_msg);
        } else if (strcmp(type, CMD_TYPE_GET_CONFIG) == 0) {
            sendConfig(client);
        } else if (strcmp(type, CMD_TYPE_SET_CONFIG) == 0) {
            saveConfig(command_json_msg);
        }
    }
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT){
    Serial.println("Websocket client connection received");
    // Send initial status as a JSON object, which the frontend expects.
    String json_output;
    compass_json_msg["type"] = 0;
    compass_json_msg["adc_value"] = analog_in;
    compass_json_msg["az_angle"] = int(heading);
    compass_json_msg["rssi"] = WiFi.RSSI();
    
    serializeJson(compass_json_msg, json_output);
    client->text(json_output);


  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
  }
  else if(type == WS_EVT_DATA){
    handleWebSocketMessage(server, client, arg, data, len);
  }
}

// MAP ADC to AZ
int mapADC(int adcValue) {
  // Check if adcValue is outside the calibrated range and extrapolate if necessary.
  // This works for both increasing and decreasing ADC tables.
  bool isIncreasing = ADCValue[0] < ADCValue[AzAdcTableSize - 1];

  // Extrapolate using the first segment if below the start of the table
  if ((isIncreasing && adcValue < ADCValue[0]) || (!isIncreasing && adcValue > ADCValue[0])) {
    return map(adcValue, ADCValue[0], ADCValue[1], Az[0], Az[1]);
  }
  // Extrapolate using the last segment if above the end of the table
  if ((isIncreasing && adcValue > ADCValue[AzAdcTableSize - 1]) || (!isIncreasing && adcValue < ADCValue[AzAdcTableSize - 1])) {
    return map(adcValue, ADCValue[AzAdcTableSize - 2], ADCValue[AzAdcTableSize - 1], Az[AzAdcTableSize - 2], Az[AzAdcTableSize - 1]);
  }

  // Iterate through the table to find the correct segment for interpolation.
  // This works regardless of whether the ADC values are ascending or descending.
  for (int i = 0; i < AzAdcTableSize - 1; i++) {
    // Check if adcValue is between the current and next ADC points.
    bool in_segment = (adcValue >= ADCValue[i] && adcValue <= ADCValue[i + 1]) || (adcValue <= ADCValue[i] && adcValue >= ADCValue[i + 1]);
    if (in_segment) {
      return map(adcValue, ADCValue[i], ADCValue[i + 1], Az[i], Az[i + 1]);
    }
  }

  // Fallback: if no segment is found (should not happen with a valid table), return the last angle.
  // This might occur with a non-monotonic table that isn't handled by the extrapolation logic.
  return Az[AzAdcTableSize - 1];
}

void setupAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Rotor-Setup");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // DNS server for captive portal
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/www/wifi_setup.html", "text/html");
  });
  // Serve static files for the setup page
  server.serveStatic("/css", SPIFFS, "/www/css");
  server.serveStatic("/js", SPIFFS, "/www/js");

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      String ssid_ap = request->getParam("ssid", true)->value();
      String password_ap = request->getParam("password", true)->value();
      
      preferences.putString("ssid", ssid_ap);
      preferences.putString("password", password_ap);

      String response_html = "<html><body><h1>Configuracoes salvas!</h1><p>O dispositivo sera reiniciado. Conecte-se a sua rede Wi-Fi e acesse o rotor pelo seu novo IP ou pelo endereco <code>" + hostname + ".local</code>.</p></body></html>";
      request->send(200, "text/html", response_html);

      delay(2000);
      ESP.restart();
    } else {
      request->send(400, "text/plain", "SSID ou senha ausentes.");
    }
  });

  // Redirect all other requests to the root
  server.onNotFound([](AsyncWebServerRequest *request){
    request->redirect("/");
  });
}

void loadPreferences() {
    preferences.begin("Rotor", false);
    for (int i = 0; i < AzAdcTableSize; i++) {
        String adcKey = "ADCValue" + String(i);
        String azKey = "Az" + String(i);
        ADCValue[i] = preferences.getInt(adcKey.c_str(), ADCValue[i]);
        Az[i] = preferences.getInt(azKey.c_str(), Az[i]);
    }
    // Load new preferences
    motor_type = preferences.getInt("motor_type", motor_type);
    analogPin = preferences.getInt("analogPin", analogPin);
    adcReadInterval = preferences.getULong("adcReadInterval", adcReadInterval);
    alphaFilter = preferences.getFloat("alphaFilter", alphaFilter);
    hostname = preferences.getString("hostname", hostname);

    // Load WiFi credentials
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");

    max_az_analog = ADCValue[AzAdcTableSize - 1];
    max_az_angle  = Az[AzAdcTableSize - 1];
    min_az_analog = ADCValue[0];
    min_az_angle  = Az[0];
}

void setup() {
  
  analogSetAttenuation(ADC_11db);
  analogReadResolution(10);

  pinMode(motor_ac_ccw, OUTPUT);
  pinMode(motor_ac_cw, OUTPUT);
 
  pinMode(motor_dc_ib, OUTPUT);
  pinMode(motor_dc_eb, OUTPUT);
  
  
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(motor_ac_speed,OUTPUT);
  analogWrite(motor_ac_speed,255);
  
  Serial.begin(115200);

  loadPreferences();

  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  if (ssid == "") {
    Serial.println("Nenhuma credencial Wi-Fi encontrada. Iniciando modo de configuracao (AP)...");
    setupAP();
  } else {
    Serial.println("Conectando ao Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname.c_str()); // Set the DHCP hostname BEFORE begin
    WiFi.begin(ssid.c_str(), password.c_str());
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Falha ao conectar. Reiniciando em modo de configuracao...");
        // Clear invalid credentials and restart
        preferences.remove("ssid");
        preferences.remove("password");
        delay(1000);
        ESP.restart();
    }

    Serial.print("Conectado! IP Address: ");
    Serial.println(WiFi.localIP());

    // MDNS-SD
    if (!MDNS.begin(hostname.c_str())) { // Hostname should be lowercase
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.println("mDNS responder started: http://" + hostname + ".local");
        MDNS.addService("http", "tcp", 80);
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Endpoint para ir para uma direção via URL
    server.on("/go", HTTP_GET, [](AsyncWebServerRequest *request) {
        String direction;
        if (request->hasParam("direction")) {
            direction = request->getParam("direction")->value();
            request->send(200, "text/plain", direction);

            goTarget = direction.toInt();
            goTarget = constrain(goTarget, min_az_angle, max_az_angle);
            goToAngle = 1;
            Serial.print("URL Go Target: ");
            Serial.println(goTarget);
        }
    });

    server.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.html");
    server.serveStatic("/favicon.ico", SPIFFS, "/www/favicon.ico");
    server.onNotFound(notFound);
  }

  server.begin();
        
}

void updateMotorController() {
  if(goToAngle == 1){ // Turn motor on until reach the disered angle

    if(abs(goTarget - (int)heading) <= 1){
      motor = MOTOR_OFF;
      goToAngle = 0;
    }else if ((int)heading > goTarget) {
      motor = MOTOR_CCW;
    }else if ((int)heading < goTarget) {
      motor = MOTOR_CW;
    }

    // If motor type = DC and Angle diference is less then 5 speed down the motor
    if(abs(goTarget - heading) < 12){ 
      pwm_speed = 100;
    }else{
      pwm_speed = 250;
    }
  }
  
  if(motor == MOTOR_OFF) { // Set motor off
    digitalWrite(motor_ac_ccw,LOW);
    digitalWrite(motor_ac_cw,LOW);
    analogWrite(motor_ac_speed, 0); // Ensure speed PWM is off
    digitalWrite(LED_BUILTIN,LOW);
  }
    
  if(motor == MOTOR_CCW) { // Set motor to turn CCW (Counter-Clockwise)
    digitalWrite(LED_BUILTIN,HIGH);
    if( motor_type == 1 ){
      analogWrite(motor_dc_eb,pwm_speed); // DC MOTOR PWM
      digitalWrite(motor_dc_ib,LOW); // DC MOTOR PWM
    }else {
      digitalWrite(motor_ac_ccw,HIGH); // AC MOTOR
      digitalWrite(motor_ac_cw,LOW); // AC MOTOR
    }
  }
  
  if(motor == MOTOR_CW) { // Set motor to turn CW (Clockwise)
    digitalWrite(LED_BUILTIN,HIGH);

    if( motor_type == 1 ){
      analogWrite(motor_dc_eb,pwm_speed); // DC MOTOR PWM
      digitalWrite(motor_dc_ib,HIGH); // DC MOTOR PWM
    } else {
      digitalWrite(motor_ac_ccw,LOW); // AC MOTOR
      digitalWrite(motor_ac_cw,HIGH); // AC MOTOR
    }
  }

  // Apply PWM speed for AC motor if it's on
  if (motor != MOTOR_OFF && motor_type == 0) {
      analogWrite(motor_ac_speed, pwm_speed);
  }
}

void readSensorAndBroadcast() {
    // Exponential Moving Average (EMA) filter to smooth ADC readings.
    float new_reading = (float)analogRead(analogPin);
    analog_in = (int)((alphaFilter * new_reading) + ((1.0 - alphaFilter) * analog_in));

    heading = mapADC(analog_in); 
    
    if (int(heading) != last_heading) {
      last_heading = int(heading);
      rssi = WiFi.RSSI();

      String json_output;
      compass_json_msg["type"] = 0;
      compass_json_msg["adc_value"] = analog_in;
      compass_json_msg["az_angle"] = int(heading);
      compass_json_msg["rssi"] = int(rssi);
      
      serializeJson(compass_json_msg, json_output);
      ws.textAll(json_output);
    }
}

void checkRssiAndBroadcast() {
    rssi = WiFi.RSSI();
    if (rssi != last_rssi) {
        last_rssi = rssi;

        // We can reuse the compass_json_msg object, but only send RSSI
        JsonDocument rssi_msg;
        rssi_msg["type"] = 0; // Status type
        rssi_msg["rssi"] = rssi;

        String json_output;
        serializeJson(rssi_msg, json_output);
        ws.textAll(json_output);
    }
}

void loop() {
  if(micros()  - lastAdcTime >= adcReadInterval){
    lastAdcTime = micros();
    readSensorAndBroadcast();
  }
  if (micros() - lastRssiTime >= rssiReadInterval) {
    lastRssiTime = micros();
    checkRssiAndBroadcast();
  }
  updateMotorController();
}
