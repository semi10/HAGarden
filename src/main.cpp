/**************************************************************************************************************************************************
* File name     : ESP32_HA_Discovery.c
* Compiler      : 
* Autor         : VIGASAN   
* Created       : 27/01/2023
* Modified      : 01/02/2024 By Semion
* Last modified :
*
*
* Description   : 
*
* Other info    : Discovery MQTT Device Example
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Local definitions------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#define PERIOD_MILLSEC_1000    500
#define PERIOD_MILLSEC_500     500
#define PERIOD_MILLSEC_250     250

#define DHTTYPE             AM2301
#define NUMB_OF_MOISTURE_SENS    5

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const int DHTPIN = 7;
const int MOIST_PIN[] = {A0, A1, A2, A3, A4, A5};

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Configuration --------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*         g_ssid = "O&S";
const char*         g_password = "0525362505";
const char*         g_mqtt_server = "192.168.0.141";                          // MQTT Server IP, same of Home Assistant
const char*         g_mqttUser = "hass";                                      // MQTT Server User Name
const char*         g_mqttPsw = "m8800gts";                                   // MQTT Server password
int                 g_mqttPort = 1883;                                        // MQTT Server Port

// Variable used for MQTT Discovery
const char*         g_deviceModel = "ESP32Device";                            // Hardware Model
const char*         g_swVersion = "1.0";                                      // Firmware Version
const char*         g_manufacturer = "Vigasan";                               // Manufacturer Name
String              g_deviceName = "CustomSensor";                            // Device Name
String              g_mqttStatusTopic = "esp32iotsensor/" + g_deviceName;     // MQTT Topic
            

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Public variables-------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
unsigned long       g_Time = 0;
int                 g_count = 0;
int                 g_mqttCounterConn = 0;
float               g_Humidity = 0.0;
float               g_Temperature = 0.0;
bool                g_InitSystem = true;
String              g_UniqueId;
bool                g_SendMqttData = false;
DHT                 g_dht(DHTPIN, DHTTYPE);
int                 g_moist[6] = {0};


void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length);       
void setup_wifi();
void MqttReconnect();
void MqttHomeAssistantDiscovery();
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Input output Pin definitions
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_dht.begin();

    Serial.begin(115200);
    delay(500);

    Serial.println("");
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(g_deviceModel);
    Serial.print("DEVICE: ");
    Serial.println(g_deviceName);
    Serial.print("SW Rev: ");
    Serial.println(g_swVersion);
    Serial.println("----------------------------------------------");
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Wifi Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    }

    if(g_InitSystem)
    {
        delay(100);
        g_InitSystem = false;
        Serial.println("INIT SYSTEM...");
        MqttHomeAssistantDiscovery();     // Send Discovery Data
    }

    if(millis() - g_Time > PERIOD_MILLSEC_500)  // Every 500 [msec]
    {
        g_Time = millis();
        
        if(g_count++ == 20) // Every 10 [sec] or if input status changed
        {
            g_count = 0;
            ////////////////////////////////////////////////////////////////
            // TEMPERATURE HUMIDITY
            ////////////////////////////////////////////////////////////////
            g_Temperature = g_dht.readTemperature();
            g_Humidity = g_dht.readHumidity();
            
            for (int i = 0; i < NUMB_OF_MOISTURE_SENS; i++)
            {
                g_moist[i] = analogRead(MOIST_PIN[i]);
                g_moist[i] = constrain(g_moist[i], 2500, 4095);
                g_moist[i] = map(g_moist[i], 2500, 4095, 100, 0);

                Serial.print("Moisture sensor ");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(g_moist[i]);
                Serial.println(" %");
            }

            Serial.print("Temperature: ");
            Serial.print(g_Temperature);
            Serial.println(" °C");

            Serial.print("Humidity: ");
            Serial.print(g_Humidity);
            Serial.println(" %");

            
            g_SendMqttData = true;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SEND MQTT DATA
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
        if(g_SendMqttData == true)        
        {
            JsonDocument payload;
            String valTemplate;
            payload["temp"] = g_Temperature;
            payload["hum"] = g_Humidity;
            payload["moist0"] =  g_moist[0];
            payload["moist1"] =  g_moist[1];
            payload["moist2"] =  g_moist[2];
            payload["moist3"] =  g_moist[3];
            payload["moist4"] =  g_moist[4];

            // for (int i = 0; i < NUMB_OF_MOISTURE_SENS; i++)
            // {
            //     valTemplate = "moist" + i;
            //     payload[valTemplate] = g_moist[i];
            // }

            String strPayload;
            serializeJson(payload, strPayload);

            if(g_mqttPubSub.connected())
            {
                g_mqttPubSub.publish(g_mqttStatusTopic.c_str(), strPayload.c_str()); 
                Serial.println("MQTT: Send Data!!!");
                Serial.println(" ");
                Serial.println(" ");
                g_SendMqttData = false;
            }
        }
    }
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);
    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(g_ssid);

    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);

    Serial.print("Unique ID: ");
    Serial.println(g_UniqueId);    
   
    while(WiFi.status() != WL_CONNECTED && counter++ < 8) 
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");

    if(WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else
    {
        Serial.println("WiFi NOT connected!!!");
    }
}

void MqttReconnect() 
{
    // Loop until we're reconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (g_mqttPubSub.connect(g_deviceName.c_str(), g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");
            // Subscribe
            g_mqttPubSub.subscribe("homeassistant/status");
            delay(100);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 1 seconds");
            delay(1000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;
    String valTemplate;
    if(g_mqttPubSub.connected())
    {
        Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");
        JsonDocument payload;
        JsonObject device;
        JsonArray identifiers;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Temperature
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_temp" + "/config";
        
        payload["name"] = g_deviceName + ".temp";
        payload["uniq_id"] = g_UniqueId + "_temp";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "temperature";
        payload["val_tpl"] = "{{ value_json.temp | is_defined }}";
        payload["unit_of_meas"] = "°C";
        device = payload["device"].to<JsonObject>();
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device["identifiers"].to<JsonArray>();
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Humidity
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        strPayload.clear();
        identifiers.clear();
        device.clear();
        payload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_hum" + "/config";
        
        payload["name"] = g_deviceName + ".hum";
        payload["uniq_id"] = g_UniqueId + "_hum";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "humidity";
        payload["val_tpl"] = "{{ value_json.hum | is_defined }}";
        payload["unit_of_meas"] = "%";
        device = payload["device"].to<JsonObject>();
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device["identifiers"].to<JsonArray>();
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Moisture
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        for(int i = 0; i < NUMB_OF_MOISTURE_SENS; i++)
        {
            strPayload.clear();
            identifiers.clear();
            device.clear();
            payload.clear();

            discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_moist" + i + "/config";
            valTemplate = String("{{ value_json.moist") + i + " | is_defined }}";
            
            payload["name"] = g_deviceName + ".moist" + i;
            payload["uniq_id"] = g_UniqueId + "_moist" + i;
            payload["stat_t"] = g_mqttStatusTopic;
            payload["dev_cla"] = "moisture";
            payload["val_tpl"] = valTemplate;
            payload["unit_of_meas"] = "%";
            device = payload["device"].to<JsonObject>();
            device["name"] = g_deviceName;
            device["model"] = g_deviceModel;
            device["sw_version"] = g_swVersion;
            device["manufacturer"] = g_manufacturer;
            identifiers = device["identifiers"].to<JsonArray>();
            identifiers.add(g_UniqueId);

            serializeJsonPretty(payload, Serial);
            Serial.println(" ");
            serializeJson(payload, strPayload);

            g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        }

    }
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    byte state = 0;
    String messageTemp;
    
    for (int i = 0; i < length; i++) 
    {
        Serial.print((char)inFrame[i]);
        messageTemp += (char)inFrame[i];
    }
    Serial.println();
  
    if(String(topic) == String("homeassistant/status")) 
    {
        if(messageTemp == "online")
            MqttHomeAssistantDiscovery();
    }
}