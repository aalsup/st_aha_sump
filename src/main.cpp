//******************************************************************************************
//  File: ST_Anything_Ultrasonic_ESP01WiFi.ino
//  Authors: Dan G Ogorchock & Daniel J Ogorchock (Father and Son)
//
//  Summary:  This Arduino Sketch, along with the ST_Anything library and the
//  revised SmartThings
//            library, demonstrates the ability of one ESP8266-01 (ESP-01) to
//            implement a multi input/output custom device for integration into
//            SmartThings. The ST_Anything library takes care of all of the work
//            to schedule device updates as well as all communications with the
//            ESP-01's WiFi.
//
//            ST_Anything_Multiples implements the following ST Capabilities in
//            multiples of 1 as a demo of what is possible with a single ESP-01
//              - 1 x Ultrasonmic Sensor device (used to monitor water level)
//
//  Note:  The tiny ESP-01 only has 2 GPIO pins, so this example is somewhat
//  limited.  Use the ST_ANything_Multiples_ESP8266WiFi.ino example to see
//         what else is possible.  As long as you only try using 2 pins, you can
//         use them for whatever you'd like.
//
//  Change History:
//
//    Date        Who            What
//    ----        ---            ----
//    2018-02-15  Dan Ogorchock  Original Creation
//
//******************************************************************************************
#include <Arduino.h>

//******************************************************************************************
// SmartThings Library for ESP8266WiFi
//******************************************************************************************
#include <SmartThingsESP8266WiFi.h>

//******************************************************************************************
// ST_Anything Library
//******************************************************************************************
#include <Constants.h>        //Constants.h is designed to be modified by the end user to adjust behavior of the ST_Anything library
#include <Device.h>           //Generic Device Class, inherited by Sensor and Executor classes
#include <Everything.h>       //Master Brain of ST_Anything library that ties everything together and performs ST Shield communications
#include <Executor.h>         //Generic Executor Class, typically receives data from ST Cloud (e.g. Switch)
#include <InterruptSensor.h>  //Generic Interrupt "Sensor" Class, waits for change of state on digital input
#include <PollingSensor.h>    //Generic Polling "Sensor" Class, polls Arduino pins periodically
#include <Sensor.h>           //Generic Sensor Class, typically provides data to ST Cloud (e.g. Temperature, Motion, etc...)

// #include <PS_Illuminance.h> //Implements a Polling Sensor (PS) to measure
// light levels via a photo resistor

// #include <PS_TemperatureHumidity.h> //Implements a Polling Sensor (PS) to
// measure Temperature and Humidity via DHT library #include
// <PS_DS18B20_Temperature.h> //Implements a Polling Sesnor (PS) to measure
// Temperature via DS18B20 libraries #include <PS_Water.h> //Implements a
// Polling Sensor (PS) to measure presence of water (i.e. leak detector)
// #include <IS_Motion.h>              //Implements an Interrupt Sensor (IS) to
// detect motion via a PIR sensor #include <IS_Contact.h> //Implements an
// Interrupt Sensor (IS) to monitor the status of a digital input pin #include
// <IS_Smoke.h>               //Implements an Interrupt Sensor (IS) to monitor
// the status of a digital input pin #include <IS_DoorControl.h> //Implements an
// Interrupt Sensor (IS) and Executor to monitor the status of a digital input
// pin and control a digital output pin #include <IS_Button.h> //Implements an
// Interrupt Sensor (IS) to monitor the status of a digital input pin for button
// presses #include <EX_Switch.h>              //Implements an Executor (EX) via
// a digital output to a relay #include <EX_Alarm.h>               //Implements
// Executor (EX)as an Alarm Siren capability via a digital output to a relay
// #include <S_TimedRelay.h>           //Implements a Sensor to control a
// digital output pin with timing capabilities

#include <PS_Ultrasonic.h>  //Ultrasonic Distance Measurement Sensor, being used to monitor water level in cylindrical tank
#include <PS_Water.h>

#include <LittleFS.h>
#include <Math.h>
#include "ArduinoJson.h"

//*************************************************************************************************
// ESP01 Pin Definitions - it only has 2 GPIO pins!
//*************************************************************************************************
#define PIN0 0
#define PIN2 2

//******************************************************************************************
// Define which Arduino Pins will be used for each device
//******************************************************************************************

#define PIN_ULTRASONIC_T 2  // digital output to trigger ultrasonic
#define PIN_ULTRASONIC_E 0  // digital input to read the echo

struct config_settings_t {
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gateway;
    IPAddress subnet;
    IPAddress dnsserver;
    uint16_t serverPort;
    IPAddress hubIp;
    uint16_t hubPort;
};

//******************************************************************************************
// ESP8266 WiFi Information
//******************************************************************************************
// String str_ssid = "yourSSIDhere";              //  <---You must edit this line!
// String str_password = "yourWiFiPasswordhere";  //  <---You must edit this line!
// IPAddress ip(192, 168, 1, 227);  // Device IP Address       //  <---You must edit this line!
// IPAddress gateway(192, 168, 1, 1);  // Router gateway          //  <---You must edit this line!
// IPAddress subnet(255, 255, 255, 0);  // LAN subnet mask         //  <---You must edit this line!
// IPAddress dnsserver(192, 168, 1, 1);  // DNS server              //  <---You must edit this line!
// const unsigned int serverPort = 8090;  // port to run the http server on

// Smartthings / Hubitat Hub TCP/IP Address
// IPAddress hubIp(192, 168, 1, 149);  // smartthings/hubitat hub ip //  <---You must edit this line!

// SmartThings / Hubitat Hub TCP/IP Address: UNCOMMENT line that corresponds to
// your hub, COMMENT the other
// const unsigned int hubPort = 39500;  // smartthings hub port
// const unsigned int hubPort = 39501;   // hubitat hub port

IPAddress strToIP(const String& octet1,
                  const String& octet2,
                  const String& octet3,
                  const String& octet4) {
    uint8_t octets[4];
    octets[0] = (uint8_t)octet1.toInt();
    octets[1] = (uint8_t)octet2.toInt();
    octets[2] = (uint8_t)octet3.toInt();
    octets[3] = (uint8_t)octet4.toInt();

    return IPAddress(octets[0], octets[1], octets[2], octets[3]);
}

IPAddress strToIP(const String& ipString) {
    String octet[4];
    uint8_t nOctet = 0;

    int lastIdx = 0;
    int idx = ipString.indexOf(".", lastIdx);
    while (idx >= 0) {
        octet[nOctet] = ipString.substring(lastIdx, idx);
        nOctet++;
        lastIdx = idx + 1;
        idx = ipString.indexOf(".", lastIdx);
    }
    octet[nOctet] = ipString.substring(lastIdx);

    return strToIP(octet[0], octet[1], octet[2], octet[3]);
}

void parseSettings(JsonObject& json,
                   config_settings_t& settings) {
    if (json.containsKey("ssid")) {
        settings.ssid = json["ssid"].as<String>();
    }
    if (json.containsKey("password")) {
        settings.password = json["password"].as<String>();
    }
    if (json.containsKey("ip")) {
        settings.ip = strToIP(json["ip"].as<String>());
    }
    if (json.containsKey("gateway")) {
        settings.gateway = strToIP(json["gateway"].as<String>());
    }
    if (json.containsKey("subnet")) {
        settings.subnet = strToIP(json["subnet"].as<String>());
    }
    if (json.containsKey("dnsserver")) {
        settings.dnsserver = strToIP(json["dnsserver"].as<String>());
    }
    if (json.containsKey("serverPort")) {
        settings.serverPort = json["serverPort"];
    }
    if (json.containsKey("hubIp")) {
        settings.hubIp = strToIP(json["hubIp"].as<String>());
    }
    if (json.containsKey("hubPort")) {
        settings.hubPort = json["hubPort"];
    }

    Serial.println(F("*** Settings ***"));
    Serial.print(F("ssid: "));
    Serial.println(settings.ssid);
    Serial.print(F("password: "));
    Serial.println(settings.password);
    Serial.print(F("ip: "));
    Serial.println(settings.ip);
    Serial.print(F("gateway: "));
    Serial.println(settings.gateway);
    Serial.print(F("subnet: "));
    Serial.println(settings.subnet);
    Serial.print(F("dnsserver: "));
    Serial.println(settings.dnsserver);
    Serial.print(F("serverPort: "));
    Serial.println(settings.serverPort);
    Serial.print(F("hubIp: "));
    Serial.println(settings.hubIp);
    Serial.print(F("hubPort: "));
    Serial.println(settings.hubPort);
}

void loadSettings(config_settings_t& settings) {
    const String configFileName = "/settings.json";
    boolean okSettings = false;

    if (LittleFS.exists(configFileName)) {
        File configFile = LittleFS.open(configFileName, "r");
        if (configFile) {
            const size_t buf_size = configFile.size();
            // Allocate a buffer to store contents of the file.
            char buf[buf_size];
            // Load the config file into the buffer
            configFile.readBytes((char*)buf, buf_size);
            DynamicJsonDocument jsonDoc(1024);
            DeserializationError err = deserializeJson(jsonDoc, buf);
            okSettings = !err;
            if (okSettings) {
                JsonObject json = jsonDoc.as<JsonObject>();
                parseSettings(json, settings);
            }
        }
    }

    if (!okSettings) {
        Serial.println("Failed to load settings from '" + configFileName + "'");
        const char buf[] = "{}";
        DynamicJsonDocument jsonDoc(32);
        deserializeJson(jsonDoc, buf);
        JsonObject json = jsonDoc.as<JsonObject>();
        parseSettings(json, settings);
    }
}

//******************************************************************************************
// st::Everything::callOnMsgSend() optional callback routine.  This is a sniffer
// to monitor
//    data being sent to ST.  This allows a user to act on data changes locally
//    within the Arduino sktech.
//******************************************************************************************
void callback(const String& msg) 
{
    Serial.print(F("ST_Anything Callback: Sniffed data = "));
    Serial.println(msg);
}

//******************************************************************************************
// Arduino Setup() routine
//******************************************************************************************
void setup() {
    Serial.begin(st::Constants::SERIAL_BAUDRATE);
    delay(100);

    Serial.println("Starting device...");

    //******************************************************************************************
    // Declare each Device that is attached to the Arduino
    //  Notes: - For each device, there is typically a corresponding "tile"
    //  defined in your
    //           SmartThings Device Hanlder Groovy code, except when using new
    //           COMPOSITE Device Handler
    //         - For details on each device's constructor arguments below, please
    //         refer to the
    //           corresponding header (.h) and program (.cpp) files.
    //         - The name assigned to each device (1st argument below) must match
    //         the Groovy
    //           Device Handler names.  (Note: "temphumid" below is the exception
    //           to this rule as the DHT sensors produce both "temperature" and
    //           "humidity".  Data from that particular sensor is sent to the ST
    //           Hub in two separate updates, one for "temperature" and one for
    //           "humidity")
    //         - The new Composite Device Handler is comprised of a Parent DH and
    //         various Child
    //           DH's.  The names used below MUST not be changed for the Automatic
    //           Creation of child devices to work properly.  Simply increment the
    //           number by +1 for each duplicate device (e.g. contact1, contact2,
    //           contact3, etc...)  You can rename the Child Devices to match your
    //           specific use case in the ST Phone Application.
    //******************************************************************************************
    const unsigned int POLLING_INTERVAL_SEC = 60;

    // Polling Sensors
    static st::PS_Ultrasonic sensor1(F("ultrasonic1"), POLLING_INTERVAL_SEC, 0, PIN_ULTRASONIC_T, PIN_ULTRASONIC_E);

    // Interrupt Sensors

    // Special sensors/executors (uses portions of both polling and executor
    // classes)

    // Executors

    //*****************************************************************************
    //  Configure debug print output from each main class
    //  -Note: Set these to "false" if using Hardware Serial on pins 0 & 1
    //         to prevent communication conflicts with the ST Shield
    //         communications
    //*****************************************************************************
    st::Everything::debug = true;
    st::Executor::debug = true;
    st::Device::debug = true;
    st::PollingSensor::debug = true;
    st::InterruptSensor::debug = true;

    //*****************************************************************************
    // Initialize the "Everything" Class
    //*****************************************************************************

    // Initialize the optional local callback routine (safe to comment out if not
    // desired)
    st::Everything::callOnMsgSend = callback;

    // Start Filesystem
    LittleFS.begin();

    // Load settings from EEPROM
    config_settings_t settings;
    loadSettings(settings);

    // Create the SmartThings ESP8266WiFi Communications Object
    // STATIC IP Assignment - Recommended
    st::Everything::SmartThing = new st::SmartThingsESP8266WiFi(
        settings.ssid, settings.password,
        settings.ip, settings.gateway, settings.subnet, settings.dnsserver,
        settings.serverPort, settings.hubIp, settings.hubPort,
        st::receiveSmartString);

    // DHCP IP Assigment - Must set your router's DHCP server to provice a static
    // IP address for this device's MAC address st::Everything::SmartThing = new
    // st::SmartThingsESP8266WiFi(str_ssid, str_password, serverPort, hubIp,
    // hubPort, st::receiveSmartString);

    // Run the Everything class' init() routine which establishes WiFi
    // communications with SmartThings Hub
    st::Everything::init();

    //*****************************************************************************
    // Add each sensor to the "Everything" Class
    //*****************************************************************************
    st::Everything::addSensor(&sensor1);

    //*****************************************************************************
    // Add each executor to the "Everything" Class
    //*****************************************************************************

    //*****************************************************************************
    // Initialize each of the devices which were added to the Everything Class
    //*****************************************************************************
    st::Everything::initDevices();
}

//******************************************************************************************
// Arduino Loop() routine
//******************************************************************************************
void loop() {
    //*****************************************************************************
    // Execute the Everything run method which takes care of "Everything"
    //*****************************************************************************
    st::Everything::run();
}