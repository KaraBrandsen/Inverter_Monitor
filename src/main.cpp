#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <M2M_LM75A.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Ticker.h> 
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <SPI.h>
#include <RTCMemory.h>
#include "LittleFS.h"
#include "CRC16.h"
#include "CRC.h"


#define LM75_ADDRESS        0x49
#define IO0_PIN             0
#define IO16_PIN            16
#define LED_PIN             4
#define SCL_PIN             12
#define SDA_PIN             13
#define RX_PIN              14
#define TX_PIN              5

#define TIMEZONE_OFFSET     7200
#define PRIMARY_NTP_SERVER  "za.pool.ntp.org"
#define BACKUP_NTP_SERVER   "igubu.saix.net"
#define NTP_UPDATE          86400
#define DATASET_SIZE        10

typedef struct{
    float grid_voltage;
    float grid_current;
    float grid_power;
    float grid_frequency;

    float output_voltage;
    float output_current;
    float output_power;
    float output_frequency;

    float percent_load;
    float battery_voltage;
    float pv_power;
    float pv_voltage;

    float boost_temp;
    float inverter_temp;
    float internal_temp;
    float external_temp;

    int total_energy;
    float cumulative_pv_total;
    int last_update_time;
    int startup_time;
}data_struct;

typedef struct{
    char hostname[20];
    char ssid[32];
    char password[32];
    char mqtt_broker[16];
    char topic[60];
    int mqtt_port;
    int timeout;
    int serial_timeout;
    int collection_interval;
    int discovery_interval;
    int inverter_protocol;
    float cumulative_pv_total;
    bool valid_wifi;
}settings_struct;

typedef struct{
    float cumulative_pv_total;
}rtc_data_struct;

CRC16 crc;
Ticker rx_busy, tx_busy;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, PRIMARY_NTP_SERVER, TIMEZONE_OFFSET, NTP_UPDATE);
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
M2M_LM75A lm75(LM75_ADDRESS);
AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient pubsubClient(espClient);
RTCMemory<rtc_data_struct> rtcMemory;

settings_struct settings = {
    "Inverter-2",
    "Chris",
    "DHXSLTLPLFSL",
    "192.168.192.15",
    "inverter2/",
    8888,
    120,
    1000,
    10,
    60,
    1,
    0,
    true
};

int run_count = 0;
int dataset_index = 0;
data_struct data[DATASET_SIZE];
bool update = true;
bool debug_enabled = false;
String qpigs = "";
String qtpr = "";
String qet = "";
String error_txt = "";
rtc_data_struct initial_struct = {0.0};
rtc_data_struct *rtc_data = &initial_struct;

void debug_print(String text){
    if(debug_enabled){
        char current_buf[] = "YY/MM/DD hh:mm:ss";
        DateTime current_time(timeClient.getEpochTime());
        
        Serial.println(String(current_time.toString(current_buf)) + " - " + text);
    }
}

void log_error(String text){
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    DateTime current_time(timeClient.getEpochTime());

    error_txt = String(current_time.toString(current_buf)) + " - " + text;
}

void transmitting(){
    digitalWrite(TX_PIN, !(digitalRead(TX_PIN))); 
}

void receiving(){
    digitalWrite(RX_PIN, !(digitalRead(RX_PIN))); 
}

void send_uart(String tx_text){
    tx_busy.attach_ms(40, transmitting);
    Serial.println(tx_text);
    tx_busy.detach();
    digitalWrite(TX_PIN, LOW);
}

String receive_uart(){
    int time_elapsed = 0;

    while((Serial.available() <= 0) && (time_elapsed < settings.serial_timeout))
    {
        time_elapsed += 50;
        delay(50);
    }

    rx_busy.attach_ms(40, receiving);
    String result = Serial.readStringUntil('\r');
    rx_busy.detach();
    digitalWrite(RX_PIN, LOW);
    return result;
}

void setLed(int index, int brightness, int colour){
    if(brightness > 50)
        brightness = 50;
    led.setBrightness(brightness);
    led.setPixelColor(0, colour);
    led.show();
}

void checkWifi(){
    debug_print("Checking Wifi");
    int connection_time = 0;

    if(WiFi.status() != WL_CONNECTED)
    {
        debug_print("Wifi Disconnected");
        log_error("Wifi Disconnected");
    }

    while (WiFi.status() != WL_CONNECTED) {
        if(connection_time == settings.timeout){
            setLed(0, 10, 0XFF0000);
            settings.cumulative_pv_total = data[dataset_index].cumulative_pv_total;
            settings.valid_wifi = false;
            EEPROM.put(0x0, settings);
            EEPROM.commit();
            ESP.restart();
            break;
        }
        delay(1000);
        connection_time++;
        if(connection_time%2 == 0)
        {
            setLed(0, 10, 0X00FF00);
        }
        else
        {
            setLed(0, 10, 0X000000);
        }
    }

    setLed(0, 10, 0X00FF00);
    if(settings.valid_wifi == false){
        settings.valid_wifi = true;
        EEPROM.put(0x0, settings);
        EEPROM.commit();
    }
    debug_print("Wifi Connected - " + WiFi.localIP().toString());
}

void checkMQTT()
{
    debug_print("Checking MQTT");
    checkWifi();
    int connection_time = 0;

    if(!pubsubClient.setBufferSize(512))
        setLed(0, 10, 0XFF0000);

    while (!pubsubClient.connected()) 
    {
        if(connection_time == settings.timeout){
            setLed(0, 10, 0XFF0000);
            debug_print("MQTT Disconnected");
            log_error("MQTT Disconnected");
            break;
        }
    
        if (!pubsubClient.connect(settings.hostname)) 
        {
            delay(1000);
            connection_time++;
        } 
    }
    debug_print("MQTT Connected");
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

String processor(const String& var){
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    char startup_buf[] = "YY/MM/DD hh:mm:ss";
    char update_buf[] = "YY/MM/DD hh:mm:ss";
    int cur_time = timeClient.getEpochTime();
    int uptime = cur_time - data[dataset_index].startup_time;
    int uptime_days = uptime/(86400);
    int uptime_hours = (uptime%86400)/3600;
    int uptime_min = ((uptime%86400)%3600)/60;
    int uptime_sec = ((uptime%86400)%3600)%60;

    DateTime current_time(cur_time);
    DateTime startup_time(data[dataset_index].startup_time);
    DateTime last_update_time(data[dataset_index].last_update_time);

    if(var == "host"){
        return settings.hostname;
    }
    if(var == "ssid"){
        return settings.ssid;
    }
    if(var == "password"){
        return settings.password;
    }
    if(var == "mqtt_host"){
        return settings.mqtt_broker;
    }
    if(var == "mqtt_port"){
        return String(settings.mqtt_port);
    }
    if(var == "mqtt_topic"){
        return settings.topic;
    }
    if(var == "interval"){
        return String(settings.collection_interval);
    }
    if(var == "discovery"){
        return String(settings.discovery_interval);
    }
    if(var == "serial_timeout"){
        return String(settings.serial_timeout);
    }
    if(var == "wifi_timeout"){
        return String(settings.timeout);
    }
    if(var == "inverters"){
        return String(settings.inverter_protocol);
    }

    if(var == "grid_voltage"){
        return String(data[dataset_index].grid_voltage, 1);
    }
    if(var == "grid_current"){
        return String(data[dataset_index].grid_current, 1);
    }
    if(var == "grid_power"){
        return String(data[dataset_index].grid_power);
    }
    if(var == "grid_frequency"){
        return String(data[dataset_index].grid_frequency, 1);
    }   
    if(var == "output_voltage"){
        return String(data[dataset_index].output_voltage, 1);
    }
    if(var == "output_current"){
        return String(data[dataset_index].output_current, 1);
    }
    if(var == "output_power"){
        return String(data[dataset_index].output_power);
    }
    if(var == "output_frequency"){
        return String(data[dataset_index].output_frequency, 1);
    }
    if(var == "output_load"){
        return String(data[dataset_index].percent_load, 1);
    }
    if(var == "battery_voltage"){
        return String(data[dataset_index].battery_voltage, 1);
    }
    if(var == "pv_power"){
        return String(data[dataset_index].pv_power);
    }
    if(var == "pv_voltage"){
        return String(data[dataset_index].pv_voltage, 1);
    }
    if(var == "boost_section_temp"){
        return String(data[dataset_index].boost_temp, 1);
    }
    if(var == "inverter_section_temp"){
        return String(data[dataset_index].inverter_temp, 1);
    }
    if(var == "internal_temp"){
        return String(data[dataset_index].internal_temp, 1);
    }
    if(var == "external_temp"){
        return String(data[dataset_index].external_temp, 1);
    }
    if(var == "total_generated"){
        return String(data[dataset_index].total_energy);
    }
    if(var == "cumulative_pv_generated"){
        return String(data[dataset_index].cumulative_pv_total, 3);
    }
    if(var == "current_time"){
        return String(current_time.toString(current_buf));
    }
    if(var == "Last_update_time"){
        return String(last_update_time.toString(update_buf));
    }
    if(var == "startup_time"){
        return String(startup_time.toString(startup_buf));
    }
    if(var == "uptime"){
        return String(uptime_days) + "d " + String(uptime_hours) + "h " + String(uptime_min) + "m " + String(uptime_sec) + "s";
    }
    return String();
}

String api_data() {
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    char startup_buf[] = "YY/MM/DD hh:mm:ss";
    char update_buf[] = "YY/MM/DD hh:mm:ss";
    int cur_time = timeClient.getEpochTime();
    int uptime = cur_time - data[dataset_index].startup_time;
    int uptime_days = uptime/(86400);
    int uptime_hours = (uptime%86400)/3600;
    int uptime_min = ((uptime%86400)%3600)/60;
    int uptime_sec = ((uptime%86400)%3600)%60;

    DateTime current_time(cur_time);
    DateTime startup_time(data[dataset_index].startup_time);
    DateTime last_update_time(data[dataset_index].last_update_time);

    String message = "{";

    message += "\"grid_voltage\":\"" + String(data[dataset_index].grid_voltage, 1) + "\",";
    message += "\"grid_current\":\"" + String(data[dataset_index].grid_current, 1) + "\",";
    message += "\"grid_power\":\"" + String(data[dataset_index].grid_power) + "\",";
    message += "\"grid_frequency\":\"" + String(data[dataset_index].grid_frequency, 1) + "\",";

    message += "\"output_voltage\":\"" + String(data[dataset_index].output_voltage, 1) + "\",";
    message += "\"output_current\":\"" + String(data[dataset_index].output_current, 1) + "\",";
    message += "\"output_power\":\"" + String(data[dataset_index].output_power) + "\",";
    message += "\"output_frequency\":\"" + String(data[dataset_index].output_frequency, 1) + "\",";

    message += "\"percent_load\":\"" + String(data[dataset_index].percent_load, 1) + "\",";
    message += "\"battery_voltage\":\"" + String(data[dataset_index].battery_voltage, 1) + "\",";
    message += "\"pv_power\":\"" + String(data[dataset_index].pv_power) + "\",";
    message += "\"pv_voltage\":\"" + String(data[dataset_index].pv_voltage, 1) + "\",";

    message += "\"boost_temp\":\"" + String(data[dataset_index].boost_temp, 1) + "\",";
    message += "\"inverter_temp\":\"" + String(data[dataset_index].inverter_temp, 1) + "\",";
    message += "\"internal_temp\":\"" + String(data[dataset_index].internal_temp, 1) + "\",";
    message += "\"external_temp\":\"" + String(data[dataset_index].external_temp, 1) + "\",";

    message += "\"total_energy\":\"" + String(data[dataset_index].total_energy) + "\",";
    message += "\"cumulative_pv_total\":\"" + String(data[dataset_index].cumulative_pv_total, 3) + "\",";

    message += "\"last_error\":\"" + error_txt + "\",";

    message += "\"current_time\":\"" + String(current_time.toString(current_buf)) + "\",";
    message += "\"last_update_time\":\"" + String(last_update_time.toString(update_buf)) + "\",";
    message += "\"startup_time\":\"" + String(startup_time.toString(startup_buf)) + "\",\"uptime\": \"" + String(uptime_days) + "d " + String(uptime_hours) + "h " + String(uptime_min) + "m " + String(uptime_sec) + "s\"";

    message += "}";
    
    return message;
}

void startOTA(){
    debug_print("Start OTA");
    ArduinoOTA.onStart([]() 
    {
        setLed(0, 100, 0XFF00FF);
    });
    ArduinoOTA.onEnd([]() 
    {
        setLed(0, 100, 0X00FFFF);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
    {
        int colour = 255.0 * float(progress) / float(total);
        setLed(0, 100, ((255 - colour)) + (colour << 8));
    });
    ArduinoOTA.onError([](ota_error_t error) 
    {
        if (error == OTA_AUTH_ERROR) {
            log_error("OTA Auth Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_BEGIN_ERROR) {
            log_error("OTA Begin Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_CONNECT_ERROR) {
            log_error("OTA Connect Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_RECEIVE_ERROR) {
            log_error("OTA Receive Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_END_ERROR) {
            log_error("OTA End Error");
            setLed(0, 100, 0XFF0000);
        }
    });
    ArduinoOTA.setRebootOnSuccess(true);
    ArduinoOTA.setHostname("Inverter");
    ArduinoOTA.begin();
}

void getAxpertData(){
    int send_retry = 3;
    int retry_cnt = 0;
    int crc_recv = 0;
    int crc_calc = -1;
    String qpigs_result = "";
    String qtpr_result = "";
    String qet_result = "";

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QPIGS");
        qpigs_result = receive_uart();
        String qpigs_data = qpigs_result.substring(0, 133);
        String qpigs_crc = qpigs_result.substring(133, 135);
        crc_recv = ((int)qpigs_crc.charAt(0) << 8) + (int)qpigs_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qpigs_data.c_str(), qpigs_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qpigs = qpigs_result + " - " + String(qpigs_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qpigs_result.charAt(133) + qpigs_result.charAt(134);
    debug_print(qpigs);
    retry_cnt = 0;
    crc_recv = 0;
    crc_calc = -1;

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QET");
        qet_result = receive_uart();
        String qet_data = qet_result.substring(0, 9);
        String qet_crc = qet_result.substring(9, 11);
        crc_recv = ((int)qet_crc.charAt(0) << 8) + (int)qet_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qet_data.c_str(), qet_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qet = qet_result + " - " + String(qet_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qet_result.charAt(9) + qet_result.charAt(10);
    debug_print(qet);
    retry_cnt = 0;
    crc_recv = 0;
    crc_calc = -1;

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QTPR");
        qtpr_result = receive_uart();
        String qtpr_data = qtpr_result.substring(0, 24);
        String qtpr_crc = qtpr_result.substring(24, 26);
        crc_recv = ((int)qtpr_crc.charAt(0) << 8) + (int)qtpr_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qtpr_data.c_str(), qtpr_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qtpr = qtpr_result + " - " + String(qtpr_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qtpr_result.charAt(24) + qtpr_result.charAt(25);
    debug_print(qtpr);
    if(!timeClient.isTimeSet())
        timeClient.update();

    int current_time =  timeClient.getEpochTime();

    debug_print("Parsing QPIGS");
    if(qpigs_result.length() == 135)
    {
        int power_mult = -1;
        if(qpigs_result.substring(7,8) == "1")
        {
            power_mult = 1;
        }

        data[dataset_index].grid_voltage = qpigs_result.substring(1, 5).toFloat();
        data[dataset_index].grid_power = power_mult * qpigs_result.substring(8, 13).toFloat();
        data[dataset_index].grid_frequency = qpigs_result.substring(14, 18).toFloat();
        data[dataset_index].grid_current = qpigs_result.substring(19, 25).toFloat();

        data[dataset_index].output_voltage = qpigs_result.substring(26, 31).toFloat();
        data[dataset_index].output_power = qpigs_result.substring(32, 37).toFloat();
        data[dataset_index].output_frequency = qpigs_result.substring(38, 42).toFloat();
        data[dataset_index].output_current = qpigs_result.substring(43, 48).toFloat();

        data[dataset_index].percent_load = qpigs_result.substring(49, 52).toFloat();
        data[dataset_index].battery_voltage = qpigs_result.substring(65, 70).toFloat();
        data[dataset_index].pv_power = qpigs_result.substring(81, 87).toFloat();
        data[dataset_index].pv_voltage = qpigs_result.substring(99, 104).toFloat();
    }

    debug_print("Parsing QTPR");
    if(qtpr_result.length() == 26)
    {
        data[dataset_index].boost_temp = qtpr_result.substring(1, 5).toFloat();
        data[dataset_index].inverter_temp = qtpr_result.substring(7, 11).toFloat();
        data[dataset_index].internal_temp = qtpr_result.substring(13, 17).toFloat();
    }

    debug_print("Parsing QET");
    if(qet_result.length() == 11)
        data[dataset_index].total_energy = qet_result.substring(1, 9).toFloat();

    debug_print("Parsing TEMP");
    data[dataset_index].external_temp = lm75.getTemperature();

    
    if(data[dataset_index].cumulative_pv_total == 0)
    {
        data[dataset_index].cumulative_pv_total = data[dataset_index].total_energy;
    }

    debug_print("Parsing Last Update");
    if((current_time - data[dataset_index].last_update_time) > 120)
    {
        data[dataset_index].last_update_time = current_time;
    }

    debug_print("Parsing Cumulative PV");
    if((data[dataset_index].last_update_time != 0) && (current_time > data[dataset_index].last_update_time))
    {
        data[dataset_index].cumulative_pv_total += ((data[dataset_index].pv_power/3600.0)*(current_time - data[dataset_index].last_update_time))/1000.0;
        data[dataset_index].last_update_time = current_time; 

        if(data[dataset_index].total_energy > data[dataset_index].cumulative_pv_total)
        {
            data[dataset_index].cumulative_pv_total = data[dataset_index].total_energy;
        }

        debug_print("Saving Cumulative Data to RTC");
        rtc_data->cumulative_pv_total = data[dataset_index].cumulative_pv_total;
        rtcMemory.save();
    }
    debug_print("Done Getting Data");
}

void getVoltronicData(){
    int send_retry = 3;
    int retry_cnt = 0;
    int crc_recv = 0;
    int crc_calc = -1;
    String qpigs_result = "";
    String qtpr_result = "";
    String qet_result = "";

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QPIGS");
        qpigs_result = receive_uart();
        String qpigs_data = qpigs_result.substring(0, 133);
        String qpigs_crc = qpigs_result.substring(133, 135);
        crc_recv = ((int)qpigs_crc.charAt(0) << 8) + (int)qpigs_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qpigs_data.c_str(), qpigs_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qpigs = qpigs_result + " - " + String(qpigs_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qpigs_result.charAt(133) + qpigs_result.charAt(134);
    debug_print(qpigs);
    retry_cnt = 0;
    crc_recv = 0;
    crc_calc = -1;

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QET");
        qet_result = receive_uart();
        String qet_data = qet_result.substring(0, 9);
        String qet_crc = qet_result.substring(9, 11);
        crc_recv = ((int)qet_crc.charAt(0) << 8) + (int)qet_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qet_data.c_str(), qet_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qet = qet_result + " - " + String(qet_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qet_result.charAt(9) + qet_result.charAt(10);
    debug_print(qet);
    retry_cnt = 0;
    crc_recv = 0;
    crc_calc = -1;

    while((crc_recv != crc_calc) && (retry_cnt < send_retry))
    {
        send_uart("QTPR");
        qtpr_result = receive_uart();
        String qtpr_data = qtpr_result.substring(0, 24);
        String qtpr_crc = qtpr_result.substring(24, 26);
        crc_recv = ((int)qtpr_crc.charAt(0) << 8) + (int)qtpr_crc.charAt(1);
        crc_calc = crc16((uint8_t *) qtpr_data.c_str(), qtpr_data.length(), 0x1021, 0, 0, false, false);
        retry_cnt++;
    }

    qtpr = qtpr_result + " - " + String(qtpr_result.length()) + " - CRC Calc:" + String(crc_calc, HEX) + " - CRC Recv:" + String(crc_recv, HEX) + " - " + qtpr_result.charAt(24) + qtpr_result.charAt(25);
    debug_print(qtpr);
    if(!timeClient.isTimeSet())
        timeClient.update();

    int current_time =  timeClient.getEpochTime();

    debug_print("Parsing QPIGS");
    if(qpigs_result.length() == 135)
    {
        int power_mult = -1;
        if(qpigs_result.substring(7,8) == "1")
        {
            power_mult = 1;
        }

        data[dataset_index].grid_voltage = qpigs_result.substring(1, 5).toFloat();
        data[dataset_index].grid_power = power_mult * qpigs_result.substring(8, 13).toFloat();
        data[dataset_index].grid_frequency = qpigs_result.substring(14, 18).toFloat();
        data[dataset_index].grid_current = qpigs_result.substring(19, 25).toFloat();

        data[dataset_index].output_voltage = qpigs_result.substring(26, 31).toFloat();
        data[dataset_index].output_power = qpigs_result.substring(32, 37).toFloat();
        data[dataset_index].output_frequency = qpigs_result.substring(38, 42).toFloat();
        data[dataset_index].output_current = qpigs_result.substring(43, 48).toFloat();

        data[dataset_index].percent_load = qpigs_result.substring(49, 52).toFloat();
        data[dataset_index].battery_voltage = qpigs_result.substring(65, 70).toFloat();
        data[dataset_index].pv_power = qpigs_result.substring(81, 87).toFloat();
        data[dataset_index].pv_voltage = qpigs_result.substring(99, 104).toFloat();
    }

    debug_print("Parsing QTPR");
    if(qtpr_result.length() == 26)
    {
        data[dataset_index].boost_temp = qtpr_result.substring(1, 5).toFloat();
        data[dataset_index].inverter_temp = qtpr_result.substring(7, 11).toFloat();
        data[dataset_index].internal_temp = qtpr_result.substring(13, 17).toFloat();
    }

    debug_print("Parsing QET");
    if(qet_result.length() == 11)
        data[dataset_index].total_energy = qet_result.substring(1, 9).toFloat();

    debug_print("Parsing TEMP");
    data[dataset_index].external_temp = lm75.getTemperature();

    
    if(data[dataset_index].cumulative_pv_total == 0)
    {
        data[dataset_index].cumulative_pv_total = data[dataset_index].total_energy;
    }

    debug_print("Parsing Last Update");
    if((current_time - data[dataset_index].last_update_time) > 120)
    {
        data[dataset_index].last_update_time = current_time;
    }

    debug_print("Parsing Cumulative PV");
    if((data[dataset_index].last_update_time != 0) && (current_time > data[dataset_index].last_update_time))
    {
        data[dataset_index].cumulative_pv_total += ((data[dataset_index].pv_power/3600.0)*(current_time - data[dataset_index].last_update_time))/1000.0;
        data[dataset_index].last_update_time = current_time; 

        if(data[dataset_index].total_energy > data[dataset_index].cumulative_pv_total)
        {
            data[dataset_index].cumulative_pv_total = data[dataset_index].total_energy;
        }

        debug_print("Saving Cumulative Data to RTC");
        rtc_data->cumulative_pv_total = data[dataset_index].cumulative_pv_total;
        rtcMemory.save();
    }
    debug_print("Done Getting Data");
}

void sendData(){
    String base[18] = {
        "grid_voltage",
        "grid_current",
        "grid_power",
        "grid_frequency",
        "output_voltage",
        "output_current",
        "output_power",
        "output_frequency",
        "percent_load",
        "battery_voltage",
        "pv_power",
        "pv_voltage",
        "boost_temp",
        "inverter_temp",
        "internal_temp",
        "external_temp",
        "total_energy",
        "cumulative_pv_total"
    };

    String inverter_name = String(settings.hostname);
    String topic = "";

    if(!((data[dataset_index].grid_voltage == 0) && (data[dataset_index].output_voltage == 0) && (data[dataset_index].battery_voltage == 0)))
    {
        topic = inverter_name + "/" + base[0]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].grid_voltage).c_str(), String(data[dataset_index].grid_voltage).length());

        topic = inverter_name + "/" + base[1]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].grid_current).c_str(), String(data[dataset_index].grid_current).length());

        topic = inverter_name + "/" + base[2]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].grid_power).c_str(), String(data[dataset_index].grid_power).length());

        topic = inverter_name + "/" + base[3]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].grid_frequency).c_str(), String(data[dataset_index].grid_frequency).length());

        topic = inverter_name + "/" + base[4]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].output_voltage).c_str(), String(data[dataset_index].output_voltage).length());

        topic = inverter_name + "/" + base[5]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].output_current).c_str(), String(data[dataset_index].output_current).length());

        topic = inverter_name + "/" + base[6]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].output_power).c_str(), String(data[dataset_index].output_power).length());

        topic = inverter_name + "/" + base[7]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].output_frequency).c_str(), String(data[dataset_index].output_frequency).length());
        
        topic = inverter_name + "/" + base[8]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].percent_load).c_str(), String(data[dataset_index].percent_load).length());
        
        topic = inverter_name + "/" + base[9]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].battery_voltage).c_str(), String(data[dataset_index].battery_voltage).length());

        topic = inverter_name + "/" + base[10]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].pv_power).c_str(), String(data[dataset_index].pv_power).length());

        topic = inverter_name + "/" + base[11]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].pv_voltage).c_str(), String(data[dataset_index].pv_voltage).length());
        }

    if(!((data[dataset_index].boost_temp == 0) && (data[dataset_index].inverter_temp == 0) && (data[dataset_index].internal_temp == 0)))
    {
        topic = inverter_name + "/" + base[12]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].boost_temp).c_str(), String(data[dataset_index].boost_temp).length());

        topic = inverter_name + "/" + base[13]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].inverter_temp).c_str(), String(data[dataset_index].inverter_temp).length());
        
        topic = inverter_name + "/" + base[14]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].internal_temp).c_str(), String(data[dataset_index].internal_temp).length());
    }
        
    topic = inverter_name + "/" + base[15]; 
    checkMQTT();
    pubsubClient.publish(topic.c_str(), String(data[dataset_index].external_temp).c_str(), String(data[dataset_index].external_temp).length());
            
    if(!(data[dataset_index].total_energy == 0))
    {
        topic = inverter_name + "/" + base[16]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].total_energy).c_str(), String(data[dataset_index].total_energy).length());
                    
        topic = inverter_name + "/" + base[17]; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), String(data[dataset_index].cumulative_pv_total).c_str(), String(data[dataset_index].cumulative_pv_total).length());
    }
    debug_print("Done Sending Data");
}

void sendMQTTDiscovery(){
    debug_print("Sending Discovery");
    String base[18] = {
        "Grid Voltage",
        "Grid Current",
        "Grid Power",
        "Grid Frequency",
        "Output Voltage",
        "Output Current",
        "Output Power",
        "Output Frequency",
        "Percent Load",
        "Battery Voltage",
        "PV Power",
        "PV Voltage",
        "Boost Temp",
        "Inverter Temp",
        "Internal Temp",
        "External Temp",
        "Total Energy",
        "Cumulative PV Total"
    };

    String inverter_name = String(settings.hostname);

    for(int i = 0 ; i < 18 ; i++)
    {
        const size_t capacity = (20*JSON_ARRAY_SIZE(1) + 2*JSON_OBJECT_SIZE(1) + 2*20*JSON_OBJECT_SIZE(2) + 
                            JSON_OBJECT_SIZE(3) + 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5))*2;
        DynamicJsonDocument dataObject(capacity);

        String underscored = base[i];
        underscored.toLowerCase();
        underscored.replace(" ", "_");

        String topic = "homeassistant/sensor/" + inverter_name + "/" + underscored + "/config";

        dataObject["unique_id"] = inverter_name + "_" + underscored;
        dataObject["name"] = inverter_name + " " + base[i];

        if(underscored.lastIndexOf("voltage") != -1)
        {
            dataObject["device_class"] =  "voltage";
            dataObject["unit_of_measurement"] = "V";
        }

        if(underscored.lastIndexOf("current") != -1)
        {
            dataObject["device_class"] =  "current";
            dataObject["unit_of_measurement"] = "A";
        }

        if(underscored.lastIndexOf("power") != -1)
        {
            dataObject["device_class"] =  "power";
            dataObject["unit_of_measurement"] = "W";
        }

        if(underscored.lastIndexOf("frequency") != -1)
        {
            dataObject["device_class"] =  "frequency";
            dataObject["unit_of_measurement"] = "Hz";
        }

        if(underscored.lastIndexOf("temp") != -1)
        {
            dataObject["device_class"] =  "temperature";
            dataObject["unit_of_measurement"] = "°C";
        }

        if(underscored.lastIndexOf("total") != -1)
        {
            dataObject["state_class"] = "total";
            dataObject["device_class"] =  "energy";
            dataObject["unit_of_measurement"] = "kWh";
        }

        if(underscored.lastIndexOf("load") != -1)
        {
            dataObject["device_class"] =  "battery";
            dataObject["unit_of_measurement"] = "%";
        }
        
        dataObject["json_attributes_topic"] = inverter_name + "/" + underscored;
        dataObject["state_topic"] = inverter_name + "/" + underscored;
        
        JsonObject device = dataObject.createNestedObject("device");

        device["manufacturer"] = "Voltronic Power";
        device["model"] = "3kW";
        device["name"] = "Icon Inverter";
        device["sw_version"] = "1.0";

        JsonArray identifiers = device.createNestedArray("identifiers");
        identifiers.add(inverter_name);

        String json_data = "";
        serializeJson(dataObject, json_data);
        checkMQTT();
        pubsubClient.publish(topic.c_str(), json_data.c_str(), json_data.length());
    }
    debug_print("Done Sending Discovery");
}

void resetData(){
    int current_time =  timeClient.getEpochTime();

    for(int i = 0 ; i < DATASET_SIZE ; i++)
    {
        data[i] = {
            0.0,
            0.0,
            0.0,
            0.0,

            0.0,
            0.0,
            0.0,
            0.0,

            0.0,
            0.0,
            0.0,
            0.0,

            0.0,
            0.0,
            0.0,
            0.0,

            0,
            0.0,
            current_time,
            current_time
        };
    }
}

IRAM_ATTR void BTInterrupt() {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200)
    {
        update = !update;

        if(update){
            startOTA();
        }
        else{
            setLed(0, 10, 0X00FF00);
        }
    }
    last_interrupt_time = interrupt_time;
}

void mqtt_receive(char *topic, byte *payload, unsigned int length) {
}

void setup() {
    EEPROM.begin(256);
    Serial.begin(2400);

    EEPROM.get(0x0, settings); 

    if(!isnan(settings.cumulative_pv_total)){
        debug_print("Error reading settings from EEPROM");
        EEPROM.put(0x0, settings);
        EEPROM.commit();
    }

    pinMode(IO0_PIN, INPUT);
    pinMode(IO16_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);

    digitalWrite(RX_PIN, HIGH);
    digitalWrite(TX_PIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(IO0_PIN), BTInterrupt, FALLING);  

    led.begin();
    led.clear();
    setLed(0, 10, 0XFFFFFF);

    if(!LittleFS.begin()){
        log_error("An Error has occurred while mounting SPIFFS");
    }

    if(settings.valid_wifi){
        debug_print("Trying old wifi credentials");
        debug_print("Connecting to: " + String(settings.ssid));
        WiFi.mode(WIFI_STA);
        WiFi.setPhyMode(WIFI_PHY_MODE_11G);
        WiFi.hostname(settings.hostname);
        WiFi.begin(settings.ssid, settings.password);

        checkWifi();
    }
    else{
        debug_print("No valid wifi credentials");
        IPAddress local_IP(192,168,0,1);
        IPAddress gateway(192,168,0,1);
        IPAddress subnet(255,255,255,0);

        WiFi.softAPConfig(local_IP, gateway, subnet);
        WiFi.softAP(settings.hostname, "12345678");
        debug_print(WiFi.softAPIP().toString());
        
    }

    timeClient.begin();
    Wire.begin(SDA_PIN, SCL_PIN);

    lm75.begin();
    if(lm75.isShutdown()){
        lm75.wakeup();
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", String(), false, processor);
    });

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){
        int args = request->args();
        for(int i=0;i<args;i++){
            if(request->arg(i) == ""){
                continue;
            }

            if(request->argName(i) == "host"){
                strcpy(settings.hostname, request->arg(i).c_str());
            }

            if(request->argName(i) == "ssid"){
                strcpy(settings.ssid, request->arg(i).c_str());
                settings.valid_wifi = true;
            }

            if(request->argName(i) == "password"){
                strcpy(settings.password, request->arg(i).c_str());
                settings.valid_wifi = true;
            }

            if(request->argName(i) == "mqtt_host"){
                strcpy(settings.mqtt_broker, request->arg(i).c_str());
            }

            if(request->argName(i) == "mqtt_topic"){
                strcpy(settings.topic, request->arg(i).c_str());
            }

            if(request->argName(i) == "mqtt_port"){
                settings.mqtt_port = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "interval"){
                settings.collection_interval = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "discovery"){
                settings.discovery_interval = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "serial_timeout"){
                settings.serial_timeout = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "wifi_timeout"){
                settings.timeout = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "inverters"){
                settings.inverter_protocol = atoi(request->arg(i).c_str());
            }
        }
        settings.cumulative_pv_total = data[dataset_index].cumulative_pv_total;
        EEPROM.put(0x0, settings);
        EEPROM.commit();
        request->send(LittleFS, "/index.html", String(), false, processor);
        ESP.restart();
    });

    server.on("/get_data", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", api_data().c_str());
    });

    server.on("/send_data", HTTP_GET, [](AsyncWebServerRequest *request){
        debug_print("Sending: " + request->getParam("send")->value());
        Serial.println(request->getParam("send")->value());

        String result = "";

        if(Serial.available() > 0)
        {
            result = Serial.readStringUntil('\r');
        }
        request->send_P(200, "text/plain", result.c_str());
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/style.css", "text/css");
    });


    server.onNotFound(notFound);
    server.begin();

    timeClient.update();
    int retry_count[] = {3, 1};

    while(!timeClient.isTimeSet())
    {
        if(retry_count[0] == 0)
        {
            if(retry_count[1] == 0)
            {
                ESP.restart();
            }
            checkWifi();
            log_error("NTP Error: Using backup server.");
            timeClient.setPoolServerName(BACKUP_NTP_SERVER);
            retry_count[0] = 3;
            retry_count[1]--;
        }

        delay(3000);
        timeClient.forceUpdate();
        retry_count[0]--;
    }

    setLed(0, 10, 0X00FF00);
    digitalWrite(RX_PIN, LOW);
    digitalWrite(TX_PIN, LOW);

    pubsubClient.setServer(settings.mqtt_broker, settings.mqtt_port);
    pubsubClient.setCallback(mqtt_receive);

    startOTA();
    resetData();

    bool result = rtcMemory.begin();
    if(result){
        debug_print("Previous Data Found");
    }

    float cumulative_pv_total = settings.cumulative_pv_total;
    if(!isnan(cumulative_pv_total))
    {
        debug_print("Recovered solar cumulative data from EEPROM");
        data[dataset_index].cumulative_pv_total = cumulative_pv_total;
    }
    else
    {
        rtc_data = rtcMemory.getData();

        if(result)
        {
            data[dataset_index].cumulative_pv_total = rtc_data->cumulative_pv_total;
        }
        else
        {
            rtc_data->cumulative_pv_total = 0;
            log_error("Error getting last solar cumulative data");
            debug_print("Error getting last solar cumulative data");
        }
    }
}

void loop() {
    if(update)
    {
        ArduinoOTA.handle();
    }
    delay(1000);

    if(run_count % settings.collection_interval == 0)
    {
        debug_print("Getting Data");
        if(settings.inverter_protocol == 0){
            getVoltronicData();
        }

        if(settings.inverter_protocol == 1){
            getAxpertData();
        }
        debug_print("Sending Data");
        //sendData();
    }

    if(run_count % settings.discovery_interval == 0)
    {
        //sendMQTTDiscovery();
        run_count = 0;
    }

    run_count++;
    timeClient.update();
}