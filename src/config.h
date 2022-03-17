#ifndef config_h
#define config_h

// device drivers
#include "pcap04.h"
#include "Adafruit_BMP3XX.h"
#include "MPU.hpp"
#include <ArduinoJson.h>

#define LED_PIN 13 //onboard LED

#define MPU_CS_PIN 10
#define MPU_INT_PIN 8

#define PCAP1_CS_PIN 5
#define PCAP1_INTN_PG5 16

#define PCAP2_CS_PIN 6
#define PCAP2_INTN_PG5 14

#define HEATER1_PWM_PIN 2
#define HEATER1_SEL_PIN 3

#define HEATER2_PWM_PIN 23
#define HEATER2_SEL_PIN 22

#define BMP390_I2C_ADDR 0x76

const unsigned int ros_domain_id = 20; 
// Publishing frequency (Hz)
const unsigned int diagnostic_frequency_hz = 1;
const unsigned int sensor_frequency_hz = 1;

const char *metsensor_id = "999";  // ID - match to serial device name of teensy

pcap_config_handler_t metsensor_pcap_config_handler;
pcap_config_t metsensor_pcap_config;

pcap_results_t* pcap1_results;
pcap_status_t* pcap1_status;

PCAP04 pcap1(PCAP04_V1, PCAP_SPI_MODE, STANDARD, PCAP1_CS_PIN, metsensor_pcap_config);
//PCAP04 pcap2(PCAP04_V0, PCAP_SPI_MODE, HUMIDITY, PCAP2_CS_PIN);

MPU_t mpu(MPU_CS_PIN);

Adafruit_BMP3XX bmp390;

DynamicJsonDocument results_json(1024);

unsigned long current_micros = 0;

#endif
