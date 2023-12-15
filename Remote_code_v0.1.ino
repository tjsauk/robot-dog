/*
 * 
 *   NovaSM3 Remote - a Spot-Mini Micro clone Remote Control
 *   Version: 5.2
 *   Version Date: 2021-10-26
 *   
 *   Author:  Chris Locke - cguweb@gmail.com
 *   Nova's website:  https://novaspotmicro.com
 *   GitHub Project:  https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
 *   YouTube Playlist:  https://www.youtube.com/watch?v=00PkTcGWPvo&list=PLcOZNHwM_I2a3YZKf8FtUjJneKGXCfduk
 *   
 *   RELEASE NOTES:
 *
 *   DEV NOTES:
 *     
 *   =============================================================
 *     
 *   Copyright (c) 2020-2021 Christopher M. Locke and others
 *   Distributed under the terms of the MIT License. 
 *   SPDX-License-Identifier: MIT
 *   
 *   Permission is hereby granted, free of charge, to any person obtaining
 *   a copy of this software and associated documentation files (the
 *   "Software"), to deal in the Software without restriction, including
 *   without limitation the rights to use, copy, modify, merge, publish,
 *   distribute, sublicense, and/or sell copies of the Software, and to
 *   permit persons to whom the Software is furnished to do so, subject to
 *   the following conditions:
 *   
 *   The above copyright notice and this permission notice shall be
 *   included in all copies or substantial portions of the Software.
 *   
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *   OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *   
 *      
*/


float VERSION = 5.2;

int debug = 0;                    //general debugging
int debug2 = 0;                   //nrf24 serial comm
int debug3 = 0;                   //controls to serial
int debug4 =  0;                  //controls interactive test mode 
int debug5 =  0;                  //battery monitor 
int plotter = 0;                  //control to plotter (also set debug3 = 1)

byte nrf_active = 1;
byte batt_active = 1;             //activate battery level monitoring
byte activity_active = 0;         //activate inactivity monitor / sleep mode

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//========================================
//  NRF24 Pin Wiring for Arduino Mega/Nano:
//  pin 7/7 -> CE pin
//  pin 8/8 -> CSN pin
//  pin 52/13 -> SCK pin
//  pin 51/11 -> MOSI pin
//  pin 50/12 -> MISO pin
//  pin xx -> IRQ pin (unused)
//========================================
// NRF24L01 transceiver setup
RF24 radio(7, 8); // Initialize the RF24 radio object with pins 7 and 8 for CE and CSN respectively
unsigned int nrfInterval = 60; // Interval in seconds for NRF updates
unsigned long lastNRFUpdate = 0; // Timestamp of the last NRF update
uint8_t address[][6] = {"1Node", "2Node"}; // Addresses for the NRF nodes
bool radioNumber = 0; // Flag to select which address to use for transmitting (0 or 1)
const int data_num = 14; // Number of elements in data arrays
bool tm_resp; // Response flag for transmission
uint8_t tm_data[data_num] = { // Transmit data array initialization
  0, 0, 0, 0, // btn1, btn2, btn3, btn4
  0, 0, // sel1, sel2
  0, 0, 0, 0, // p1, p2, p3, p4
  0, 0, 0, 0 // lx, ly, rx, ry
};

// DEV: Setup for currently unused two-way communication
uint8_t rc_data[data_num] = { // Receive data array initialization
  0, 0, 0, // battery, spd, Y
  0, 0, 0, // 
  1, 0, 0, 0, // Current gate, Standing up
  0, 0, 0, 0 // Unused elements
};

// Definitions for slide potentiometer pins
#define POT1_PIN A8
#define POT2_PIN A9
#define POT3_PIN A10
#define POT4_PIN A11

// Variables for potentiometer percentage values and their previous states
int per1, per2, per3, per4; // Current percentage values of pots
int pper1, pper2, pper3, pper4; // Previous percentage values of pots

// Variables for reading the potentiometer values and their previous states
int p1 = 0, p1p = 0; // Current and previous reading of POT1
int p2 = 0, p2p = 0; // Current and previous reading of POT2
int p3 = 0, p3p = 0; // Current and previous reading of POT3
int p4 = 0, p4p = 0; // Current and previous reading of POT4
int pot_slack = 10; // Threshold for potentiometer change detection
int pot_min = 0, pot_max = 255; // Min and max values for potentiometer mapping


// Joystick related variables

// Center deadzone values for both joysticks
int jstick_center[2] = {12, 12}; // Array holding the deadzone threshold for each joystick

// Minimum and maximum values for generic joystick mapping
int j_min = 0; // Minimum value for joystick position mapping
int j_max = 255; // Maximum value for joystick position mapping
// Note: If sticks are oriented oppositely to this configuration, swap j_min and j_max

// Pin definitions for the right joystick
#define RJX_PIN A4 // Right Joystick X-axis pin
#define RJY_PIN A5 // Right Joystick Y-axis pin
int rj_min = 0; // Minimum raw value for right joystick
int rj_max = 1023; // Maximum raw value for right joystick
// Note: If right joystick is oriented oppositely, swap rj_min and rj_max

// Pin definitions for the left joystick
#define LJX_PIN A6 // Left Joystick X-axis pin
#define LJY_PIN A7 // Left Joystick Y-axis pin
int lj_min = 0; // Minimum raw value for left joystick
int lj_max = 1023; // Maximum raw value for left joystick
// Note: If left joystick is oriented oppositely, swap lj_min and lj_max

// The values j_min, j_max, rj_min, rj_max, lj_min, and lj_max are used for mapping
// the raw joystick input values (which can vary based on hardware specifics) to a
// standard range that the rest of the program can consistently work with.


//rgb leds (5)
#define LED_STRIP_PIN 2
#define LED_STRIP_NUM 5
Adafruit_NeoPixel led_strip = Adafruit_NeoPixel(LED_STRIP_NUM, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
unsigned int rgbInterval = 45;
unsigned long lastRGBUpdate = 0;
int rgb_del = 256;
int current_led = 0;
int fadeStep = 0;
int fade_steps = 400;
int fade_back = 1;
int pattern = 0;                  //set which light pattern is used
int pattern_int = 400;            //set pattern delay between
int pattern_cnt = 3;              //set number of loops of pattern
int pattern_step = 0;
int cur_rgb_val1[3] = {55, 0, 200};
int cur_rgb_val2[3] = {125, 125, 0};
int rgb_num = 99;
int rgb_bright = 25;
byte set_led[5] = {0, 0, 0, 0, 0};

// Button related variables

// "Fire" buttons (4)
#define BTN1_PIN 3 // Pin number for left front fire button
#define BTN2_PIN 5 // Pin number for right front fire button
#define BTN3_PIN 4 // Pin number for left back fire button
#define BTN4_PIN 6 // Pin number for right back fire button

int b1 = 0; // Current state of left front fire button
int b1p = 0; // Previous state of left front fire button
int b2 = 0; // Current state of right front fire button
int b2p = 0; // Previous state of right front fire button
int b3 = 0; // Current state of left back fire button
int b3p = 0; // Previous state of left back fire button
int b4 = 0; // Current state of right back fire button
int b4p = 0; // Previous state of right back fire button

// The variables b1, b2, b3, b4 hold the current state (pressed or not) of each fire button,
// while b1p, b2p, b3p, b4p keep track of their previous states for change detection.

// Select / Stop buttons (2)
#define SEL1_PIN 9  // Pin number for left green select/stop button
#define SEL2_PIN 10 // Pin number for right red select/stop button

int s1 = 0; // Current state of left green select/stop button
int s1p = 0; // Previous state of left green select/stop button
int s2 = 0; // Current state of right red select/stop button
int s2p = 0; // Previous state of right red select/stop button

// Similar to the fire buttons, s1 and s2 hold the current state of the select/stop buttons,
// while s1p and s2p store their previous states to detect changes.

//OLED screens (2)
//D20 = SDA
//D21 = SCL

//#define RST 16

//pinMode(RST,OUTPUT);
//digitalWrite(RST, LOW);                          // set low to reset OLED before initialising in setup
//delay(50); 
//digitalWrite(RST, HIGH); 


//waveshare oled, wire res to digital oin 16 and give that as argument in stead of -1 couple rows down
//to get the screen to address 0x3D dc pin of the screen needs to be pulled to vcc 
//or it will default to 0x3c
#define OLED_ADDR 0x3D
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 16);//OLED_RESET
unsigned int oledInterval = 90;
unsigned long lastOLEDUpdate = 0;
unsigned int displayInterval = 30;
unsigned long lastDisplayUpdate = 0;
  
#define OLED_ADDR2 0x3C
#define SCREEN_WIDTH2 128
#define SCREEN_HEIGHT2 32
Adafruit_SSD1306 display2(SCREEN_WIDTH2, SCREEN_HEIGHT2, &Wire, OLED_RESET);
unsigned int oled2Interval = 90;
unsigned long lastOLED2Update = 0;

//battery monitor
#define BATT_MONITOR A3
unsigned long batteryInterval = 60000;             //60 seconds
unsigned long lastBatteryUpdate = 0;
int batt_cnt = 0;
float batt_voltage = 7.4;                          //approx fully charged battery minimum nominal voltage
float batt_voltage_prev = 7.4;                     //comparison voltage to prevent false positives
float batt_voltage2 = 11.4;                        //approx fully charged Nova battery minimum nominal voltage
float batt_voltage_prev2 = 11.4;                   //comparison voltage to prevent false positives
float avg_volts = 0;
float batt_levels[10] = {                          //voltage drop alarm levels(10)
   7.3, 7.1, 6.9, 6.7
};

//misc vars
unsigned int controlsInterval = 10;//milliseconds between button checks
unsigned long controlsUpdate = 0;

byte inactivity_set = 1;
int inactivity_loop = 5;
int inactivity_cnt = 0;
unsigned int inactivityInterval = 3000;
unsigned long inactivityUpdate = 0;


//spotmicro bmp for oled
const unsigned char smbmp [] PROGMEM = {
  // 'spot, 128x64px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x0e, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x07, 0x7f, 0xff, 0x7f, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xf0, 0x7f, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xfd, 0xbf, 0x00, 0x00, 0x00, 0xff, 0x80, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xf1, 0x7e, 0x00, 0x00, 0x01, 0xfe, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0xc3, 0xfc, 0x00, 0x00, 0x00, 0xf8, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xfe, 0x03, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x37, 0xf8, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x70, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xe0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x70, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x30, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x38, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x18, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x80, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x07, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xc0, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xe0, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x78, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x38, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x6f, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x21, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x20, 0x3e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x30, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x30, 0x03, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00, 0x40, 0x00, 0x70, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x20, 0x00, 0x00, 0x78, 0x00, 0x0e, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x0d, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x20, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x00, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x20, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x30, 0x00, 0x03, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


//function prototypes
void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack, int rgbNum = 99);
void blink_rgb_num(int rgb_val[3], int rgb_val2[3], int rgbNum=99);


void setup() {
  Serial.begin(19200);
  while (!Serial) {
  }

  delay(3000);
  if (nrf_active) {
    if (!radio.begin()) {
      if (debug2) Serial.println(F("radio hardware is not responding!!"));
      while (1) {}
    } else {
      if (debug2) Serial.println(F("radio hardware is ready!"));
      radio.setPALevel(RF24_PA_LOW);
      radio.setPayloadSize(sizeof(tm_data));
      radio.setChannel(124);
      radio.openWritingPipe(address[radioNumber]);
      radio.enableAckPayload();
      radio.setRetries(5,5); // delay, count

      if (debug2) Serial.println(F("radio ready to transmit"));
    }
  }

  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);
  pinMode(POT4_PIN, INPUT);

  pinMode(LJX_PIN, INPUT);
  pinMode(LJY_PIN, INPUT);

  pinMode(RJX_PIN, INPUT);
  pinMode(RJY_PIN, INPUT);

  led_strip.begin();
  led_strip.setBrightness(rgb_bright);
  wipe_strip();

  pinMode(BTN1_PIN, INPUT);//INPUT_PULLUP
  pinMode(BTN2_PIN, INPUT);
  pinMode(BTN3_PIN, INPUT);
  pinMode(BTN4_PIN, INPUT);

  pinMode(SEL1_PIN, INPUT);
  pinMode(SEL2_PIN, INPUT);
  
  // initialize with the I2C addr
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);  
  display2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR2);  

  rgb_check(8);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
    
  display2.clearDisplay();
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10,0);
  display2.println("Initializing...");
  display2.display();
  
  delay(2000);
  display.clearDisplay();
  display2.clearDisplay();
  display2.setTextSize(2);
  
  //animate bitmap
  display.drawBitmap(-10, 5,  smbmp, 128, 64, WHITE);
  display.display();
  display.startscrollright(0x00, 0x07);

  //strobe version banner
  for (int i=25;i>0;i--) {
    int d = (i*4);
    if (i > 22) d = (i*10);
    if (i == 25) d = (i*100);
    display2.setCursor(22, 2);
    display2.setTextSize(2);
    display2.println("NOVA SM3");
    display2.setCursor(16, 18);
    display2.print("REMOTE");
    display2.setCursor(86, 22);
    display2.setTextSize(1);
    display2.print(" v");
    display2.println(VERSION);
    display2.display();
    delay(d);
    display2.clearDisplay();
    display2.display();
    if (i == 25) d = (i*10);
    delay(d);
  }
  display2.clearDisplay();
  display2.display();
  delay(2000);

  display2.setCursor(30,12);
  display2.setTextSize(3);
  display2.print("READY!");
  display2.display();

  display.stopscroll();
  display.clearDisplay();
  display.display();

  //init remote with Nova
  if (nrf_active) {
    for (int i = 0; i < data_num; i++) {
      tm_data[i] = 0;
    }
    tm_resp = nrf_check();
  }

  if (debug) Serial.println(F("Ready!"));
  pattern = 10;
  pattern_cnt = 16;
  pattern_int = 150;
}

void loop() {
  if (millis() - controlsUpdate > controlsInterval) {
    check_pots();//näissä asetetaan toimintoja
    check_buttons();
    check_joysticks();
  }

  if (nrf_active && (millis() - lastNRFUpdate > nrfInterval)) tm_resp = nrf_check();//Sends the data and receiwes the response

  if (millis() - lastRGBUpdate > rgbInterval) rgb_check(pattern);//päivittää rgb värien tilan jos sitä on vaihdettu
  if (millis() - lastOLEDUpdate > oledInterval) oled_refresh();//päivittää näyttöjen tilat jos sitä on vaihdettu
  if (millis() - lastOLED2Update > oled2Interval) oled2_refresh();
  if (millis() - lastDisplayUpdate > displayInterval) update_display(2);//this writes stuff to the bigger screen

  if (batt_active && (millis() - lastBatteryUpdate > batteryInterval)) battery_check(0); 
  if (activity_active && millis() - inactivityUpdate > inactivityInterval) inactivity_check(); 

  

}


bool nrf_check() {
  bool resp = false;

  if (tm_data) {
    resp = radio.write(&tm_data, sizeof(tm_data));
    if (resp) {
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rc_data, sizeof(rc_data));
        if (debug2) {
          for (int i = 0; i < data_num; i++) {
            Serial.print(rc_data[i]);Serial.print("\t");
          }
          Serial.println(" ACK REC");
        }
      }

      if (debug2) {
        for (int i = 0; i < data_num; i++) {
          Serial.print(tm_data[i]);Serial.print("\t");
        }
        Serial.println(" DATA SENT");
      }
    } else {
      //handle failed transmission
      if (debug2) {
        Serial.print(F("Transmission failed! "));
        Serial.println(" ");
      }
    }

    lastNRFUpdate = millis();
  }

  return resp;
}


void inactivity_check() {
  if (inactivity_set) {
    inactivity_cnt++;
    if (debug) {
      Serial.print(F("icnt: "));
      Serial.println(inactivity_cnt);
    }

    //put into sleep mode
    if (inactivity_cnt == inactivity_loop) {
      if (debug) {
        Serial.println(F("update display"));
      }
      update_display(4);
    } else if (inactivity_cnt == (inactivity_loop * 2)) {
      inactivity_cnt = 0;
      display.clearDisplay();
      display2.clearDisplay();
    }
  } else {
    inactivity_cnt = 0;
  }
  inactivity_set = 1;
  inactivityUpdate = millis();
}

/*

Nova PS2 compatible controls
  
SEL1_PIN = PSB_START
SEL2_PIN = PSB_SELECT

RJX_PIN = PSS_RX
RJY_PIN = PSS_RY
LJX_PIN = PSS_LX
LJY_PIN = PSS_LY

BTN1_PIN = PSB_L1
BTN3_PIN = PSB_L2
BTN2_PIN = PSB_R1
BTN4_PIN = PSB_R2

POT1_PIN = (step_height_factor)
POT3_PIN = (ramp_spd/ramp_dist)
POT2_PIN = (move_steps)
POT4_PIN = (spd)

- unassigned PS2 controls -
PSB_PAD_UP
PSB_PAD_RIGHT
PSB_PAD_LEFT
PSB_PAD_DOWN

PSB_TRIANGLE
PSB_CROSS
PSB_CIRCLE
PSB_SQUARE

PSB_L3
PSB_R3

*/

// Function to check and process potentiometer values
void check_pots() {
  int updis = 0; // Flag to indicate if there is an update in potentiometer values

  // Processing for POT1
  int p1c = analogRead(POT1_PIN); // Read current value of POT1
  if (p1c != p1p) { // Check if there is a significant change in POT1 value
    p1 = map(p1c, 0, 1023, pot_max, pot_min); // Map the read value to the defined range
    if (p1c > (p1p + pot_slack) || p1c < (p1p - pot_slack)) { // Check if change is above slack threshold
      p1p = p1c; // Update the previous value
      tm_data[6] = p1; // Update the transmit data array
      per1 = map(p1, pot_max, pot_min, 100, -100); // Convert reading to a percentage
      if (per1 < 3 && per1 > -3) per1 = 0; // Zeroing small values for stability
      if (per1 != pper1) { // Check for significant change to update display
        pper1 = per1; // Update previous percentage value
        updis = 1; // Set update flag
      }
    }
  }

  // Similar processing for POT2
  int p2c = analogRead(POT2_PIN);
  if (p2c != p2p) {
    p2 = map(p2c, 0, 1023, pot_max, pot_min);
    if (p2c > (p2p + pot_slack) || p2c < (p2p - pot_slack)) {
      p2p = p2c;
      tm_data[7] = p2;
      per2 = map(p2, pot_max, pot_min, 100, -100);
      if (per2 < 3 && per2 > -3) per2 = 0;
      if (per2 != pper2) {
        pper2 = per2;
        updis = 1;
      }
    }
  }

  // Similar processing for POT3
  int p3c = analogRead(POT3_PIN);
  if (p3c != p3p) {
    p3 = map(p3c, 0, 1023, pot_max, pot_min);
    if (p3c > (p3p + pot_slack) || p3c < (p3p - pot_slack)) {
      p3p = p3c;
      tm_data[8] = p3;
      per3 = map(p3, pot_max, pot_min, 100, -100);
      if (per3 < 3 && per3 > -3) per3 = 0;
      if (per3 != pper3) {
        pper3 = per3;
        updis = 1;
      }
    }
  }

  // Similar processing for POT4
  int p4c = analogRead(POT4_PIN);
  if (p4c != p4p) {
    p4 = map(p4c, 0, 1023, pot_max, pot_min);
    if (p4c > (p4p + pot_slack) || p4c < (p4p - pot_slack)) {
      p4p = p4c;
      tm_data[9] = p4;
      per4 = map(p4, pot_max, pot_min, 100, -100);
      if (per4 < 3 && per4 > -3) per4 = 0;
      if (per4 != pper4) {
        pper4 = per4;
        updis = 1;
      }
    }
  }

  // Update display if any potentiometer value has changed
  if (updis) {
    // update_display(1); // Uncomment this line if you have a display update function
  }
}

// Function to check the state of buttons and update corresponding values
void check_buttons() {
  // Check and update the state of Button 1
  int b1c = digitalRead(BTN1_PIN); // Read the current state of Button 1

  if (b1c != b1p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b1p = b1c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (b1c == b1p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b1p = b1c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[0] = b1c; // Update transmit data array
  }
if (false){
 // Similar process for Button 2
  int b2c = digitalRead(BTN2_PIN);
  if (b2c != b2p) { // Check if state has changed since the last read
    b2 = b2c; // Update current state
    b2p = b2c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[1] = b2; // Update transmit data array

    // Update LED color based on button state
    if (b2) {
      led_strip.setPixelColor(4, led_strip.Color(125, 125, 0)); // Button pressed
    } else {
      led_strip.setPixelColor(4, led_strip.Color(0, 0, 0)); // Button released
    }
    led_strip.show(); // Refresh LED strip to reflect changes
  }
}
 

  // Check and update the state of Button 1
  int b2c = digitalRead(BTN2_PIN); // Read the current state of Button 1

  if (b2c != b2p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b2p = b2c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (b2c == b2p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b2p = b2c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[1] = b2c; // Update transmit data array
  }
  // Check and update the state of Button 1
  int b3c = digitalRead(BTN3_PIN); // Read the current state of Button 1

  if (b3c != b3p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b3p = b3c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (b3c == b3p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b3p = b3c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[2] = b3c; // Update transmit data array
  }

  // Check and update the state of Button 1
  int b4c = digitalRead(BTN4_PIN); // Read the current state of Button 1

  if (b4c != b4p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b4p = b4c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (b4c == b4p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    b4p = b4c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[3] = b4c; // Update transmit data array
  }

// Check and update the state of Button 1
  int s1c = digitalRead(SEL1_PIN); // Read the current state of Button 1

  if (s1c != s1p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    s1p = s1c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (s1c == s1p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    s1p = s1c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[4] = s1c; // Update transmit data array
  }
  // Similar process for Select Button 2
   int s2c = digitalRead(SEL2_PIN); // Read the current state of Button 1

  if (s2c != s2p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    s2p = s2c; // Update previous state
    inactivity_set = 0; // Reset inactivity
  }else if (s2c == s2p) { // Check if state has changed since the last read
    //b1 = b1c; // Update current state
    s2p = s2c; // Update previous state
    inactivity_set = 0; // Reset inactivity
    tm_data[5] = s2c; // Update transmit data array
  }
}


// Function to check joystick positions and update corresponding values
void check_joysticks() {
  // Read and normalize right joystick X-axis value to -1, 0, or 1
  int rjx = map(analogRead(RJX_PIN), rj_min, rj_max, j_min, j_max);
  //if (abs(rjx) < jstick_center[1]) rjx = 0; // Set to 0 if within center deadzone
  tm_data[12] = rjx; // Update transmit data array for right joystick X-axis

  // Read and normalize left joystick X-axis value to -1, 0, or 1
  int ljx = map(analogRead(LJX_PIN), lj_min, lj_max, j_min, j_max);
  //if (abs(ljx) < jstick_center[0]) ljx = 0; // Set to 0 if within center deadzone
  tm_data[10] = ljx; // Update transmit data array for left joystick X-axis

  // Read and normalize right joystick Y-axis value to -1, 0, or 1
  int rjy = map(analogRead(RJY_PIN), rj_min, rj_max, j_max , j_min);
  //if (abs(rjy) < jstick_center[1]) rjy = 0; // Set to 0 if within center deadzone
  tm_data[13] = rjy; // Update transmit data array for right joystick Y-axis

  // Read and normalize left joystick Y-axis value to -1, 0, or 1
  int ljy = map(analogRead(LJY_PIN), lj_min, lj_max, j_max , j_min);
  //if (abs(ljy) < jstick_center[0]) ljy = 0; // Set to 0 if within center deadzone
  tm_data[11] = ljy; // Update transmit data array for left joystick Y-axis

  controlsUpdate = millis(); // Update control timestamp
}//lx  , ly käännä, rx, ry käännä ekat on 147 tokat 148 keskellä


/*
   -------------------------------------------------------
   OLED Functions
   -------------------------------------------------------
*/

void update_display(int dis) {
  if (dis == 1) {
    display.clearDisplay();

    display.setCursor(0, 16);
    display.setTextSize(1);
    display.print("P1");
    display.setCursor(15, 16);
    display.setTextSize(2);
    display.print(per1);
    display.print("%");
    display.setCursor(67, 16);
    display.setTextSize(1);
    display.print("P2");
    display.setCursor(79, 16);
    display.setTextSize(2);
    display.print(per2);
    display.print("%");

    display.setCursor(0, 40);
    display.setTextSize(1);
    display.print("P3");
    display.setCursor(15, 40);
    display.setTextSize(2);
    display.print(per3);
    display.print("%");
    display.setCursor(67, 40);
    display.setTextSize(1);
    display.print("P4");
    display.setCursor(79, 40);
    display.setTextSize(2);
    display.print(per4);
    display.print("%");

    display.display();
  } else if (dis == 2 && !inactivity_set) {
    display.clearDisplay();

    display.setCursor(0, 2);
    display.setTextSize(1);
    display.print("Spd");
    display.setCursor(22, 2);
    display.setTextSize(2);
    display.print(float(rc_data[1])/100);

    display.setCursor(73, 2);
    display.setTextSize(1);
    display.print("Y");
    display.setCursor(87, 2);
    display.setTextSize(2);
    display.print(rc_data[2]);

    if (true){//show balancing x and y offset amounts
    

    display.setCursor(0, 26);
    display.setTextSize(1);
    display.print("XC");
    display.setCursor(12, 26);
    display.setTextSize(2);
    display.print(rc_data[3]);
    display.setCursor(67, 26);
    display.setTextSize(1);
    display.print("YC");
    display.setCursor(80, 26);
    display.setTextSize(2);
    display.print(rc_data[4]);
    }
    

    

    display.setCursor(0, 50);
    display.setTextSize(1);
    display.print("Gate");
    display.setCursor(27, 50);
    display.setTextSize(2);
    display.print(rc_data[6]);

if (false){
 display.setCursor(50, 50);
    display.setTextSize(1);
    display.print("MO");
    display.setCursor(65, 50);
    display.setTextSize(2);
    display.print(rc_data[6]);
}
   

    display.setCursor(60, 50);
    display.setTextSize(1);
    display.print("Enabled");
    display.setCursor(115, 50);
    display.setTextSize(2);
    display.print(rc_data[7]);

    display.display();

    lastDisplayUpdate = millis();
  } else if (dis == 3) {
    byte mod = 1;
    if (rc_data[6] < 5) {
      mod = rc_data[6] + 1;
    }
    display2.clearDisplay();
    display2.setCursor(55, 0);
    display2.setTextSize(1);
    display2.print("MODE ");
    display2.print(mod);
    display2.setTextSize(2);
    switch (mod) {
      case 1:
        display2.setCursor(44, 9);
        display2.print("March");
        break;
      case 2:
        display2.setCursor(52, 9);
        display2.print("Walk");
        break;
      case 3:
        display2.setCursor(19, 9);
        display2.print("Freestyle");
        break;
      case 4:
        display2.setCursor(51, 9);
        display2.print("Trot");
        break;
      case 5:
        display2.setCursor(39, 9);
        display2.print("Follow");
        break;
    }
    display2.setCursor(40, 25);
    display2.setTextSize(1);
    display2.print("Press Start");
    display2.display();

    lastDisplayUpdate = millis();
  } else if (dis == 4) {
    display.clearDisplay();
    display.setCursor(12, 4);
    display.setTextSize(3);
    display.print("Zzz...");
    display.display();

    display2.clearDisplay();

    lastDisplayUpdate = millis();
  }
}


void bfillcircle(int d, int cx, int cy, int r) {
  if (d == 2) {
    display2.fillCircle(cx, cy, r, SSD1306_BLACK);
    display2.display();
  } else {
    display.fillCircle(cx, cy, r, SSD1306_BLACK);
    oled_refresh();
  }
}
void wfillcircle(int d, int cx, int cy, int r) {
  if (d == 2) {
    display2.fillCircle(cx, cy, r, SSD1306_WHITE);
    display2.display();
  } else {    
    display.fillCircle(cx, cy, r, SSD1306_WHITE);
    oled_refresh();
  }
}

void bdrawline(int d, int sx, int sy, int ex, int ey) {
  if (d == 2) {
    display2.drawLine(sx, sy, ex, ey, SSD1306_BLACK);
    display2.display();
  } else {
    display.drawLine(sx, sy, ex, ey, SSD1306_BLACK);
    oled_refresh();
  }
}

void wdrawline(int d, int sx, int sy, int ex, int ey) {
  if (d == 2) {
    display2.drawLine(sx, sy, ex, ey, SSD1306_WHITE);
    display2.display();
  } else {
    display.drawLine(sx, sy, ex, ey, SSD1306_WHITE);
    oled_refresh();
  }
}

void oled_refresh() {
  display.display();
  lastOLEDUpdate = millis();
}
void oled2_refresh() {
  display2.display();
  lastOLED2Update = millis();
}




/*
   -------------------------------------------------------
   Check Battery Level
    :provide general description and explanation here
   -------------------------------------------------------
*/
void battery_check(byte bshow) {
  int syshalt = 0;
  int batt_danger = 0;
  int sensorValue = analogRead(BATT_MONITOR);
  batt_voltage = sensorValue * (5 / 1023.00) * (680/390)*(6.7/2.83);     // Convert the reading values from 5v to 7.5V
  batt_voltage2 = (float(rc_data[0]) / 10);//

  display2.clearDisplay();
  display2.setTextSize(1);
  display2.setCursor(14,0);
  display2.print("Remote  ");
  display2.setTextSize(2);
  display2.print(batt_voltage);
  display2.println("v");
  display2.setTextSize(1);
  display2.setCursor(14,18);
  display2.print("Nova  ");
  display2.setTextSize(2);
  display2.print(batt_voltage2);
  display2.println("v");
  oled2_refresh();

  if (debug) {
    Serial.print(batt_voltage); Serial.print(" volts ");
    Serial.print(batt_voltage2); Serial.println(" volts2");
  }
  if (batt_voltage2 && (batt_voltage2 <= (batt_voltage_prev2 - .05))) {
    bshow = 1;
  }
  if ((batt_voltage && (batt_voltage <= (batt_voltage_prev - .05))) || bshow) {
    if (batt_cnt == 3 || bshow) {
      if (batt_cnt > 1) {
        batt_voltage = avg_volts / batt_cnt;
      }
      for (int i = 0; i < 4; i++) {
        if (batt_voltage <= batt_levels[i]) {
          batt_danger++;
        } else if ((batt_voltage >= (batt_levels[i] - 0.02)) && (batt_voltage <= (batt_levels[i] + 0.02))) {
          batt_danger = i;
        }
      }
      if (debug || debug5) {
        Serial.print(batt_voltage); Serial.print(" volts - BATTERY LEVEL #"); Serial.println(batt_danger);
      }

      if (!bshow) {
        display2.clearDisplay();
        display2.setTextSize(2);
        if (batt_danger > 3) {
          display2.setCursor(12,2);
          display2.println("BATT LOW");
          display2.setCursor(6,18);
          display2.println("SYSTEM HALT");
          syshalt = 1;
  
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 0; cur_rgb_val1[2] = 0;
          cur_rgb_val2[0] = 0; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 0;
          rgb_num = 99;
          pattern = 9;
          pattern_cnt = 10;
          pattern_int = 5;
        } else if (batt_danger > 0) {
          display2.setCursor(6,2);
          display2.println("BATT WARN");
          display2.setCursor(6,19);
          display2.print(batt_voltage);
          display2.println(" VOLTS");
  
          cur_rgb_val1[0] = 255; cur_rgb_val1[1] = 255; cur_rgb_val1[2] = 0;
          cur_rgb_val2[0] = 0; cur_rgb_val2[1] = 0; cur_rgb_val2[2] = 0;
          rgb_num = 99;
          pattern = 9;
          pattern_cnt = 10;
          pattern_int = 5;
        }
        oled2_refresh();
      }
      batt_voltage_prev = batt_voltage;
      batt_cnt = 0;
      avg_volts = 0; 
       
      if (syshalt) {
        if (debug || debug5) {
          Serial.print(batt_voltage); Serial.println(F(" volts - SYSTEM HALTED!"));
        }
        while(1) {           //simulate system halt
          delay(3000);
        }
      }
    } else {
      batt_cnt++;
      avg_volts += batt_voltage;
      if (debug5) {
        Serial.print("batt change #"); Serial.print(batt_cnt); 
        Serial.print(": sensor / volts "); Serial.print(sensorValue); Serial.print(F(" / ")); Serial.println(batt_voltage);
      }
    }
  }

  lastBatteryUpdate = millis();
}


/*
   -------------------------------------------------------
   LED Functions
   -------------------------------------------------------
*/

void rgb_check(int pat) {
  switch (pat) {
    case 0:
      colorWipe(led_strip.Color(0, 0, 0)); // off
      break;
    case 1:
      fade(0, 0, 0, 0, 0, 125, fade_steps, fade_back, rgb_num); // fade from blue to black
      break;
    case 2:
      fade(0, 255, 0, 64, 0, 0, fade_steps, 1, rgb_num); // fade from orange to black
      break;
    case 3:
      fade(0, 0, 0, 125, 0, 0, fade_steps, 1, rgb_num); // fade from green to black
      break;
    case 4:
      fade(0, 125, 0, 125, 0, 0, fade_steps, 1, rgb_num); // fade from yellow to black
      break;
    case 5:
      fade(0, 125, 0, 0, 0, 0, fade_steps, 1, rgb_num); // fade from red to black
      break;
    case 6:
      fade(0, 125, 0, 0, 0, 125, fade_steps, 1, rgb_num); // fade from purple to black
      break;
    case 7:
      colorWipe(led_strip.Color(255, 255, 255)); //white
      break;
    case 8:
      rainbow(pattern_int);
      break;
    case 9:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        blink_rgb_num(cur_rgb_val1, cur_rgb_val2, rgb_num);
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_strip();
      }
      break;
    case 10:
      if (pattern_cnt) {
        rgbInterval = pattern_int;
        colorWipeWave(led_strip.Color(0, 32, 0)); // green wipe wave
      } else if (rgbInterval == pattern_int) {
        rgbInterval = 30;
        wipe_strip();
      }
      break;
    case 11:
      fade(cur_rgb_val1[0], cur_rgb_val2[0], cur_rgb_val1[1], cur_rgb_val2[1], cur_rgb_val1[2], cur_rgb_val2[2], fade_steps, fade_back, rgb_num); // fade from color1 to color2
      break;
  }
}



void fade(int redStartValue, int redEndValue, int greenStartValue, int greenEndValue, int blueStartValue, int blueEndValue, int totalSteps, int fadeBack, int rgbNum = 99) {
  static float redIncrement, greenIncrement, blueIncrement;
  static float red, green, blue;
  static boolean fadeUp = false;

  if (fadeStep == 0) { // first step is to initialise the initial colour and increments
    red = redStartValue;
    green = greenStartValue;
    blue = blueStartValue;
    fadeUp = false;

    redIncrement = (float)(redEndValue - redStartValue) / (float)totalSteps;
    greenIncrement = (float)(greenEndValue - greenStartValue) / (float)totalSteps;
    blueIncrement = (float)(blueEndValue - blueStartValue) / (float)totalSteps;
    fadeStep = 1; // next time the function is called start the fade
  } else { // all other steps make a new colour and display it
    // make new colour
    red += redIncrement;
    green +=  greenIncrement;
    blue += blueIncrement;

    // set up the pixel buffer
    if (rgbNum != 99) {
      led_strip.setPixelColor(rgbNum, led_strip.Color((int)red, (int)green, (int)blue));
    } else {
      for (int i = 0; i < led_strip.numPixels(); i++) {
        led_strip.setPixelColor(i, led_strip.Color((int)red, (int)green, (int)blue));
      }
    }

    // now display it
    led_strip.show();
    fadeStep += 1; // go on to next step
    if (fadeStep >= totalSteps) { // finished fade
      if (fadeUp) { // finished fade up and back
        fadeStep = 0;
        return; // so next call recalabrates the increments
      }
      // now fade back
      if (fadeBack) {
        fadeUp = true;
        redIncrement = -redIncrement;
        greenIncrement = -greenIncrement;
        blueIncrement = -blueIncrement;
        fadeStep = 1; // don't calculate the increments again but start at first change
      }
    }
  }
}

void colorWipe(uint32_t c) {
  static int i = 0;
  static int i2 = 1;
  wipe_strip();
  led_strip.setPixelColor(i, c);
  led_strip.setPixelColor(i2, c);
  led_strip.show();
  i++;
  i2++;
  if (i >= led_strip.numPixels()) {
    i = 0;
    wipe_strip(); // blank out led_strip
  }
  if (i2 >= led_strip.numPixels()) {
    i2 = 0;
    wipe_strip(); // blank out led_strip
  }
  lastRGBUpdate = millis();
}

void colorWipeWave(uint32_t c) {
  if (!current_led) {
    wipe_strip();
  } else {  
    if (pattern_cnt) pattern_cnt--;
    led_strip.setPixelColor((current_led-1), c);
    led_strip.show();
  }

  if (current_led == led_strip.numPixels()) {
    current_led = 0;
  } else {
    current_led++;
  }

  lastRGBUpdate = millis();
}

void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < led_strip.numPixels(); i++) {
      int pixelHue = firstPixelHue + (i * 65536L / led_strip.numPixels());
      led_strip.setPixelColor(i, led_strip.gamma32(led_strip.ColorHSV(pixelHue)));
    }
    led_strip.show(); // Update strip with new contents
    delayMicroseconds(wait);
  }
  lastRGBUpdate = millis();
}


void wipe_strip() { // clear all LEDs
  for (int i = 0; i < led_strip.numPixels(); i++) {
    led_strip.setPixelColor(i, led_strip.Color(0, 0, 0));
  }
  led_strip.show();
}

void blink_rgb_num(int rgb_val[3], int rgb_val2[3], int rgbNum=99) {
  if (pattern_cnt) {
    if (pattern_step) {
      if (rgbNum != 99) {
        led_strip.setPixelColor(rgbNum, led_strip.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
      } else {
        for (int i = 0; i < led_strip.numPixels(); i++) {
          led_strip.setPixelColor(i, led_strip.Color(rgb_val[0], rgb_val[1], rgb_val[2]));
        }
      }
      pattern_step = 0;
    } else {
      if (rgbNum != 99) {
        led_strip.setPixelColor(rgbNum, led_strip.Color(rgb_val2[0], rgb_val2[1], rgb_val2[2]));
      } else {
        for (int i = 0; i < led_strip.numPixels(); i++) {
          led_strip.setPixelColor(i, led_strip.Color(rgb_val2[0], rgb_val2[1], rgb_val2[2]));
        }
      }
      pattern_step = 1;
    }
    led_strip.show();
    pattern_cnt--;
  } else {
    wipe_strip();
  }

  lastRGBUpdate = millis();
}
