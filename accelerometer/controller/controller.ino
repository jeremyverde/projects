/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"
#include "SdFat.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
// Definitions for data logging functions
// Battery pin
#define VBATPIN A7

// I2C clock speed
#define I2C_SPEED 1000000
// Set range to 2G, 4G, 8G, 16G
#define ACCEL_RANGE LIS3DH_RANGE_8_G
// 5khz data rate
#define ACCEL_DATARATE LIS3DH_DATARATE_LOWPOWER_5KHZ
// Decimal precision for acceleration
#define ACCEL_PRECISION 3
// Separator character
#define SEP ","

// Enable debug logger.
// Note: Comment out before running in real-world.
// #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

RTC_PCF8523 rtc;

// change this to match your SD shield or module;
// Adafruit SD shields and modules: pin 10
const int chipSelect = 10;

// Filename format:  YYYYMMDDHHmm.csv
char filename[17];
unsigned long begin_us;
unsigned long begin_epoch;
unsigned long start_us;
unsigned long stop_us;
unsigned long counter = 0;

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// set up variables using the SD utility library functions:
// File system object.
SdFat sd;
// Log file.
SdFile dataFile;


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  DEBUG_PRINTLN(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup_ble(void)
{
  #ifdef DEBUG
  whie(!Serial);
  delay(500);
  #endif
  
  Serial.begin(115200);
  DEBUG_PRINTLN(F("Adafruit Bluefruit App Controller Example"));
  DEBUG_PRINTLN(F("-----------------------------------------"));

  /* Initialise the module */
  DEBUG_PRINTLN(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  DEBUG_PRINTLN( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    DEBUG_PRINTLN(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  DEBUG_PRINTLN("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  DEBUG_PRINTLN(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  DEBUG_PRINTLN();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  DEBUG_PRINTLN(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    DEBUG_PRINTLN(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  DEBUG_PRINTLN( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  DEBUG_PRINTLN(F("******************************"));
}

/**
 * Data logging setup procedures.
 */
void setup_data() {
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif

  #ifdef DEBUG
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    DEBUG_PRINT("VBat: " ); DEBUG_PRINTLN(measuredvbat);
  #endif

  /**
   * Initialize the Real Time Clock.
   */
  DEBUG_PRINTLN("Initializing RTC...");
  if (! rtc.begin()) {
    DEBUG_PRINTLN("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()) {
    DEBUG_PRINTLN("RTC is NOT running!");
  }
  DEBUG_PRINTLN("RTC initialized.");
  begin_us = micros();
  DateTime now = rtc.now();
  begin_epoch = now.unixtime();

  // Set filename based on timestamp.
  sprintf_P(filename, PSTR("%4d%02d%02d%02d%02d.csv"), now.year(), now.month(), now.day(), now.hour(), now.minute());
  DEBUG_PRINT("Filename: ");
  DEBUG_PRINTLN(filename);

  /**
   * Initialize the accelerometer.
   */
  DEBUG_PRINTLN("Initializing LIS3DH Sensor...");
  if (! lis.begin(LIS3DH_DEFAULT_ADDRESS)) {   // change this to 0x19 for alternative i2c address
    DEBUG_PRINTLN("Couldnt start");
    while (1);
  }
  // Set I2C high speedmode
  Wire.setClock(I2C_SPEED);
  lis.setRange(ACCEL_RANGE);
  lis.setDataRate(ACCEL_DATARATE);
  DEBUG_PRINTLN("LIS3DH initialized.");


  /**
   * Initialize the SD card and log file.
   */
  DEBUG_PRINTLN("Initializing SD card...");
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    DEBUG_PRINTLN("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  DEBUG_PRINTLN("Card initialized.");

  if (! dataFile.open(filename, O_CREAT | O_APPEND | O_WRITE)) {
     DEBUG_PRINTLN("Could not open file...");
     while (1);
  }
  // Write header row to file.
  dataFile.println("timestamp (s), start (µs), accel x (G), accel y (G), accel z (G)");
  dataFile.flush();
  
  // Check to see if the file exists:
  if (! sd.exists(filename)) {
    DEBUG_PRINTLN("Log file doesn't exist.");
    while (1);
  }

  DEBUG_PRINTLN("Ready!");
  DEBUG_PRINTLN("timestamp (s), start (µs), delta (µs), accel x (G), accel y (G), accel z (G)");
}

void setup(){
  setup_ble();
  setup_data();
  }

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
    if (buttnum <= 4 & pressed){startLogging(buttnum);} // number of button is minutes to log for
  }

}

bool startLogging(uint32_t period_min){
  DEBUG_PRINT("started logging...");
  ble.print("Logging for "); ble.print(period_min); ble.println(" minutes");
  ble.println("Logging Start");
  uint32_t period = period_min * 60000L;
  uint32_t log_interval = 10000L; // how often to write to SD card (every 10 seconds.. see if that's enough)
  uint32_t t_last_logged = millis();
  for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
        read_accel();
        log_timers();
        log_accel();
        
        if (millis()-t_last_logged > log_interval) {
          write_sd();
          t_last_logged = millis();
        }
    }
  ble.println("Logging Stop");  
  return true;
  }
  
/**
 * Read from the accelerometer sensor and measure how long the op takes.
 */
void read_accel() {
  start_us = micros();
  lis.read();
  stop_us = micros();
}

/**
 * Output the timing info.
 */
void log_timers() {
  // Write timestamp to file.
  // Roughly equivalent to calling rtc.now().unixtime(), without 1ms latency.
  dataFile.print(begin_epoch + ((stop_us - begin_us) / 1000000));
  dataFile.print(SEP);
  // Write timers to file.
  dataFile.print(stop_us);
  dataFile.print(SEP);
}

/**
 * Output the acceleration info.
 */
void log_accel() {
  // Write acceleration to file.
  dataFile.print(lis.x_g, ACCEL_PRECISION);
  dataFile.print(SEP);
  dataFile.print(lis.y_g, ACCEL_PRECISION);
  dataFile.print(SEP);
  dataFile.print(lis.z_g, ACCEL_PRECISION);
  // Write newline.
  dataFile.println();
}

/**
 * Flush buffer, actually write to disk.
 * This can take between 9 and 26 milliseconds. Or ~10ms with SDfat.
 */
void write_sd() {
  DEBUG_PRINTLN("Writing to SD card");
  #ifdef DEBUG
  unsigned long pre_write = micros();
  unsigned long data_size = dataFile.fileSize();
  unsigned long mem_left = dataFile.available();
  # endif
  dataFile.flush();
  DEBUG_PRINT("Write ops took: ");
  DEBUG_PRINTLN(micros() - pre_write);
  DEBUG_PRINT("File size: ");
  DEBUG_PRINTLN(data_size);
}
