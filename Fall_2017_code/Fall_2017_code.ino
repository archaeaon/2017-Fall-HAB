#include <Adafruit_Sensor.h>
#include <Wire.h>

const int ledPinRed = 5, ledPinGn = 6, ledPinBlue = 7;
const long interval = 1000;   // interval at which to call the function (milliseconds)

unsigned long previousMillis = 0;
bool ledState = LOW;


#include <SD.h>
//#include <SPI.h>
File dataFile;
const int ChipSelectPin = 10;
/* SD connection information */
/* 5V pin to the 5V    *
 * GND pin to the GND  *
 * CLK to pin 13 or 52 *
 * DO to pin 12 or 50  *
 * DI to pin 11 or 51  *
 * CS to pin 10 or 53  */


#include <Adafruit_MMA8451.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
/* MMA connection information */
/* Vin pin to the 5V    *
 * GND pin to the GND   *
 * SCL to pin A5 or D21 *
 * SDA to pin A4 or D20 */


#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float SeaLevelPressure;
/* BMP connection information */
/* Vin pin to the 3V     *
 *  GND pin to the GND   *
 *  SCL to pin A5 or D21 *
 *  SDA to pin A4 or D20 */


#define GPS_RXPIN 2
#define GPS_TXPIN 3
#define GPS_BAUD 9600 // GPS module baud rate. GY-NE06MV2 is 9600

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus tinyGPS;
SoftwareSerial GPSModule(GPS_RXPIN, GPS_TXPIN);
/* GPS connection information */
/* VCC pin to the 5V   *
 *  GND pin to the GND *
 *  TX to pin D2       *
 *  RX to pin D3       */


void setup() {

        pinMode(ledPinRed, OUTPUT);  //BMP, pin 5
        pinMode(ledPinBlue, OUTPUT); //MMA, pin 6
        pinMode(ledPinGn, OUTPUT);   //SD, pin 7


        digitalWrite(ledPinBlue, HIGH);
        digitalWrite(ledPinGn, HIGH);
        digitalWrite(ledPinRed, HIGH);
        delay(2500);
        digitalWrite(ledPinBlue, LOW);
        digitalWrite(ledPinGn, LOW);
        digitalWrite(ledPinRed, LOW);

        Serial.begin(9600);
        GPSModule.begin(GPS_BAUD);

        Wire.begin();
        bmpSetup();
        mmaSetup();
        sd_Setup();

        digitalWrite(ledPinRed, HIGH);
        digitalWrite(ledPinBlue, HIGH);
        digitalWrite(ledPinGn, HIGH);
        delay(2500);
        digitalWrite(ledPinBlue, LOW);
        digitalWrite(ledPinGn, LOW);
        digitalWrite(ledPinRed, LOW);

}

void loop() {

        unsigned long currentMillis = millis();
        int Time = currentMillis * .001;


        if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                Serial.print(F("Time = ")); Serial.print(Time);
                Serial.println(F(""));

                dataFile = SD.open(F("data.txt"),FILE_WRITE);
                dataFile.print(Time);
                dataFile.print(F(","));
                dataFile.close();

                bmpData();
                mmaData();
                gpsData();

                if (ledState == LOW) {
                        ledState = HIGH;
                } else {
                        ledState = LOW;
                }
                // set the LED with the ledState of the variable:

                digitalWrite(ledPinBlue, ledState);
                digitalWrite(ledPinGn, ledState);
                digitalWrite(ledPinRed, ledState);
        }

}

void bmpSetup() {

        if (!bmp.begin()) {
                Serial.println(F("Could not find a valid BMP085 sensor."));

                while(1)
                {
                  if (ledState == LOW) {
                                ledState = HIGH;
                        } else {
                                ledState = LOW;
                        }
                        // set the LED with the ledState of the variable:
                        digitalWrite(ledPinGn, ledState);
                        delay(250);
                }
        }

        sensors_event_t event;
        bmp.getEvent(&event);
        SeaLevelPressure = event.pressure;
        Serial.print(event.pressure);

}

void mmaSetup() {

        if (!mma.begin()) {
                Serial.println(F("Couldnt start MMA sensor"));

                while(1)
                {
                  if (ledState == LOW) {
                                ledState = HIGH;
                        } else {
                                ledState = LOW;
                        }
                        // set the LED with the ledState of the variable:
                        digitalWrite(ledPinBlue, ledState);
                        delay(250);
                }
        }

        mma.setRange(MMA8451_RANGE_8_G);

}

void sd_Setup() {

        if (!SD.begin(ChipSelectPin)) {
                Serial.println(F("SD initialization failed!"));

                while(1)
                {
                  if (ledState == LOW) {
                                ledState = HIGH;
                        } else {
                                ledState = LOW;
                        }
                        // set the LED with the ledState of the variable:
                        digitalWrite(ledPinRed, ledState);
                        delay(250);
                }
        }

        dataFile = SD.open(F("data.txt"),FILE_WRITE);
        dataFile.print(F("Time(sec), Temperature(C), Pressure(Pa), Altitude(m), "));
        dataFile.print(F("x, y, z(m/s^2), Orientation, "));
        dataFile.print(F("Latitude, Longitude, gpsAltitude, Course, Speed, Satellites, SeaLevelPressure = "));
        dataFile.print(SeaLevelPressure);
        dataFile.println(F(""));
        dataFile.close();

}

void bmpData() {

        sensors_event_t bmp_event;
        bmp.getEvent(&bmp_event);

        float Altitude, Pressure, Temperature;

        if (bmp_event.pressure)
        {
                Pressure = bmp_event.pressure;

                Serial.print(F("Pressure:    "));
                Serial.print(Pressure);
                Serial.println(F(" hPa"));

                /* Calculating altitude with reasonable accuracy requires pressure    *
                 * sea level pressure for your position at the moment the data is     *
                 * converted, as well as the ambient temperature in degress           *
                 * celcius. If you don't have these values, a 'generic' value of      *
                 * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
                 * in sensors.h), but this isn't ideal and will give variable         *
                 * results from one day to the next.                                  */

                /* First we get the current temperature from the BMP085 */
                bmp.getTemperature(&Temperature);
                Serial.print(F("Temperature: "));
                Serial.print(Temperature);
                Serial.println(F(" C"));

                /* Then convert the atmospheric pressure, SLP and temp to altitude    */
                //float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; //uncomment to use generic sea level value
                Altitude = bmp.pressureToAltitude(SeaLevelPressure, bmp_event.pressure, Temperature);
                Serial.print(F("Altitude:    "));
                Serial.print(Altitude);
                Serial.println(F(" m"));
                Serial.println(F(""));

                /* Print the aquired data to the bmp file */
                dataFile = SD.open(F("data.txt"),FILE_WRITE);
                dataFile.print(Temperature);
                dataFile.print(F(","));
                dataFile.print(Pressure);
                dataFile.print(F(","));
                dataFile.print(Altitude);
                dataFile.print(F(","));
                dataFile.close();
        }

}

void mmaData() {

        sensors_event_t event;
        mma.getEvent(&event);

        float x = 0, y = 0, z = 0;

        x = event.acceleration.x;
        y = event.acceleration.y;
        z = event.acceleration.z;

        Serial.print(F("X: \t")); Serial.print(x); Serial.print(F("\t"));
        Serial.print(F("Y: \t")); Serial.print(y); Serial.print(F("\t"));
        Serial.print(F("Z: \t")); Serial.print(z); Serial.print(F("\t"));
        Serial.println(F("m/s^2 ")); Serial.println(F(""));

        /* Get the orientation of the sensor */
        uint8_t Orientation = mma.getOrientation();
        int Orient = 0;

        switch (Orientation) {
        case MMA8451_PL_PUF:
                Serial.println(F("Portrait Up Front"));
                Orient = 1;
                break;
        case MMA8451_PL_PUB:
                Serial.println(F("Portrait Up Back"));
                Orient = 2;
                break;
        case MMA8451_PL_PDF:
                Serial.println(F("Portrait Down Front"));
                Orient = 3;
                break;
        case MMA8451_PL_PDB:
                Serial.println(F("Portrait Down Back"));
                Orient = 4;
                break;
        case MMA8451_PL_LRF:
                Serial.println(F("Landscape Right Front"));
                Orient = 5;
                break;
        case MMA8451_PL_LRB:
                Serial.println(F("Landscape Right Back"));
                Orient = 6;
                break;
        case MMA8451_PL_LLF:
                Serial.println(F("Landscape Left Front"));
                Orient = 7;
                break;
        case MMA8451_PL_LLB:
                Serial.println(F("Landscape Left Back"));
                Orient = 8;
                break;
        }

        /* Print the aquired data to the mma file */
        dataFile = SD.open(F("data.txt"),FILE_WRITE);
        dataFile.print(x);
        dataFile.print(F(","));
        dataFile.print(y);
        dataFile.print(F(","));
        dataFile.print(z);
        dataFile.print(F(","));
        dataFile.print(Orient);
        dataFile.print(F(","));
        dataFile.close();

}

void gpsData() {

        float Latitude = tinyGPS.location.lat(), Longitude = tinyGPS.location.lng();
        double gpsAltitude = tinyGPS.altitude.feet(), Course = tinyGPS.course.deg(), Speed = tinyGPS.speed.mph();
        int Satellites = tinyGPS.satellites.value();

        Serial.print(F("Lat: ")); Serial.println(Latitude, 6);
        Serial.print(F("Long: ")); Serial.println(Longitude, 6);
        Serial.print(F("Alt: ")); Serial.println(gpsAltitude);
        Serial.print(F("Course: ")); Serial.println(Course);
        Serial.print(F("Speed: ")); Serial.println(Speed);
        Serial.print(F("Satillites: ")); Serial.println(Satellites);
        Serial.println();

        dataFile = SD.open(F("data.txt"),FILE_WRITE);
        dataFile.print(Latitude, 6);
        dataFile.print(F(","));
        dataFile.print(Longitude, 6);
        dataFile.print(F(","));
        dataFile.print(gpsAltitude);
        dataFile.print(F(","));
        dataFile.print(Course);
        dataFile.print(F(","));
        dataFile.print(Speed);
        dataFile.print(F(","));
        dataFile.print(Satellites);
        dataFile.println(F(""));
        dataFile.close();

}
