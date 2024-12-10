/*
Environment_Calculations.ino

This code shows how to record data from the BME280 environmental sensor
and perform various calculations.

GNU General Public License

Written: Dec 30 2015.
Last Updated: Oct 07 2017.

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>

int in1 = 2;
int in2 = 4;
int enA = 3;

int in3 = 5;
int in4 = 7;
int enB = 6;



const int IR_RECEIVE_PIN = 8;




#include <IRremote.hpp>
#include <Arduino.h>


#define SERIAL_BAUD 9600

// Assumed environmental values:
float referencePressure = 1018.6;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 4.7;           // °C  measured local outdoor temp.
float barometerAltitude = 1650.3;  // meters ... map readings + barometer position


BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);

//////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(SERIAL_BAUD);

  while(!Serial) {} // Wait

  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  Serial.print("Assumed outdoor temperature: "); Serial.print(outdoorTemp);
  Serial.print("°C\nAssumed reduced sea level Pressure: "); Serial.print(referencePressure);
  Serial.print("hPa\nAssumed barometer altitude: "); Serial.print(barometerAltitude);
  Serial.println("m\n***************************************");


IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); //start the receiver

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

//////////////////////////////////////////////////////////////////
void loop()
{

    // put your main code here, to run repeatedly:
   if (IrReceiver.decode())
   {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.printIRResultShort(&Serial); //optional use new print version
    switch(IrReceiver.decodedIRData.decodedRawData) {
      case 0xBF40FF00: //Keypad button 5
      Serial.println("5");
      //drive
  digitalWrite(in1, HIGH); //move forward for half a second
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);

  digitalWrite(in1, LOW); //stop for half a second
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);
break;
      case 0xE916FF00: //Keypad button *
      Serial.println("*");
      
      printBME280Data(&Serial);
      break;
      case 0xB946FF00:
            Serial.println("2");
      //drive
  digitalWrite(in1, LOW); //move forward for half a second
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);

  digitalWrite(in1, LOW); //stop for half a second
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);
  break;

  case 0xBC43FF00:
  digitalWrite(in1, HIGH); //move forward for half a second
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);

  digitalWrite(in1, LOW); //stop for half a second
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enA, 100);
  digitalWrite(enB, 100);
  delay(1500);
  break;
    }
    IrReceiver.resume(); //Enable more receiving of next

   }
}

//////////////////////////////////////////////////////////////////
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? "C" :"F"));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->print(String(presUnit == BME280::PresUnit_hPa ? "hPa" : "Pa")); // expected hPa and Pa only

   EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
   EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

   /// To get correct local altitude/height (QNE) the reference Pressure
   ///    should be taken from meteorologic messages (QNH or QFF)
   float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);

   float dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);

   /// To get correct seaLevel pressure (QNH, QFF)
   ///    the altitude value should be independent on measured pressure.
   /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
   float seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temp, pres, envAltUnit, envTempUnit);

   float absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);

   client->print("\t\tAltitude: ");
   client->print(altitude);
   client->print((envAltUnit == EnvironmentCalculations::AltitudeUnit_Meters ? "m" : "ft"));
   client->print("\t\tDew point: ");
   client->print(dewPoint);
   client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));
   client->print("\t\tEquivalent Sea Level Pressure: ");
   client->print(seaLevel);
   client->print(String( presUnit == BME280::PresUnit_hPa ? "hPa" :"Pa")); // expected hPa and Pa only

   client->print("\t\tHeat Index: ");
   float heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
   client->print(heatIndex);
   client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));

   client->print("\t\tAbsolute Humidity: ");
   client->println(absHum);

   delay(1000);
}
