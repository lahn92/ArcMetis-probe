#include <Arduino.h>
#include <SensirionI2cScd30.h>
#include <Wire.h>

SensirionI2cScd30 SCD30;

static char errorMessage[128]; //An array to hold error messages
static int16_t SCD30Error; //A variable to hold error codes
byte SCD30_addr = 0x61;
void setup() {


    Serial.begin(9600);
    while (!Serial) { //Forsinkelse grundet hardware så der er kontakt til serial port
        delay(100);
    }
    Wire.begin();
    SCD30.begin(Wire, SCD30_addr);

    SCD30.stopPeriodicMeasurement();
    SCD30.softReset(); //Sensor reset
    SCD30.setMeasurementInterval(7);
    delay(2000); //Venter på at sensoren er klar
    SCD30Error = SCD30.startPeriodicMeasurement('ambientPressure'); //Start periodiske målinger

    if (SCD30Error != NO_ERROR) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(SCD30Error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
}

void loop() {

//Initialiserer variabler til at holde værdierne for CO2 koncentration, temperatur og fugtighed
    float co2Concentration = 0.0;
    float temperature = 0.0;
    float humidity = 0.0;
    SCD30Error = SCD30.blockingReadMeasurementData(co2Concentration, temperature, humidity); //Læser måleværdier
    if (SCD30Error != NO_ERROR) { //Tjekke for fejl
        Serial.print("Error trying to execute blockingReadMeasurementData(): ");
        errorToString(SCD30Error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    //Printer i Serial Monitor
    Serial.print("CO2Concentration: ");
    Serial.print(co2Concentration);
    Serial.print(" ppm ");
    Serial.print("\t");
    Serial.print("temperature: ");
    Serial.print(temperature);
    Serial.print(" Deg C ");
    Serial.print("\t");
    Serial.print("humidity: ");
    Serial.print(humidity);
    Serial.print(" %RS ");
    Serial.println();
    }
