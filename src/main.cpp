
//Samling af biblioteker, der inkluderes i Arduino-sketchen for at give funktionalitet til forskellige de forskellige hardwarekomponenter og kommunikationsprotokoller:

#include <Adafruit_ADS1X15.h>  //Inkluderer bibliotek for A/D konverter fra Adafruit
#include <Servo.h>             //Inkluderer bibliotek for servomotor til vandprøve
#include <Wire.h>              //Inkluderer bibliotek for I2C kommunikation
#include <SD.h>                //Inkluderer bibliotek for SD-kort
#include <SPI.h>               //Inkluderer bibliotek for SPI kommunikation
#include <KellerLD.h>          //Inkluderer bibliotek for Bar100
#include <MS5837.h>            //Inkluderer bibliotek for Bar30
#include <TSYS01.h>            //Inkluderer bibliotek for Temperatur sensor fra Blue Robotics
#include <ArduinoBLE.h>        //Inkluderer bibliotek for arduinoen
#include <Adafruit_SCD30.h>    //Inkluderer bibliotek for SCD30
#include <EthernetENC.h>       //Inkluderer bibliotek for ethernet
#include <Adafruit_HTU21DF.h>  //Inkluderer bibliotek for HTU21
#include <string>              // Bibliotek for konventering fra float til string
#include <queue>               //Kø bibliotek
//#include <Ezo_i2c.h>
//#include <Ezo_i2c_util.h>
//Biblioteker for konventering fra Char til float og omvendt
#include <stdlib.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <PCA9540BD.h>  //Inkluderer bibliotek for PCA9540BD
#include <malloc.h>
#include "ADS1X15.h"


//EZO biblioteker
//#include <Ezo_i2c.h>
//#include <Ezo_i2c_util.h>
//#include <iot_cmd.h>
//#include <sequencer1.h>
//#include <sequencer2.h>
//#include <sequencer3.h>
//#include <sequencer4.h>


//                                             Defination af diverse startbetingelser samt variabler.

//Ethernet - arduino
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  //MAC-adresse for Ethernet-modul (selvvalgt)
IPAddress ip(192, 168, 1, 63);                        //IP-adresse for Ethernet-modul (Selvvalgt)
IPAddress subnet(255, 255, 255, 0);                   //Skal matche den tilsluttede PC
IPAddress gateway(192, 168, 1, 1);                    //Skal matche den tilsluttede PC
unsigned int localPort = 61556;                       // lokal port til at lytte på
char packetBuffer[255];                               //buffer til at holde indgående pakke
EthernetUDP Udp;                                      //Opretter en instans af EthernetUDP-klassen ved navn Udp
const int CS = 10;                                    //Pin til ethernet modul.
float standby_data[3];                                //Array til at hole på tid og ude trykket, til når sonden ikke skal fortage målinger
int standby_str;                                      //Bruges til at bestemme størrelsen af standby arrayet


//Ethernet - platform
IPAddress remoteIp(192, 168, 1, 64);  //Platformens IP-adresse. Den der benyttes den generelle IP-adresse
unsigned int remotePort = 61557;      //Platformens port
String dataToSend;                    //Opretter er variabel dataToSend som en string
int packetSize;                       //Variabel til at holde på antallet af indkommende data
float interval = 5000;                //Vaeriabel til at holde på ændring i måleinterval fra platform. Angives i ms.

//SD-kort
// File dataFile; FJERNES
char fileName[20];               //Buffer til at holde på fil-navn
const int chipSelect = 9;        //Chip select pin for SD kort.
const int buffer_size_SD = 100;  //Buffer størrelsen er 8 - kan ske at den skal ændres.
char buffer_SD[buffer_size_SD];
int fileNum;
String dataString = "";

// Laver en buffer til konventering fra float til char, som er nødvendig for at kunne skrive til SD-kortet.
const int buffer_size = 8;  //Buffer størrelsen er 8 - kan ske at den skal ændres.
char buffer[buffer_size] = "";

//SCD30
Adafruit_SCD30 SCD30;
float CO2;
float SCD30_temp;

//Bar30
MS5837 Bar30;
float P_inde = 1000;
float P_inde_tjek;
float TidligereP_inde = 0.0;
float P_tolerance = 25;

//TSYS01 - Blue Robotics temperatur sensor
TSYS01 TSYS;
float T_ude;

//Bar100
KellerLD Bar100;
float P_ude;
float P_ude_tjek;
//Trykdifferens til styring af aktuator
float Pdiff = 35;     //Tilladelige tryk differens i mbar.
float adjP = 0;        //Variabel til justering af tryksensorer.
                       //AD konverter
//Adafruit_ADS1115 ads;  // Initialiserer ADS1115 objektet
float CH4;             //Variabel til at holde på den beregne CH4 spænding
int16_t adc3;          // CH4 sensoren er forbundet til ADC analog port 3
ADS1115 ADS(0x48);

//HTU-21 fugtsensor
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float RH = 55;        //Variabel til at holde på fugtigheds målinger fra HTU-21 sensoren
float htu_temp = 55;  //Variabel til at holde på temperatur målinger fra HTU-21 sensoren

// //EZO™ sensorer

float ezo_data;    //Variabel til at holde på data fra ezo sensorer
byte in_char = 0;  //Bruges som en 1 byte buffer til at gemme indgående bytes fra o2-sensoren.
byte i = 0;        //Tæller for data-arrayet.
float O2 = 500;    //Variabel til at holde på O2 værdier
char* EZO_data_char;
String tempString;  // String til at holde på string temperaturer til temperations kompensation for EZO sensorer
float EC;           //float var used to hold the float value of the conductivity.
int time_;
char data[32];
int dataStr;
char ec_data[32];
//Variablen sørger for at kompensationen kun fungerer en gang pr gennemkørsel af main-loop.

//Stopur
float startTid;       //Tid hvornår arduinoens loop begynder.
float nuTid;          //Tid hvornår arduien logger tiden igen
int tidGaaet = 0;     //Variabel til at beregning af hvor lang tid der er gået totalt
float tidSidste = 0;  //Variabel til at beregne hvor lang tid der er gået siden sidste gennemkørsel.

//Batteriovervågning
float Perc_bat = 100;            //Variabel til batteriprocent
const int analogInPin = A7;      // Angiver den analoge indgang som der skal læses på
const float maxVoltage = 3.13;   // Angiver den maksimale spænding arduinoen kan læse fra batterierne
const float minVoltage = 1.9;    // Angiver den laveste spænding som arduinoen kan læse fra batterierne
int analog_maaling;              //Variabel til at holde på målingen fra analogpin 0.
float voltage;                   //Variabel til at holde på den konventerede spænding
float percentage;                //Variabel til at holde på den beregnede batteriprocent
std::queue<float> last15Values;  // Kø til at gemme de sidste 10 værdier

//Boolean til main-loop
bool Main = true;


//Diverse I2C Adresser som ikke er defineret i deres biblioteker
byte O2_addr = 0x6C;  //Standard ECO O2 sensor I2C address.

const byte ECrxPin = 2;
const byte ECtxPin = 4;
// Set up a new SoftwareSerial object for the EC sensor
UART ECSerial (ECrxPin, ECtxPin, NC, NC);


//Array til at holde på standby-værdierne
float standby[3];
char standby_char[3];

//Lækage sensorer
int leakPin = 8;  // Leak Signal Pin
int leak = 0;     // 0 = Dry , 1 = Leak
unsigned long TidNuLeak = 0;
unsigned long TidSidsteLeak = 0;
const unsigned long IntervalLeak = 10000;
bool reset = false;  //Startværdi for lækage interupt loo

// Mipex sensor

unsigned long tidLyt = 500;  //Tid som der lyttes efter svar fra mipexsensoren
float MIPEX = 0;             //Variabel til at gemme værdier fra MIPEX sensoren.
char inByte;
int arrayIndex = 0;
char dataArray[5] = { 0, 0, 0, 0, 0 };



//Definering af PWM signal til servomotor
#define MinPulseWidth 500       // Korteste puls
#define MaxPulseWidth 2500      // Længste puls
#define DEFAULT_PULSE_WIDTH 1500  // Standard pulsbredde når servomotoren er tilsluttet
#define REFRESH_INTERVAL 20000    // Minmimums opdateringstid i ms
Servo servo;
const int servoPin = 3;  //Definerer hvilken pin servomotorens PWN signal genereres på
bool vandprove = false;

unsigned long SidsteTid = 0;
float previousMillis;
const long interval1 = 2000;  //Åbnetid
const long interval2 = 5000;  //Tid åben
const long interval3 = 2000;  //Lukketid
const long interval4 = 100;   //Fjern evt holdemoment
unsigned long previousMillisSetup = 0;
const long interval1Setup = 1000;
const long interval2Setup = 100;

//Aktuator
float P_diff = 0;
const int motorPWMOP = 5;                  // PWM pin for M1B
const int motorPWMNED = 6;                 // PWM pin for M2B
unsigned long TidNuAktu;                   //Nuværende tid til aktuator stop
const unsigned long AktuStartTid = 10000;  //Tid som aktuatoren kører for at opnå være i top.
bool AktuSetup = true;
unsigned long NulstilStart;  //Bruges til at nulstille aktuatorens position
bool motorActiv = false; // bool to track if the motor is active 
bool movingBool = true; // bool to track if motor is currently doing other jobs (resetting etc.) 
bool movingDir = false; //bool to keep track of the current direction of the motor false == vacum 
int movingTimeAccumulating = 0; //variable to keep track of the time the motor has moved in 1 direction.  
const int timeToMoveFull = 7500; // the time it takes for the actuator to move full stroke
const int settlingTime = 1.8e+6;      // Settling time before switching direction (in milliseconds)
unsigned long movementTimeAccumulated = 0; // Total movement time in current direction
unsigned long lastUpdateTime = 0; // Last time movement was updated
unsigned long directionSwitchTime = 0; // Time when direction was switched
bool isMoving = false; // Flag to check if the motor is currently running
bool isSettling = false; // Flag to track if we are in the settling phase
long aktuTidSidste = 0;
const long aktuInterval = 100;

//Array til at holde på alle værdier i main-loopet
float mainData[15] = { tidGaaet, P_ude, P_inde, RH, SCD30_temp, htu_temp, T_ude, CO2, O2, CH4, MIPEX, EC, Perc_bat,movingDir};
int mainStr;  //Variabel til bestemmelse af antal karakter i main-arrayet

// Multiplexer
PCA9540BD multiplexer;
int currentChannel = 1;  // Start med kanal 0

////// Function prototypes //////
void Data_fra_platform();
void Stopur();
void float_til_char(float* data, int size);
float EZO_sensorer(byte addr);
float EC_float();
void batteriniveau();
void PdiffTjek();
void Send_til_platform(char* char_data);
void Send_til_platform2(float data);
void NulstilAktu();
void SkrivTilSD(float data);
void delay500ms();
int freeMemory();
int getFreeMemory();
void printMemoryStats();

size_t freeHeap;
void printMemoryStats() {
    struct mallinfo mi = mallinfo();
    Serial.print("Total Allocated Heap: ");
    Serial.println(mi.uordblks); // Used heap
    Serial.print("Total Free Heap: ");
    Serial.println(mi.fordblks); // Free heap
    freeHeap = mi.fordblks;
}

void Data_fra_platform()  //Læse-funktion
{
  packetSize = Udp.parsePacket();

  if (packetSize) {                        // Hvis der er en pakke tilgængelig
    int len = Udp.read(packetBuffer, 24);  // Læser dataene fra den modtagne UDP-pakke ind i packetBuffer
    if (len > 0) {                         // Hvis der blev læst nogen data fra UDP-pakken
      packetBuffer[len] = 0;               // Tilføjer en null-terminator til slutningen af dataene i packetBuffer
    }
    String input = String(packetBuffer);  // Konverterer packetBuffer til en streng
    Serial.print("UDP packet is: ");
    Serial.println(input);
    if (input == "ON") {                  // Tjekker, om den indgående streng er "ON", hvis den er starter main-loopet
      Main = true;
    }

    if (input == "Stop") {
      Main = false;
    }

    if (input.startsWith("Maaleinterval")) {
      String tal_fra_platform = input.substring(String("Maaleinterval").length() + 1);  // +1 for at springe over mellemrummet
      interval = tal_fra_platform.toFloat();
    }
    if (input == "Vandprove") {
      Serial.println("conducting water sample");
      servo.write(180);                            // move the servo to 180 degrees
      while (millis() - SidsteTid < interval1) {}  // Venter på at servomotren er åben

      previousMillis = millis();
      servo.write(176);                            // Drejer 4 grader for at undgå unødvendig holdemoment og derved strømforbrug imens vandprøven tages
      while (millis() - SidsteTid < interval2) {}  // Lader ventilen være åben i 4 sek.

      previousMillis = millis();
      servo.write(0);                              // lukker ventilen igen
      while (millis() - SidsteTid < interval3) {}  // Delay som sikre at ventilen er lukket

      previousMillis = millis();
      servo.write(6);                              // Drejer motoren 6 grader for at undgå unødvendige holdemoment og derved strømforbrug
      while (millis() - SidsteTid < interval4) {}  // Delay som sikre at ventilen er lukket
    }
    if (input == "Reset") {
      reset = true;
    }
    if (input.startsWith("Pdiff")) {
      String tal_fra_platform = input.substring(String("Pdiff").length() + 1);  // +1 for at springe over mellemrummet
      Pdiff = tal_fra_platform.toFloat();
    }
    if (input == "Nulstil") {
      NulstilAktu();
    }
    if (input == "Moving") {
      NulstilAktu();
      movingBool = true;
    }

    if (input == "steady") {
      movingBool = false;
    }

    if (input == "ned") {                    // Aktuatoren skal bevæge sig ned
      unsigned long startTidNED = millis();  // Gemmer starttidspunktet

      // Start aktuatoren med 50% PWM
      analogWrite(motorPWMOP, 0);     // 0% PWM
      analogWrite(motorPWMNED, 127);  // 50% PWM
      while (millis() - startTidNED < 1500) {
        // Aktuatoren bevæger sig med 50% PWM.
      }

      // Skift til 75% PWM efter 3 sekunder
      analogWrite(motorPWMNED, 191);  // 75% PWM
      while (millis() - startTidNED < 2500) {
        // Aktuatoren bevæger sig med 75% PWM.
      }

      // Skift til 100% PWM efter 6 sekunder
      analogWrite(motorPWMNED, 255);  // 100% PWM
      while (millis() - startTidNED < 10000) {
        // Aktuatoren bevæger sig med 100% PWM.
      }

      analogWrite(motorPWMNED, 0);  // Stop aktuatoren efter 10 sekunder
    }

    if (input == "op") {                    // Aktuatoren skal bevæge sig op
      unsigned long startTidOP = millis();  // Gemmer starttidspunktet
      analogWrite(motorPWMNED, 0);          // 0% PWM
      analogWrite(motorPWMOP, 127);         // 50% PWM
      while (millis() - startTidOP < 1500) {
        // Aktuatoren bevæger sig med 50% PWM.
      }

      // Skift til 75% PWM efter 3 sekunder
      analogWrite(motorPWMOP, 191);  // 75% PWM
      while (millis() - startTidOP < 2500) {
        // Aktuatoren bevæger sig med 75% PWM.
      }

      // Skift til 100% PWM efter 6 sekunder
      analogWrite(motorPWMOP, 255);  // 100% PWM
      while (millis() - startTidOP < 10000) {
        // Aktuatoren bevæger sig med 100% PWM.
      }

      analogWrite(motorPWMOP, 0);  // Stop aktuatoren efter 10 sekunder
    }

    if (input == "adjP") {
      adjP = P_ude - P_inde;
    }
  }
}


void Stopur() {
  // Beregn, hvor lang tid der er gået siden starten af stopuret
  nuTid = millis();                        //Tid [ms] hvor funktionen kaldes
  tidGaaet = ((nuTid - startTid) / 1000);  //Tid [s] gået siden funktionen sidst blev kaldt.
}

void float_til_char(float* data, int size) {  //Funktion til at konvetere float værdier til string og tilføje det til en samlet string array (buffer)
  char temp[100];                             //Buffer til string
  memset(buffer, 0, sizeof(buffer));          //Nulstiller buffer
  memset(buffer, 0, sizeof(temp));            //Nulstiller temp
  for (int i = 0; i < size; i++) {
    dtostrf(data[i], 6, 2, temp);
    strcat(buffer, temp);  // Tilføjer den nye string til buffer
    if (i < size - 1) {    // Tilføj komma-separeret værdi mellem hver værdi undtagen den sidste
      strcat(buffer, ",");
    }
  }
}


float EZO_sensorer(byte addr) {
  Wire.beginTransmission(addr);  // Call the circuit by its address.
  if (addr == O2_addr) {
    dataStr = 20;
    time_ = 600;
    char O2_data[20];
    tempString = String(htu_temp, 0);
    Wire.write(("RT," + tempString).c_str());  // Send "RT,internal temperature" command to sensor for a single reading.
    
    // Check if transmission was successful
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
      Serial.print("I2C transmission error: ");
      Serial.println(error);
      return -1.0; // Return an error value
    }
  }
  delay(time_);  // Wait for the circuit to complete its instruction.

  Wire.requestFrom(addr, dataStr, 1);  // Request data size bytes from the circuit at the relevant address
  if (Wire.available() < 1) {
    Serial.println("No data available from sensor");
    return -1.0;  // Return an error value if no data is available
  }

  // Read data
  i = 0;
  while (Wire.available()) {
    in_char = Wire.read();
    if (i < sizeof(data) - 1) {
      data[i++] = in_char;
    }
    if (in_char == '\0' || i >= sizeof(data) - 1) break;  // Null-terminate or prevent buffer overflow
  }
  data[i] = '\0';

  ezo_data = atof(data);  // Convert the data to float format
  return ezo_data;
}

float EC_float() {
  ECSerial.println("R");  // Send the "read" command to the sensor

  const unsigned long timeout = 1000;  // 1-second timeout
  unsigned long startTime = millis();

  // Reset the buffer and counter
  memset(ec_data, 0, sizeof(ec_data));
  i = 0;

  while (millis() - startTime < timeout) {  // Timeout for reading
    if (ECSerial.available()) {
      char in_char = ECSerial.read();  // Read a byte
      if (i < sizeof(ec_data) - 1) {   // Ensure we don't overflow the buffer
        ec_data[i++] = in_char;        // Store the byte and increment the counter
      }
      if (in_char == '\n' || in_char == '\r') { // End of line detected
        ec_data[i] = '\0';  // Null-terminate the string
        break;
      }
    }
  }

  if (i == 0) {  // No data received within timeout
    Serial.println("EC sensor did not respond within timeout");
    return -1.0;  // Return an error value
  }

  // Process the data from ec_data
  int commaIndex = -1;
  for (int j = 0; j < i; j++) {
    if (ec_data[j] == ',') {
      commaIndex = j;
      break;
    }
  }

  if (commaIndex != -1) {
    ec_data[commaIndex] = '.';  // Replace comma with a period
  }

  // Convert to float
  float ec_fdata = atof(ec_data);  // Convert the char[] to float
  if (ec_fdata == 0.0 && ec_data[0] != '0') {
    // Handle invalid or unexpected response
    Serial.println("Error: Invalid EC data");
    return -1.0;  // Return a sentinel value indicating an error
  }

  return ec_fdata;  // Return the converted float
}




void batteriniveau() {
  // Beregn spændingsniveauet i procent
  analog_maaling = analogRead(analogInPin);
  voltage = analog_maaling * (3.3 / 1023.0);                                // Konverter analogt input til spænding
  percentage = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;  //Beregner batteriniveau i %

  // Tilføj den nye procentdel til køen
  last15Values.push(percentage);

  // Hvis køen har mere end 15 elementer, fjern det ældste
  if (last15Values.size() > 15) {
    last15Values.pop();
  }

  // Beregn gennemsnittet af de sidste 15 værdier
  float sum = 0;
  std::queue<float> temp = last15Values;
  while (!temp.empty()) {
    sum += temp.front();
    temp.pop();
  }
  float average = sum / last15Values.size();  //Finder gennemsnittet af værdierne i bufferen

  Perc_bat = constrain(average, 0, 100);  // Sørger for, at procentdelen er inden for intervallet 0-100%. Ligger den udenfor intervallet vil den returnere et nærmeste ekstrema.
}

void PdiffTjek() {
    //Serial.println("running pdifftjek");

    if (movingBool == true || motorActiv == false || Perc_bat< 15) {
        //Serial.println("returning due to movingBool == true");
        return;
    }

    TidligereP_inde = P_inde;
    multiplexer.selectChannel(1);
    Bar100.read();
    P_ude = Bar100.pressure();
    multiplexer.selectChannel(0);
    Bar30.read();
    P_inde = adjP + Bar30.pressure();
    unsigned long currentTime = millis(); // Get the current time

    // Accumulate movement time only when the motor is running
    if (isMoving) {
        movementTimeAccumulated += (currentTime - lastUpdateTime);
    }

    lastUpdateTime = currentTime; // Update time tracking

    // Handle Settling Time
    if (isSettling) {
        if (currentTime - directionSwitchTime >= settlingTime) {
            Serial.println("Settling time over, resuming movement.");
            isSettling = false; // Settling period is over
        } else {
            Serial.println("Waiting for settling time...");
            return; // Exit function to keep the motor stopped
        }
    }

    // Check if total movement time exceeds the limit
    if (movementTimeAccumulated >= timeToMoveFull) {
        Serial.println("Switching direction due to accumulated time limit");

        movingDir = !movingDir;  // Reverse direction
        movementTimeAccumulated = 0; // Reset accumulated time

        // Start settling period
        isSettling = true;
        directionSwitchTime = currentTime;

        // Stop the actuator during settling time
        analogWrite(motorPWMOP, 0);
        analogWrite(motorPWMNED, 0);
        isMoving = false;

        return; // Exit function to enforce settling period
    }

    // Control actuator movement
    if (movingDir == false && (P_ude - P_inde) <= Pdiff && P_inde <= 20000) {
        analogWrite(motorPWMNED, 150);
        analogWrite(motorPWMOP, 0);
        isMoving = true;
    }
    else if (movingDir == true && (P_ude - P_inde) >= -1*Pdiff && P_inde <= 20000) {
        analogWrite(motorPWMOP, 150);
        analogWrite(motorPWMNED, 0);
        isMoving = true;
    }
    else {
        // Stop the actuator if the pressure difference is within range
        analogWrite(motorPWMOP, 0);
        analogWrite(motorPWMNED, 0);
        isMoving = false; // Stop tracking time when motor is not moving
    }
}

void Send_til_platform(char* char_data) {
  // Sender bufferen via UDP
  Udp.beginPacket(remoteIp, remotePort);
  Udp.write(char_data);
  Udp.endPacket();
}

void Send_til_platform2(float data) {  //Benyttet til fejlfinding.
  char char_data2[10];                 // Buffer to hold the converted float
  dtostrf(data, 6, 2, char_data2);     // Convert float to string (6 digits, 2 decimal)

  Send_til_platform(char_data2);  // Send the char data via UDP
}

void NulstilAktu() {
  if (P_inde >= 20000) {
    return;
  }
  NulstilStart = millis();
  analogWrite(motorPWMOP, 255);

  if (NulstilStart - millis() > 5000) {
    analogWrite(motorPWMOP, 0);
    return;
  }
}

void SkrivTilSD(const char* dataLine) {
  File dataFile = SD.open(fileName, FILE_WRITE);  // Open the file for writing
  if (dataFile) {
    dataFile.println(dataLine);  // Write the string (with a newline at the end)
    dataFile.flush();            // Ensure data is written to the SD card
    dataFile.close();            // Close the file
  } else {
    // Handle file open error
    Serial.println("Error opening file!");
  }
  SD.end();  // Release SD card resources
}


void delay500ms() {                      //Sikre mere stabilt program. Ikke nødvendigvis 500 ms.
  unsigned long startMillis = millis();  // Record the start time
  while (millis() - startMillis < 200) {
  }
}

void setup()
{

  // Wait for serial connection
  Serial.begin(9600);
  printMemoryStats();

  // while (!Serial) {}
  //  Wait for Serial connection to establish
  printMemoryStats();

  Serial.println("Serial connection established.");

  // Set the baud rate for the SoftwareSerial object
  ECSerial.begin(9600);

  // Delay to ensure stability
  delay(2000);
  Serial1.begin(57600); // For UART communication to MIPEX
  Serial.println("Serial1 initialized at 57600 baud.");

  // Start I2C communication
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C communication started at 400 kHz.");

  // Ethernet initialization
  Ethernet.init(CS);
  Serial.println("Ethernet initialized with Chip Select pin.");
  Ethernet.begin(mac, ip, gateway, subnet);
  Serial.println("Ethernet network configured.");
  Udp.begin(localPort);
  Serial.println("UDP communication started on port " + String(localPort) + ".");

  // Start communication with sensors

  // Bar30 sensor
  multiplexer.selectChannel(0);
  Bar30.init();
  Bar30.setModel(MS5837::MS5837_30BA);
  Serial.println(F("Bar30 sensor initialized on multiplexer channel 0."));

  // SCD30 sensor
  SCD30.begin();
  Serial.println("SCD30 sensor initialized.");

  // HTU-21 humidity sensor
  htu.begin();
  Serial.println("HTU-21 sensor initialized.");

  // Tsys and Bar100 sensors
  multiplexer.selectChannel(1);
  TSYS.init();
  Bar100.init();
  Serial.println(F("TSYS and Bar100 sensors initialized on multiplexer channel 1."));

  // A/D converter
  //ads.begin();
  //ads.setGain(GAIN_TWOTHIRDS);
  ADS.begin();
  ADS.setGain(0);
  Serial.println(F("ADS A/D converter initialized with gain 2/3x."));

  // SD card initialization
  if (SD.begin(chipSelect))
  {
    fileNum = 1;
    char tempFileName[13] = "Data1.txt";
    while (SD.exists(tempFileName))
    {
      fileNum++;
      sprintf(tempFileName, "DATA%d.txt", fileNum);
    }

    strcpy(fileName, tempFileName);
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
      dataFile.println(F("Tid[s],P_out[mbar],P_in[mbar],RH[%],T_SCD[C],T_HTU21[C],T_out[C],CO2[ppm],O2[ppt],CH4[V],MIPEX,EC[muS/cm],Batteriniveau[%],motorDir,Total Free Heap:"));
      dataFile.flush();
      dataFile.close();
      Serial.println("SD card initialized. File " + String(fileName) + " created.");
    }
    else
    {
      Serial.println(F("Error: Failed to open file on SD card."));
    }
    SD.end();
  }
  else
  {
    Serial.println(F("Error: SD card initialization failed."));
  }

  // Leak sensor
  pinMode(leakPin, INPUT);
  Serial.println("Leak sensor pin configured as input.");

  // Servo motor setup
  servo.attach(servoPin, MinPulseWidth, MaxPulseWidth);
  servo.write(0);
  Serial.println("Servo motor initialized and set to closed position.");
  while (millis() - previousMillisSetup < interval1Setup)
  {
  }

  previousMillisSetup = millis();
  servo.write(6);
  Serial.println(F("Servo motor moved to 6 degrees to minimize holding torque."));
  while (millis() - previousMillisSetup < interval2Setup)
  {
  }

  // Actuator setup
  pinMode(motorPWMOP, OUTPUT);
  pinMode(motorPWMNED, OUTPUT);
  analogWrite(motorPWMOP, 150);
  Serial.println(F("Actuator motor initialized and set to move up."));

  delay(5000);

  movingBool = false;
  AktuSetup = false;

  multiplexer.selectChannel(1);
  Bar100.read();
  P_ude = Bar100.pressure();
  multiplexer.selectChannel(0);
  Bar30.read();
  P_inde = Bar30.pressure();
  adjP = P_ude - P_inde;

  // Initialization complete
  Serial.println("Setup complete. System is ready.");
}

void loop()
{
  Serial.println("Starting loop");

  for (int i = 0; i < 15; i++)
  {
    mainData[i] = -1; // Initialize all values in main data to -1
  }

  TidNuAktu = millis();
  if ((AktuSetup == true && TidNuAktu - startTid >= AktuStartTid) || (P_inde >= 20000))
  {
    Serial.println("Stopping motor due to condition");
    analogWrite(motorPWMOP, 0); // Stop motor
    movingBool = false;
    AktuSetup = false;
  }

  while (Main == false)
  {

    // Serial.println("Entering standby loop");
    delay(1); // Prevent watchdog timeout
    leak = 0; // Reset leak value
    if (millis() - tidSidste >= interval && !isMoving)
    {
      tidSidste = millis();
      Serial.println(F("Performing periodic tasks in standby loop"));
      Stopur();

      mainData[0] = tidGaaet;
      standby_data[0] = tidGaaet;

      multiplexer.selectChannel(1); // Selecting SD0 and SC0
      Bar100.read();
      P_ude = Bar100.pressure();
      standby_data[1] = P_ude;
      mainData[1] = P_ude;

      Serial.print("External pressure: ");
      Serial.println(P_ude);

      batteriniveau();
      standby_data[2] = Perc_bat;
      mainData[12] = Perc_bat;

      Serial.print("Battery percentage: ");
      Serial.println(Perc_bat);

      standby_str = sizeof(standby_data) / sizeof(standby_data[0]);
      float_til_char(standby_data, standby_str);

      Send_til_platform(buffer);

      mainStr = sizeof(mainData) / sizeof(mainData[0]);
      float_til_char(mainData, mainStr);

      SkrivTilSD(buffer);
    }

    Data_fra_platform();

    TidNuLeak = millis();
    leak = digitalRead(leakPin);
    if (leak == 1 && TidNuLeak - TidSidsteLeak >= IntervalLeak && !isMoving)
    {
      Serial.println("Leak detected");
      Udp.beginPacket(remoteIp, remotePort);
      Udp.write("Leak");
      Udp.endPacket();
      TidSidsteLeak = TidNuLeak;
    }

    if (millis() - aktuTidSidste >= aktuInterval)
    {
      PdiffTjek();
      aktuTidSidste = millis();
    }
  }

  while (Main == true)
  {
    // Serial.println("Entering main loop");
    delay(1); // Prevent watchdog timeout
    leak = 0; // Reset leak value
    if (millis() - tidSidste >= interval)
    {
      tidSidste = millis();
      Serial.println(F("Performing periodic tasks in main loop"));

      Stopur();

      multiplexer.selectChannel(0);

      Bar30.read();
      P_inde = Bar30.pressure();
      P_inde += adjP;
      Serial.print("Internal pressure: ");
      Serial.println(P_inde);

      SCD30.read();
      CO2 = SCD30.CO2;
      SCD30_temp = SCD30.temperature;
      Serial.print("SCD30 temp: ");
      Serial.println(SCD30_temp);
      Serial.print("SCD30 co2: ");
      Serial.println(CO2);

      RH = htu.readHumidity();
      htu_temp = htu.readTemperature();
      Serial.print("HTU temp: ");
      Serial.println(htu_temp);

      multiplexer.selectChannel(1);
      Bar100.read();
      P_ude = Bar100.pressure();
      Serial.print("external pressure: ");
      Serial.println(P_ude);

      TSYS.read();
      T_ude = TSYS.temperature();
      if (isnan(T_ude) || T_ude < 0 || T_ude > 100)
      {
        T_ude = 99;
      }
      Serial.print("external temp: ");
      Serial.println(T_ude);

      O2 = EZO_sensorer(O2_addr);
      Serial.print("O2 level: ");
      Serial.println(O2);

      EC = EC_float();
      Serial.print("EC level: ");
      Serial.println(EC);

      printMemoryStats();
      
      adc3 = ADS.readADC(1);
      //adc3 = ads.readADC_SingleEnded(1);
      CH4 = adc3 * 0.1875;
      Serial.print("CH4 level: ");
      Serial.println(CH4);

      // Inside your main loop where MIPEX is read
      Serial1.write("DATA");
      Serial1.write('\r');                         // Request data from Serial1
      unsigned long timeoutMIPEX = millis() + 100; // 100ms timeout
      bool dataReceived = false;                   // Flag to track if data was received

      while (millis() < timeoutMIPEX)
      {
        if (Serial1.available())
        {
          inByte = Serial1.read(); // Read incoming byte
          dataReceived = true;     // Mark that we got data

          if (inByte != '\r')
          {
            dataArray[arrayIndex] = inByte;
            arrayIndex++;
          }
          else
          {
            dataArray[arrayIndex] = '\0'; // Null-terminate string
            arrayIndex = 0;
          }

          // If we have enough characters (5), process MIPEX data
          if (arrayIndex == 5)
          {
            dataArray[5] = '\0'; // Null-terminate string
            String MIPEX_S = String((char *)dataArray);
            int MIPEX_int = MIPEX_S.toInt();
            MIPEX = (float)MIPEX_int;
            break; // Exit loop since we've received data
          }
        }
      }

      if (!dataReceived)
      {
        Serial.println(F("Warning: No response from Serial1 within timeout!"));
        MIPEX = -1.0; // or some error value to indicate no reading
      }

      batteriniveau();
      Serial.print(F("bat:"));
      Serial.println(Perc_bat);

      mainData[0] = tidGaaet;
      mainData[1] = P_ude;
      mainData[2] = P_inde;
      mainData[3] = RH;
      mainData[4] = SCD30_temp;
      mainData[5] = htu_temp;
      mainData[6] = T_ude;
      mainData[7] = CO2;
      mainData[8] = O2;
      mainData[9] = CH4;
      mainData[10] = MIPEX;
      mainData[11] = EC;
      mainData[12] = Perc_bat;
      mainData[13] = movingDir;
      mainData[14] = freeHeap;

      mainStr = sizeof(mainData) / sizeof(mainData[0]);
      float_til_char(mainData, mainStr);
      Send_til_platform(buffer);
      SkrivTilSD(buffer);
      multiplexer.selectChannel(1);
    }

    Data_fra_platform();

    TidNuLeak = millis();
    leak = digitalRead(leakPin);
    if (leak == 1 && TidNuLeak - TidSidsteLeak >= IntervalLeak)
    {
      Serial.println("Leak detected in main loop");
      Udp.beginPacket(remoteIp, remotePort);
      Udp.write("Leak");
      Udp.endPacket();
      TidSidsteLeak = TidNuLeak;
    }
    if (millis() - aktuTidSidste >= aktuInterval)
    {
      PdiffTjek();
      aktuTidSidste = millis();
    }
  }
}
