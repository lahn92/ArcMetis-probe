
// Samling af biblioteker, der inkluderes i Arduino-sketchen for at give funktionalitet til forskellige de forskellige hardwarekomponenter og kommunikationsprotokoller:

// #include <Adafruit_ADS1X15.h>  //Inkluderer bibliotek for A/D konverter fra Adafruit
#include <Servo.h>            //Inkluderer bibliotek for servomotor til vandprøve
#include <Wire.h>             //Inkluderer bibliotek for I2C kommunikation
#include <SD.h>               //Inkluderer bibliotek for SD-kort
#include <SPI.h>              //Inkluderer bibliotek for SPI kommunikation
#include <KellerLD.h>         //Inkluderer bibliotek for Bar100
#include <MS5837.h>           //Inkluderer bibliotek for Bar30
#include <TSYS01.h>           //Inkluderer bibliotek for Temperatur sensor fra Blue Robotics
#include <ArduinoBLE.h>       //Inkluderer bibliotek for arduinoen
#include <Adafruit_SCD30.h>   //Inkluderer bibliotek for SCD30
#include <EthernetENC.h>      //Inkluderer bibliotek for ethernet
#include <Adafruit_HTU21DF.h> //Inkluderer bibliotek for HTU21
#include <string>             // Bibliotek for konventering fra float til string
#include <queue>              //Kø bibliotek
// #include <Ezo_i2c.h>
// #include <Ezo_i2c_util.h>
// Biblioteker for konventering fra Char til float og omvendt
#include <stdlib.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <PCA9540BD.h> //Inkluderer bibliotek for PCA9540BD
#include <malloc.h>
#include <ADS1X15.h>
#include <nrfx.h>

const int settlingTime = 30000;           // Settling time before switching direction (in milliseconds)
float Pdiff = 10000;                       // Tilladelige tryk differens i mbar.



const char recoveryFileName[] = "RECOVERY.txt";

// EZO biblioteker
// #include <Ezo_i2c.h>
// #include <Ezo_i2c_util.h>
// #include <iot_cmd.h>
// #include <sequencer1.h>
// #include <sequencer2.h>
// #include <sequencer3.h>
// #include <sequencer4.h>

//                                             Defination af diverse startbetingelser samt variabler.

// Ethernet - arduino
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC-adresse for Ethernet-modul (selvvalgt)
IPAddress ip(192, 168, 1, 63);                     // IP-adresse for Ethernet-modul (Selvvalgt)
IPAddress subnet(255, 255, 255, 0);                // Skal matche den tilsluttede PC
IPAddress gateway(192, 168, 1, 1);                 // Skal matche den tilsluttede PC
unsigned int localPort = 61556;                    // lokal port til at lytte på
char packetBuffer[255];                            // buffer til at holde indgående pakke
EthernetUDP Udp;                                   // Opretter en instans af EthernetUDP-klassen ved navn Udp
const int CS = 10;                                 // Pin til ethernet modul.
float standby_data[3];                             // Array til at hole på tid og ude trykket, til når sonden ikke skal fortage målinger
int standby_str;                                   // Bruges til at bestemme størrelsen af standby arrayet

// Ethernet - platform
IPAddress remoteIp(192, 168, 1, 64); // Platformens IP-adresse. Den der benyttes den generelle IP-adresse
unsigned int remotePort = 61557;     // Platformens port
String dataToSend;                   // Opretter er variabel dataToSend som en string
int packetSize;                      // Variabel til at holde på antallet af indkommende data
float interval = 5000;               // Vaeriabel til at holde på ændring i måleinterval fra platform. Angives i ms.

// SD-kort
//  File dataFile; FJERNES
char fileName[20];              // Buffer til at holde på fil-navn
const int chipSelect = 9;       // Chip select pin for SD kort.
const int buffer_size_SD = 100; // Buffer størrelsen er 8 - kan ske at den skal ændres.
char buffer_SD[buffer_size_SD];
int fileNum;
String dataString = "";

// Laver en buffer til konventering fra float til char, som er nødvendig for at kunne skrive til SD-kortet.
const int buffer_size = 256; // Adjust based on your needs
char buffer[buffer_size] = "";

// SCD30
Adafruit_SCD30 SCD30;
float CO2;
float SCD30_temp;

// Bar30
MS5837 Bar30;
float P_inde = 1000;
float P_inde_tjek;
float TidligereP_inde = 0.0;
float P_tolerance = 25;

// TSYS01 - Blue Robotics temperatur sensor
TSYS01 TSYS;
float T_ude;

// Bar100
KellerLD Bar100;
float P_ude;
float P_ude_raw;
float P_ude_tjek;
// Trykdifferens til styring af aktuator

float adjP = 0;               // Variabel til justering af tryksensorer.
float Pdiff_high = Pdiff + 5; // Upper threshold
float Pdiff_low = Pdiff - 5;  // Lower threshold

const int P_ude_WINDOW_SIZE = 10;              // Number of readings to average
float P_ude_pressureHistory[P_ude_WINDOW_SIZE] = {0}; // Array to store past readings
int P_ude_historyIndex = 0;                   // Current index in the history array
float P_ude_runningSum = 0.0;                 // Keep track of the sum for efficiency




// AD konverter

ADS1115 ADS(0x48);
// Adafruit_ADS1115 ads;  // Initialiserer ADS1115 objektet
float CH4;    // Variabel til at holde på den beregne CH4 spænding
int16_t adc3; // CH4 sensoren er forbundet til ADC analog port 3

// HTU-21 fugtsensor
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float RH = 55;       // Variabel til at holde på fugtigheds målinger fra HTU-21 sensoren
float htu_temp = 55; // Variabel til at holde på temperatur målinger fra HTU-21 sensoren

// //EZO™ sensorer

float ezo_data;   // Variabel til at holde på data fra ezo sensorer
byte in_char = 0; // Bruges som en 1 byte buffer til at gemme indgående bytes fra o2-sensoren.
byte i = 0;       // Tæller for data-arrayet.
float O2 = 500;   // Variabel til at holde på O2 værdier
char *EZO_data_char;
String tempString; // String til at holde på string temperaturer til temperations kompensation for EZO sensorer
float EC;          // float var used to hold the float value of the conductivity.
int time_;
char data[32];
int dataStr;
char ec_data[32];
// Variablen sørger for at kompensationen kun fungerer en gang pr gennemkørsel af main-loop.

// Stopur
float startTid;      // Tid hvornår arduinoens loop begynder.
float nuTid;         // Tid hvornår arduien logger tiden igen
int tidGaaet = 0;    // Variabel til at beregning af hvor lang tid der er gået totalt
int oldTime = 0;
float tidSidste = 0; // Variabel til at beregne hvor lang tid der er gået siden sidste gennemkørsel.

// Batteriovervågning
float Perc_bat = 100;           // Variabel til batteriprocent
const int analogInPin = A7;     // Angiver den analoge indgang som der skal læses på
const float maxVoltage = 3.13;  // Angiver den maksimale spænding arduinoen kan læse fra batterierne
const float minVoltage = 1.9;   // Angiver den laveste spænding som arduinoen kan læse fra batterierne
int analog_maaling;             // Variabel til at holde på målingen fra analogpin 0.
float voltage;                  // Variabel til at holde på den konventerede spænding
float percentage;               // Variabel til at holde på den beregnede batteriprocent
std::queue<float> last15Values; // Kø til at gemme de sidste 10 værdier

// Boolean til main-loop
bool Main = true;

// Diverse I2C Adresser som ikke er defineret i deres biblioteker
byte O2_addr = 0x6C; // Standard ECO O2 sensor I2C address.

const byte ECrxPin = 2;
const byte ECtxPin = 4;
// Set up a new SoftwareSerial object for the EC sensor
UART ECSerial(ECrxPin, ECtxPin, NC, NC);

// Array til at holde på standby-værdierne
float standby[3];
char standby_char[3];

// Lækage sensorer
int leakPin = 8; // Leak Signal Pin
int leak = 0;    // 0 = Dry , 1 = Leak
unsigned long TidNuLeak = 0;
unsigned long TidSidsteLeak = 0;
const unsigned long IntervalLeak = 10000;
bool reset = false; // Startværdi for lækage interupt loo

// Mipex sensor

unsigned long tidLyt = 500; // Tid som der lyttes efter svar fra mipexsensoren
float MIPEX = -1;            // Variabel til at gemme værdier fra MIPEX sensoren.
char inByte;
int arrayIndex = 0;
char dataArray[5] = {0, 0, 0, 0, 0};

// Definering af PWM signal til servomotor
#define MinPulseWidth 500        // Korteste puls
#define MaxPulseWidth 2500       // Længste puls
#define DEFAULT_PULSE_WIDTH 1500 // Standard pulsbredde når servomotoren er tilsluttet
#define REFRESH_INTERVAL 20000   // Minmimums opdateringstid i ms
Servo servo;
const int servoPin = 3; // Definerer hvilken pin servomotorens PWN signal genereres på
bool vandprove = false;

unsigned long SidsteTid = 0;
float previousMillis;
const long interval1 = 2000; // Åbnetid
const long interval2 = 5000; // Tid åben
const long interval3 = 2000; // Lukketid
const long interval4 = 100;  // Fjern evt holdemoment
unsigned long previousMillisSetup = 0;
const long interval1Setup = 1000;
const long interval2Setup = 100;

// Aktuator
const int motorPWMOP = 5;                 // PWM pin for M1B
const int motorPWMNED = 6;                // PWM pin for M2B
unsigned long TidNuAktu;                  // Nuværende tid til aktuator stop
const unsigned long AktuStartTid = 10000; // Tid som aktuatoren kører for at opnå være i top.
bool AktuSetup = true;
unsigned long NulstilStart;                // Bruges til at nulstille aktuatorens position
bool motorActiv = false;                   // bool to track if the motor is active
bool movingBool = true;                    // bool to track if motor is currently doing other jobs (resetting etc.)
bool movingDir = false;                    // bool to keep track of the current direction of the motor false == vacum
int movingTimeAccumulating = 0;            // variable to keep track of the time the motor has moved in 1 direction.
const int timeToMoveFull = 7500;           // the time it takes for the actuator to move full stroke

unsigned long movementTimeAccumulated = 0; // Total movement time in current direction
unsigned long lastUpdateTime = 0;          // Last time movement was updated
unsigned long directionSwitchTime = 0;     // Time when direction was switched
bool isMoving = false;                     // Flag to check if the motor is currently running
bool isSettling = false;                   // Flag to track if we are in the settling phase
long aktuTidSidste = 0;
const long aktuInterval = 100;

// Array til at holde på alle værdier i main-loopet
float mainData[13] = {tidGaaet, P_ude, P_inde, RH, SCD30_temp, htu_temp, T_ude, CO2, O2, CH4, MIPEX, EC, Perc_bat};
int mainStr; // Variabel til bestemmelse af antal karakter i main-arrayet

// Multiplexer
PCA9540BD multiplexer;
int currentChannel = 1; // Start med kanal 0

////// Function prototypes //////
void Data_fra_platform();
void Stopur();
void float_til_char(float *data, int size);
float EZO_sensorer(byte addr);
float EC_float();
void batteriniveau();
void PdiffTjek();
void Send_til_platform(char *char_data);
void Send_til_platform2(float data);
void NulstilAktu();
void SkrivTilSD(float data);
void delay500ms();
int freeMemory();
int getFreeMemory();
void printMemoryStats();
void triggerReset();
float PUdegetMovingAverage(float newReading);

size_t freeHeap;
void printMemoryStats()
{
  struct mallinfo mi = mallinfo();
  Serial.print("Total Allocated Heap: ");
  Serial.println(mi.uordblks); // Used heap
  Serial.print("Total Free Heap: ");
  Serial.println(mi.fordblks); // Free heap
  freeHeap = mi.fordblks;
}

void Data_fra_platform() // Læse-funktion
{
  packetSize = Udp.parsePacket();

  if (packetSize)
  {                                       // Hvis der er en pakke tilgængelig
    int len = Udp.read(packetBuffer, 24); // Læser dataene fra den modtagne UDP-pakke ind i packetBuffer
    if (len > 0)
    {                        // Hvis der blev læst nogen data fra UDP-pakken
      packetBuffer[len] = 0; // Tilføjer en null-terminator til slutningen af dataene i packetBuffer
    }
    String input = String(packetBuffer); // Konverterer packetBuffer til en streng
    Serial.print("UDP packet is: ");
    Serial.println(input);
    if (input == "ON")
    { // Tjekker, om den indgående streng er "ON", hvis den er starter main-loopet
      Main = true;
    }

    if (input == "Stop")
    {
      Main = false;
    }

    if (input.startsWith("Maaleinterval"))
    {
      String tal_fra_platform = input.substring(String("Maaleinterval").length() + 1); // +1 for at springe over mellemrummet
      interval = tal_fra_platform.toFloat();
    }
    if (input == "Vandprove")
    {
      Serial.println("conducting water sample");
      servo.write(180); // move the servo to 180 degrees
      while (millis() - SidsteTid < interval1)
      {
      } // Venter på at servomotren er åben

      previousMillis = millis();
      servo.write(176); // Drejer 4 grader for at undgå unødvendig holdemoment og derved strømforbrug imens vandprøven tages
      while (millis() - SidsteTid < interval2)
      {
      } // Lader ventilen være åben i 4 sek.

      previousMillis = millis();
      servo.write(0); // lukker ventilen igen
      while (millis() - SidsteTid < interval3)
      {
      } // Delay som sikre at ventilen er lukket

      previousMillis = millis();
      servo.write(6); // Drejer motoren 6 grader for at undgå unødvendige holdemoment og derved strømforbrug
      while (millis() - SidsteTid < interval4)
      {
      } // Delay som sikre at ventilen er lukket
    }
    if (input == "Reset")
    {
      reset = true;
    }
    if (input.startsWith("Pdiff"))
    {
      String tal_fra_platform = input.substring(String("Pdiff").length() + 1); // +1 for at springe over mellemrummet
      Pdiff = tal_fra_platform.toFloat();
      Pdiff_high = Pdiff + 5; // Upper threshold
      Pdiff_low = Pdiff - 5;  // Lower threshold
    }
    if (input == "Nulstil")
    {
      NulstilAktu();
    }
    if (input == "Moving")
    {
      NulstilAktu();
      movingBool = true;
    }

    if (input == "steady")
    {
      movingBool = false;
    }

    if (input == "ned")
    {                                       // Aktuatoren skal bevæge sig ned
      unsigned long startTidNED = millis(); // Gemmer starttidspunktet

      // Start aktuatoren med 50% PWM
      analogWrite(motorPWMOP, 0);    // 0% PWM
      analogWrite(motorPWMNED, 127); // 50% PWM
      while (millis() - startTidNED < 1500)
      {
        // Aktuatoren bevæger sig med 50% PWM.
      }

      // Skift til 75% PWM efter 3 sekunder
      analogWrite(motorPWMNED, 191); // 75% PWM
      while (millis() - startTidNED < 2500)
      {
        // Aktuatoren bevæger sig med 75% PWM.
      }

      // Skift til 100% PWM efter 6 sekunder
      analogWrite(motorPWMNED, 255); // 100% PWM
      while (millis() - startTidNED < 10000)
      {
        // Aktuatoren bevæger sig med 100% PWM.
      }

      analogWrite(motorPWMNED, 0); // Stop aktuatoren efter 10 sekunder
    }

    if (input == "op")
    {                                      // Aktuatoren skal bevæge sig op
      unsigned long startTidOP = millis(); // Gemmer starttidspunktet
      analogWrite(motorPWMNED, 0);         // 0% PWM
      analogWrite(motorPWMOP, 127);        // 50% PWM
      while (millis() - startTidOP < 1500)
      {
        // Aktuatoren bevæger sig med 50% PWM.
      }

      // Skift til 75% PWM efter 3 sekunder
      analogWrite(motorPWMOP, 191); // 75% PWM
      while (millis() - startTidOP < 2500)
      {
        // Aktuatoren bevæger sig med 75% PWM.
      }

      // Skift til 100% PWM efter 6 sekunder
      analogWrite(motorPWMOP, 255); // 100% PWM
      while (millis() - startTidOP < 10000)
      {
        // Aktuatoren bevæger sig med 100% PWM.
      }

      analogWrite(motorPWMOP, 0); // Stop aktuatoren efter 10 sekunder
    }

    if (input == "adjP")
    {
      adjP = P_ude - P_inde;
    }
    if (input == "motorActivate")
    {
      motorActiv = true;
    }
    if (input == "motorDeactivate")
    {
      motorActiv = false;
    }
  }
}

void Stopur()
{
  // Beregn, hvor lang tid der er gået siden starten af stopuret
  nuTid = millis();                       // Tid [ms] hvor funktionen kaldes
  tidGaaet = ((nuTid - startTid) / 1000)+oldTime; // Tid [s] gået siden funktionen sidst blev kaldt.
}

void float_til_char(float *data, int size)
{
  char temp[20]; // Temporary buffer for each float

  memset(buffer, 0, sizeof(buffer)); // Reset buffer safely

  for (int i = 0; i < size; i++)
  {
    snprintf(temp, sizeof(temp), "%.2f", data[i]); // Convert float safely

    // Ensure we don't exceed buffer size
    if (strlen(buffer) + strlen(temp) + 1 < buffer_size)
    {
      strcat(buffer, temp);
      if (i < size - 1)
      {
        strcat(buffer, ",");
      }
    }
    else
    {
      // Handle overflow case
      break;
    }
  }
}

float EZO_sensorer(byte addr)
{
  Wire.beginTransmission(addr); // Kalder kredsløbet ved dets adresse.
  if (addr == O2_addr)
  {
    dataStr = 20;
    time_ = 600;
    tempString = String(htu_temp, 0);
    Wire.write(("RT," + tempString).c_str()); // Sender en "RT,den indvendige temperatur" kommando til sensoren for at anmode om en enkelt aflæsning.
    Wire.endTransmission();                   // Afslutter I2C-dataoverførslen.
  }
  delay(time_); // Venter det korrekte tidsinterval for at kredsløbet kan fuldføre sin instruktion.

  Wire.requestFrom(addr, dataStr, 1); // Anmoder om datastørrelse byte fra kredsløbet ved den relevante addresse
  Wire.read();
  i = 0;
  while (Wire.available())
  {
    in_char = Wire.read();
    data[i] = in_char;
    i += 1; // Forøger tælleren for arrayelementet.
    if (Wire.available() == 0)
    { // Tjekker om I2C-bufferen er tom. Hvis den er tom, nulstilles indekset.
      i = 0;
      break;
    }
    // }
    while (Wire.available() && i < sizeof(data) - 1)
    {
      data[i] = Wire.read();
      i++;
    }
    data[i] = '\0';        // Null-terminate the string
    ezo_data = atof(data); // Konverterer dataet til float-format
    return ezo_data;
  }
  return -1;
}

float EC_float()
{
  ECSerial.println("R"); // Send the "read" command to the sensor

  delay(600); // Allow the sensor to process and respond

  // Reset the buffer and counter
  memset(ec_data, 0, sizeof(ec_data));
  i = 0;

  unsigned long startTime = millis(); // Track start time for timeout
  while (millis() - startTime < 1000)
  { // 1-second timeout
    if (ECSerial.available())
    {
      char in_char = ECSerial.read(); // Read a byte
      if (i < sizeof(ec_data) - 1)
      {                         // Ensure we don't overflow the buffer
        ec_data[i++] = in_char; // Store the byte and increment the counter
      }
      if (ECSerial.available() == 0)
      { // Exit when no more data is available
        break;
      }
    }
  }

  ec_data[i] = '\0'; // Null-terminate the string

  // Directly process the data from ec_data
  int commaIndex = -1;
  for (int j = 0; j < i; j++)
  {
    if (ec_data[j] == ',')
    {
      commaIndex = j;
      break;
    }
  }

  if (commaIndex != -1)
  {
    ec_data[commaIndex] = '.'; // Replace comma with a period
  }

  // Convert to float
  float ec_fdata = atof(ec_data); // Convert the char[] to float
  if (ec_fdata == 0.0 && ec_data[0] != '0')
  {
    // Handle invalid or unexpected response
    Serial.println("Error: Invalid EC data");
    return -1.0; // Return a sentinel value indicating an error
  }

  return ec_fdata; // Return the converted float
}

void batteriniveau()
{
  // Beregn spændingsniveauet i procent
  analog_maaling = analogRead(analogInPin);
  voltage = analog_maaling * (3.3 / 1023.0);                               // Konverter analogt input til spænding
  percentage = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0; // Beregner batteriniveau i %

  // Tilføj den nye procentdel til køen
  last15Values.push(percentage);

  // Hvis køen har mere end 15 elementer, fjern det ældste
  if (last15Values.size() > 15)
  {
    last15Values.pop();
  }

  // Beregn gennemsnittet af de sidste 15 værdier
  float sum = 0;
  std::queue<float> temp = last15Values;
  while (!temp.empty())
  {
    sum += temp.front();
    temp.pop();
  }
  float average = sum / last15Values.size(); // Finder gennemsnittet af værdierne i bufferen

  Perc_bat = constrain(average, 0, 100); // Sørger for, at procentdelen er inden for intervallet 0-100%. Ligger den udenfor intervallet vil den returnere et nærmeste ekstrema.
}

void PdiffTjek()
{
  multiplexer.selectChannel(1);
  Bar100.read();
  P_ude_raw = Bar100.pressure();
  P_ude = PUdegetMovingAverage(P_ude_raw);

  if (movingBool == true || motorActiv == false || Perc_bat < 15)
  {
    return;
  }

  multiplexer.selectChannel(0);
  Bar30.read();
  P_inde = adjP + Bar30.pressure();
  unsigned long currentTime = millis();

  if (isMoving)
  {
    movementTimeAccumulated += (currentTime - lastUpdateTime);
  }

  lastUpdateTime = currentTime;

  if (isSettling)
  {
    if (currentTime - directionSwitchTime >= settlingTime)
    {
      Serial.println("Settling time over, resuming movement.");
      isSettling = false;
    }
    else
    {
      Serial.println("Waiting for settling time...");
      return;
    }
  }

  if (movementTimeAccumulated >= timeToMoveFull)
  {
    Serial.println("Switching direction due to accumulated time limit");

    movingDir = !movingDir;
    movementTimeAccumulated = 0;

    isSettling = true;
    directionSwitchTime = currentTime;

    analogWrite(motorPWMOP, 0);
    analogWrite(motorPWMNED, 0);
    isMoving = false;

    return;
  }

  // Control actuator movement with hysteresis
  if (isMoving)
  {
    // Stop movement when the pressure difference reaches Pdiff
    if ((movingDir == false && (P_ude - P_inde) >= Pdiff) ||
        (movingDir == true && (P_ude - P_inde) <= -Pdiff))
    {
      analogWrite(motorPWMOP, 0);
      analogWrite(motorPWMNED, 0);
      isMoving = false;
    }
  }
  else
  {
    // Start movement again only when the hysteresis range is exceeded
    if (movingDir == false && (P_ude - P_inde) <= Pdiff_low)
    {
      analogWrite(motorPWMNED, 150);
      analogWrite(motorPWMOP, 0);
      isMoving = true;
    }
    else if (movingDir == true && (P_ude - P_inde) >= -Pdiff_high)
    {
      analogWrite(motorPWMOP, 150);
      analogWrite(motorPWMNED, 0);
      isMoving = true;
    }
  }
}

void Send_til_platform(char *char_data)
{
  // Sender bufferen via UDP
  Udp.beginPacket(remoteIp, remotePort);
  Udp.write(char_data);
  Udp.endPacket();
}

void Send_til_platform2(float data)
{                                  // Benyttet til fejlfinding.
  char char_data2[10];             // Buffer to hold the converted float
  dtostrf(data, 6, 2, char_data2); // Convert float to string (6 digits, 2 decimal)

  Send_til_platform(char_data2); // Send the char data via UDP
}

void NulstilAktu()
{
  if (P_inde >= 20000)
  {
    return;
  }
  NulstilStart = millis();
  analogWrite(motorPWMOP, 255);

  delay(5000);
  
  analogWrite(motorPWMOP, 0);
  movementTimeAccumulated = 0;
  directionSwitchTime = 0; 
  return;
  
}

void SkrivTilSD(const char *dataLine)
{
  File dataFile = SD.open(fileName, FILE_WRITE); // Open the file for writing
  if (dataFile)
  {
    dataFile.println(dataLine); // Write the string (with a newline at the end)
    dataFile.flush();           // Ensure data is written to the SD card
    dataFile.close();           // Close the file
  }
  else
  {
    // Handle file open error
    Serial.println("Error opening file!");
  }
  SD.end(); // Release SD card resources
}

void delay500ms()
{                                       // Sikre mere stabilt program. Ikke nødvendigvis 500 ms.
  unsigned long startMillis = millis(); // Record the start time
  while (millis() - startMillis < 200)
  {
  }
}

void saveToRecoveryFile() {
  Serial.println("Starting saveToRecoveryFile");

  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed in saveToRecoveryFile!");
    return;
  }

  File recoveryFile = SD.open(recoveryFileName, FILE_WRITE);
  if (recoveryFile) {
    // Write variables in a comma-separated format
    recoveryFile.print(movingDir);         recoveryFile.print(",");
    recoveryFile.print(motorActiv);        recoveryFile.print(",");
    recoveryFile.print(movingTimeAccumulating); recoveryFile.print(",");
    recoveryFile.print(adjP, 6);           recoveryFile.print(",");
    recoveryFile.print(tidGaaet);          recoveryFile.print(",");
    recoveryFile.print(Main);              // Save Main (bool)

    recoveryFile.println();  // Newline to finish the entry

    recoveryFile.flush();  // Ensure data is written
    recoveryFile.close();  // Close the file
    Serial.println("Variables saved to recovery file: " + String(recoveryFileName));
  } else {
    Serial.println("Error: Failed to open recovery file for writing!");
  }

  SD.end(); // Release SD card resources
}

void restoreFromRecoveryFile() {
  Serial.println("Starting restoreFromRecoveryFile");

  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card initialization failed in restoreFromRecoveryFile!");
    return;
  }

  File recoveryFile = SD.open(recoveryFileName, FILE_READ);
  if (recoveryFile) {
    String line = recoveryFile.readStringUntil('\n'); // Read the first line
    recoveryFile.close();

    // Parse the comma-separated values
    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);
    int comma3 = line.indexOf(',', comma2 + 1);
    int comma4 = line.indexOf(',', comma3 + 1);
    int comma5 = line.indexOf(',', comma4 + 1);

    if (comma1 != -1 && comma2 != -1 && comma3 != -1 && comma4 != -1 && comma5 != -1) {
      movingDir = line.substring(0, comma1).toInt(); // bool stored as 0 or 1
      motorActiv = line.substring(comma1 + 1, comma2).toInt(); // bool stored as 0 or 1
      movingTimeAccumulating = line.substring(comma2 + 1, comma3).toInt();
      adjP = line.substring(comma3 + 1, comma4).toFloat();
      tidGaaet = line.substring(comma4 + 1, comma5).toInt();
      Main = line.substring(comma5 + 1).toInt(); // Restore Main (bool stored as 0 or 1)

      Serial.print("Restored movingDir: "); Serial.println(movingDir);
      Serial.print("Restored motorActiv: "); Serial.println(motorActiv);
      Serial.print("Restored movingTimeAccumulating: "); Serial.println(movingTimeAccumulating);
      Serial.print("Restored adjP: "); Serial.println(adjP);
      Serial.print("Restored tidGaaet: "); Serial.println(tidGaaet);
      Serial.print("Restored Main: "); Serial.println(Main);
    } else {
      Serial.println("Error: Invalid format in recovery file!");
    }

    // Delete the recovery file after successful read
    if (SD.remove(recoveryFileName)) {
      Serial.println("Recovery file deleted after restore.");
    } else {
      Serial.println("Error: Failed to delete recovery file!");
    }
  } else {
    Serial.println("No recovery file found, skipping restore.");
  }

  SD.end(); // Release SD card resources
}


void triggerReset()
{
  Serial.println("starting reset");

  saveToRecoveryFile();
  Serial.println("vars saved to flash");

  delay(100);
  NVIC_SystemReset(); // Perform software reset
}

float PUdegetMovingAverage(float newReading) {
  // Subtract the oldest reading from the running sum
  P_ude_runningSum -= P_ude_pressureHistory[P_ude_historyIndex];
  
  // Add the new reading to the history array
  P_ude_pressureHistory[P_ude_historyIndex] = newReading;
  
  // Add the new reading to the running sum
  P_ude_runningSum += newReading;
  
  // Move to the next index, wrapping around to 0 if needed
  P_ude_historyIndex = (P_ude_historyIndex + 1) % P_ude_WINDOW_SIZE;
  
  // Return the moving average
  return P_ude_runningSum / P_ude_WINDOW_SIZE;
}

void setup()
{
  Serial.begin(9600);
  // while (!Serial) {}
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

  // Check for recovery file and handle reset logic
  bool resetOccurred = false;
  if (SD.begin(chipSelect))
  {
    if (SD.exists(recoveryFileName))
    {
      Serial.println("Recovery file detected, restoring variables...");
      restoreFromRecoveryFile(); // Restores variables and deletes RECOVERY.txt
      resetOccurred = true;
    }
    else
    {
      Serial.println("No recovery file found, initializing variables...");
    }

    // SD card setup for data logging
    fileNum = 1;
    char tempFileName[13] = "DATA1.txt";

    if (resetOccurred)
    {
      // Find the last used file to continue appending
      while (SD.exists(tempFileName))
      {
        fileNum++;
        sprintf(tempFileName, "DATA%d.txt", fileNum);
      }
      fileNum--; // Step back to the last existing file
      if (fileNum < 1)
        fileNum = 1; // Ensure we don’t go below 1
      sprintf(tempFileName, "DATA%d.txt", fileNum);
      Serial.println("Reset detected, continuing with existing file: " + String(tempFileName));
    }
    else
    {
      // Normal boot: find the next available file
      while (SD.exists(tempFileName))
      {
        fileNum++;
        sprintf(tempFileName, "DATA%d.txt", fileNum);
      }
      Serial.println("Normal boot, creating new file: " + String(tempFileName));
    }

    strcpy(fileName, tempFileName);
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
      if (!resetOccurred)
      {
        // Only write header if it’s a new file (normal boot)
        dataFile.println(F("Tid[s],P_out[mbar],P_in[mbar],RH[%],T_SCD[C],T_HTU21[C],T_out[C],CO2[ppm],O2[ppt],CH4[V],MIPEX,EC[muS/cm],Batteriniveau[%]"));
      }
      dataFile.flush();
      dataFile.close();
      Serial.println("SD initialized: " + String(fileName));
    }
    else
    {
      Serial.println(F("Error: Failed to open file on SD card."));
    }
    SD.end();
  }
  else
  {
    Serial.println("Error: SD card initialization failed during setup!");
  }

  // Start communication with sensors
  multiplexer.selectChannel(0);
  delay(200);
  Bar30.init();
  Bar30.setModel(MS5837::MS5837_30BA);
  Serial.println(F("Bar30 sensor initialized on multiplexer channel 0."));

  SCD30.begin();
  Serial.println("SCD30 sensor initialized.");

  htu.begin();
  Serial.println("HTU-21 sensor initialized.");

  multiplexer.selectChannel(1);
  TSYS.init();
  Bar100.init();
  Serial.println(F("TSYS and Bar100 sensors initialized on multiplexer channel 1."));

  if (!ADS.begin() || !ADS.isConnected())
  {
    Serial.println(F("Warning: ADS1115 A/D converter not found at 0x48."));
  }
  ADS.setGain(0);
  Serial.println(F("ADS A/D converter initialized with gain 2/3x."));

  // Rest of your setup (leak sensor, servo, actuator)...
  pinMode(leakPin, INPUT);
  Serial.println("Leak sensor pin configured as input.");

  servo.attach(servoPin, MinPulseWidth, MaxPulseWidth);
  servo.write(0);
  Serial.println("Servo motor initialized and set to closed position.");

  delay(interval1Setup);
  servo.write(6);
  Serial.println(F("Servo motor moved to 6 degrees to minimize holding torque."));
  delay(interval2Setup);

  pinMode(motorPWMOP, OUTPUT);
  pinMode(motorPWMNED, OUTPUT);

  if (!resetOccurred)
  {
    analogWrite(motorPWMOP, 150);
    Serial.println(F("Actuator motor initialized and set to move up."));
    delay(5000);
    analogWrite(motorPWMOP, 0);
    multiplexer.selectChannel(1);
    for (size_t i = 0; i < P_ude_WINDOW_SIZE; i++)
    {
      Bar100.read();
      P_ude = PUdegetMovingAverage(Bar100.pressure());
      delay(100);
    }
    
    Bar100.read();
    P_ude = Bar100.pressure();
    multiplexer.selectChannel(0);
    Bar30.read();
    P_inde = Bar30.pressure();
    adjP = P_ude - P_inde;
  }

  movingBool = false;
  AktuSetup = false;

  Serial.println("Setup complete. System is ready.");
}


void loop()
{
  Serial.println("Starting loop");

  for (int i = 0; i < 13; i++)
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
      if (P_ude > 150000 || isnan(P_ude))
      {
        triggerReset();
      }

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
      Serial.println("conversion done");

      Send_til_platform(buffer);
      Serial.println("udp sent");

      mainStr = sizeof(mainData) / sizeof(mainData[0]);
      float_til_char(mainData, mainStr);

      SkrivTilSD(buffer);

      Serial.println("sd card done");
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
    if (millis() - tidSidste >= interval && !isMoving)
    {
      tidSidste = millis();
      Serial.println(F("Performing periodic tasks in main loop"));

      Stopur();

      multiplexer.selectChannel(0);

      Bar30.read();
      P_inde = Bar30.pressure();
      if (P_inde < 100 || P_inde > 300000)
      {
        triggerReset();
      }
      Serial.print("Internal pressure: ");
      Serial.println(P_inde);

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
      Serial.print("external pressure: ");
      Serial.println(P_ude);

      TSYS.read();
      T_ude = TSYS.temperature();
      if (isnan(T_ude) || T_ude < 0 || T_ude > 100)
      {
        T_ude = 99;
      }
      if (isnan(T_ude))
      {
        triggerReset();
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

      adc3 = ADS.readADC(3);
      // adc3 = ads.readADC_SingleEnded(1);
      if (ADS.getError() == ADS1X15_OK)
      {
        CH4 = adc3 * 0.1875;
        Serial.print("CH4 level: ");
        Serial.println(CH4);
      }
      //  Use value
      else
      {
        Serial.println(F("ADC Error"));
        triggerReset();
      }
      //  handle error

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


      mainStr = sizeof(mainData) / sizeof(mainData[0]);
      float_til_char(mainData, mainStr);
      Send_til_platform(buffer);
      SkrivTilSD(buffer);
      multiplexer.selectChannel(1);
    }

    if (Serial.available() > 0) {
      char receivedChar = Serial.read();
      if (receivedChar == 'r') {
          Serial.println(F("Forcing reset"));
          triggerReset();
      }
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
