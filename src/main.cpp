
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
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>
//Biblioteker for konventering fra Char til float og omvendt
#include <stdlib.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <PCA9540BD.h>  //Inkluderer bibliotek for PCA9540BD

//EZO biblioteker
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>
#include <iot_cmd.h>
#include <sequencer1.h>
#include <sequencer2.h>
#include <sequencer3.h>
#include <sequencer4.h>


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
float Pdiff = 200;     //Tilladelige tryk differens i mbar.
float adjP = 0;        //Variabel til justering af tryksensorer.
                       //AD konverter
Adafruit_ADS1115 ads;  // Initialiserer ADS1115 objektet
float CH4;             //Variabel til at holde på den beregne CH4 spænding
int16_t adc3;          // CH4 sensoren er forbundet til ADC analog port 3

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
bool Main;


//Diverse I2C Adresser som ikke er defineret i deres biblioteker
byte O2_addr = 0x6C;  //Standard ECO O2 sensor I2C address.
byte EC_addr = 0x64;  //Standard ECO EC I2C addresse


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

//Array til at holde på alle værdier i main-loopet
float mainData[13] = { tidGaaet, P_ude, P_inde, RH, SCD30_temp, htu_temp, T_ude, CO2, O2, CH4, MIPEX, EC, Perc_bat };
int mainStr;  //Variabel til bestemmelse af antal karakter i main-arrayet

//Definering af PWM signal til servomotor
#define MIN_PULSE_WIDTH 500       // Korteste puls
#define MAX_PULSE_WIDTH 2500      // Længste puls
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
bool movingBool = false;


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


void Data_fra_platform()  //Læse-funktion
{
  packetSize = Udp.parsePacket();

  if (packetSize) {                        // Hvis der er en pakke tilgængelig
    int len = Udp.read(packetBuffer, 24);  // Læser dataene fra den modtagne UDP-pakke ind i packetBuffer
    if (len > 0) {                         // Hvis der blev læst nogen data fra UDP-pakken
      packetBuffer[len] = 0;               // Tilføjer en null-terminator til slutningen af dataene i packetBuffer
    }
    String input = String(packetBuffer);  // Konverterer packetBuffer til en streng
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
  Wire.beginTransmission(addr);  // Kalder kredsløbet ved dets adresse.
  if (addr == O2_addr) {
    dataStr = 20;
    time_ = 600;
    char O2_data[20];
    tempString = String(htu_temp, 0);
    Wire.write(("RT," + tempString).c_str());  // Sender en "RT,den indvendige temperatur" kommando til sensoren for at anmode om en enkelt aflæsning.
    Wire.endTransmission();                    // Afslutter I2C-dataoverførslen.
  }
  if (addr == EC_addr) {
    dataStr = 32;
    time_ = 700;
    char EC_data[32];  //Bruges til at holde I2C-responskoden fra EC sensor.
    tempString = String(T_ude, 0);
    Wire.write(("RT," + tempString).c_str());  //Sender den temperatur kompensations kommando med den nuværende udetremperatur
    Wire.endTransmission();                    // Afslutter I2C-dataoverførslen.
  }
  delay(time_);  // Venter det korrekte tidsinterval for at kredsløbet kan fuldføre sin instruktion.

  Wire.requestFrom(addr, dataStr, 1);  // Anmoder om datastørrelse byte fra kredsløbet ved den relevante addresse
  Wire.read();
  i = 0;
  while (Wire.available()) {
    in_char = Wire.read();
    data[i] = in_char;
    i += 1;                       // Forøger tælleren for arrayelementet.
    if (Wire.available() == 0) {  //Tjekker om I2C-bufferen er tom. Hvis den er tom, nulstilles indekset.
      i = 0;
      break;
    }
    // }
    while (Wire.available() && i < sizeof(data) - 1) {
      data[i] = Wire.read();
      i++;
    }
    data[i] = '\0';         // Null-terminate the string
    ezo_data = atof(data);  // Konverterer dataet til float-format
    return ezo_data;
  }
}
float EC_float() {

  Wire.beginTransmission(0x64);  // call the circuit by its ID number.
  Wire.write("R");               // transmit the command that was sent through the serial port.
  Wire.endTransmission();        // end the I2C data transmission.

  delay(570);  // wait the correct amount of time for the circuit to complete its instruction.

  Wire.requestFrom(0x64, 32, 1);  // call the circuit and request 32 bytes.

  int code = Wire.read();  // the first byte is the response code, we read this separately.

  // Make sure to reset the ec_data buffer and the counter i
  memset(ec_data, 0, sizeof(ec_data));
  i = 0;

  while (Wire.available()) {
    in_char = Wire.read();        // receive a byte.
    ec_data[i] = in_char;         // load this byte into our array.
    i += 1;                       // increment the counter for the array element.
    if (Wire.available() == 0) {  // if we see that we have been sent a null command.
      i = 0;                      // reset the counter i to 0.
      break;                      // exit the while loop.
    }
  }

  float ec_fdata = atof(ec_data);

  return ec_fdata;
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


  if (movingBool == true) {
    return;
  }

  if (abs(P_inde - TidligereP_inde) <= P_tolerance) {  //Hvis det indre tryk ikke justeres efter sidste gennemkørsel, kan det være en tegn på at aktuatoren er i bund/top
    return;
  }

  TidligereP_inde = P_inde;

  if (P_ude - P_inde > Pdiff && P_inde <= 20000) {
    while (true) {
      analogWrite(motorPWMOP, 150);
      Bar100.read();
      Bar30.read();
      P_ude_tjek = Bar30.pressure();
      P_inde_tjek = Bar30.pressure() + adjP;
      if (P_ude_tjek - P_inde_tjek <= 0) {
        analogWrite(motorPWMOP, 0);
        break;
      }
    }
  }
  if (P_ude - P_inde < Pdiff && P_inde <= 20000) {
    while (true) {
      analogWrite(motorPWMNED, 150);
      Bar100.read();
      Bar30.read();
      P_ude_tjek = Bar30.pressure();
      P_inde_tjek = Bar30.pressure() + adjP;
      if (P_ude_tjek - P_inde_tjek >= 0) {
        analogWrite(motorPWMNED, 0);
        break;
      }
    }
  } else {
    return;
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

void SkrivTilSD(float data) {
  File dataFile = SD.open(fileName, FILE_WRITE);  // Åbner det oprettede dokument for skrivning
  if (dataFile) {
    String dataString = String(data);
    dataFile.print(dataString);  // Printer kolonne i tekstdokumentet
    dataFile.print(",");
    dataFile.flush();  // Sikrer at data bliver skrevet til SD-kortet
    dataFile.close();  // Lukker filen efter skrivning
  } else {
    return;
  }
  // Forsøg på at frigive SD-kortet efter operationerne
  SD.end();  // Frigør SD-kortet
}

void delay500ms() {                      //Sikre mere stabilt program. Ikke nødvendigvis 500 ms.
  unsigned long startMillis = millis();  // Record the start time
  while (millis() - startMillis < 200) {
  }
}


void setup() {


  //Generelt
  Serial.begin(9600);
  delay(2000);
  Serial1.begin(57600);  //Til UART kommunikation til MIPEX

  Wire.begin();  //Begynder I2C kommunikation
  //Ethernet
  Ethernet.init(CS);                         //Definerer hvilken pin chip select til ethernet board er tilsluttet
  Ethernet.begin(mac, ip, gateway, subnet);  //Opretter et netværk for arduinoen med de definerede netværksoplysninger
  Udp.begin(localPort);                      //Starter UDP kommunikation på den definerede port

  //Start af kommunikation til diverse sensorer:

  //Bar30
  multiplexer.selectChannel(0);  // for selecting SD1 and SC1
  Bar30.init();
  Bar30.setModel(MS5837::MS5837_30BA);  //Sætter modellen til bar30

  //SCD30
  SCD30.begin();

  //Tsys
  TSYS.init();

  //A/D konverter
  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/arduino-code

  //HTU-21 fugtsensor
  htu.begin();

  // //SD-kort
  SD.begin(chipSelect);                 // Initialiserer SD-kortet på den specificerede CS (Chip Select) pin
  fileNum = 1;                          // Startværdi til datafilX.
  char tempFileName[13] = "Data1.txt";  // Opretter en midlertidig filnavn streng
  // Hvis Data1.txt allerede findes på SD kortet vil den øge det sidste nummer indtil den finder et ledigt navn.
  while (SD.exists(tempFileName)) {                // Tjekker om filen allerede eksisterer på SD-kortet
    fileNum++;                                     // Hvis filen eksisterer, øges filnummeret
    sprintf(tempFileName, "DATA%d.txt", fileNum);  // Genererer et nyt filnavn med det opdaterede filnummer
  }
  strcpy(fileName, tempFileName);                                                                                                                                // Kopierer det endelige filnavn fra den midlertidige filnavn streng
  File dataFile = SD.open(fileName, FILE_WRITE);                                                                                                                 // Åbner det oprettede dokument for skrivning
  dataFile.println("Tid [s],P_out [mbar],P_in [mbar],RH [%],T_SCD [C],T_HTU21 [C],T_out [C],CO2 [ppm],O2 [ppt],CH4 [V],MIPEX ,EC [muS/cm], Batteriniveau [%]");  // Printer kolonne overskrift i tekstdoumentet

  dataFile.flush();  // Sikrer at data bliver skrevet til SD-kortet
  dataFile.close();  // Lukker filen efter skrivning
  SD.end();          // Frigør SD-kortet
  startTid = millis();

  //Indtil den modtager et start signal fra platformen skal den kun fortage dybdemålinger samt logge tiden
  Main = false;

  //Til lækage sensorer
  pinMode(leakPin, INPUT);  // Set pin as input



  // //Til servomotor

  servo.attach(servoPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servo.write(0);                                             // Sørger for at ventilen er lukket når programmet starter
  while (millis() - previousMillisSetup < interval1Setup) {}  // Sikre at sermotoren har tid nok til at dreje ventil i lukkeposition

  previousMillisSetup = millis();
  servo.write(6);                                             // Drejer motoren 5 grader for at undgå unødvendige holdemoment og derved strømforbrug
  while (millis() - previousMillisSetup < interval2Setup) {}  // Sikre at sermotoren har tid nok til at dreje ventil i lukkeposition

  //Aktuator
  pinMode(motorPWMOP, OUTPUT);
  pinMode(motorPWMNED, OUTPUT);

  analogWrite(motorPWMOP, 255);  //Morten kører op


  //Leak sensor
  pinMode(leakPin, INPUT);

  //Bar100
  multiplexer.selectChannel(1);  // for selecting SD0 and SC0
  Bar100.init();
}



void loop() {
  TidNuAktu = millis();
  if ((AktuSetup == true && TidNuAktu - startTid >= AktuStartTid) || (P_inde >= 20000)) {
    analogWrite(motorPWMOP, 0);  // Stop motoren
    AktuSetup = false;
  }
  while (Main == false) {
    leak = 0;                                //Nulstille lækageværdien i tilfælde af fejl.
    if (millis() - tidSidste >= interval) {  //Da delay() funktionen giver en unødvendigt delay i datamodtagelse fra platform køres loopet kun når x tid er gået
      tidSidste = millis();                  // Opdaterer variabel tidSidste til nuværende tid

      Stopur();  //Tiden noteres
      SkrivTilSD(tidGaaet);


      standby_data[0] = tidGaaet;  //Opdater standby arrayet med den nye tid
      //Der fortages en måling af det udvendige tryk
      multiplexer.selectChannel(1);  // for selecting SD0 and SC0
      Bar100.read();
      P_ude = Bar100.pressure();  //Opdater standby arrayet med den nye trykmåling
      standby_data[1] = P_ude;
      SkrivTilSD(P_ude);

      //Læser batteriniveau
      batteriniveau();
      standby_data[2] = Perc_bat;
      SkrivTilSD(Perc_bat);

      // I din kode kan du nu kalde denne funktion med dit array og dets størrelse:
      standby_str = sizeof(standby_data) / sizeof(standby_data[0]);

      float_til_char(standby_data, standby_str);

      // Send bufferen via UDP
      Send_til_platform(buffer);

      PdiffTjek();

    }  //Lukker tids-if-statement

    Data_fra_platform();  //Tjekker om der er data fra platformen
    //Input fra lækagesensorer
    TidNuLeak = millis();
    leak = digitalRead(leakPin);                                   // Read the Leak Sensor Pin
    if (leak == 1 && TidNuLeak - TidSidsteLeak >= IntervalLeak) {  //
      Udp.beginPacket(remoteIp, remotePort);
      Udp.write("Leak");
      Udp.endPacket();
      TidSidsteLeak = TidNuLeak;
    }

  }  //Lukker Standby-loop

  while (Main == true) {
    TidNuAktu = millis();
    if (AktuSetup == true && TidNuAktu - startTid >= AktuStartTid) {
      analogWrite(motorPWMOP, 0);  // Stop motoren
      AktuSetup == false;
    }
    leak = 0;                                //Nulstiller lækage informationen i tilfælde af fejl.
    if (millis() - tidSidste >= interval) {  //Da delay() funktionen giver en unødvendigt delay i datamodtagelse fra platform udnødvendigt forsinket køres loopet kun når x tid er gået
      tidSidste = millis();                  // Opdaterer variabel tidSidste til nuværende tid
      multiplexer.selectChannel(0);
      delay500ms();

      //Drifttiden logges
      Stopur();
      SkrivTilSD(tidGaaet);


      //Måling af det udvendige tryk
      multiplexer.selectChannel(1);  // for selecting SD0 and SC0
      Bar100.read();
      P_ude = Bar100.pressure();  //Opdater arrayet med den nye trykmåling
      delay500ms();
      multiplexer.selectChannel(0);  // for selecting SD1 and SC1 (Øvre)
      delay500ms();
      SkrivTilSD(P_ude);

      //Måling af det indvendige tryk

      Bar30.read();
      P_inde = Bar30.pressure();
      P_inde += adjP;
      delay500ms();
      SkrivTilSD(P_inde);



      //Temperatur samt CO2 måling fra SCD30
      SCD30.read();
      CO2 = SCD30.CO2;
      SCD30_temp = SCD30.temperature;

      delay500ms();
      SkrivTilSD(CO2);
      SkrivTilSD(SCD30_temp);


      //Ude temperatur fra TSYS01
      TSYS.read();
      T_ude = TSYS.temperature();

      // Check for NaN or extreme values
      if (isnan(T_ude) || T_ude < 0 || T_ude > 100) {
        T_ude = 99;  // Set default value for humidity
      }
      delay500ms();
      SkrivTilSD(T_ude);



      //O2 måling
      O2 = EZO_sensorer(O2_addr);
      delay500ms();
      SkrivTilSD(O2);

      // //EC måling
      EC = EC_float();
      delay500ms();
      SkrivTilSD(EC);


      //Temperatur samt relativfugtigheds måling fra 1899 - HTU sensor
      RH = htu.readHumidity();
      SkrivTilSD(RH);

      htu_temp = htu.readTemperature();
      SkrivTilSD(htu_temp);


      //ADC til CH4 sensor
      adc3 = ads.readADC_SingleEnded(1);  // Læser ADC-kortets AIN3-indgang
      CH4 = adc3 * 0.1875;                //Regner de målte værdier om til millivolt.
      delay500ms();
      SkrivTilSD(CH4);


      //KODE TIL MIPEXSENSOR
      Serial1.write("DATA");
      Serial1.write('\r');
      tidLyt = millis();


      while ((millis() - tidLyt) < 100) {  // Der lyttes efter respons i 0,1 sekunder.
        if (Serial1.available()) {
          inByte = Serial1.read();

          if (inByte != '\r') {
            dataArray[arrayIndex] = inByte;  // Gemmer dataene i arrayet
            arrayIndex++;                    // Øger arrayets indeks
          } else {
            dataArray[arrayIndex] = '\0';  // Null-terminer arrayet
            arrayIndex = 0;                // Nulstiller array indeks
          }

          if (arrayIndex == 5) {  // Hvis der er foretaget 5 gennemkørsler
            dataArray[5] = '\0';  // Sørg for, at arrayet er korrekt afsluttet

            String MIPEX_S = String((char*)dataArray);  // Konverter til String
            int MIPEX_int = MIPEX_S.toInt();

            MIPEX = (float)MIPEX_int;  // Konverter til float
            SkrivTilSD(MIPEX);
          }
        }
      }
      delay500ms();

      //Batteriniveauet opdateres
      batteriniveau();
      delay500ms();
      SkrivTilSD(Perc_bat);



      //Array med main data
      float mainData[13] = { tidGaaet, P_ude, P_inde, RH, SCD30_temp, htu_temp, T_ude, CO2, O2, CH4, MIPEX, EC, Perc_bat };

      //Størrelse af main data array
      mainStr = sizeof(mainData) / sizeof(mainData[0]);

      //Float main data array konverteres til char
      float_til_char(mainData, mainStr);  //Konventerer float værdierne til char
      delay500ms();
      //Char array med main data sendes til platform
      Send_til_platform(buffer);  //Sender char bufferen via UDP
      //Ny linje til tekstdokument
      File dataFile = SD.open(fileName, FILE_WRITE);  // Åbner det oprettede dokument for skrivning
      dataFile.println("");                           // Linjeskift
      dataFile.close();                               // Lukker filen efter skrivning
      PdiffTjek();                                    //Tjekker om aktuatoren skal justere ift. det ønsket tryk differens
      multiplexer.selectChannel(1);                   // for selecting SD0 and SC0
      delay500ms();
    }  //Lukker tids IF-loop

    Data_fra_platform();  //Tjekker om der er data fra platformen

    //Input fra lækagesensorer
    TidNuLeak = millis();
    leak = digitalRead(leakPin);                                   // Read the Leak Sensor Pin
    if (leak == 1 && TidNuLeak - TidSidsteLeak >= IntervalLeak) {  //
      Udp.beginPacket(remoteIp, remotePort);
      Udp.write("Leak");
      Udp.endPacket();
      TidSidsteLeak = TidNuLeak;
    }  //Lukker LEAK loop
  }    //Lukker main loop
}  //Lukker void-loop