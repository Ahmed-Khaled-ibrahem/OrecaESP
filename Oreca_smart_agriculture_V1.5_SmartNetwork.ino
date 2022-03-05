/*
   EEPROM LOCATIONS
   ----------------

   EEPROM 0 >> Device Type  not used here
   EEPROM 1 >> First Load Status location
   EEPROM 2 >> Number of Restarts occured
   EEPROM 3 >> Number of FACTORY RESETs
   EEPROM 4 >> Smart Network Role
               'M' for Master, 'C' for SubMaster,  'S' for Slave
   EEPROM 5 >> Master ip only used by subMaster to check master live

   EEPROM 10 - 13 >> Local IP
   EEPROM 14 - 18 >> Gateway IP

   EEPROM 100 - 600 >> Loads Status Value     // Changing every restart by 6 for not exceeding 100,000 write to ESP8266

*/

#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <SoftwareSerial.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <time.h>
#include <DHT.h>
#include "DFRobot_EC.h"

// ************************************************************************************
#include <addons/RTDBHelper.h>

/* If work with RTDB, define the RTDB URL and database secret */
//#define DATABASE_URL "klix-2020.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
//#define DATABASE_SECRET "Tk5c4JryptxOBawjqV956Ud4rXTwipYufJo4AcSF"

#define DATABASE_URL "agritech-6bcfe-default-rtdb.firebaseio.com"
#define DATABASE_SECRET "P3emJtDiaObDBDU15Sz0f3c7TjsbqRcMvmFbj6Dh"

///* If work with RTDB, define the RTDB URL and database secret */
//#define DATABASE_URL "new-work-54a1f-default-rtdb.firebaseio.com"
//#define DATABASE_SECRET "bhV9QwIM5hpUxtEy2JRm7PQ8oMihEMa2xy8dZ4Q4"

/* Define the Firebase Data object */
FirebaseData fbdo;

/* Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

//********************************************************************

#define Clk 5     // D1=GPIO5          // D-flipFlop Clock
#define FFD1 4    // D2=GPIO4          // Flip Flop D1 
#define FFD2 0    // D3=GPIO0          // Flip Flop D2 
#define FFD3 2    // D4=GPIO2          // Flip Flop D3 
#define MS1 14    // D5=GPIO14         // Flip Flop D4          // Mux Selector 1 
#define MS2 12    // D6=GPIO12         // Flip Flop D5          // Mux Selector 2 
#define MS3 13    // D7=GPIO13         // Flip Flop D6          // Mux Selector 3 
#define MPin A0   // A0=ADC0           // Mux Common pin
#define RST 9     // SD2=GPIO9   ******* sholud be another pin ********  like mux buttons
#define led 8     // Tx=GPIO1


byte loadsPins[6] = {FFD1, FFD2, FFD3, MS1, MS2, MS3};      // Loads pins List for looping
bool Load[6];            // Ex: [1,0,1,1,1,0]               // Loads States
int LoadsLocation[6];    // Ex: [175,176,177,178,179,180]   // Memory locations
float Sensor[5];           // Ex: [200,10,7,80,600]           // Sensores Reading values

// Sensor[0] => Temperature
// Sensor[1] => Humidity air
// Sensor[2] => EC_Sensor
// Sensor[3] => Ph
// Sensor[4] => Humidity soil  || Moisture Sensor

DHT dht( MPin , DHT22 );
DFRobot_EC ec;


//***********************************************************************

const String FirmwareVer = {"1.0"};
#define URL_fw_Version "/ASa3ed/ESP12/master/version.txt"

#define URL_fw_Bin "https://raw.githubusercontent.com/ASa3ed/ESP12/master/newFW.bin"
const char* host = "raw.githubusercontent.com";

//#define URL_fw_Bin "https://firebasestorage.googleapis.com/v0/b/klix-2020.appspot.com/o/newFW.bin?alt=media&token=8bad48da-e4fb-45ea-ad52-45978dca07fe"
//const char* host = "firebasestorage.googleapis.com";

const int httpsPort = 443;

//**************************************************************************************************************************************

unsigned long firebasede = 0;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 5000;             // Period to check Firmware Updates in "milliseconds"
const long mini_interval = 1000;

//**************************************************************************************************************************************

WiFiServer WifiServer(555);

IPAddress LocalIP;
IPAddress Gateway;
IPAddress Subnet(255, 255, 255, 0);
IPAddress DNS(8, 8, 8, 8);
//IPAddress StaticIP(192, 168, 1, 200);

String Status;

String NodeStatus;
String LED_status;
String TcpOrder;
String FireOrder = "0";
String Order[4];
String defaultOrder[3];
//String MasterOrder = 0;
bool FireFlag[4];
int RestartNumber;
int RESETNumber;
char devicetype = 'G';
// Change for many devices
// S, Shutter                     => not used here
// R, Load ,Load (Regular Use)
// L, L2 >Load , L1 >Dimming      => not used here
// D, L2 >Dimming , L1 >Load      => not used here
// M, Dimming , Dimming           => not used here

char NetworkRole = 'M';  //  M => "Master" , C => "SubMaster" ,S =>  "Slave"
// save it to eeprom
bool SubMasterActive = false ;      // if true submaster get data from firebase
String SubMasterIP = "MasterOnly";   // Master ip only used by subMaster to check master live

HTTPClient http;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  EEPROM.begin(4096);

  pinMode(Clk, OUTPUT);

  pinMode(FFD1, OUTPUT);
  pinMode(FFD2, OUTPUT);
  pinMode(FFD3, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  pinMode(MPin, INPUT);
  pinMode(RST, INPUT);
  //pinMode(led, OUTPUT);

  //buttonReset();
  GetEEPROM();
  digitalWrite(Clk, 0);

  //----------------------------------------------------- Connect to WiFi --------------------------------------------
  int wifiwait = 0;
  int count = 0;
  WiFi.mode(WIFI_STA);

  //    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
  //    WL_IDLE_STATUS      = 0,
  //    WL_NO_SSID_AVAIL    = 1,
  //    WL_SCAN_COMPLETED   = 2,
  //    WL_CONNECTED        = 3,
  //    WL_CONNECT_FAILED   = 4,
  //    WL_CONNECTION_LOST  = 5,
  //    WL_DISCONNECTED     = 6

  if (WiFi.SSID() == 0) {
    while (WiFi.SSID() == 0) // ----------------------------- First Time >> No Credentials
    {
      Serial.println("First Time No Credentials.. Waiting for Smart Config ... ... ...");
      WiFi.beginSmartConfig();
      while (1) {
        delay(50);
        Devices(devicetype, defaultOrder, 0);
        if (WiFi.smartConfigDone()) {
          Serial.println("SmartConfig Success");
          break;
        }
      }
      WiFi.begin(WiFi.SSID(), WiFi.psk());

      while (WiFi.status() != WL_CONNECTED) {
        delay(20);
        Serial.print(".");
        Serial.print(WiFi.status());
        Devices(devicetype, defaultOrder, 0);
        if (digitalRead(RST) == HIGH) {
          FactoryReset();
        }
      }
    }
    // ----- Save IP address to EEPROM
    Serial.println("");
    Serial.print("Local ip : ");
    Serial.println(WiFi.localIP());


    LocalIP = WiFi.localIP();
    Gateway = WiFi.gatewayIP();

    Save_ip();


  } // -----------------------------------------------------------------------------------------

  else { // -------------------------------------- I have Credentials

    for (int i = 10; i < 14 ; i++)
    {
      LocalIP[i - 10] = EEPROM.read(i);
    }
    for (int i = 14; i < 18 ; i++)
    {
      Gateway[i - 14] = EEPROM.read(i);
    }

    Serial.println(LocalIP);
    Serial.println(Gateway);

    WiFi.config( LocalIP, DNS, Gateway, Subnet );

    WiFi.begin( WiFi.SSID(), WiFi.psk() );

    while (WiFi.status() != WL_CONNECTED) {
      delay(20);
      Serial.print(".");
      Serial.print(WiFi.status());
      Devices(devicetype, defaultOrder, 0);

      if (digitalRead(RST) == HIGH) {
        FactoryReset();
      }
    }
  }

  Serial.println("");
  Serial.println("Wireless Connection Established");

  //----------------------------------------------------- Firebase Begin & Send DeviceType --------------------------------------------

  /* Assign the database URL and database secret(required) */
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;

  Firebase.reconnectWiFi(true);

  /* Initialize the library with the Firebase authen and config */
  Firebase.begin(&config, &auth);

  //Or use legacy authenticate method
  //Firebase.begin(DATABASE_URL, DATABASE_SECRET);

  // Send Devicetype & # of Restarts to Firebase & # of RESETS
  FireFlag[1] = 1;
  FireFlag[2] = 1;
  FireFlag[3] = 1;

  WifiServer.begin();

  if (digitalRead(RST) == HIGH)
  {
    FactoryReset();
  }

  Sensors_init();
  Serial.println("... Starting ...");
  //ReadSensors();
  //PrintNuts();
}
//---------------------------------------------------------------------------------------------------------
//----------------------------------------------------- LOOP ----------------------------------------------
//---------------------------------------------------------------------------------------------------------

void loop()
{
  yield(); // For ESP8266 to not dump *************************** SEARCH FOR STABILITY
  delay(1); // a work around for prevent WDT :(

  while (WiFi.status() != WL_CONNECTED) {
    delay(20);
    Serial.print(".");
    Serial.print(WiFi.status());
    Devices(devicetype, defaultOrder, 0);

  }

  WiFiClient client = WifiServer.available();

  if (client) {
    if (client.connected()) {
      if (client.available() > 0) {

        TcpOrder = (client.readStringUntil('\r'));
        Serial.print("Packet: ");
        Serial.println(TcpOrder);

        TcpOrder.remove(0, 5);

        TcpOrder = TcpOrder.substring(0, TcpOrder.indexOf(' '));
        Serial.println("Request : " + TcpOrder);

        client.println("HTTP/1.1 200 OK\nContent-type:text/html\nConnection: close\n");


        //DeviceType
        if (TcpOrder == "DeviceType") {
          //String WifiRssi = String( WiFi.RSSI() );
          // if( WifiRssi.length() > 5){ WifiRssi = "-70";}

          //Serial.print(WiFi.RSSI());
          String WifiRssi = String( WiFi.RSSI() );
          String repo = String(devicetype) + "," + WifiRssi ;
          Serial.println(repo);
          client.println(repo);
          // Take care here if TCP FAILED code will continue anyway ! without acknowledgment to User !!!
          // Hint client.print() returns the number of bytes written,
          // #ofBytes = client.println("");
          // if #ofBytes then continue
        }

        //Status
        else if (TcpOrder == "NodeStatus") {
          ReadSensors();
          Status = "";
          for (int i = 0; i < 6; i++)
          {
            Status += String(Load[i]) + ",";
          }
          for (int i = 0; i < 5; i++)
          {
            Status += String(Sensor[i]) + ",";
          }

          client.println(Status);
          // Take care here if TCP FAILED code will continue anyway ! without acknowledgment to User !!!
          // Hint client.print() returns the number of bytes written,
          // #ofBytes = client.println("");
          // if #ofBytes then continue
        }

        //RESET
        else if (TcpOrder == "RESET") {
          client.println("RESET");
          // Take care here if TCP FAILED code will continue anyway ! without acknowledgment to User !!!
          // Hint client.print() returns the number of bytes written,
          // #ofBytes = client.println("");
          // if #ofBytes then continue
          client.stop();  // Remove !!!
          FactoryReset();
        }
        else if (TcpOrder == "UPDATE") {
          client.println("UPDATE");
          // Take care here if TCP FAILED code will continue anyway ! without acknowledgment to User !!!
          // Hint client.print() returns the number of bytes written,
          // #ofBytes = client.println("");
          // if #ofBytes then continue
          client.stop();  // Remove !!!
          FirmwareUpdate();
        }
        //Action Operation
        else if (TcpOrder == "Live") {
          client.println( "Yes" );
          client.stop();
          SubMasterActive = false ;
          previousMillis = millis();
        }

        else {
          client.println(TcpOrder);
          // Take care here if TCP FAILED code will continue anyway ! without acknowledgment to User !!!
          // Hint client.print() returns the number of bytes written,
          // #ofBytes = client.println("");
          // if #ofBytes then continue
          client.stop();  // Remove !!!

          SplitOrder(TcpOrder, Order, ',');
          Serial.println("TCPOrder >>> " + Order[0] + " : " + Order[1] + " : " + Order[2] + " : " + Order[3]);

          if (Order[0] == "networkType") {
            if ( Order[1] == "Master" ) {
              Serial.println("Device converted to master");
              //client.println( TcpOrder );
              NetworkRole = 'M';
              EEPROM.write(4, 'M');
              EEPROM.commit();

              SubMasterIP = Order[2];

              //client.stop();
            }
            else if ( Order[1] == "SubMaster") {
              Serial.println("Device converted to Submaster");
              //client.println( TcpOrder );
              NetworkRole = 'C';

              EEPROM.write(4, 'C');
              EEPROM.commit();
              //client.stop();
            }
            else if (Order[1] == "Slave") {
              Serial.println("Device converted to Slave");
              // client.println( TcpOrder );
              NetworkRole = 'S';
              EEPROM.write(4, 'S');
              EEPROM.commit();
              //client.stop();
            }
            else {
              Serial.println("Bad network type request");
            }
          }

          else if ( Order[0] == "GenIp" ) {
            String ipString = Order[1];
            String oct[4] ;
            SplitOrder( ipString, oct, '.');

            LocalIP[0] = oct[0].toInt();
            LocalIP[1] = oct[1].toInt();
            LocalIP[2] = oct[2].toInt();
            LocalIP[3] = oct[3].toInt();

            Save_ip();
          }

          else {
            Devices(devicetype, Order , 0);
          }
        }
        client.stop();  // Remove !!!
      }
      TcpOrder = "";
      Devices(devicetype, defaultOrder, 0);
    }
  }

  if ( NetworkRole == 'M' || ( SubMasterActive && NetworkRole == 'C' ) ) {
    // Master sholuld only ping to firebase
    // if master fail subMaster will ping to firebase

    if (Firebase.getString(fbdo, WiFi.macAddress() + "/FireOrder"))
    {
      FireOrder = fbdo.to<const char *>();
      FireOrder = String(FireOrder);
    }

    else
    {
      Serial.print("FireOrder Error: ");
      Serial.println(fbdo.errorReason().c_str());
    }
  }

  if ( FireOrder != "0" ) {
    if (FireOrder == "RESET") {
      Serial.printf("Set RESET... %s\n", Firebase.setBool(fbdo, WiFi.macAddress() + "/ResetAck", 1) ? "ok" : fbdo.errorReason().c_str());
      // Take care here if set firebase FAILED code will continue anyway ! without acknowledgment to User !!!

      FactoryReset();
    }
    if (FireOrder == "UPDATE") {
      Serial.printf("Set FW_UPDATE... %s\n", Firebase.setBool(fbdo, WiFi.macAddress() + "/FW_UPDATE", 1) ? "ok" : fbdo.errorReason().c_str());
      // Take care here if set firebase FAILED code will continue anyway ! without acknowledgment to User !!!

      //SendHTTPOrder("192.168.1.3", FireOrder);

      FirmwareUpdate();
    }

    //Action Operation
    else {
      Serial.println("FireOrder: " + FireOrder);
      SplitOrder(FireOrder, Order, ',');

      if (Order[3] == "Self") {
        Devices(devicetype, Order, 1);
      }
      else {
        //Devices(devicetype, Order, 1);  // remove it
        SendHTTPOrder(client, Order[3], FireOrder);
      }
    }
    FireOrder = "0";   // Dont forget to clear from flutter also after check Acknoledgement on NodeStatus
  }

  //Devices IO
  Devices(devicetype, defaultOrder, 0);
  FireReport();

  if (digitalRead(RST) == HIGH) {
    FactoryReset();
  }

  //  if( NetworkRole == 'C' ){
  //          Serial.println("\n\n Make http request for master ");
  //          String Target = "http://"+MasterIP+":555/" + "Live" ;
  //          http.begin(client, Target.c_str());
  //
  //          int httpResponseCode = http.GET();
  //          Serial.print( "Master httpResponseCode : " );
  //            Serial.println( httpResponseCode );
  //
  //            if(httpResponseCode != 200){
  //               SubMasterActive = true;
  //              }
  //              else{
  //                SubMasterActive = false;
  //                }
  //          http.end();
  //    }

  if ( NetworkRole == 'p' &&  SubMasterIP != "MasterOnly") {
    Serial.println("\n\n Make http request for Submaster ");
    String Target = "http://" + SubMasterIP + ":555/" + "Live" ;
    http.begin(client, Target.c_str());

    int httpResponseCode = http.GET();
    Serial.print( "Master httpResponseCode : " );
    Serial.println( httpResponseCode );
    http.end();
  }

  if ( millis() - previousMillis  > interval) { // after 15 second subMaster Become Master
    SubMasterActive = true ;
  }
}

//----------------------------------------------------# Functions # -----------------------------------------

//---------------------------------------------------- Spliting Order -----------------------------------------
String *SplitOrder(String Order, String ArrOrder[4], char spliter) {

  String DType = Order.substring(0, Order.indexOf(spliter));
  Order.remove(0, Order.indexOf(spliter) + 1);
  //  Serial.println(DType);
  String DChannel = Order.substring(0, Order.indexOf(spliter));
  Order.remove(0, Order.indexOf(spliter) + 1);
  //  Serial.println(DChannel);
  String DValue = Order.substring(0, Order.indexOf(spliter));
  Order.remove(0, Order.indexOf(spliter) + 1);
  //  Serial.println(DValue);
  String Net_Code = Order.substring(0, Order.indexOf(spliter));

  ArrOrder[0] = DType;
  ArrOrder[1] = DChannel;
  ArrOrder[2] = DValue;
  ArrOrder[3] = Net_Code;

  return ArrOrder;
}
//--------------------------------------------------- Firebase Reporter ---------------------------------------
void FireReport()
{
  yield();  // For ESP8266 to not dump

  if (FireFlag[0] == 1)
  {
    ReadSensors();
    Status = "";
    for (int i = 0; i < 6; i++)
    {
      Status += String(Load[i]) + ",";
    }
    for (int i = 0; i < 5; i++)
    {
      Status += String(Sensor[i]) + ",";
    }


    if (Firebase.setString(fbdo, WiFi.macAddress() + "/NodeStatus", Status)) {
      FireFlag[0] = 0;
      Serial.println("FireReporter >>> Node Status : " + Status + " ... Reported");
    }
    else {
      Serial.print("FireReporter Error : ");
      Serial.println(fbdo.errorReason().c_str());
    }
  }


  if (FireFlag[1] == 1)
  {
    if (Firebase.setString(fbdo, WiFi.macAddress() + "/DeviceType", String(devicetype) + "," + String( WiFi.RSSI()))) {
      FireFlag[1] = 0;
      Serial.println("FireReporter >>> DeviceType : " + String(devicetype) + " ... Reported");
    }
    else {
      Serial.print("FireReporter Error : ");
      Serial.println(fbdo.errorReason().c_str());
    }
  }

  if (FireFlag[2] == 1)
  {
    if (Firebase.setInt(fbdo, WiFi.macAddress() + "/RestartNumber", RestartNumber)) {
      FireFlag[2] = 0;
      Serial.println("FireReporter >>> RestartNumber : " + String(RestartNumber) + " ... Reported");
    }
    else {
      Serial.print("FireReporter Error : ");
      Serial.println(fbdo.errorReason().c_str());
    }
  }

  if (FireFlag[3] == 1)
  {
    if (Firebase.setInt(fbdo, WiFi.macAddress() + "/RESET", RESETNumber)) {
      FireFlag[3] = 0;
      Serial.println("FireReporter >>> RESET : " + String(RESETNumber) + " ... Reported");
    }
    else {
      Serial.print("FireReporter Error : ");
      Serial.println(fbdo.errorReason().c_str());
    }
  }

}

//--------------------------------------------------- FACTORY RESET ---------------------------------------
void FactoryReset()
{
  return;
  Serial.println("**************** Starting FACTORY RESET ****************");
  delay(1000);


  EEPROM.write(3, RESETNumber + 1);

  WiFi.disconnect();
  ESP.eraseConfig();

  EEPROM.write(1, 100); // Status_location

  for (int i = 10; i <= 600; i++)   // From 1 if you need store devicetype
  {
    EEPROM.write(i, 0);
    Serial.print("Wrote: ");
    Serial.println(i);
  }

  EEPROM.commit();    //Store data to EEPROM
  //  ESP.reset();
  ESP.restart();    // just restart

}

//---------------------------------------------------- Read from EEPROM ----------------------------------------
void GetEEPROM()
{
  //  devicetype = char(EEPROM.read(0));
  //  Serial.print("DeviceType: ");
  //  Serial.println(devicetype);

  // ------------------------------------------
  LoadsLocation[0] = int(EEPROM.read(1));
  for (byte i = 1; i < 6; i++) {
    LoadsLocation[i] =  LoadsLocation[i - 1] + 1;
  }

  // ------------------------------------------
  Serial.println(" Initial Load States : ");
  for (byte i = 0; i < 6; i++) {
    Load[i] = int( EEPROM.read(LoadsLocation[i]) );
    MuxIO( 'O', i, Load[i] );  /////////////////         **************  change after for
    Serial.println(" Load: " + String(i) + " = " + String(Load[i]));
  }
  // ------------------------------------------
  RestartNumber = int(EEPROM.read(2));
  Serial.println("RestartNumber: " + String(RestartNumber));
  EEPROM.write(2, RestartNumber + 1);
  // ------------------------------------------
  RESETNumber = int(EEPROM.read(3));
  Serial.println("RESETNumber: " + String(RESETNumber));

  // ------------------------------------------
  if ( LoadsLocation[5] <= 594 ) {
    for (byte i = 0; i < 6; i++) {
      LoadsLocation[i] = LoadsLocation[i] + 6;
      EEPROM.write(LoadsLocation[i], Load[i]);
    }
    EEPROM.write(1, LoadsLocation[0]);

  }

  else {
    for (byte i = 0; i < 6; i++) {
      LoadsLocation[i] = i + 100;
      EEPROM.write(LoadsLocation[i], Load[i]);
    }

    EEPROM.write(1, LoadsLocation[0]);
  }

  // ------------------------------------------
  char network_stat = char(EEPROM.read(4));
  if (network_stat == 'M' || network_stat == 'C' || network_stat == 'S') {
    NetworkRole = network_stat;
  }
  //------------------------------------------
  EEPROM.commit();
}

//--------------------------------------------------- Devices IO ------------------------------------------------
void Devices(char devicetype, String XOrder[3], bool FireOrTCP)
{
  yield();  // For ESP8266 to not dump

  if (analogRead(A0) > 150) {
    buttonReset();
  }

  if (XOrder[0] == "D") // ---------------------------------------- Digital Outputs
  {
    String LoadNo = XOrder[1].substring(1);
    Serial.println("Load number: " + LoadNo);

    MuxIO('O', byte(LoadNo.toInt()), XOrder[2].toInt());
    Load[LoadNo.toInt()] = XOrder[2].toInt();

    EEPROM.write( LoadsLocation[LoadNo.toInt()], Load[LoadNo.toInt()]);
    EEPROM.commit();
    Serial.println("Load >> " + XOrder[1] + " Value: " + XOrder[2] + " In Location: " + String(LoadsLocation[LoadNo.toInt()]));
    FireFlag[0] = 1;
    if (FireOrTCP == 1) {
      FireReport();
    }
  }

  // else {        // ----------------------------------------------- 5 switches/plugs/loads
  //
  //  for(byte i=0; i<3; i++){
  //   //                        ----------- Looping in all 3 switchs --------------
  //   if ( MuxIO('S',i,0) >= 150 )
  //    {
  //        Load[i] = !Load[i];
  //        MuxIO('O',i, Load[i]);
  //
  //        EEPROM.write( LoadsLocation[i], Load[i] );
  //        EEPROM.commit();
  //        Serial.println("Load "+ String(i)+ " is "+ String(Load[i]) +" In Location: " + String(LoadsLocation[i]));
  //        FireFlag[0] = 1;
  //        if (FireOrTCP == 1) {
  //        FireReport();
  //        }
  //
  //      while (MuxIO('S',i,0) >= 150) {
  //        delay(1);
  //      }
  //      if(i==2){
  //        buttonReset();
  //        }
  //    }
  //   }
  //  }

}

//--------------------------------------------------- Print Nuts ------------------------------------------------
void PrintNuts()
{
  Serial.println("*****************************************");
  Serial.println("************* Hello ESP8266 *************");
  Serial.println("*****************************************");
  WiFi.printDiag(Serial);
  Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
  Serial.printf("Password: %s\n", WiFi.psk().c_str());
  Serial.printf("MacAddress: %s\n", WiFi.macAddress().c_str());
  Serial.printf("BSSID: %s\n", WiFi.BSSIDstr().c_str());
  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  Serial.printf("Gataway IP: %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.print("DNS #1, #2 IP: ");
  WiFi.dnsIP().printTo(Serial);
  Serial.print(", ");
  WiFi.dnsIP(1).printTo(Serial);
  Serial.println();
  Serial.print("Local ip : ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet: ");
  //Serial.println(WiFi.subnet());
  Serial.printf("Default hostname: %s\n", WiFi.hostname().c_str());
  WiFi.hostname("OrecaTech");
  Serial.printf("New hostname: %s\n", WiFi.hostname().c_str());
  Serial.printf("Flash Chip Real Size : %d\n", ESP.getFlashChipRealSize());
  Serial.println("*****************************************");
  Serial.println("***************** Nuts ******************");
  Serial.println("*****************************************");
}
//--------------------------------------------------- Firmware Update Functions ------------------------------------------------
//--------------------------------------------------- Firmware Update Functions ------------------------------------------------
//--------------------------------------------------- Firmware Update Functions ------------------------------------------------
//------------------------------------------------- Firmware Update (Core_Function) --------------------------------------------
void FirmwareUpdate()
{
  yield();
  delay(1);

  WiFiClientSecure client;
  //  client.setTrustAnchors(&cert);
  client.setInsecure();
  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed");
    return;
  }
  client.print(String("GET ") + URL_fw_Version + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("Headers received");
      break;
    }
  }
  String payload = client.readStringUntil('\n');

  payload.trim();
  if (payload.equals(FirmwareVer) )
  {
    Serial.println("Device already on latest firmware version");
  }
  else
  {
    Serial.println("New firmware detected");
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, URL_fw_Bin);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}



//------------------------------------------------- Multiplexer Input Output --------------------------------------

/*
           ------------  Connections  ----------

  MUX:   inputs    outputs
        MPin     DHTSensor      y0
        MS1      EC_Sensor      y1
        MS2      PHSensor       y2
        MS3      Soil_Moisture  y3
                 Sensor 5       y4
                 Switch 1       y5
                 Switch 2       y6
                 Switch 3       y7


*/

//  ----------  Switchs logic  -------
// We Have 3 Real Switchs control 6 loads
// - to control Load(1) we Push Switch(1)
// - to control Load(2) we Push Switch(2)
// - to control Load(3) we Push Switch(3)
// - to control Load(4) we Push Switch(1) for a while  "not implemented yet"
// - to control Load(5) we Push Switch(2) for a while  "not implemented yet"
// - to control Load(6) we Push Switch(3) for a while  "not implemented yet"



// ------------- How to Use MuxIO --------------
// --- (#) represent the index Start from 0 ---

// we have 5 Sensors
// read Sensor 1 =>   MuxIO('N',1,0);
// read Sensor # =>   MuxIO('N',#,0);

// we have 3 Switchs
// read Switch #  =>  MuxIO('S',#,0);

// we have 6 outputs
// Write to output #  =>  MuxIO('O',#,value);

// Selectors table
// index   M1S3 M1S2 M1S1
//   1      000
//   2      001
//   3      010
//   4      011
//   5      100
//   6      101
//   7      110
//   8      111

int MuxIO(char type, byte index, bool value) {
  int Reading = 0;
  //Serial.println("\n --  MuxIO Function  -- ");

  if ( type == 'N' ) {
    // write Selectors
    digitalWrite(MS1, index == 1 || index == 3);
    digitalWrite(MS2, index == 2 || index == 3);
    digitalWrite(MS3, index == 4);
    //delay(1);
    Reading = analogRead(MPin);
    Serial.print("Sensor ("); Serial.print(index); Serial.print(") is  "); Serial.println(Reading);
    return Reading;
  }

  else if ( type == 'S') {
    // write Selectors
    digitalWrite(MS1, index != 1 );
    digitalWrite(MS2, index != 0 );
    digitalWrite(MS3, 1);
    //delay(1);
    Reading = analogRead(MPin);
    //Serial.print("Switch (");Serial.print(index);Serial.print(") is  ");Serial.println(Reading);
    return Reading;
  }

  else {
    // -----  output to D-flipFlop

    digitalWrite(Clk, 0);   // Making Sure the clk is low
    Load[index] = value;

    for (byte i = 0; i < 6 ; i++) {
      digitalWrite( loadsPins[i] , Load[i] );
    }
    digitalWrite(Clk, 1);
    delay(5);
    digitalWrite(Clk, 0);

    Serial.print("Output ("); Serial.print(index); Serial.print(") is  "); Serial.println(value);
    return 5;
  }
  Serial.print("Wrong Call for MuxIO Function");
  return 0 ;
}

void ReadSensors() {
  yield();

  MuxIO('N', 0, 0);               // prepare selectors to read from DHT sensor
  float readTemp = dht.readTemperature();     // Read temperature as Fahrenheit (isFahrenheit = true)
  if (isnan(readTemp)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  else {
    Sensor[0] = readTemp;
    Serial.println("Dht reading done");
  }

  // Read Humidity
  float readHum = dht.readHumidity();
  if (isnan(readHum)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  else {
    Sensor[1] = readHum;
    Serial.println("Dht reading done");
  }
  ////----------------------------------------

  MuxIO('N', 1, 0);
  float temperature = 25;
  int voltage = analogRead(MPin) / 1024.0 * 5000; // read the voltage
  //float temperature = readTemperature();             // read our temperature sensor
  float ecValue =  ec.readEC(voltage, temperature); // convert voltage to EC with temperature
  Sensor[2] = ecValue;
  ec.calibration(voltage, temperature);

  ////----------------------------------------
  int Value = MuxIO('N', 2, 0);
  float ph_voltage = Value * (3.3 / 4095.0);
  float ph = (3.3 * ph_voltage);
  Sensor[3] = ph;
  ////----------------------------------------
  int moisure_volt = MuxIO('N', 3, 0);
  float moisture_percentage = ( 100.00 - ( (moisure_volt / 1023.00) * 100.00 ) );
  Sensor[4] = moisture_percentage;
}

void SendHTTPOrder(WiFiClient client, String IP, String Order) {

  Serial.println("\n\n Make http resgest");

  String Target = "http://" + IP + ":555/" + Order ;
  http.begin(client, Target.c_str());

  int httpResponseCode = http.GET();
  Serial.print( "httpResponseCode : " );
  Serial.println( httpResponseCode );
  http.end();
}

void buttonReset() {

  Serial.println("**************** Starting FACTORY RESET ****************");
  delay(1000);


  EEPROM.write(3, RESETNumber + 1);

  WiFi.disconnect();
  ESP.eraseConfig();

  EEPROM.write(1, 100); // Status_location

  for (int i = 10; i <= 600; i++)   // From 1 if you need store devicetype
  {
    EEPROM.write(i, 0);
    Serial.print("Wrote: ");
    Serial.println(i);
  }

  EEPROM.commit();    //Store data to EEPROM
  //  ESP.reset();
  ESP.restart();    // just restart

}

void Save_ip() {

  for (int i = 10; i < 14; i++)
  {
    EEPROM.write(i, LocalIP[i - 10]);
    Serial.print("Wrote: ");
    Serial.println(LocalIP[i - 10]);
  }
  for (int i = 14; i < 18; i++)
  {
    EEPROM.write(i, Gateway[i - 14]);
    Serial.print("Wrote: ");
    Serial.println(Gateway[i - 14]);
  }

  EEPROM.commit();    //Store data to EEPROM
}

void Sensors_init() {
  MuxIO('N', 0, 0);
  dht.begin();
  MuxIO('N', 1, 0);
  ec.begin();
}
"# OrecaESP" 
