# 1 "/var/folders/1p/qkhbg0_50g74ggt0v9m9r54r0000gn/T/tmp__y25a__"
#include <Arduino.h>
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
#if defined (ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ESP32_ARCH 1
#elif defined(ARDUINO_ARCH_ESP8266)

#else
# error "Architecture unknown and not supported"
#endif




#include "configGway.h"
#include "configNode.h"

#include <Esp.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <string>

#include <SPI.h>
#include <Wire.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include "GatewayBase64.h"

#if defined(ESP32_ARCH) && (_BOARD_PROFILE == BOARD_PROFILE_HELTEC_WIFI_LORA)
#define USE_HELTEC_DISPLAY 1
#endif


#include "loraModem.h"
#include "loraFiles.h"
#include "oLED.h"

extern "C" {
# include "lwip/err.h"
# include "lwip/dns.h"
}

#if (_GATEWAYNODE==1) || (_LOCALSERVER>=1)
# include "AES-128_V10.h"
#endif


#if defined(ESP32_ARCH)
# include <WiFi.h>
# include <ESPmDNS.h>
# include <SPIFFS.h>
# include <WebServer.h>
# include <DNSServer.h>
# include <WiFiManager.h>

# if (_BOARD_PROFILE == BOARD_PROFILE_HELTEC_WIFI_LORA)
# include "heltec.h"
# endif

# ifdef ESP_getChipId
#undef ESP_getChipId
# endif
#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())


# if _SERVER==1
# include <Streaming.h>
  WebServer server(_SERVERPORT);
# endif

# if _OTA==1

# include <ArduinoOTA.h>
# endif



#elif defined(ARDUINO_ARCH_ESP8266)
 extern "C" {
# include "user_interface.h"
# include "c_types.h"
 }
# include <ESP8266WiFi.h>
# include <ESP8266mDNS.h>
# include <DNSServer.h>
# include <ESP8266WebServer.h>
# include <WiFiManager.h>

# ifdef ESP_getChipId
#undef ESP_getChipId
# endif
#define ESP_getChipId() (ESP.getChipId())

# if _SERVER==1
# include <Streaming.h>
  ESP8266WebServer server(_SERVERPORT);
# endif

# if _OTA==1
# include <ESP8266httpUpdate.h>
# include <ArduinoOTA.h>
# endif


#else
# error "Architecture not supported"

#endif




uint8_t debug=1;
uint8_t pdebug= P_MAIN ;
uint32_t txDones=0;

#if _GATEWAYNODE==1
# if _GPS==1
# include <TinyGPS++.h>
  TinyGPSPlus gps;
  HardwareSerial sGps(1);
# endif
#endif

using namespace std;

bool sx1276 = true;
uint8_t MAC_array[6];
uint8_t currentMode = 0x81;







uint8_t protocol = _PROTOCOL;


sf_t sf = _SPREADING;




float lat = _LAT;
float lon = _LON;
int alt = _ALT;
char platform[24] = _PLATFORM;
char email[40] = _EMAIL;
char description[64]= _DESCRIPTION;


StaticJsonDocument<312> jsonBuffer;



IPAddress ntpServer;
IPAddress ttnServer;
IPAddress thingServer;

WiFiUDP Udp;

time_t startTime = 0;
uint32_t eventTime = 0;
uint32_t sendTime = 0;
uint32_t doneTime = 0;
uint32_t statTime = 0;
uint32_t pullTime = 0;
uint32_t rstTime = 0;
uint32_t fileTime = 0;

#define TX_BUFF_SIZE 1024
#define RX_BUFF_SIZE 1024
#define STATUS_SIZE 512

#if _SERVER==1
 uint32_t wwwtime = 0;
#endif
#if _NTP_INTR==0
 uint32_t ntptimer = 0;
#endif
#if _GATEWAYNODE==1
 uint16_t LoraUp.fcnt = 0;
 uint16_t LoraDown.fcnt = 0;
#endif
#ifdef _PROFILER
 uint32_t endTmst=0;
#endif



uint16_t iMoni=0;
uint16_t iSeen=0;
uint16_t iSens=0;




int16_t mutexSPI = 1;

uint8_t buff[64];
uint8_t buff_down[TX_BUFF_SIZE];
IPAddress remoteIpNo;
unsigned int remotePortNo;
# 224 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
void ICACHE_RAM_ATTR Interrupt_0();
void ICACHE_RAM_ATTR Interrupt_1();

int sendPacket(uint8_t *buf, uint8_t len);

void printIP(IPAddress ipa, const char sep, String & response);
void setupWWW();

void mPrint(String txt);
int getNtpTime(time_t *t);
int mStat(uint8_t intr, String & response);
void SerialStat(uint8_t intr);
void printHexDigit(uint8_t digit, String & response);
int inDecodes(char * id);
static void stringTime(time_t t, String & response);

int initMonitor(struct moniLine *monitor);
void initConfig(struct espGwayConfig *c);
int printSeen(const char *fn, struct nodeSeen *listSeen);
int readGwayCfg(const char *fn, struct espGwayConfig *c);

void init_oLED();
void acti_oLED();
void addr_oLED();
void msg_oLED(String mesg);

void setupOta(char *hostname);

void initLoraModem();
void rxLoraModem();
void writeRegister(uint8_t addr, uint8_t value);
void cadScanner();
void startReceiver();

void stateMachine();

bool connectUdp();
int readUdp(int packetSize);
int sendUdp(IPAddress server, int port, uint8_t *msg, uint16_t length);
void sendStat();
void pullData();

#if _MUTEX==1
 void ICACHE_FLASH_ATTR CreateMutux(int *mutex);
 bool ICACHE_FLASH_ATTR GetMutex(int *mutex);
 void ICACHE_FLASH_ATTR ReleaseMutex(int *mutex);
#endif
# 282 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
void setup();
void loop ();
int WlanStatus();
int wifiMgr();
void configModeCallback (WiFiManager *myWiFiManager);
int WlanConnect(int maxTry);
IPAddress resolveHost(String svrName, int maxTry);
void gateway_mgt(uint8_t size, uint8_t *buff);
void id_print (String id, String val);
int readConfig(const char *fn, struct espGwayConfig *c);
int writeGwayCfg(const char *fn, struct espGwayConfig *c);
int writeConfig(const char *fn, struct espGwayConfig *c);
int addLog(const unsigned char * line, int cnt);
int initSeen(struct nodeSeen *listSeen);
int readSeen(const char *fn, struct nodeSeen *listSeen);
int addSeen(struct nodeSeen *listSeen, struct stat_t stat);
uint8_t readRegister(uint8_t addr);
void writeBuffer(uint8_t addr, uint8_t *buf, uint8_t len);
void setRate(uint8_t sf, uint8_t crc);
void setFreq(uint32_t freq);
void setPow(uint8_t pow);
void opmode(uint8_t mode);
void hop();
uint8_t receivePkt(uint8_t *payload);
void initDown(struct LoraDown *LoraDown);
bool sendPkt(uint8_t *payLoad, uint8_t payLength);
int loraWait(struct LoraDown *LoraDown);
void txLoraModem(struct LoraDown *LoraDown);
void ICACHE_RAM_ATTR Interrupt_2();
void msg_lLED(String mesg, String mesg2);
void updateOtaa();
int repeatLora(struct LoraUp * LUP);
void smartDelay(uint32_t ms);
int LoRaSensors(uint8_t *buf);
void mXor(uint8_t *buf, uint8_t *key);
void shift_left(uint8_t * buf, uint8_t len);
void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2);
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * NwkSKey, uint8_t dir);
void checkMic(uint8_t *buf, uint8_t len, uint8_t *key);
int sensorPacket();
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction);
void connectTtn();
int readTtn(int Packetsize);
int sendTtn(IPAddress server, int port, uint8_t *msg, int length);
int buildPacket(uint8_t *buff_up, struct LoraUp *LoraUp, bool internal);
int receivePacket();
void printInt (uint32_t i, String & response);
void printRegs(struct LoraDown *LoraDown, String & response);
void printDwn(struct LoraDown *LoraDown, String & response);
void printHex(uint32_t hexa, const char sep, String & response);
void ftoa(float f, char *val, int p);
void printDigits(uint32_t digits);
int sendNtpRequest(IPAddress timeServerIP);
void setupTime();
int SerialName(uint32_t a, String & response);
void die(String s);
void gway_failed(const char *file, uint16_t line);
boolean YesNo();
boolean GoOn();
void wwwFile(String fn);
void buttonDocu();
void buttonRegs();
void buttonLog();
static void wwwButtons();
static void setVariables(const char *cmd, const char *arg);
static void openWebPage();
static void gatewaySettings();
static void statisticsData();
static void messageHistory();
static void nodeHistory();
void monitorData();
static void wifiConfig();
static void systemStatus();
static void interruptData();
void sendWebPage(const char *cmd, const char *arg);
static void websiteFooter();
#line 282 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
void setup() {

 char MAC_char[19];
 MAC_char[18] = 0;
 char hostname[12];

 initDown(&LoraDown);
 initConfig(&gwayConfig);

# if defined(ESP32_ARCH) && (_BOARD_PROFILE == BOARD_PROFILE_HELTEC_WIFI_LORA)
 Heltec.begin(false , true , true , true , 868E6);
# endif

# if _DUSB>=1
  Serial.println("serial begin 115200");
  Serial.flush();
# endif

# if _OLED>=1
  init_oLED();
# endif

# if _GPS==1

  sGps.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
# endif

 delay(500);

 bool spiffsOk = false;
# if defined(ESP32_ARCH)
 spiffsOk = SPIFFS.begin(false);
 if (!spiffsOk) {
  mPrint(F("SPIFFS mount failed, attempting format"));
  SPIFFS.end();
  SPIFFS.format();
  spiffsOk = SPIFFS.begin(false);
 }
 if (!spiffsOk) {
  spiffsOk = SPIFFS.begin(true);
 }
# else
 spiffsOk = SPIFFS.begin();
# endif

 if (spiffsOk) {
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_MAIN)) {
   mPrint(F("SPIFFS begin"));
  }
# endif
 }
 else {
  mPrint(F("SPIFFS mount failed"));
  msg_oLED("NO FS");
  while (true) {
   delay(1000);
  }
 }


# if _SPIFFS_FORMAT>=1
 msg_oLED("FORMAT");
 SPIFFS.format();
 delay(500);
 initConfig(&gwayConfig);
 gwayConfig.formatCntr++;
 if ((debug>=1) && (pdebug & P_MAIN)) {
  mPrint("SPIFFS Format Done");
 }
# endif

# if _MONITOR>=1
  msg_oLED("MONITOR");
  initMonitor(monitor);

# if defined CFG_noassert
   mPrint("No Asserts");
# else
   mPrint("Do Asserts");
# endif
# endif

 delay(500);






 if (readGwayCfg(_CONFIGFILE, &gwayConfig) > 0) {
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("readGwayCfg:: return OK");
  }
# endif
 }
 else {
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("setup:: readGwayCfg: ERROR readGwayCfg Failed");
  }
# endif
 };
 delay(500);

# if _WIFIMANAGER==1
  msg_oLED("WIFIMGR");
# if _MONITOR>=1
   mPrint(F("setup:: WiFiManager"));
# endif
  delay(500);

  wifiMgr();


  if (WiFi.status() != WL_CONNECTED) {
   mPrint("setup:: WiFiManager failed, forcing AP mode");
   WiFi.disconnect(true, true);
   delay(200);
   WiFi.mode(WIFI_AP);
   bool apStarted = WiFi.softAP(AP_NAME, AP_PASSWD, 1, 0);
   IPAddress apIP(192, 168, 4, 1);
   WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
   char ipBuf[16];
   strncpy(ipBuf, apIP.toString().c_str(), sizeof(ipBuf));
   ipBuf[sizeof(ipBuf)-1] = '\0';
   mPrint(String("setup:: AP start ") + (apStarted ? "OK" : "FAILED"));
   msg_lLED(apStarted ? String("AP ")+String(AP_NAME) : String("AP FAIL"), String(ipBuf));
   if (!apStarted) {
    return;
   }
  }
# endif

 msg_oLED("WIFI STA");
 WiFi.mode(WIFI_STA);
 WiFi.setAutoReconnect(true);
 WiFi.macAddress(MAC_array);
    sprintf(MAC_char,"%02x:%02x:%02x:%02x:%02x:%02x",
  MAC_array[0],MAC_array[1],MAC_array[2],MAC_array[3],MAC_array[4],MAC_array[5]);

# if _MONITOR>=1
  mPrint("MAC: " + String(MAC_char) + ", len=" + String(strlen(MAC_char)) );
# endif



 while (WlanConnect(0) <= 0) {
# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_MAIN)) {
   mPrint("setup:: Error Wifi network connect(0)");
  }
# endif
 }

 yield();

# if _MONITOR>=1
 if ((debug>=1) & (pdebug & P_MAIN)) {
  mPrint("setup:: WlanConnect="+String(WiFi.SSID()) );
 }
# endif


# if defined(ESP32_ARCH)

  sprintf(hostname, "%s%02x%02x%02x", "esp32-", MAC_array[3], MAC_array[4], MAC_array[5]);
  WiFi.setHostname(hostname);
  MDNS.begin(hostname);
# else

  sprintf(hostname, "%s%02x%02x%02x", "esp8266-", MAC_array[3], MAC_array[4], MAC_array[5]);
  wifi_station_set_hostname(hostname);
# endif

# if _MONITOR>=1
 if (debug>=0) {
  String response = "Host=";
# if defined(ESP32_ARCH)
   response += String(WiFi.getHostname());
# else
   response += String(wifi_station_get_hostname());
# endif

  response += " WiFi Connected to " + String(WiFi.SSID());
  response += " on IP=" + String(WiFi.localIP().toString() );
  mPrint(response);
 }
# endif

 delay(500);


# if _UDPROUTER==1

  if (!connectUdp()) {
# if _MONITOR>=1
    mPrint("Error connectUdp");
# endif
  }
# elif _TTNROUTER==1
  if (!connectTtn()) {
# if _MONITOR>=1
    mPrint("Error connectTtn");
# endif
  }
# else
# if _MONITOR>=1
   mPrint("Setup:: ERROR, No Router Connection defined");
# endif
# endif

 delay(200);


    pinMode(pins.ss, OUTPUT);
 pinMode(pins.rst, OUTPUT);
    pinMode(pins.dio0, INPUT);
 pinMode(pins.dio1, INPUT);



#if defined(ESP32_ARCH)
 SPI.begin(SCK, MISO, MOSI, SS);
#else
 SPI.begin();
#endif

 SPISettings settings(SPISPEED, MSBFIRST, SPI_MODE0);
 SPI.beginTransaction(settings);

 delay(500);




# if _MONITOR>=1
 if (debug>=0) {
  String response= "Gateway ID: ";
  printHexDigit(MAC_array[0], response);
  printHexDigit(MAC_array[1], response);
  printHexDigit(MAC_array[2], response);
  printHexDigit(0xFF, response);
  printHexDigit(0xFF, response);
  printHexDigit(MAC_array[3], response);
  printHexDigit(MAC_array[4], response);
  printHexDigit(MAC_array[5], response);

  response += ", Listening at SF" + String(sf) + " on ";
  response += String(freqs[gwayConfig.ch].upFreq) + " Hz.";
  mPrint(response);
 }
# endif


 msg_lLED("GET TIME",".");
 ntpServer = resolveHost(NTP_TIMESERVER, 15);
 if (ntpServer.toString() == "0:0:0:0") {
# if _MONITOR>=1
   mPrint("setup:: NTP Server not found, found="+ntpServer.toString());
# endif
  delay(10000);
  ntpServer = resolveHost(NTP_TIMESERVER, 10);
 }



# if _NTP_INTR==1
  setupTime();

# else
 {



  String response = ".";
  while (timeStatus() == timeNotSet) {

   time_t newTime;
   if (getNtpTime(&newTime)<=0) {
# if _MONITOR>=1
    if (debug>=0) {
     mPrint("setup:: ERROR Time not set (yet). Time="+String(newTime) );
    }
# endif
    response += ".";
    msg_lLED("GET TIME",response);
    delay(800);
    continue;
   }
   response += ".";
   msg_lLED("GET TIME",response);
   delay(1000);
   setTime(newTime);
  }


  startTime = now();
# if _MONITOR>=1
  if (debug>=0) {
   String response= "Time set=";
   stringTime(now(),response);
   mPrint(response);
  }
# endif

  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }
# endif

 delay(100);


#if _GWAYSCAN==1
 mPrint("setup:: _GWAYSCAN=1: Setup OK");

#else


# ifdef _TTNSERVER
  ttnServer = resolveHost(_TTNSERVER, 10);
  if (ttnServer.toString() == "0:0:0:0") {
# if _MONITOR>=1
   if (debug>=1) {
    mPrint("setup:: TTN Server not found");
   }
# endif
   delay(10000);
   ttnServer = resolveHost(_TTNSERVER, 10);
  }
  delay(100);
# endif

# ifdef _THINGSERVER
  thingServer = resolveHost(_THINGSERVER, 10);
  delay(100);
# endif

#endif



#if _OTA==1
 setupOta(hostname);
#endif

 readSeen(_SEENFILE, listSeen);

#if _SERVER==1

 setupWWW();
#endif

 delay(100);


 _state = S_INIT;
 initLoraModem();

 if (gwayConfig.cad) {
  _state = S_SCAN;
  sf = SF7;
  cadScanner();
 }
 else {
  _state = S_RX;
  rxLoraModem();
 }
 LoraUp.payLoad[0]= 0;
 LoraUp.size = 0;



 if (pins.dio0 == pins.dio1) {
  attachInterrupt(pins.dio0, Interrupt_0, RISING);
 }

 else {
  attachInterrupt(pins.dio0, Interrupt_0, RISING);
  attachInterrupt(pins.dio1, Interrupt_1, RISING);

 }

 writeConfig(_CONFIGFILE, &gwayConfig);
 printSeen(_SEENFILE, listSeen);


# if _OLED>=1
  acti_oLED();
  addr_oLED();
# endif

 writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
 writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);


#define printReg(x) {int i=readRegister(x); if(i<=0x0F) Serial.print('0'); Serial.print(i,HEX);}
 Serial.print("RegInvertiQ :: "); printReg(REG_INVERTIQ); Serial.println();
 Serial.print("RegInvertiQ2:: "); printReg(REG_INVERTIQ2); Serial.println();

 mPrint(" --- Setup() ended, Starting loop() ---");

}
# 704 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
void loop ()
{
 int packetSize;
 uint32_t nowSeconds = now();



 if (WlanConnect(1) < 0) {
# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_MAIN)) {
   mPrint("loop:: ERROR reconnect WLAN");
  }
# endif
  yield();
  return;
 }

 yield();







 while( (packetSize= Udp.parsePacket()) > 0) {
# if _MONITOR>=1
  if ((debug>=3) && (pdebug & P_TX)) {
   mPrint("loop:: readUdp available");
  }
# endif







  if (readUdp(packetSize) < 0) {
# if _MONITOR>=1
   if (debug>=0)
    mPrint("v readUdp ERROR, returning < 0");
# endif
   break;
  }



  else {


  }
 }

 yield();





 stateMachine();







 if ( ((nowSeconds - statr[0].time) > _MSG_INTERVAL) &&
  (msgTime <= statr[0].time) )
 {
# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_MAIN)) {
   String response="";
   response += "REINIT:: ";
   response += String( _MSG_INTERVAL );
   response += (" ");
   mStat(0, response);
   mPrint(response);
  }
# endif

  yield();

  if ((gwayConfig.cad) || (gwayConfig.hop)) {
   _state = S_SCAN;
   sf = SF7;
   cadScanner();
  }
  else {
   _state = S_RX;
   rxLoraModem();
  }

  msgTime = nowSeconds;
 }

# if _OTA==1




 yield();
 ArduinoOTA.handle();
# endif





 if (_event == 1) {
  return;
 }
 else yield();

# if _SERVER==1



 server.handleClient();
# endif






    if ((nowSeconds - statTime) >= _STAT_INTERVAL) {
  yield();
        sendStat();
# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_MAIN)) {
   mPrint("Send Pushdata sendStat");
  }
# endif






# if _GATEWAYNODE==1
  if (gwayConfig.isNode) {

   yield();




   if (sensorPacket() < 0) {
# if _MONITOR>=1
    if ((debug>=1) || (pdebug & P_MAIN)) {
     mPrint("sensorPacket: Error");
    }
# endif
   }
  }
# endif
  statTime = nowSeconds;
    }
# 874 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/src.ino"
 nowSeconds = now();
    if ((nowSeconds - pullTime) >= _PULL_INTERVAL) {

  yield();
        pullData();
  pullTime = nowSeconds;

# if _MONITOR>=1
  if ((debug>=3) && (pdebug & P_RX)) {
   String response = "^ PULL_DATA:: ESP-sc-gway: message micr=";
   printInt(micros(), response);
   mPrint(response);
  }
# endif
    }





 nowSeconds = now();
    if ((nowSeconds - rstTime) >= _RST_INTERVAL) {

  yield();
  startReceiver();
  rstTime = nowSeconds;

# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_MAIN)) {
   String response = "^ ESP-sc-gway:: RST_DATA message sent: micr=";
   printInt(micros(), response);
   mPrint(response);
  }
# endif
    }





# if _NTP_INTR==0



  yield();
  if (nowSeconds - ntptimer >= _NTP_INTERVAL) {
   yield();
   time_t newTime;
   if (getNtpTime(&newTime)<=0) {
# if _MONITOR>=1
    if (debug>=2) {
     mPrint("loop:: WARNING Time not set (yet). Time="+String(newTime) );
    }
# endif
   }
   else {
    setTime(newTime);
    if (year(now()) != 1970) {
     ntptimer = nowSeconds;
    }
   }
  }
# endif

# if _MAXSEEN>=1
  if ((nowSeconds - fileTime) >= _FILE_INTERVAL) {
   printSeen(_SEENFILE, listSeen);
   fileTime = nowSeconds;
  }
# endif

}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_WiFi.ino"
# 31 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_WiFi.ino"
int WlanStatus() {

 switch (WiFi.status()) {
  case WL_CONNECTED:
# if _MONITOR>=1
   if ( debug>=1 ) {
    mPrint("WlanStatus:: CONNECTED ssid=" + String(WiFi.SSID()));
   }
# endif
   WiFi.setAutoReconnect(true);
   return(1);
   break;



  case WL_DISCONNECTED:
# if _MONITOR>=1
   if ( debug>=0 ) {
    mPrint("WlanStatus:: DISCONNECTED, IP=" + String(WiFi.localIP().toString()));
   }
# endif

    delay(100);

   return(0);
   break;


  case WL_IDLE_STATUS:
# if _MONITOR>=1
   if ( debug>=0 ) {
    mPrint("WlanStatus:: IDLE");
   }
# endif
   break;



  case WL_NO_SSID_AVAIL:
# if _MONITOR>=1
   if ( debug>=0 )
    mPrint("WlanStatus:: NO SSID");
# endif
   break;

  case WL_CONNECT_FAILED:
# if _MONITOR>=1
   if ( debug>=0 )
    mPrint("WlanStatus:: Connect FAILED");
# endif
   break;


  case WL_SCAN_COMPLETED:
# if _MONITOR>=1
   if ( debug>=0 )
    mPrint("WlanStatus:: SCAN COMPLETE");
# endif
   break;


  case WL_CONNECTION_LOST:
# if _MONITOR>=1
   if ( debug>=0 )
    mPrint("WlanStatus:: Connection LOST");
# endif
   break;



  case WL_NO_SHIELD:
# if _MONITOR>=1
   if ( debug>=0 )
    mPrint("WlanStatus:: WL_NO_SHIELD");
# endif
   break;

  default:
# if _MONITOR>=1
   if ( debug>=0 ) {
    mPrint("WlanStatus Error:: code=" + String(WiFi.status()));
   }
# endif
   break;
 }
 return(-1);

}





int wifiMgr()
{
#if _WIFIMANAGER==1

    WiFi.mode(WIFI_STA);
    WiFiManager wifiManager;

# if _MONITOR>=1
    if (debug>=1) {
        mPrint("Starting WiFiManager portal");
        mPrint("Connect Wifi to accesspoint: "+String(AP_NAME)+"-"+String(ESP_getChipId(), HEX)+" and browse to 192.168.4.1");
    }
# endif

    wifiManager.setDebugOutput(false);
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setConfigPortalTimeout(180);


    String portalSsid = String(AP_NAME) + "-" + String(ESP_getChipId(), HEX);
    if (!wifiManager.autoConnect(portalSsid.c_str(), AP_PASSWD)) {
# if _MONITOR>=1
        Serial.println("wifiMgr:: autoConnect failed - forcing config portal");
# endif
        if (!wifiManager.startConfigPortal(portalSsid.c_str(), AP_PASSWD)) {
# if _MONITOR>=1
            Serial.println("wifiMgr:: config portal timed out, restarting");
# endif
            delay(1000);
            ESP.restart();
        }
    }

# if _MONITOR>=1
 if ((debug>=1) && (pdebug & P_MAIN)) {
  mPrint("WlanConnect:: Now starting WlanStatus");
  delay(1);
  int i = WlanStatus();
  switch (i) {
   case 1: mPrint("WlanConnect:: WlanStatus Connected"); break;
   case 0: mPrint("WlanConnect:: WlanStatus Disconnected"); break;
   default: mPrint("WlanConnect:: WlanStatus other");
  }
 }
# endif





# if defined(ARDUINO_ARCH_ESP8266)

  struct station_config sta_conf;
  wifi_station_get_config(&sta_conf);
# else
  mPrint("wifiMgr:: define arch specific");
# endif

#endif
 return 1;
}






#if _WIFIMANAGER==1
void configModeCallback (WiFiManager *myWiFiManager)
{

  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());

}
#endif
# 226 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_WiFi.ino"
int WlanConnect(int maxTry) {

 unsigned char agains = 0;
 unsigned char wpa_index = 0;







 int i=0;

 while ((WiFi.status() != WL_CONNECTED) && (( i<= maxTry ) || (maxTry==0)) )
 {

  for (unsigned int j=wpa_index; (j<(sizeof(wpa)/sizeof(wpa[0]))) && (WiFi.status() != WL_CONNECTED ); j++)
  {

   char *ssid = wpa[j].login;
   char *password = wpa[j].passw;

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)) {
    Serial.print(i);
    Serial.print(':');
    Serial.print(j);
    Serial.print(':');
    Serial.print(sizeof(wpa)/sizeof(wpa[0]));
    Serial.print(F(". WlanConnect SSID="));
    Serial.print(ssid);
    if ( debug>=2 ) {
     Serial.print(F(", pass="));
     Serial.print(password);
    }
    Serial.println();
   }
# endif


   gwayConfig.wifis++;

   WiFi.mode(WIFI_STA);
   delay(1000);
   WiFi.begin(ssid, password);
   delay(8000);





   int stat = WlanStatus();
   if ( stat == 1) {
    writeGwayCfg(_CONFIGFILE, &gwayConfig );
    return(1);
   }



   agains=1;
   while ((WiFi.status() != WL_CONNECTED) && (agains < 8)) {
    agains++;
    delay(8000);
# if _MONITOR>=1
    if ( debug>=0 ) {
     Serial.print(".");
    }
# endif
   }



   WiFi.persistent(false);
   WiFi.mode(WIFI_OFF);

  }

  i++;
 }

 yield();
 return(1);

}
# 322 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_WiFi.ino"
IPAddress resolveHost(String svrName, int maxTry)
{
 IPAddress svrIP;

 if (svrName.endsWith(".local")) {
# if defined(ESP32_ARCH)
   svrName=svrName.substring(0,svrName.length()-6);
   svrIP = MDNS.queryHost(svrName);
   for (uint8_t i=0; i<maxTry; i++) {
    svrIP = MDNS.queryHost(svrName);
    if (svrIP.toString() != "0.0.0.0") break;
# if (_MONITOR>=1)
     mPrint("ReTrying to resolve with mDNS");
# endif
    delay(12000);
   }
# else
   char cc[svrName.length() +1 ];
   strncpy(cc, svrName.c_str(),svrName.length());
   cc[svrName.length()]=0;

   for (uint8_t i=0; i<maxTry; i++) {
    if (!WiFi.hostByName(cc, svrIP))
    {
     mPrint("resolveHost:: ERROR hostByName="+ String(cc)+", len=" + String(sizeof(cc)));
    };
    delay(1000);
   }
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)) {
    mPrint("resolveHost:: "+ String(cc) +" IP=" + String(svrIP.toString()) );
   }
# endif
# endif
 }
 else
 {
  char cc[svrName.length() +1 ];
  strncpy(cc, svrName.c_str(),svrName.length());
  cc[svrName.length()]=0;

  for (uint8_t i=0; i<maxTry; i++) {
   if (WiFi.hostByName(cc, svrIP))
   {
# if _MONITOR>=1
     mPrint("resolveHost:: OK="+ String(cc) +" IP=" + String(svrIP.toString()) );
# endif
     return svrIP;
   }
   else
   {
    mPrint("resolveHost:: ERROR hostByName="+ String(cc)+", len=" + String(sizeof(cc)));
   };
   delay(1000);
  }
 }
 return svrIP;
}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_gatewayMgt.ino"
# 29 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_gatewayMgt.ino"
#if _GATEWAYMGT==1

#if !defined _THINGPORT
#error "The management functions needs _THINGPORT defined (and not over _TTNPORT)"
#endif
# 54 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_gatewayMgt.ino"
void gateway_mgt(uint8_t size, uint8_t *buff) {

 uint8_t opcode = buff[3];

 switch (opcode) {
  case MGT_RESET:
# if _MONITOR>=1
    mPrint(F("gateway_mgt:: RESET"));
# endif

   setup();


  break;
  case MGT_SET_SF:
# if _MONITOR>=1
    mPrint(F("gateway_mgt:: SET SF"));
# endif

  break;
  case MGT_SET_FREQ:
# if _MONITOR>=1
    mPrint(F("gateway_mgt:: SET FREQ"));
# endif

  break;
  default:
# if _MONITOR>=1
    mPrint(F("gateway_mgt:: Unknown UDP code=") + String(opcode) );
# endif
   return;
  break;
 }
}

#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
# 25 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int initMonitor(struct moniLine *monitor)
{
#if _MONITOR>=1
 for (int i=0; i< gwayConfig.maxMoni; i++) {
  monitor[i].txt= "-";
 }
 iMoni=0;
#endif
 return(1);
}
# 46 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
void id_print (String id, String val)
{
#if _MONITOR>=1
 if ((debug>=0) && (pdebug & P_MAIN)) {
  Serial.print(id);
  Serial.print(F("=\t"));
  Serial.println(val);
 }
#endif
}
# 68 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
void initConfig(struct espGwayConfig *c)
{
 (*c).ch = _CHANNEL;
 (*c).sf = _SPREADING;
 (*c).debug = 1;
 (*c).pdebug = P_GUI | P_MAIN | P_TX;
 (*c).cad = _CAD;
 (*c).hop = false;
 (*c).seen = true;
 (*c).expert = _EXPERT;
 (*c).monitor = true;
 (*c).trusted = 1;
 (*c).txDelay = 0;
 (*c).dusbStat = true;
 (*c).d_fcnt = 0;

 (*c).maxSeen = _MAXSEEN;
 (*c).maxStat = _MAXSTAT;
 (*c).maxMoni = _MAXMONITOR;



 (*c).logFileRec = 0;
 (*c).logFileNo = 0;




 free(statr);
 delay(5);

 statr = (struct stat_t *) malloc((*c).maxStat * sizeof(struct stat_t));
 for (int i=0; i<(*c).maxStat; i++) {
  statr[i].time=0;
  statr[i].upDown=0;
  statr[i].node=0;
  statr[i].ch=0;
  statr[i].sf=0;
 }

 free(listSeen); delay(50);
 listSeen = (struct nodeSeen *) malloc((*c).maxSeen * sizeof(struct nodeSeen));
 for (int i=0; i<(*c).maxSeen; i +=1 ) {
  listSeen[i].idSeen=0;
 }

}




int readGwayCfg(const char *fn, struct espGwayConfig *c)
{

 if (readConfig(fn, c)<0) {
# if _MONITOR>=1
   mPrint("readConfig:: Error reading config file");
   return 0;
# endif
 }

 if (gwayConfig.sf != (uint8_t) 0) {
  sf = (sf_t) gwayConfig.sf;
 }
 debug = (*c).debug;
 pdebug = (*c).pdebug;
 (*c).boots++;

# if _GATEWAYNODE==1
  if (gwayConfig.fcnt != (uint8_t) 0) {
   LoraUp.fcnt = gwayConfig.fcnt+10;
  }
# endif

 writeGwayCfg(_CONFIGFILE, &gwayConfig );

 return 1;

}





int readConfig(const char *fn, struct espGwayConfig *c)
{

 int tries = 0;

 if (!SPIFFS.exists(fn)) {
# if _MONITOR>=1
   mPrint("readConfig ERR:: file="+String(fn)+" does not exist ..");
# endif
  initConfig(c);
  return(-1);
 }

 File f = SPIFFS.open(fn, "r");
 if (!f) {
# if _MONITOR>=1
   Serial.println(F("ERROR:: SPIFFS open failed"));
# endif
  return(-1);
 }

 while (f.available()) {

# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_MAIN)) {
   Serial.print('.');
  }
# endif




  if (tries >= 15) {
   f.close();
# if _MONITOR>=1
   if (debug>=0) {
    mPrint("readConfig:: Formatting");
   }
# endif
   SPIFFS.format();
   f = SPIFFS.open(fn, "r");
   tries = 0;
   initSeen(listSeen);
  }
  initConfig(c);

  String id =f.readStringUntil('=');
  String val=f.readStringUntil('\n');

  if (id == "MONITOR") {
   id_print(id, val);
   (*c).monitor = (bool) val.toInt();
  }
  else if (id == "BOOTS") {
   id_print(id, val);
   (*c).boots = (uint16_t) val.toInt();
  }
  else if (id == "CAD") {
   id_print(id, val);
   (*c).cad = (bool) val.toInt();
  }
  else if (id == "CH") {
   id_print(id,val);
   (*c).ch = (uint8_t) val.toInt();
  }
  else if (id == "DEBUG") {
   id_print(id, val);
   (*c).debug = (uint8_t) val.toInt();
  }
  else if (id == "DELAY") {
   id_print(id, val);
   (*c).txDelay = (int32_t) val.toInt();
  }
  else if (id == "EXPERT") {
   id_print(id, val);
   (*c).expert = (bool) val.toInt();
  }
  else if (id == "FCNT") {
   id_print(id, val);
   (*c).u_fcnt = (uint16_t) val.toInt();
  }




  else if (id == "FILENO") {
   id_print(id, val);
   (*c).logFileNo = (uint16_t) val.toInt();
  }
  else if (id == "FILEREC") {
   id_print(id, val);
   (*c).logFileRec = (uint16_t) val.toInt();
  }
  else if (id == "FORMAT") {
   id_print(id, val);
   (*c).formatCntr= (int8_t) val.toInt();
  }
  else if (id == "HOP") {
   id_print(id, val);
   (*c).hop = (bool) val.toInt();
  }
  else if (id == "NODE") {
   id_print(id, val);
   (*c).isNode = (bool) val.toInt();
  }
  else if (id == "REFR") {
   id_print(id, val);
   (*c).refresh = (bool) val.toInt();
  }
  else if (id == "REENTS") {
   id_print(id, val);
   (*c).reents = (uint16_t) val.toInt();
  }
  else if (id == "RESETS") {
   id_print(id, val);
   (*c).resets = (uint16_t) val.toInt();
  }
  else if (id == "NTPERR") {
   id_print(id, val);
   (*c).ntpErr = (uint16_t) val.toInt();
  }
  else if (id == "NTPETIM") {
   id_print(id, val);
   (*c).ntpErrTime = (uint32_t) val.toInt();
  }
  else if (id == "NTPS") {
   id_print(id, val);
   (*c).ntps = (uint16_t) val.toInt();
  }
  else if (id == "PDEBUG") {
   id_print(id, val);
   (*c).pdebug = (uint8_t) val.toInt();
  }
  else if (id == "SEEN") {
   id_print(id, val);
   (*c).seen = (bool) val.toInt();
  }
  else if (id == "SF") {
   id_print(id, val);
   (*c).sf = (uint8_t) val.toInt();
  }
  else if (id == "SHOWDATA") {
   id_print(id, val);
   (*c).showdata = (uint8_t) val.toInt();
  }
  else if (id == "TRUSTED") {
   id_print(id, val);
   (*c).trusted= (int8_t) val.toInt();
  }
  else if (id == "VIEWS") {
   id_print(id, val);
   (*c).views = (uint16_t) val.toInt();
  }
  else if (id == "WAITERR") {
   id_print(id, val);
   (*c).waitErr = (uint16_t) val.toInt();
  }
  else if (id == "WAITOK") {
   id_print(id, val);
   (*c).waitOk = (uint16_t) val.toInt();
  }
  else if (id == "WIFIS") {
   id_print(id, val);
   (*c).wifis = (uint16_t) val.toInt();
  }
  else {
# if _MONITOR>=1
    mPrint(F("readConfig:: tries++"));
# endif
   tries++;
  }
 }
 f.close();

 return(1);

}
# 338 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int writeGwayCfg(const char *fn, struct espGwayConfig *c)
{


 (*c).sf = (uint8_t) sf;
 (*c).debug = debug;
 (*c).pdebug = pdebug;

# if _GATEWAYNODE==1
  (*c).u_fcnt = LoraUp.fcnt;
# endif
 (*c).d_fcnt = LoraDown.fcnt;

 return(writeConfig(fn, c));
}
# 364 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int writeConfig(const char *fn, struct espGwayConfig *c)
{


 File f = SPIFFS.open(fn, "w");
 if (!f) {
#if _MONITOR>=1
  mPrint("writeConfig:: ERROR open file="+String(fn));
#endif
  return(-1);
 }

 f.print("CH"); f.print('='); f.print((*c).ch); f.print('\n');
 f.print("SF"); f.print('='); f.print((*c).sf); f.print('\n');
 f.print("FCNT"); f.print('='); f.print((*c).u_fcnt); f.print('\n');
 f.print("DCNT"); f.print('='); f.print((*c).d_fcnt); f.print('\n');
 f.print("DEBUG"); f.print('='); f.print((*c).debug); f.print('\n');
 f.print("PDEBUG"); f.print('='); f.print((*c).pdebug); f.print('\n');
 f.print("CAD"); f.print('='); f.print((*c).cad); f.print('\n');
 f.print("HOP"); f.print('='); f.print((*c).hop); f.print('\n');
 f.print("NODE"); f.print('='); f.print((*c).isNode); f.print('\n');
 f.print("BOOTS"); f.print('='); f.print((*c).boots); f.print('\n');
 f.print("RESETS"); f.print('='); f.print((*c).resets); f.print('\n');
 f.print("WIFIS"); f.print('='); f.print((*c).wifis); f.print('\n');
 f.print("VIEWS"); f.print('='); f.print((*c).views); f.print('\n');
 f.print("REFR"); f.print('='); f.print((*c).refresh); f.print('\n');
 f.print("REENTS"); f.print('='); f.print((*c).reents); f.print('\n');
 f.print("NTPETIM"); f.print('='); f.print((*c).ntpErrTime); f.print('\n');
 f.print("NTPERR"); f.print('='); f.print((*c).ntpErr); f.print('\n');
 f.print("WAITERR"); f.print('='); f.print((*c).waitErr); f.print('\n');
 f.print("WAITOK"); f.print('='); f.print((*c).waitOk); f.print('\n');
 f.print("NTPS"); f.print('='); f.print((*c).ntps); f.print('\n');
 f.print("FILEREC"); f.print('='); f.print((*c).logFileRec); f.print('\n');
 f.print("FILENO"); f.print('='); f.print((*c).logFileNo); f.print('\n');
 f.print("FORMAT"); f.print('='); f.print((*c).formatCntr); f.print('\n');
 f.print("DELAY"); f.print('='); f.print((*c).txDelay); f.print('\n');
 f.print("SHOWDATA");f.print('='); f.print((*c).showdata); f.print('\n');
 f.print("TRUSTED"); f.print('='); f.print((*c).trusted); f.print('\n');
 f.print("EXPERT"); f.print('='); f.print((*c).expert); f.print('\n');
 f.print("SEEN"); f.print('='); f.print((*c).seen); f.print('\n');
 f.print("MONITOR"); f.print('='); f.print((*c).monitor); f.print('\n');

 f.close();
 return(1);
}
# 426 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int addLog(const unsigned char * line, int cnt)
{

#if _STAT_LOG==1
 File f;
 char fn[16];



 if (gwayConfig.logFileRec > LOGFILEREC) {

  gwayConfig.logFileRec = 0;
  gwayConfig.logFileNo++;

  f.close();

  sprintf(fn,"/log-%d", gwayConfig.logFileNo);

  if (gwayConfig.logFileNo > LOGFILEMAX) {
   sprintf(fn,"/log-%d", gwayConfig.logFileNo-LOGFILEMAX );
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_RX)) {
    mPrint("addLog:: Too many logfiles, deleting="+String(fn));
   }
# endif
   SPIFFS.remove(fn);
  }
 }

 sprintf(fn,"/log-%d", gwayConfig.logFileNo);



 if (!SPIFFS.exists(fn)) {
# if _MONITOR>=1
  if ((debug >= 1) && (pdebug & P_RX)) {
   mPrint("addLog:: WARNING file="+String(fn)+" does not exist. record="+String(gwayConfig.logFileRec) );
  }
# endif
 }

 f = SPIFFS.open(fn, "a");
 if (!f) {
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_RX)) {
   mPrint("addLOG:: ERROR file open failed="+String(fn));
  }
# endif
  return(0);
 }
# if _MONITOR>=2
 else {
  mPrint("addLog:: Opening adding file="+String(fn));
 }
# endif


 int i=0;
# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_RX)) {
  char s[256];
  i += 12;
  sprintf(s, "addLog:: fileno=%d, rec=%d : %s",gwayConfig.logFileNo,gwayConfig.logFileRec,&line[i]);
  mPrint(s);
 }
# endif

 for (i=0; i< 12; i++) {

  f.print('*');
 }
 f.print(now());
 f.print(':');
 f.write(&(line[i]), cnt-12);
 f.print('\n');

 f.close();

#endif

 gwayConfig.logFileRec++;

 return(1);
}
# 526 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int initSeen(struct nodeSeen *listSeen)
{
#if _MAXSEEN>=1
 for (int i=0; i< gwayConfig.maxSeen; i++) {
  listSeen[i].idSeen=0;
  listSeen[i].upDown=0;
  listSeen[i].sfSeen=0;
  listSeen[i].cntSeen=0;
  listSeen[i].chnSeen=0;
  listSeen[i].timSeen=(time_t) 0;
 }
 iSeen= 0;
#endif
 return(1);

}
# 554 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int readSeen(const char *fn, struct nodeSeen *listSeen)
{
#if _MAXSEEN>=1
 int i;
 iSeen= 0;

 if (!SPIFFS.exists(fn)) {
# if _MONITOR>=1
   mPrint("WARNING:: readSeen, history file not exists "+String(fn) );
# endif
  initSeen(listSeen);
  return(-1);
 }

 File f = SPIFFS.open(fn, "r");
 if (!f) {
# if _MONITOR>=1
   mPrint("readSeen:: ERROR open file="+String(fn));
# endif
  return(-1);
 }

 delay(1000);

 for (i=0; i<gwayConfig.maxSeen; i++) {
  delay(200);
  String val="";

  if (!f.available()) {
# if _MONITOR>=2
    mPrint("readSeen:: No more info left in file, i=" + String(i));
# endif
   break;
  }
  val=f.readStringUntil('\t'); listSeen[i].timSeen = (time_t) val.toInt();
  val=f.readStringUntil('\t'); listSeen[i].upDown = (uint8_t) val.toInt();
  val=f.readStringUntil('\t'); listSeen[i].idSeen = (int64_t) val.toInt();
  val=f.readStringUntil('\t'); listSeen[i].cntSeen = (uint32_t) val.toInt();
  val=f.readStringUntil('\t'); listSeen[i].chnSeen = (uint8_t) val.toInt();
  val=f.readStringUntil('\n'); listSeen[i].sfSeen = (uint8_t) val.toInt();

  if ((int32_t)listSeen[i].idSeen != 0) {
   iSeen++;
  }

# if _MONITOR>=1

  if ((debug>=2) && (pdebug & P_MAIN)) {
    mPrint(" readSeen:: idSeen ="+String(listSeen[i].idSeen,HEX)+", i="+String(i));
  }
# endif
 }

 f.close();

#endif


 return 1;

}
# 626 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int printSeen(const char *fn, struct nodeSeen *listSeen)
{
#if _MAXSEEN>=1
 int i;
 if (!SPIFFS.exists(fn)) {
# if _MONITOR>=1
   mPrint("WARNING:: printSeen, file not exists="+String(fn));
# endif

 }

 File f = SPIFFS.open(fn, "w");
 if (!f) {
# if _MONITOR>=1
   mPrint("printSeen:: ERROR open file="+String(fn)+" for writing");
# endif
  return(-1);
 }
 delay(500);

 for (i=0; i<iSeen; i++) {
  if ((int32_t)listSeen[i].idSeen == 0) break;

  f.print((time_t)listSeen[i].timSeen); f.print('\t');
  f.print((uint8_t)listSeen[i].upDown); f.print('\t');
  f.print((int32_t)listSeen[i].idSeen); f.print('\t');
  f.print((uint32_t)listSeen[i].cntSeen); f.print('\t');
  f.print((uint8_t)listSeen[i].chnSeen); f.print('\t');
  f.print((uint8_t)listSeen[i].sfSeen); f.print('\n');

# if _MONITOR>=1
  if ((debug >= 2) && (pdebug & P_TX)) {
   if (listSeen[i].upDown == 1) {
    mPrint("printSeen:: ERROR f.print, upDown="+String(listSeen[i].upDown)+", i="+String(i) );
   }
  }
# endif
 }

 f.close();
#endif
 return(1);

}
# 686 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraFiles.ino"
int addSeen(struct nodeSeen *listSeen, struct stat_t stat)
{

#if _MAXSEEN>=1
 int i;

 for (i=0; i<iSeen; i++) {



  if ((listSeen[i].idSeen==stat.node) &&
   (listSeen[i].upDown==stat.upDown))
  {


   listSeen[i].timSeen = (time_t)stat.time;
   listSeen[i].chnSeen = stat.ch;
   listSeen[i].sfSeen = stat.sf;
   listSeen[i].cntSeen++;


# if _MONITOR>=2
   if ((debug>=3) && (pdebug & P_MAIN)) {
    mPrint("addSeen:: adding i="+String(i)+", node="+String(stat.node,HEX));
   }
# endif

   return 1;
  }
 }


 if ((i>=iSeen) && (i<gwayConfig.maxSeen)) {
  listSeen[i].upDown = stat.upDown;
  listSeen[i].idSeen = stat.node;
  listSeen[i].chnSeen = stat.ch;
  listSeen[i].sfSeen = stat.sf;
  listSeen[i].timSeen = (time_t)stat.time;
  listSeen[i].cntSeen = 1;
  iSeen++;
 }

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_MAIN)) {
  String response= "  addSeen:: i=";
  response += i;
  response += ", tim=";
  stringTime(stat.time, response);
  response += ", iSeen=";
  response += String(iSeen);
  response += ", node=";
  response += String(stat.node,HEX);
  response += ", listSeen[0]=";
  printHex(listSeen[0].idSeen,':',response);

  mPrint(response);
 }
# endif

#endif
 return 1;

}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
# 51 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
#if _MUTEX==1
 void CreateMutux(int *mutex) {
  *mutex=1;
 }

#define LIB__MUTEX 1

#if LIB__MUTEX==1
 bool GetMutex(int *mutex) {

  if (*mutex==1) {
   *mutex=0;

   return(true);
  }

  return(false);
 }
#else
 bool GetMutex(int *mutex) {

 int iOld = 1, iNew = 0;

 asm volatile (
  "rsil a15, 1\n"
  "l32i %0, %1, 0\n"
  "bne %0, %2, 1f\n"
  "s32i %3, %1, 0\n"
  "1:\n"
  "wsr.ps a15\n"
  "rsync\n"
  : "=&r" (iOld)
  : "r" (mutex), "r" (iOld), "r" (iNew)
  : "a15", "memory"
 );
 return (bool)iOld;
}
#endif

 void ReleaseMutex(int *mutex) {
  *mutex=1;
 }

#endif
# 109 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
uint8_t readRegister(uint8_t addr)
{

    digitalWrite(pins.ss, LOW);
 SPI.transfer(addr & 0x7F);
 uint8_t res = (uint8_t) SPI.transfer(0x00);
    digitalWrite(pins.ss, HIGH);

    return((uint8_t) res);
}
# 130 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void writeRegister(uint8_t addr, uint8_t value)
{
 noInterrupts();
 digitalWrite(pins.ss, LOW);

 SPI.transfer((int8_t)((addr | 0x80) & 0xFF));
 SPI.transfer(value & 0xFF);

    digitalWrite(pins.ss, HIGH);
 interrupts();
}
# 153 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void writeBuffer(uint8_t addr, uint8_t *buf, uint8_t len)
{
 noInterrupts();
 digitalWrite(pins.ss, LOW);

 if (len >= 24) {
  len=24;
  mPrint("WARNING writeBuffer:: len > 24");
 }

 SPI.transfer((int8_t)(addr | 0x80) );

# if _BUF_WRITE==1
  SPI.transfer((uint8_t *) buf, len);
# else
  for (int i=0; i< len; i++) {
   SPI.transfer(buf[i]);
  }
# endif

    digitalWrite(pins.ss, HIGH);
 interrupts();

# if _MONITOR>=1
 if ((debug>=1) && (pdebug & P_TX)) {
  mPrint("v writeBuffer:: payLength=0x"+String(len,HEX)+", len FIFO=0x"+String((int8_t)(readRegister(REG_FIFO_ADDR_PTR)-readRegister(REG_FIFO_TX_BASE_AD)),HEX) );
 }
 if ((debug>=1) && (pdebug & P_MAIN)) {
  String response = "v writeBuffer:: after  : buf=";
  for (int j=0; j<len; j++) {
   response += String(buf[j],HEX)+" ";
  }
  mPrint(response);
 }
# endif
}
# 206 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void setRate(uint8_t sf, uint8_t crc)
{
 uint8_t mc1=0, mc2=0, mc3=0;

 if (sf<SF7) {
  sf=7;
 }
 else if (sf>SF12) {
  sf=12;
 }



    if (sx1276) {
  mc1= 0x72;
  mc2= (sf<<4) | crc;
  mc3= 0x00;
        if (sf == SF11 || sf == SF12) {
   mc3|= 0x01;
  }
    }
 else {
  mc1= 0x0A;
  mc2= ((sf<<4) | crc) % 0xFF;

        if (sf == SF11 || sf == SF12) {
   mc1= 0x0B;
  }
# if _MONITOR>=1
  if ((debug>=1) &&(pdebug & P_MAIN)) {
   mPrint("WARNING, sx1272 selected");
  }
# endif
    }







 writeRegister(REG_MODEM_CONFIG1, (uint8_t) mc1);
 writeRegister(REG_MODEM_CONFIG2, (uint8_t) mc2);
 writeRegister(REG_MODEM_CONFIG3, (uint8_t) mc3);


    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t) 0x05);
    }
 else {
        writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t) 0x08);
    }
 return;
}
# 270 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void setFreq(uint32_t freq)
{



    uint32_t temp_bytes = (((uint64_t)freq << 19) / 32000000) & 0x00FFFFFF;

    writeRegister(REG_FRF_MSB, ((uint8_t)(temp_bytes>>16)) & 0xFF );
    writeRegister(REG_FRF_MID, ((uint8_t)(temp_bytes>> 8)) & 0xFF );
    writeRegister(REG_FRF_LSB, ((uint8_t)(temp_bytes>> 0)) & 0xFF );
 return;
}
# 290 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void setPow(uint8_t pow)
{
 uint8_t pac = 0x00;

 if (pow>17) {
  pac = 0xFF;
 }
 else if (pow<2) {
  pac = 0x42;
 }
 else if (pow<=12) {
  pac=0x40+pow;
 }
 else {


  pac=0x70+pow;
 }

# if _MONITOR>=1
 if ((debug>=2) && ( pdebug & P_MAIN)) {
  mPrint("v setPow:: pow=0x"+String(pow,HEX)+", pac=0x"+String(pac,HEX) );
 }
# endif

 ASSERT(((pac&0x0F)>=2) &&((pac&0x0F)<=20));

 writeRegister(REG_PAC, (uint8_t) pac);
 return;
}
# 328 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void opmode(uint8_t mode)
{
 if (mode == OPMODE_LORA) {
# ifdef CFG_sx1276_radio

# endif
  writeRegister(REG_OPMODE, 0xFF & (uint8_t) mode );
 }
 else {

  writeRegister(REG_OPMODE, 0xFF & (uint8_t)((readRegister(REG_OPMODE) & 0x80) | mode));
 }
}






void hop()
{


 opmode(OPMODE_STANDBY);


 gwayConfig.ch = (gwayConfig.ch + 1) % NUM_HOPS ;
 setFreq(freqs[gwayConfig.ch].upFreq);


 sf = SF7;
 setRate(sf, 0x04);


 writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);


 writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);


 writeRegister(REG_INVERTIQ,0x27);



 writeRegister(REG_MAX_PAYLOAD_LENGTH,MAX_PAYLOAD_LENGTH);
 writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);

 writeRegister(REG_FIFO_ADDR_PTR,(uint8_t)readRegister(REG_FIFO_RX_BASE_AD));
 writeRegister(REG_HOP_PERIOD,0x00);


 writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08);





 writeRegister(REG_IRQ_FLAGS_MASK, 0x00);


    writeRegister(REG_IRQ_FLAGS, 0xFF);




# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_RADIO)){
   String response = "hop:: hopTime:: " + String(micros() - hopTime);
   mStat(0, response);
   mPrint(response);
 }
# endif

 hopTime = micros();

}
# 440 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
uint8_t receivePkt(uint8_t *payload)
{
    statc.msg_ttl++;

    uint8_t irqflags = readRegister(REG_IRQ_FLAGS);
 uint8_t crcUsed = readRegister(REG_HOP_CHANNEL);
 if (crcUsed & 0x40) {
# if _MONITOR>=1
  if (( debug>=2) && (pdebug & P_RX )) {
   mPrint("R rxPkt:: CRC used");
  }
# endif
 }


    if (irqflags & IRQ_LORA_CRCERR_MASK)
    {
# if _MONITOR>=1
        if ((debug>=0) && (pdebug & P_RADIO)) {
   String response=("rxPkt:: Err CRC, t=");
   stringTime(now(), response);
   mPrint(response);
  }
# endif
  return 0;
    }




 else if ((irqflags & IRQ_LORA_HEADER_MASK) == false)
    {
# if _MONITOR>=1
   if ((debug>=0) && (pdebug & P_RADIO)) {
    mPrint("rxPkt:: Err HEADER");
   }
# endif

        writeRegister(REG_IRQ_FLAGS, (uint8_t)(IRQ_LORA_HEADER_MASK | IRQ_LORA_RXDONE_MASK));
        return 0;
    }



 else {
        statc.msg_ok++;
  switch(statr[0].ch) {
   case 0: statc.msg_ok_0++; break;
   case 1: statc.msg_ok_1++; break;
   case 2: statc.msg_ok_2++; break;
  }

  if (readRegister(REG_FIFO_RX_CURRENT_ADDR) != readRegister(REG_FIFO_RX_BASE_AD)) {
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_RADIO)) {
    mPrint("RX BASE <" + String(readRegister(REG_FIFO_RX_BASE_AD)) + "> != RX CURRENT <" + String(readRegister(REG_FIFO_RX_CURRENT_ADDR)) + ">" );
   }
# endif
  }


  uint8_t currentAddr = readRegister(REG_FIFO_RX_BASE_AD);
        uint8_t receivedCount = readRegister(REG_RX_BYTES_NB);
# if _MONITOR>=1
   if ((debug>=1) && (currentAddr > 64)) {
    mPrint("rxPkt:: ERROR Rx addr>64"+String(currentAddr));
   }
# endif
        writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) currentAddr);

  if (receivedCount > PAYLOAD_LENGTH) {
# if _MONITOR>=1
    if ((debug>=0) & (pdebug & P_RADIO)) {
     mPrint("rxPkt:: ERROR Payliad receivedCount="+String(receivedCount));
    }
# endif
   receivedCount=PAYLOAD_LENGTH;
  }

        for(int i=0; i < receivedCount; i++)
        {
            payload[i] = readRegister(REG_FIFO);
        }

# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_RX)) {
   String response="^ (" + String(receivedCount) + "): " ;
   for (int i=0; i<receivedCount; i++) {
    if (payload[i]<0x10) response += '0';
    response += String(payload[i],HEX) + " ";
   }
   mPrint(response);
  }
# endif



# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_RX)) {

   String response = "^ receivePkt:: rxPkt: t=";
   stringTime(now(), response);
   response += ", f=" + String(gwayConfig.ch) + ", sf=" + String(sf);

   response += ", a=";
   uint8_t DevAddr [4];
     DevAddr[0] = payload[4];
     DevAddr[1] = payload[3];
     DevAddr[2] = payload[2];
     DevAddr[3] = payload[1];
   printHex((IPAddress)DevAddr, ':', response);

   response += ", flags=" + String(irqflags,HEX);
   response += ", addr=" + String(currentAddr);
   response += ", len=" + String(receivedCount);



# if _LOCALSERVER>=1
   if (debug>=1) {

    int index;
    uint8_t data[receivedCount];

    if ((index = inDecodes((char *)(payload+1))) >=0 ) {
     response += (", inDecodes="+String(index));
    }
    else {
     response += (", ERROR No Index in inDecodes");
     mPrint(response);
     return(receivedCount);
    }



    Serial.print(F(", data="));

    for (int i=0; i<receivedCount; i++) {
     data[i]= payload[i];
    }


    LoraUp.fcnt= payload[6] | (payload[7] << 8);



    uint8_t CodeLength= encodePacket(
     (uint8_t *)(data + 9),
     receivedCount-9-4,
     (uint16_t)LoraUp.fcnt,
     DevAddr,
     decodes[index].appKey,
     0
    );

    Serial.print(F("- NEW fc="));
    Serial.print(LoraUp.fcnt);
    Serial.print(F(", addr="));

    for (int i=0; i<4; i++) {
     if (DevAddr[i]<=0xF) {
      Serial.print('0');
     }
     Serial.print(DevAddr[i], HEX);
     Serial.print(' ');
    }
    Serial.print(F(", len="));
    Serial.print(CodeLength);
   }
# endif

   mPrint(response);


   Serial.print(F(", paylength="));
   Serial.print(receivedCount);
   Serial.print(F(", payload="));
   for (int i=0; i<receivedCount; i++) {
     if (payload[i]<=0xF) Serial.print('0');
     Serial.print(payload[i], HEX);
     Serial.print(' ');
   }
   Serial.println();
  }
# endif

  return(receivedCount);
    }


 writeRegister(REG_IRQ_FLAGS, (uint8_t) (
  IRQ_LORA_RXDONE_MASK |
  IRQ_LORA_RXTOUT_MASK |
  IRQ_LORA_HEADER_MASK |
  IRQ_LORA_CRCERR_MASK));

 return 0;

}






void initDown(struct LoraDown *LoraDown)
{
 LoraDown->fcnt = 0x00;
}
# 664 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
bool sendPkt(uint8_t *payLoad, uint8_t payLength)
{
# if _MONITOR>=1
 if (payLength>=128) {
  if ((debug>=1) && (pdebug & P_TX)) {
   mPrint("v sendPkt:: ERROR len="+String(payLength));
  }
  return false;
 }
# endif

 payLoad[payLength] = 0x00;

 writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_TX_BASE_AD));
 writeRegister(REG_PAYLOAD_LENGTH, (uint8_t) payLength);

 ASSERT( (uint8_t)readRegister(REG_FIFO_ADDR_PTR)==(uint8_t)readRegister(REG_FIFO_TX_BASE_AD) );







# if _BUF_WRITE>=1

  writeBuffer(REG_FIFO, (uint8_t *) payLoad, (uint8_t) payLength);

# else
  SPI.transfer((int8_t)(REG_FIFO | 0x80) );
  for (int i=0; i<(uint8_t) payLength; i++) {
   writeRegister(REG_FIFO, (uint8_t) payLoad[i]);
  }

# endif

# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_TX)) {
   Serial.print("v sendPkt:: <");
   for (int i=0; i<(uint8_t) payLength; i++) {
    Serial.print(" ");
    Serial.print((uint8_t) payLoad[i],HEX);
   }
   Serial.println(">");
  }
# endif

 return true;
}
# 742 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
int loraWait(struct LoraDown *LoraDown)
{
 if (LoraDown->imme == 1) {
  if ((debug>=3) && (pdebug & P_TX)) {
   mPrint("loraWait:: imme is 1");
  }
  return(1);
 }
 LoraDown->usec = micros();
 int32_t delayTmst = (int32_t)(LoraDown->tmst - LoraDown->usec) + gwayConfig.txDelay - WAIT_CORRECTION;


 if ((delayTmst > 8000000) || (delayTmst < -1000)) {
# if _MONITOR>=1
  if (delayTmst > 8000000) {
   String response= "v loraWait:: ERROR: ";
   printDwn(LoraDown, response);
   mPrint(response);
  }
  else {
   String response= "v loraWait:: return 0: ";
   printDwn(LoraDown, response);
   mPrint(response);
  }
# endif
  gwayConfig.waitErr++;

  return(0);
 }



 while (delayTmst > 15000) {
  delay(15);
  delayTmst -= 15000;
 }




 delayMicroseconds(delayTmst);

 gwayConfig.waitOk++;
 return (1);
}
# 826 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void txLoraModem(struct LoraDown *LoraDown)
{
 _state = S_TX;


 opmode(OPMODE_SLEEP);
 delayMicroseconds(100);
 opmode(OPMODE_LORA | 0x03);;


 ASSERT((readRegister(REG_OPMODE) & OPMODE_LORA) != 0);


 opmode(OPMODE_STANDBY);


 setRate(LoraDown->sf, LoraDown->crc);





 setFreq(LoraDown->freq);



 writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);
 writeRegister(REG_PARAMP,(readRegister(REG_PARAMP) & 0xF0) | 0x08);


 writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);


 setPow(LoraDown->powe);




 writeRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ) | 0x40);
 writeRegister(REG_INVERTIQ2, 0x19);


    writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
  MAP_DIO0_LORA_TXDONE |
  MAP_DIO1_LORA_NOP |
  MAP_DIO2_LORA_NOP |
  MAP_DIO3_LORA_NOP));


    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~IRQ_LORA_TXDONE_MASK);


    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);







 writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_TX_BASE_AD));



 writeRegister(REG_PAYLOAD_LENGTH, (uint8_t) LoraDown->size);


 writeRegister(REG_MAX_PAYLOAD_LENGTH, (uint8_t) MAX_PAYLOAD_LENGTH);

# if _MONITOR >= 1
   if ((debug>=2) && (pdebug & P_TX)) {
    String response ="v txLoraModem:: Before sendPkt: Addr=";
    response+=
       String((uint8_t)LoraDown->payLoad[4],HEX) + " " +
       String((uint8_t)LoraDown->payLoad[3],HEX) + " " +
       String((uint8_t)LoraDown->payLoad[2],HEX) + " " +
       String((uint8_t)LoraDown->payLoad[1],HEX);
    response += ", FCtrl=" + String(LoraDown->payLoad[5],HEX);
    response += ", FPort=" + String(LoraDown->payLoad[8],HEX);
    mPrint(response);
   }
# endif


 sendPkt(LoraDown->payLoad, LoraDown->size);

# if _MONITOR >= 1
   if ((debug>=2) && (pdebug & P_TX)) {
    String response ="v txLoraModem::  After sendPkt: Addr=";
    response+=
       String(LoraDown->payLoad[4],HEX) + " " +
       String(LoraDown->payLoad[3],HEX) + " " +
       String(LoraDown->payLoad[2],HEX) + " " +
       String(LoraDown->payLoad[1],HEX);
    response += ", FCtrl=" + String(LoraDown->payLoad[5],HEX);
    response += ", FPort=" + String(LoraDown->payLoad[8],HEX);
    response += ", Fcnt=" + String(LoraDown->payLoad[6] | LoraDown->payLoad[7] << 8);
    mPrint(response);
   }
# endif

 for (int i=0; i< _REG_AMOUNT; i++) {
  registers[i].regvalue= readRegister(registers[i].regid);
 }


 delayMicroseconds(WAIT_CORRECTION);
 opmode(OPMODE_TX);



 yield();

}
# 960 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void rxLoraModem()
{




 opmode(OPMODE_STANDBY);


 setFreq(freqs[gwayConfig.ch].upFreq);


    setRate(sf, 0x04);


# if _GWAYSCAN!=1
 writeRegister(REG_INVERTIQ, (uint8_t) 0x27);
# else
 writeRegister(REG_INVERTIQ, (uint8_t) 0x40);
# endif
# 990 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
 writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_RX_BASE_AD));


 writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);


 writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
  IRQ_LORA_RXDONE_MASK |
  IRQ_LORA_RXTOUT_MASK |
  IRQ_LORA_HEADER_MASK |
  IRQ_LORA_CRCERR_MASK));


 if (gwayConfig.hop) {

  writeRegister(REG_HOP_PERIOD,0x00);
 }
 else {
  writeRegister(REG_HOP_PERIOD,0xFF);
 }

 writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
   MAP_DIO0_LORA_RXDONE |
   MAP_DIO1_LORA_RXTOUT |
   MAP_DIO2_LORA_NOP |
   MAP_DIO3_LORA_CRC));



 if (gwayConfig.cad) {


  _state= S_RX;
  opmode(OPMODE_RX_SINGLE);
 }
 else {

  _state= S_RX;
# if _MONITOR>=1
  if (gwayConfig.hop) {
   mPrint("rxLoraModem:: ERROR continuous receive in hop mode");
  }
# endif
  opmode(OPMODE_RX);
 }


    writeRegister(REG_IRQ_FLAGS, 0xFF);

 return;
}
# 1053 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void cadScanner()
{




 opmode(OPMODE_STANDBY);


 setFreq(freqs[gwayConfig.ch].upFreq);





 setRate(sf, 0x04);


 writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);


 writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
  MAP_DIO0_LORA_CADDONE |
  MAP_DIO1_LORA_CADDETECT |
  MAP_DIO2_LORA_NOP |
  MAP_DIO3_LORA_CRC ));


 writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
  IRQ_LORA_CDDONE_MASK |
  IRQ_LORA_CDDETD_MASK |
  IRQ_LORA_CRCERR_MASK |
  IRQ_LORA_HEADER_MASK));


 opmode(OPMODE_CAD);






 return;

}
# 1114 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void initLoraModem()
{
 _state = S_INIT;
#if defined(ESP32_ARCH)
 digitalWrite(pins.rst, LOW);
 delayMicroseconds(10000);
    digitalWrite(pins.rst, HIGH);
 delayMicroseconds(10000);
 digitalWrite(pins.ss, HIGH);
#else

 digitalWrite(pins.rst, HIGH);
 delayMicroseconds(10000);
    digitalWrite(pins.rst, LOW);
 delayMicroseconds(10000);
#endif

 opmode(OPMODE_SLEEP);


 opmode(OPMODE_LORA);


 setFreq(freqs[gwayConfig.ch].upFreq);


    setRate(sf, 0x04);


    writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);
# if _PIN_OUT==4
  delay(1);
# endif


    uint8_t version = readRegister(REG_VERSION);
 if (version == 0x12) {
# if _MONITOR>=1
           if ((debug>=2) && (pdebug & P_MAIN)) {
   mPrint("SX1276 starting");
  }
# endif
  sx1276= true;
 }
    else if (version == 0x22) {
# if _MONITOR>=1
            if ((debug>=1) && (pdebug & P_MAIN)) {
    mPrint("WARNING:: SX1272 detected");
   }
# endif
        sx1276= false;
    }
 else {




# if _DUSB>=1
   Serial.print(F("Unknown transceiver="));
   Serial.print(version,HEX);
   Serial.print(F(", pins.rst =")); Serial.print(pins.rst);
   Serial.print(F(", pins.ss  =")); Serial.print(pins.ss);
   Serial.print(F(", pins.dio0 =")); Serial.print(pins.dio0);
   Serial.print(F(", pins.dio1 =")); Serial.print(pins.dio1);
   Serial.print(F(", pins.dio2 =")); Serial.print(pins.dio2);
   Serial.println();
   Serial.flush();
# endif
  die("initLoraModem, unknown transceiver?");
    }



 writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);

# if _GWAYSCAN==0

  writeRegister(REG_INVERTIQ,0x27);
# else
  writeRegister(REG_INVERTIQ,0x40);
  mPrint("initLoraModem:: Set REG_INVERTIQ | 0x40");
# endif



 writeRegister(REG_MAX_PAYLOAD_LENGTH,PAYLOAD_LENGTH);

 writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);

 writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_RX_BASE_AD));
 writeRegister(REG_HOP_PERIOD,0x00);


 writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08);



 writeRegister(REG_PADAC_SX1276, 0x84);






 writeRegister(REG_IRQ_FLAGS_MASK, 0x00);


    writeRegister(REG_IRQ_FLAGS, 0xFF);

}
# 1233 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void startReceiver()
{
 initLoraModem();
 if (gwayConfig.cad) {
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_SCAN)) {
   mPrint("S PULL:: _state set to S_SCAN");
   if (debug>=2) Serial.flush();
  }
# endif
  _state = S_SCAN;
  sf = SF7;
  cadScanner();
 }
 else {
  _state = S_RX;
  rxLoraModem();
 }
 writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
 writeRegister(REG_IRQ_FLAGS, 0xFF);
}
# 1264 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void ICACHE_RAM_ATTR Interrupt_0()
{
 _event=1;
}
# 1281 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_loraModem.ino"
void ICACHE_RAM_ATTR Interrupt_1()
{
 _event=1;
}




void ICACHE_RAM_ATTR Interrupt_2()
{
 _event=1;
}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_oLED.ino"
# 25 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_oLED.ino"
#include <Wire.h>

#ifdef GW_DISPLAY
#undef GW_DISPLAY
#endif

#if defined(USE_HELTEC_DISPLAY)
#define GW_DISPLAY Heltec.display
#else
#include "OLEDDisplayFonts.h"
#define GW_DISPLAY (&display)
#endif

void init_oLED()
{
#if _OLED>=1
#if !defined(USE_HELTEC_DISPLAY)
#if defined(OLED_SDA) && defined(OLED_SCL)
 Wire.begin(OLED_SDA, OLED_SCL);
#endif
#if defined OLED_RST
 pinMode(OLED_RST,OUTPUT);
 digitalWrite(OLED_RST, LOW);
 delay(100);
 digitalWrite(OLED_RST, HIGH);
 delay(50);
#endif
 display.init();
 delay(100);
#endif

 auto *d = GW_DISPLAY;
 d->clear();
 d->flipScreenVertically();
 d->setFont(ArialMT_Plain_16);
 d->setTextAlignment(TEXT_ALIGN_LEFT);
 d->drawString(0, 0, "STARTING");
 d->display();
#endif
}







void acti_oLED()
{
#if _OLED>=1
 auto *d = GW_DISPLAY;
 d->clear();
 d->setFont(ArialMT_Plain_10);
 d->drawString(0, 0, "READY SSID=");
 d->drawString(0, 12, WiFi.SSID());
 d->drawString(0, 24, "IP=");
 d->drawString(0, 36, WiFi.localIP().toString().c_str());
 d->display();

#endif
 delay(4000);
}






void msg_oLED(String mesg)
{
#if _OLED>=1
    auto *d = GW_DISPLAY;
    d->clear();
    d->setFont(ArialMT_Plain_16);
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(0, 16, String(mesg));
    d->display();
 yield();
#endif
}



void msg_lLED(String mesg, String mesg2)
{
#if _OLED>=1
    auto *d = GW_DISPLAY;
    d->clear();
    d->setFont(ArialMT_Plain_10);
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(0, 0, String(mesg));
    d->drawString(0, 20, String(mesg2));
    d->display();
 yield();
#endif
}





void addr_oLED()
{
#if _OLED>=1
 #if _DUSB>=1
  Serial.print(F("OLED_ADDR=0x"));
  Serial.println(OLED_ADDR, HEX);
 #endif
#endif
}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_otaServer.ino"
# 20 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_otaServer.ino"
#if _OTA==1
# 30 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_otaServer.ino"
void setupOta(char *hostname) {

 ArduinoOTA.begin();

# if _MONITOR>=1
  mPrint("setupOta:: Started");
# endif


 ArduinoOTA.setHostname(hostname);

 ArduinoOTA.onStart([]() {
  String type;



   type = "sketch";




  Serial.println("Start updating " + type);
 });

 ArduinoOTA.onEnd([]() {
  Serial.println("\nEnd");
 });

 ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
 });

 ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR) Serial.println("End Failed");
 });

# if _MONITOR>=1
  if (debug>=1) {
   mPrint("Ready IP address: " + String(WiFi.localIP().toString()));
  }
# endif


#if _SERVER==2
 ESPhttpUpdate.rebootOnUpdate(false);

 server.on("/esp", HTTP_POST, [&](){

      HTTPUpdateResult ret = ESPhttpUpdate.update(server.arg("firmware"), "1.0.0");

      switch(ret) {
        case HTTP_UPDATE_FAILED:

   Serial.println(F("Update failed"));
            break;
        case HTTP_UPDATE_NO_UPDATES:

   Serial.println(F("Update not necessary"));
            break;
        case HTTP_UPDATE_OK:

   Serial.println(F("Update started"));
            ESP.restart();
            break;
  default:
   Serial.println(F("setupOta:: Unknown ret="));
      }
 });
#endif
}






void updateOtaa() {

 String response="";
 Serial.print(F("updateOtaa:: <unimplemented> IP="));
 printIP((IPAddress)WiFi.localIP(),'.',response);
 Serial.println(response);


}


#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_repeater.ino"
# 19 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_repeater.ino"
#if _REPEATER==1

#ifdef _TTNSERVER
# error "Please undefine _TTNSERVER, for _REPEATER"
#endif

#ifdef _THINGSERVER
# error "Please undefine _THINGSERVER, for _REPEATER"
#endif
# 48 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_repeater.ino"
int repeatLora(struct LoraUp * LUP) {

 struct LoraDown lDown;
 struct LoraDown * LDWN = & lDown;

 LDWN->tmst = LUP->tmst;
 LDWN->freq = freqs[(gwayConfig.ch-1)%3].dwnFreq;
 LDWN->size = LUP->size;
 LDWN->sf = LUP->sf;
 LDWN->powe = 14;
 LDWN->crc = 0;
 LDWN->iiq = 0x27;
 LDWN->imme = false;



 LDWN->payLoad = LUP->payLoad;

 yield();

 Serial.print("repeatLora:: size="); Serial.print(LDWN->size);
 Serial.print(", sf="); Serial.print(LDWN->sf);
 Serial.print(", iiq="); Serial.print(LDWN->iiq,HEX);
 Serial.print(", UP ch="); Serial.print(gwayConfig.ch,HEX);
 Serial.print(", DWN ch="); Serial.print((gwayConfig.ch-1)%3,HEX);
 Serial.print(", freq="); Serial.print(LDWN->freq);
 Serial.println();






 writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
 writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
 txLoraModem(LDWN);

 txDones=0;
 _event=1;
 _state=S_TXDONE;

 yield();
 startReceiver();





 Serial.print("repeatLora:: ch=");
 Serial.print(gwayConfig.ch);
 Serial.print(", to ch=");
 Serial.print((gwayConfig.ch-1)%3);
 Serial.print(", data=");

 for (int i=0; i< LDWN->size; i++) {
  Serial.print(LDWN->payLoad[i],HEX);
  Serial.print('.');
 }
 Serial.println();
 if (debug>=2) Serial.flush();

 return(1);
}


#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
# 22 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
#if _GATEWAYNODE==1

#include "LoRaCode.h"

unsigned char DevAddr[4] = _DEVADDR ;



#if _GPS==1




void smartDelay(uint32_t ms)
{
 uint32_t start = millis();
 do
 {
  while (sGps.available()) {
   gps.encode(sGps.read());
  }
  yield();
 } while (millis() - start < ms);
}
#endif
# 73 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
int LoRaSensors(uint8_t *buf) {

# if defined(_LCODE)
# if defined(_RAW)
# error "Only define ONE encoding in configNode.h, _LOCDE or _RAW"
# endif

  String response="";
  uint8_t tchars = 1;
  buf[0] = 0x86;

# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_MAIN)) {
   response += "LoRaSensors:: ";
  }
# endif


# if _GPS==1
   smartDelay(10);
   if ((millis() > 5000) && (gps.charsProcessed() < 10)) {
# if _MONITOR>=1
     mPrint("ERROR: No GPS data received: check wiring");
# endif
    return(0);
   }





# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)) {
    response += " Gps lcode:: lat="+String(gps.location.lat())+", lng="+String(gps.location.lng())+", alt="+String(gps.altitude.feet()/3.2808)+", sats="+String(gps.satellites.value());
   }
# endif
   tchars += lcode.eGpsL(gps.location.lat(), gps.location.lng(), gps.altitude.value(), gps.satellites.value(), buf + tchars);
# endif

# if _BATTERY==1
# if defined(ARDUINO_ARCH_ESP8266) || defined(ESP32)


    pinMode(35, INPUT);
    float volts=3.3 * analogRead(35) / 4095 * 2;
# else

    float volts=0;
# endif
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)){
    response += ", Battery V="+String(volts);
   }
# endif

   tchars += lcode.eBattery(volts, buf + tchars);
# endif



  lcode.eMsg(buf, tchars);

# if _MONITOR>=1
   mPrint(response);
# endif





# elif defined(_RAW)
  uint8_t tchars = 0;


# if _GPS==1
   smartDelay(10);
   if (millis() > 5000 && gps.charsProcessed() < 10) {
# if _MONITOR>=1
     mPrint("ERROR: No GPS data received: check wiring");
# endif
    return(0);
   }

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)){
    mPrint("Gps raw:: lat="+String(gps.location.lat())+", lng="+String(gps.location.lng())+", alt="+String(gps.altitude.feet()/3.2808)+", sats="+String(gps.satellites.value()) );

   }
# endif

   double lat = gps.location.lat();
   double lng = gps.location.lng();
   double alt = gps.altitude.feet() / 3.2808;
   memcpy((buf+tchars), &lat, sizeof(double)); tchars += sizeof(double);
   memcpy((buf+tchars), &lng, sizeof(double)); tchars += sizeof(double);
   memcpy((buf+tchars), &alt, sizeof(double)); tchars += sizeof(double);
# endif

# if _BATTERY==1
# if defined(ARDUINO_ARCH_ESP8266) || defined(ESP32)


    pinMode(35, INPUT);
    float volts=3.3 * analogRead(35) / 4095 * 2;
# else

    float volts=0;
# endif
   memcpy((buf+tchars), &volts, sizeof(float)); tchars += sizeof(float);

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_MAIN)){
    mPrint("Battery raw="+String(volts));
   }
# endif
# endif



# else
# error "Please define an encoding format as in configNode.h"
# endif



# if _DUSB>=1 && _GPS==1
 if ((debug>=2) && (pdebug & P_MAIN)) {
  Serial.print("GPS sensor");
  Serial.print("\tLatitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("\tLongitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("\tSatellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("\tAltitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("\tTime      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
 }
# endif

 return(tchars);
}
# 231 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
void mXor(uint8_t *buf, uint8_t *key)
{
 for (uint8_t i = 0; i < 16; ++i) buf[i] ^= key[i];
}
# 244 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
void shift_left(uint8_t * buf, uint8_t len)
{
    while (len--) {
        uint8_t next = len ? buf[1] : 0;

        uint8_t val = (*buf << 1);
        if (next & 0x80) val |= 0x01;
        *buf++ = val;
    }
}






void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2)
{

 memset(k1, 0, 16);


 AES_Encrypt(k1,key);


 if (k1[0] & 0x80) {
  shift_left(k1,16);
  k1[15] ^= 0x87;
 }
 else {
  shift_left(k1,16);
 }


 for (int i=0; i<16; i++) k2[i]=k1[i];

 if (k1[0] & 0x80) {
  shift_left(k2,16);
  k2[15] ^= 0x87;
 }
 else {
  shift_left(k2,16);
 }


 return;
}
# 315 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * NwkSKey, uint8_t dir)
{



 uint8_t Block_B[16];
 uint8_t X[16];
 uint8_t Y[16];



 Block_B[0]= 0x49;

 Block_B[1]= 0x00;
 Block_B[2]= 0x00;
 Block_B[3]= 0x00;
 Block_B[4]= 0x00;

 Block_B[5]= dir;

 Block_B[6]= DevAddr[3];
 Block_B[7]= DevAddr[2];
 Block_B[8]= DevAddr[1];
 Block_B[9]= DevAddr[0];

 Block_B[10]= (FrameCount & 0x00FF);
 Block_B[11]= ((FrameCount >> 8) & 0x00FF);
 Block_B[12]= 0x00;
 Block_B[13]= 0x00;

 Block_B[14]= 0x00;

 Block_B[15]= len;




 uint8_t k1[16];
 uint8_t k2[16];
 generate_subkey(NwkSKey, k1, k2);




 uint8_t micBuf[len+16];
 for (uint8_t i=0; i<16; i++) micBuf[i]=Block_B[i];
 for (uint8_t i=0; i<len; i++) micBuf[i+16]=data[i];




 uint8_t numBlocks = len/16 + 1;
 if ((len % 16)!=0) numBlocks++;




 uint8_t restBits = len%16;





 memset(X, 0, 16);




 for(uint8_t i= 0x0; i < (numBlocks - 1); i++) {
  for (uint8_t j=0; j<16; j++) Y[j] = micBuf[(i*16)+j];
  mXor(Y, X);
  AES_Encrypt(Y, NwkSKey);
  for (uint8_t j=0; j<16; j++) X[j] = Y[j];
 }







 if (restBits) {
  for (uint8_t i=0; i<16; i++) {
   if (i< restBits) Y[i] = micBuf[((numBlocks-1)*16)+i];
   if (i==restBits) Y[i] = 0x80;
   if (i> restBits) Y[i] = 0x00;
  }
  mXor(Y, k2);
 }
 else {
  for (uint8_t i=0; i<16; i++) {
   Y[i] = micBuf[((numBlocks-1)*16)+i];
  }
  mXor(Y, k1);
 }
 mXor(Y, X);
 AES_Encrypt(Y,NwkSKey);






 data[len+0]=Y[0];
 data[len+1]=Y[1];
 data[len+2]=Y[2];
 data[len+3]=Y[3];

 yield();

 return 4;
}


#if _CHECK_MIC==1
# 439 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
void checkMic(uint8_t *buf, uint8_t len, uint8_t *key)
{
 uint8_t cBuf[len+1];
 uint8_t NwkSKey[16] = _NWKSKEY;

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_RX)) {
  String response = "";
  for (int i=0; i<len; i++) {
   printHexDigit(buf[i], response);
   response += ' ';
  }
  mPrint("old="+response);
 }
# endif

 for (int i=0; i<len-4; i++) {
  cBuf[i] = buf[i];
 }
 len -=4;


 uint16_t FrameCount = cBuf[6] | cBuf[7] << 8;
 len += micPacket(cBuf, len, FrameCount, NwkSKey, 0);

 if ((debug>=2) && (pdebug & P_RX)) {
  String response = "";

  for (int i=0; i<len; i++) {
   printHexDigit(cBuf[i],response);
   response += " ";
  }
  mPrint("new="+response);
 }

}
#endif
# 503 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
int sensorPacket() {

 uint8_t buff_up[512];
 uint8_t message[64]={ 0 };

 uint8_t NwkSKey[16] = _NWKSKEY;
 uint8_t AppSKey[16] = _APPSKEY;
 uint8_t DevAddr[4] = _DEVADDR;



 struct LoraUp LUP;

 LUP.sf = 8;
 LUP.prssi = -50;
 LUP.rssicorr = 139;
 LUP.snr = 0;
# 528 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
 LUP.payLoad[0] = 0x40;





 LUP.payLoad[1] = DevAddr[3];
 LUP.payLoad[2] = DevAddr[2];
 LUP.payLoad[3] = DevAddr[1];
 LUP.payLoad[4] = DevAddr[0];

 LUP.payLoad[5] = 0x00;
 LUP.payLoad[6] = LoraUp.fcnt % 0x100;
 LUP.payLoad[7] = LoraUp.fcnt / 0x100;



 LUP.payLoad[8] = 0x01;


 LUP.size = 9;







 uint8_t PayLength = LoRaSensors((uint8_t *)(LUP.payLoad + LUP.size));

#if _DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO)) {
  String response="";
  Serial.print(F("old: "));
  for (int i=0; i<PayLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif


 uint8_t CodeLength = encodePacket(
  (uint8_t *)(LUP.payLoad + LUP.size),
  PayLength,
  (uint16_t)LoraUp.fcnt,
  DevAddr,
  AppSKey,
  0
 );

#if _DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO)) {
  Serial.print(F("new: "));
  for (int i=0; i<CodeLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif

 LUP.size += CodeLength;







 LUP.size += micPacket((uint8_t *)(LUP.payLoad), LUP.size, (uint16_t)LoraUp.fcnt, NwkSKey, 0);

#if _DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO)) {
  Serial.print(F("mic: "));
  for (int i=0; i<LUP.size; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif






 uint16_t buff_index = buildPacket(buff_up, &LUP, true);

 LoraUp.fcnt++;
 statc.msg_ttl++;
 statc.msg_sens++;
 switch(gwayConfig.ch) {
  case 0: statc.msg_sens_0++; break;
  case 1: statc.msg_sens_1++; break;
  case 2: statc.msg_sens_2++; break;
 }






 if ((LoraUp.fcnt % 10)==0) writeGwayCfg(_CONFIGFILE, &gwayConfig );

 if (buff_index > 512) {
  if (debug>0)
   mPrint("sensorPacket:: ERROR buffer size too large");
  return(-1);
 }

#if _GWAYSCAN==0
# ifdef _TTNSERVER
 if (!sendUdp(ttnServer, _TTNPORT, buff_up, buff_index)) {
  return(-1);
 }
# endif

# ifdef _THINGSERVER
 if (!sendUdp(thingServer, _THINGPORT, buff_up, buff_index)) {
  return(-1);
 }
# endif
#endif

#if _DUSB>=1


 if ((debug>=2) && (pdebug & P_RADIO)) {
  CodeLength = encodePacket(
   (uint8_t *)(LUP.payLoad + 9),
   PayLength,
   (uint16_t)LoraUp.fcnt-1,
   DevAddr,
   AppSKey,
   0
  );

  Serial.print(F("rev: "));
  for (int i=0; i<CodeLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.print(F(", addr="));
  for (int i=0; i<4; i++) {
   Serial.print(DevAddr[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif

 if (gwayConfig.cad) {

  _state = S_SCAN;
  sf = SF7;
  cadScanner();
 }
 else {

  _state = S_RX;
  rxLoraModem();
 }

 return(buff_index);
}

#endif


#if (_GATEWAYNODE==1) || (_LOCALSERVER>=1)
# 733 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_sensor.ino"
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction)
{

 uint8_t i, j;
 uint8_t Block_A[16];
 uint8_t bLen=16;

 uint8_t restLength = DataLength % 16;
 uint8_t numBlocks = DataLength / 16;
 if (restLength>0) numBlocks++;

 for(i = 1; i <= numBlocks; i++) {
  Block_A[0] = 0x01;

  Block_A[1] = 0x00;
  Block_A[2] = 0x00;
  Block_A[3] = 0x00;
  Block_A[4] = 0x00;

  Block_A[5] = Direction;

  Block_A[6] = DevAddr[3];
  Block_A[7] = DevAddr[2];
  Block_A[8] = DevAddr[1];
  Block_A[9] = DevAddr[0];

  Block_A[10] = (FrameCount & 0x00FF);
  Block_A[11] = ((FrameCount >> 8) & 0x00FF);
  Block_A[12] = 0x00;
  Block_A[13] = 0x00;

  Block_A[14] = 0x00;

  Block_A[15] = i;


  AES_Encrypt(Block_A, AppSKey);


  if ((i == numBlocks) && (restLength>0)) bLen = restLength;

  for(j = 0; j < bLen; j++) {
   *Data = *Data ^ Block_A[j];
   Data++;
  }
 }


 return(DataLength);
}

#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
# 70 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
void stateMachine()
{


 uint8_t flags = readRegister(REG_IRQ_FLAGS);
 uint8_t mask = readRegister(REG_IRQ_FLAGS_MASK);
 uint8_t intr = flags & ( ~ mask );
 uint8_t rssi;
 _event=0;

# if _MONITOR>=1
 if (intr != flags) {
  String response = "stateMachine:: ERROR: Int=0x"+String(intr,HEX)+", flags=0x"+String(flags,HEX)+", ";
  mStat(intr, response);
  mPrint(response);
 }
# endif





 if ((gwayConfig.hop) && (intr == 0x00))
 {
  mPrint("state:: hop==1");





  if ((_state == S_SCAN) || (_state == S_CAD)) {

   _event=0;

   uint32_t eventWait = EVENT_WAIT;
   switch (_state) {
    case S_INIT: eventWait = 0;
     break;

    case S_SCAN: eventWait = EVENT_WAIT * 1;
        mPrint("SCAN");
     break;
    case S_CAD: eventWait = EVENT_WAIT * 1;
        mPrint("CAD");
     break;
    case S_RX: eventWait = EVENT_WAIT * 8;
        mPrint("RX");
     break;
    case S_TX: eventWait = EVENT_WAIT * 1;
        mPrint("TX");
     break;
    case S_TXDONE: eventWait = EVENT_WAIT * 4;
        mPrint("TXDONE");
     break;
    default:
     eventWait=0;
# if _MONITOR>=1
      String response = "StateMachine:: Default: ";
      mStat(intr, response);
      mPrint(response);
# endif
   }






   uint32_t doneWait = DONE_WAIT;
   switch (sf) {
    case SF7: break;
    case SF8: doneWait *= 2; break;
    case SF9: doneWait *= 4; break;
    case SF10: doneWait *= 8; break;
    case SF11: doneWait *= 16; break;
    case SF12: doneWait *= 32; break;
    default:
     doneWait *= 1;
# if _MONITOR>=1
     if ((debug>=0) && (pdebug & P_PRE)) {
      mPrint("StateMachine:: PRE: DEF set");
     }
# endif
     break;
   }




   if (eventTime > micros()) eventTime=micros();
   if (doneTime > micros()) doneTime=micros();

   if (((micros() - doneTime) > doneWait) &&
    ((_state == S_SCAN) || (_state == S_CAD)))
   {
    _state = S_SCAN;
    hop();
    cadScanner();
# if _MONITOR>=1
    if ((debug >= 1) && (pdebug & P_PRE)) {
     String response = "DONE  :: ";
     mStat(intr, response);
     mPrint(response);
    }
# endif
    eventTime=micros();
    doneTime=micros();
    return;
   }



   if ((micros() - eventTime) > eventWait )
   {
    _state = S_SCAN;
    hop();
    cadScanner();
# if _MONITOR>=1
    if ((debug >= 2) && (pdebug & P_PRE)) {
     String response = "HOP ::  ";
     mStat(intr, response);
     mPrint(response);
    }
# endif
    eventTime=micros();
    doneTime=micros();
    return;
   }





# if _MONITOR>=1
   if ((debug>=3) && (pdebug & P_PRE)) {
    String response = "PRE:: eventTime=";
    response += String(eventTime);
    response += ", micros=";
    response += String(micros());
    response += ": ";
    mStat(intr, response);
    mPrint(response);
   }
# endif
  }


  else {

  }

  yield();

 }
# 232 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
 switch (_state)
 {




   case S_INIT:
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_PRE)) {
   mPrint("S_INIT");
  }
# endif

  writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );
  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00 );
  _event=0;
   break;







   case S_SCAN:





  if (intr & IRQ_LORA_CDDETD_MASK) {

   _state = S_RX;


   writeRegister(REG_DIO_MAPPING_1, (
    MAP_DIO0_LORA_RXDONE |
    MAP_DIO1_LORA_RXTOUT |
    MAP_DIO2_LORA_NOP |
    MAP_DIO3_LORA_CRC));



   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
    IRQ_LORA_RXDONE_MASK |
    IRQ_LORA_RXTOUT_MASK |
    IRQ_LORA_HEADER_MASK |
    IRQ_LORA_CRCERR_MASK));







   rssi = readRegister(REG_RSSI);
   _rssi = rssi;

   _event=0;
   detTime=micros();

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_PRE)) {
    String response = "SCAN:: ";
    mStat(intr, response);
    mPrint(response);
   }
# endif
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   opmode(OPMODE_RX_SINGLE);

  }







  else if (intr & IRQ_LORA_CDDONE_MASK) {

   opmode(OPMODE_CAD);
   rssi = readRegister(REG_RSSI);

# if _MONITOR>=1
   if ((debug>=2) && (pdebug & P_SCAN)) {
    String response = "SCAN:: CDDONE: ";
    mStat(intr, response);
    mPrint(response);
   }
# endif






   if (rssi > (RSSI_LIMIT - (gwayConfig.hop * 7)))
   {
# if _MONITOR>=1
    if ((debug>=2) && (pdebug & P_SCAN)) {
     String response = "SCAN:: -> CAD: ";
     mStat(intr, response);
     mPrint(response);
    }
# endif
    _state = S_CAD;
    _event=0;
   }



   else {
# if _MONITOR>=1
    if ((debug>=2) && (pdebug & P_SCAN)) {
     String response = "SCAN:: rssi=";
     response += String(rssi);
     response += ": ";
     mStat(intr, response);
     mPrint(response);
    }
# endif
    _state = S_SCAN;
   }


   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   doneTime = micros();

  }
# 380 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
  else if (intr == 0x00)
  {
   _event=0;
  }



  else {
# if _MONITOR>=1
   if ((debug>=0) && (pdebug & P_SCAN)) {
    String response = "SCAN unknown:: ";
    mStat(intr, response);
    mPrint(response);
   }
# endif
   _state=S_SCAN;

   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
  }

   break;
# 415 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
   case S_CAD:





  if (intr & IRQ_LORA_CDDETD_MASK) {


   writeRegister(REG_DIO_MAPPING_1, (
    MAP_DIO0_LORA_RXDONE |
    MAP_DIO1_LORA_RXTOUT |
    MAP_DIO2_LORA_NOP |
    MAP_DIO3_LORA_CRC ));


   _event=0;



   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
    IRQ_LORA_RXDONE_MASK |
    IRQ_LORA_RXTOUT_MASK |
    IRQ_LORA_HEADER_MASK |
    IRQ_LORA_CRCERR_MASK ));






   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );


   opmode(OPMODE_RX_SINGLE);

   delayMicroseconds( RSSI_WAIT );

   rssi = readRegister(REG_RSSI);
   _rssi = rssi;

   detTime = micros();
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_CAD)) {
    String response = "CAD:: ";
    mStat(intr, response);
    mPrint(response);
   }
# endif
   _state = S_RX;

  }




  else if (intr & IRQ_LORA_CDDONE_MASK) {




   if (((uint8_t)sf) < freqs[gwayConfig.ch].upHi) {

    sf = (sf_t)((uint8_t)sf+1);
    setRate(sf, 0x04);

    opmode(OPMODE_CAD);

    delayMicroseconds(RSSI_WAIT);
    rssi = readRegister(REG_RSSI);

# if _MONITOR>=1
    if ((debug>=3) && (pdebug & P_CAD)) {
     mPrint("S_CAD:: CDONE, SF=" + String(sf) );
    }


    _event=0;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);

    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );

# endif
   }




   else {


    _event=1;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );

    _state = S_SCAN;
    sf = (sf_t) freqs[gwayConfig.ch].upLo;
    cadScanner();

# if _MONITOR>=1
    if ((debug>=3) && (pdebug & P_CAD)) {
     mPrint("CAD->SCAN:: " + String(intr) );
    }
# endif
   }
   doneTime = micros();

  }







  else if (intr == 0x00) {
# if _MONITOR>=1
   if ((debug>=3) && (pdebug & P_CAD)) {
    mPrint("CAD:: intr is 0x00");
   }
# endif

  }




  else {
# if _MONITOR>=1
   if ( debug>=0) {
    mPrint("ERROR CAD: Unknown::" + String(intr) );
   }
# endif
   _state = S_SCAN;
   sf = SF7;
   cadScanner();


   _event=1;
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);

  }
   break;
# 568 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
   case S_RX:

  if (intr & IRQ_LORA_RXDONE_MASK) {

# if _CRCCHECK>=1





   if (intr & IRQ_LORA_CRCERR_MASK) {
# if _MONITOR>=1
    if ((debug>=0) && (pdebug & P_RX)) {
     String response = "^ CRC ERROR:: ";
     mStat(intr, response);
    }
# endif

    if ((gwayConfig.cad) || (gwayConfig.hop)) {
     sf = SF7;
     _state = S_SCAN;
     cadScanner();
    }
    else {
     _state = S_RX;
     rxLoraModem();
    }


    _event=0;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
    writeRegister(REG_IRQ_FLAGS, (uint8_t) (
     IRQ_LORA_RXDONE_MASK |
     IRQ_LORA_RXTOUT_MASK |
     IRQ_LORA_HEADER_MASK |
     IRQ_LORA_CRCERR_MASK ));

    break;
   }
# endif


# if _DUSB>=1 || _MONITOR>=1
    uint32_t rxDoneTime = micros();
# endif


   LoraUp.payLoad[0]= 0x00;
# 625 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
   if((LoraUp.size = receivePkt(LoraUp.payLoad)) <= 0) {
# if _MONITOR>=1
    if ((debug>=0) && (pdebug & P_RX)) {
     String response = "sMachine:: ERROR S-RX: size=" + String(LoraUp.size);
     mPrint(response);
    }
# endif

    _event=1;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);

    _state = S_SCAN;
    break;
   }

# if _MONITOR>=1
   if ((debug >=2) && (pdebug & P_RX)) {
    String response = "RXDONE:: dT=";
    response += String(rxDoneTime - detTime);
    mStat(intr, response);
    mPrint(response);
   }
# endif


   uint8_t value = readRegister(REG_PKT_SNR_VALUE);
   if ( value & 0x80 ) {

    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
    LoraUp.snr = -value;
   }
   else {

    LoraUp.snr = ( value & 0xFF ) >> 2;
   }


   LoraUp.prssi = readRegister(REG_PKT_RSSI);


   if (sx1276) {
    LoraUp.rssicorr = 157;
   } else {
    LoraUp.rssicorr = 139;
   }

   LoraUp.sf = readRegister(REG_MODEM_CONFIG2) >> 4;



   if (receivePacket() <= 0) {
# if _MONITOR>=1
    if ((debug>=0) && (pdebug & P_RX)) {
     mPrint("sMach:: ERROR receivePacket");
    }
# endif
   }



   if ((gwayConfig.cad) || (gwayConfig.hop)) {
    _state = S_SCAN;
    sf = SF7;
    cadScanner();
   }
   else {
    _state = S_RX;
    rxLoraModem();
   }

   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   eventTime=micros();
   _event=0;
  }
# 709 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
  else if (intr & IRQ_LORA_RXTOUT_MASK) {



   _event=0;
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00 );
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);




   if ((gwayConfig.cad) || (gwayConfig.hop)) {

# if _MONITOR>=1
    if ((debug>=2) && (pdebug & P_RX)) {
     String response = "RXTOUT:: ";
     mStat(intr, response);
     mPrint(response);
    }
# endif
    sf = SF7;
    cadScanner();
    _state = S_SCAN;
   }



   else {
    _state = S_RX;
    rxLoraModem();
   }

   eventTime=micros();
   doneTime = micros();

  }

  else if (intr & IRQ_LORA_HEADER_MASK) {



# if _MONITOR>=1
   if ((debug>=3 ) && (pdebug & P_RX)) {
    mPrint("RX HEADER:: " + String(intr));
   }
# endif

  }





  else if (intr == 0x00) {
# if _MONITOR>=1
   if ((debug>=3) && (pdebug & P_RX)) {
    mPrint("S_RX no INTR:: " + String(intr));
   }
# endif
  }




  else {
# if _MONITOR>=1
   if ((debug>=0) && (pdebug & P_RX)) {
    mPrint("R S_RX:: no RXDONE, RXTOUT, HEADER:: " + String(intr));
   }
# endif


  }

   break;
# 793 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
   case S_TX:




  if (intr == 0x00) {
# if _MONITOR>=1
   if ((debug>=3) && (pdebug & P_TX)) {
    mPrint("TX:: 0x00");
   }
# endif
   _event= 1;
  }

  txDones=0;
  _state=S_TXDONE;
  _event=1;

# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_TX)) {
   String response="TX fini:: ";
   mStat(intr, response);
   mPrint(response);
  }
# endif

  yield();

   break;
# 832 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_stateMachine.ino"
   case S_TXDONE:

  if (intr & IRQ_LORA_TXDONE_MASK) {

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_TX)) {
    String response = "v OK, stateMachine TXDONE: rcvd=";
    printInt(micros(),response);
    response += ", done= ";
    if (micros() < LoraDown.tmst) {
     response += "-" ;
     printInt((LoraDown.tmst-micros())/1000000, response );
    }
    else {
     printInt((micros()-LoraDown.tmst)/1000000, response);
    }
    mPrint(response);
   }
# endif


   if ((gwayConfig.cad) || (gwayConfig.hop)) {

    _state = S_SCAN;
    sf = SF7;
    cadScanner();
   }
   else {
    _state = S_RX;
    rxLoraModem();
   }

   _event=0;

   switch (protocol) {
    case 1:
# if _MONITOR>=1
     if ((debug>=3) && (pdebug & P_TX)) {
      mPrint("^ TX_ACK:: readUdp: protocol version 1");
     }
# endif
     break;
    case 2:
# if _MONITOR>=1
     if ((debug>=1) && (pdebug & P_TX)) {
      mPrint("^ TX_ACK:: readUDP: protocol version 2+");
     }
# endif
     break;
    default:
# if _MONITOR>=1
     if ((debug>=1) && (pdebug & P_TX)) {
      mPrint("^ TX_ACK:: readUDP: protocol version unknown");
     }
# endif
   }

   yield();



   buff[0]= buff_down[0];
   buff[1]= buff_down[1];
   buff[2]= buff_down[2];
   buff[3]= TX_ACK;


   buff[4]= 0;


   yield();


   Udp.beginPacket(remoteIpNo, remotePortNo);



   if (Udp.write((unsigned char *)buff, 12) != 12) {
# if _MONITOR>=1
    if (debug>=0) {
     mPrint("^ readUdp:: ERROR: PULL_ACK write");
    }
# endif
   }
   else {
# if _MONITOR>=1
    if ((debug>=1) && (pdebug & P_TX)) {
     mPrint("^ readUdp:: TX_ACK: micros="+String(micros()));
    }
# endif
   }

   if (!Udp.endPacket()) {
# if _MONITOR>=1
    if ((debug>=0) && (pdebug & P_TX)) {
     mPrint("^ readUdp:: PULL_ACK: ERROR Udp.endPacket");
    }
# endif
   }


   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0xFF);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0x00);

   yield();
  }


  else if ( intr != 0 ) {
# if _MONITOR>=1
   if ((debug>=0) && (pdebug & P_TX)) {
    String response = "TXDONE:: ERROR unknown intr=";
    mStat(intr, response);
    mPrint(response);
   }
# endif
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   _event=0;
   _state=S_SCAN;
  }



  else {




   if ( sendTime > micros() ) {
    sendTime = 0;
# if _MONITOR>=1
    if ((debug>=1) && (pdebug & P_TX)) {
     mPrint("v Warning:: intr=0 received, sendTime=0");
    }
# endif
   }

   else if(( _state == S_TXDONE ) && (( micros() - sendTime) > 7500000 )) {
# if _MONITOR>=1
    if ((debug>=1) && (pdebug & P_TX)) {
     mPrint("v Warning:: TXDONE not received, resetting receiver");
    }
# endif
    startReceiver();
   }
   else if ((debug>=2) && (pdebug & P_TX)) {

    if ((++txDones)%10==0)
     mPrint("stateMachine:: TXDONE, txDones="+String(txDones)+", uSecs elapsed="+String( micros() - sendTime));
   }
  }


# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_TX)) {



  }
# endif





   break;






   default:
# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_PRE)) {
   mPrint("ERR state=" + String(_state));
  }
# endif

  if ((gwayConfig.cad) || (gwayConfig.hop)) {
# if _MONITOR>=1
   if (debug>=0) {
    String response = "default:: ERROR: Unknown _state=";
    mStat(intr, response);
    mPrint(response);
   }
# endif
   _state = S_SCAN;
   sf = SF7;
   cadScanner();
   _event=0;
  }
  else
  {
   _state = S_RX;
   rxLoraModem();
   _event=0;
  }
  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
  writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
  eventTime=micros();

   break;
 }

 return;
}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_tcpTTN.ino"
# 34 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_tcpTTN.ino"
#ifdef _TTNROUTER

#ifdef _UDPROUTER
# error "Error: Please undefine _UDPROUTER if you like to use _TTNROUTER"
#endif
# 50 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_tcpTTN.ino"
void connectTtn() {
# error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

int readTtn(int Packetsize) {
#error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

int sendTtn(IPAddress server, int port, uint8_t *msg, int length) {
#error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
# 45 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
int sendPacket(uint8_t *buf, uint8_t len)
{
# 82 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
 char * bufPtr = (char *) (buf+4);
 bufPtr[len-4] = 0;

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_TX)) {
  mPrint("v sendPacket:: cmd="+ String(buf[3]) +", token=" + String(buf[2]<<8 | buf[1]) + ", " + String((char *)bufPtr));
 }
# endif



 auto error = deserializeJson(jsonBuffer, bufPtr);
 if (error) {
# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: ERROR: Json Decode: " + String(bufPtr) );
  }
  yield();
# endif
  return(-1);
 }





 JsonObject root = jsonBuffer.as<JsonObject>();

 LoraDown.tmst = (uint32_t) root["txpk"]["tmst"].as<unsigned long>();
 const char * data = root["txpk"]["data"];
 uint8_t psize = root["txpk"]["size"];

 const char * datr = root["txpk"]["datr"];
 const char * codr = root["txpk"]["codr"];
 const char * modu = root["txpk"]["modu"];
 const char * time = root["txpk"]["time"];

 LoraDown.modu = (char *) modu;
 LoraDown.codr = (char *) codr;

 LoraDown.ipol = root["txpk"]["ipol"];

 LoraDown.imme = root["txpk"]["imme"];
 LoraDown.powe = root["txpk"]["powe"];
 LoraDown.prea = root["txpk"]["prea"];
 LoraDown.ncrc = root["txpk"]["ncrc"];
 LoraDown.rfch = root["txpk"]["rfch"];
 LoraDown.iiq = (LoraDown.ipol==true ? 0x40: 0x27);
 LoraDown.crc = 0x00;

 LoraDown.sf = atoi(datr+2);
 int j; for (j=3; *(datr+j)!='W'; j++);
 LoraDown.bw = atoi(datr+j+1);

 LoraDown.size = base64_dec_len((char *) data, strlen(data));
 base64_decode((char *) payLoad, (char *) data, strlen(data));
 LoraDown.payLoad = payLoad;



 if (LoraDown.imme) {
# if _MONITOR>=1
  if ((debug >= 1) && (pdebug & P_TX)) {
   mPrint("sendPacket:: IMME");
  }
# endif
 }

 if (time!=0) {
# if _MONITOR>=1
  if ((debug >= 1) && (pdebug & P_TX)) {
   mPrint("sendPacket:: time="+String(time));
  }
# endif
 }

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_TX)) {
  String response = "v (" ;
  response += String(LoraDown.size);
  response += "): " ;
  for (j=0; j<LoraDown.size; j++) {
   printHexDigit(LoraDown.payLoad[j], response);
   response += " ";
  }
  mPrint(response);
 }
# endif


# if _PROFILER>=2
 uint8_t mhdr = (uint8_t) (LoraDown.payLoad[0] >> 5);
 switch (mhdr) {
  case 0x00:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Join Request");
     }
# endif
     break;
  case 0x01:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Join Accept");
     }
# endif

     break;
  case 0x02:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Unconfirmed Data Up");
     }
# endif
     break;
  case 0x03:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Unconfirmed Data Down");
     }
# endif

     break;
  case 0x04:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Confirmed Data Up");
     }
# endif
     break;
  case 0x05:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Confirmed Data Down");
     }
# endif

     break;
  case 0x06:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: RFU");
     }
# endif
     break;
  case 0x07:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: Proprietary");
     }
# endif
     break;
  default:
# if _MONITOR>=1
     if ((debug >= 1) && (pdebug & P_TX)) {
      mPrint("sendPacket:: mhdr undefined");
     }
# endif
 }
# endif

 yield();







 if (data != NULL) {
# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: LoraDown.size="+String(LoraDown.size)+", psize="+String(psize)+", strlen(data)="+String(strlen(data))+", data=" + String(data));
  }
# endif
 }
 else {
# if _MONITOR>=1
  if ((debug>=0) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: ERROR: data is NULL");
  }
# endif
  return(-1);
 }



#if _STRICT_1CH == 0
# 278 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
 LoraDown.powe = root["txpk"]["powe"];
 const float ff = root["txpk"]["freq"];


 LoraDown.freq = (uint32_t) freqs[gwayConfig.ch].dwnFreq;



#elif _STRICT_1CH == 1





 if (LoraDown.powe>14) LoraDown.powe=14;


 const uint32_t ff = (uint32_t) (freqs[gwayConfig.ch].dwnFreq);
 LoraDown.freq = (uint32_t)(ff & 0xFFFFFFFF);

 Serial.print("v _STRICT_1CH==1 done, freq=");
 Serial.print(LoraDown.freq);
 Serial.print(", HEX=");
 uint32_t effe = LoraDown.freq;
 Serial.print((effe >> 24) & 0xFF); Serial.print(" ");
 Serial.print((effe >> 16) & 0xFF); Serial.print(" ");
 Serial.print((effe >> 8) & 0xFF); Serial.print(" ");
 Serial.print((effe >> 0) & 0xFF); Serial.print(" ");
 Serial.println();

#elif _STRICT_1CH == 2





 uint32_t fff = (uint32_t)(root["txpk"]["freq"].as<double>() * 1000000);

 uint32_t diff = (freqs[gwayConfig.ch].dwnFreq > fff)
  ? (freqs[gwayConfig.ch].dwnFreq - fff)
  : (fff - freqs[gwayConfig.ch].dwnFreq);
 if (diff < 100000) {
  LoraDown.freq = (uint32_t) (freqs[gwayConfig.ch].dwnFreq) & 0xFFFFFFFF ;
  if ((debug>=2) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: _STRICT_1CH="+String(_STRICT_1CH)+", fff="+String(fff)+", abs");
  }
 }
 else {
  LoraDown.freq = fff & 0xFFFFFFFF;
  if ((debug>=2) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: _STRICT_1CH="+String(_STRICT_1CH)+", fff="+String(fff)+", fff");
  }
 }


#else
 mPrint("sendPacket:: ERROR: _STRICT_1CH value="+String(_STRICT_1CH)+" unknown");
#endif

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_TX)) {
   mPrint("v sendPacket:: _STRICT_1CH="+String(_STRICT_1CH)+", freq="+String(LoraDown.freq)+", sf="+String(LoraDown.sf));
 }
# endif

 yield();

 if (LoraDown.size != psize) {
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("v sendPacket:: WARNING size=" + String(LoraDown.size) + ", psize=" + String(psize) );
  }
# endif
 }


 statc.msg_down++;
 switch(statr[0].ch) {
  case 0: statc.msg_down_0++; break;
  case 1: statc.msg_down_1++; break;
  case 2: statc.msg_down_2++; break;
 }



 _state = S_TX;

 return 1;

}
# 388 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
int buildPacket(uint8_t *buff_up, struct LoraUp *LoraUp, bool internal)
{
 int32_t SNR;
    int16_t rssicorr;
 int16_t prssi;

 char cfreq[12] = {0};
 uint16_t buff_index=0;
 char b64[256];

 uint8_t *message = LoraUp->payLoad;
 char messageLength = LoraUp->size;

# if _CHECK_MIC==1
  unsigned char NwkSKey[16] = _NWKSKEY;
  checkMic(message, messageLength, NwkSKey);
# endif



 if (internal) {
  SNR = 12;
  prssi = 50;
  rssicorr = 157;
 }
 else {
  SNR = LoraUp->snr;
  prssi = LoraUp->prssi;
  rssicorr = LoraUp->rssicorr;
 }

#if _STATISTICS >= 1





 for (int m=( gwayConfig.maxStat -1); m>0; m--) statr[m]= statr[m-1];


# if _LOCALSERVER>=1
 statr[0].datal=0;
 int index;
 if ((index = inDecodes((char *)(LoraUp->payLoad+1))) >=0 ) {


  LoraUp->fcnt=LoraUp->payLoad[7]<<8 | LoraUp->payLoad[6];

  for (int k=0; (k<LoraUp->size) && (k<23); k++) {
   statr[0].data[k] = LoraUp->payLoad[k+9];
  };




  uint8_t DevAddr[4];
  DevAddr[0]= LoraUp->payLoad[4];
  DevAddr[1]= LoraUp->payLoad[3];
  DevAddr[2]= LoraUp->payLoad[2];
  DevAddr[3]= LoraUp->payLoad[1];

  statr[0].datal = encodePacket(
        (uint8_t *)(statr[0].data),
        LoraUp->size -9 -4,
        (uint16_t)LoraUp->fcnt,
        DevAddr,
        decodes[index].appKey,
        0);
 }
# endif

 statr[0].time = now();
 statr[0].ch = gwayConfig.ch;
 statr[0].prssi = prssi - rssicorr;
 statr[0].sf = LoraUp->sf;
 statr[0].upDown = 0;
 statr[0].node = ( message[1]<<24 | message[2]<<16 | message[3]<<8 | message[4] );
# if RSSI==1
 statr[0].rssi = _rssi - rssicorr;
# endif

# if _STATISTICS >= 2


 switch (statr[0].sf) {
  case SF7: statc.sf7++; break;
  case SF8: statc.sf8++; break;
  case SF9: statc.sf9++; break;
  case SF10: statc.sf10++; break;
  case SF11: statc.sf11++; break;
  case SF12: statc.sf12++; break;
 }
# endif

# if _STATISTICS >= 3
 if (statr[0].ch == 0) {
  statc.msg_ttl_0++;
  switch (statr[0].sf) {
   case SF7: statc.sf7_0++; break;
   case SF8: statc.sf8_0++; break;
   case SF9: statc.sf9_0++; break;
   case SF10: statc.sf10_0++; break;
   case SF11: statc.sf11_0++; break;
   case SF12: statc.sf12_0++; break;
  }
 }
 else
 if (statr[0].ch == 1) {
  statc.msg_ttl_1++;
  switch (statr[0].sf) {
   case SF7: statc.sf7_1++; break;
   case SF8: statc.sf8_1++; break;
   case SF9: statc.sf9_1++; break;
   case SF10: statc.sf10_1++; break;
   case SF11: statc.sf11_1++; break;
   case SF12: statc.sf12_1++; break;
  }
 }
 else
 if (statr[0].ch == 2) {
  statc.msg_ttl_2++;
  switch (statr[0].sf) {
   case SF7: statc.sf7_2++; break;
   case SF8: statc.sf8_2++; break;
   case SF9: statc.sf9_2++; break;
   case SF10: statc.sf10_2++; break;
   case SF11: statc.sf11_2++; break;
   case SF12: statc.sf12_2++; break;
  }
 }
# endif

#endif

# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_RADIO)) {
  Serial.print(F("R buildPacket:: pRSSI="));
  Serial.print(prssi-rssicorr);
  Serial.print(F(" RSSI: "));
  Serial.print(_rssi - rssicorr);
  Serial.print(F(" SNR: "));
  Serial.print(SNR);
  Serial.print(F(" Length: "));
  Serial.print((int)messageLength);
  Serial.print(F(" -> "));
  for (int i=0; i< messageLength; i++) {
     Serial.print(message[i],HEX);
     Serial.print(' ');
  }
  Serial.println();
  yield();
 }
# endif


# if _OLED>=1
    char timBuff[20];
    sprintf(timBuff, "%02i:%02i:%02i", hour(), minute(), second());

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    display.drawString(0, 0, "Time: " );
    display.drawString(40, 0, timBuff);

    display.drawString(0, 16, "RSSI: " );
    display.drawString(40, 16, String(prssi-rssicorr));

    display.drawString(70, 16, ",SNR: " );
    display.drawString(110, 16, String(SNR) );

 display.drawString(0, 32, "Addr: " );

    if (message[4] < 0x10) display.drawString( 40, 32, "0"+String(message[4], HEX)); else display.drawString( 40, 32, String(message[4], HEX));
 if (message[3] < 0x10) display.drawString( 61, 32, "0"+String(message[3], HEX)); else display.drawString( 61, 32, String(message[3], HEX));
 if (message[2] < 0x10) display.drawString( 82, 32, "0"+String(message[2], HEX)); else display.drawString( 82, 32, String(message[2], HEX));
 if (message[1] < 0x10) display.drawString(103, 32, "0"+String(message[1], HEX)); else display.drawString(103, 32, String(message[1], HEX));

    display.drawString(0, 48, "LEN: " );
    display.drawString(40, 48, String((int)messageLength) );
    display.display();

# endif





 int encodedLen = base64_enc_len(messageLength);

# if _MONITOR>=1
 if ((debug>=1) && (encodedLen>255) && (pdebug & P_RADIO)) {
  mPrint("R buildPacket:: b64 err, len=" + String(encodedLen));
  return(-1);
 }
# endif

 base64_encode(b64, (char *) message, messageLength);


 uint8_t token_h = (uint8_t)rand();
 uint8_t token_l = (uint8_t)rand();
# 603 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
 buff_up[0] = protocol;
 buff_up[1] = (uint8_t)token_h;
 buff_up[2] = (uint8_t)token_l;
 buff_up[3] = PUSH_DATA;


 buff_up[4] = MAC_array[0];
 buff_up[5] = MAC_array[1];
 buff_up[6] = MAC_array[2];
 buff_up[7] = 0xFF;
 buff_up[8] = 0xFF;
 buff_up[9] = MAC_array[3];
 buff_up[10] = MAC_array[4];
 buff_up[11] = MAC_array[5];

 buff_index = 12;



 memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[{", 10); buff_index += 10;




#ifdef _JSONENCODE

 StaticJsonDocument<400> doc;



 doc["chan"] = "" + gwayConfig.ch;
 doc["rfch"] = "0";
 doc["freq"] = "" + (freqs[gwayConfig.ch].upFreq / 1000000);
 doc["stat"] = "1";
 doc["modu"] = "LORA";
 doc["datr"] = "SF" + String(LoraUp->sf) + "BW" + String(freqs[gwayConfig.ch].upBW);
 doc["rssi"] = "" +(prssi-rssicorr);
 doc["lsnr"] = "" +(long)SNR;
 doc["codr"] = "4/5";



 encodedLen = base64_enc_len(messageLength);
 doc["size"] = "" + encodedLen;

 int len= base64_encode(doc["data"], (char *)message, messageLength);

 LoraUp->tmst = doc["tmst"] = "" + (uint32_t) micros() + _RXDELAY1;


 const char * p = (const char *) & (buff_up [buff_index]);
 int written = serializeJson(doc, (const char *)p, buff_index+20 );


#else


 ftoa((double)freqs[gwayConfig.ch].upFreq / 1000000, cfreq, 6);
 if ((LoraUp->sf<6) || (LoraUp->sf>12)) {
  LoraUp->sf=7;
 }






 buff_index += snprintf((char *)(buff_up + buff_index),
  RX_BUFF_SIZE-buff_index,
  "\"chan\":%1u,\"rfch\":%1u,\"freq\":%s,\"stat\":1,\"modu\":\"LORA\"" ,
  0, 0, cfreq);


 buff_index += snprintf((char *)(buff_up + buff_index), RX_BUFF_SIZE-buff_index
  , ",\"datr\":\"SF%uBW%u\",\"codr\":\"4/5\",\"lsnr\":%li,\"rssi\":%d,\"size\":%u,\"data\":\""
  , LoraUp->sf, freqs[gwayConfig.ch].upBW, (long)SNR, prssi-rssicorr, messageLength);


 encodedLen = base64_enc_len(messageLength);
 buff_index += base64_encode((char *)(buff_up + buff_index), (char *) message, messageLength);


 LoraUp->tmst = (uint32_t) micros()+ _RXDELAY1;




 buff_index += snprintf((char *)(buff_up + buff_index),
  RX_BUFF_SIZE-buff_index, "\",\"tmst\":%u",
  LoraUp->tmst);

#endif




 buff_up[buff_index] = '}';
 buff_up[buff_index+1] = ']';
 buff_up[buff_index+2] = '}';
 buff_index += 3;

 buff_up[buff_index] = 0;




# if _MAXSEEN>=1
  addSeen(listSeen, statr[0]);
# endif

# if _STAT_LOG==1


  addLog( (unsigned char *)(buff_up), buff_index );
# endif

# if _MONITOR>=1
 if ((debug>=1) && (pdebug & P_RX)) {
  mPrint("^ PUSH_DATA:: token="+String(token_h<<8 | token_l)+", data="+String((char *)(buff_up + 12))+", Buff_up Length="+String(buff_index));
 }
# endif


 return(buff_index);

}
# 747 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
int receivePacket()
{
 uint8_t buff_up[RX_BUFF_SIZE];
# 758 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
  if (LoraUp.size > 0) {

# ifdef _PROFILER
    int32_t startTime = micros();
# endif



            int build_index = buildPacket(buff_up, &LoraUp, false);





# if _REPEATER==1
   if (!repeatLora(&LoraUp)) {
    return(-3);
   }
# endif

# if _GWAYSCAN==0
# ifdef _TTNSERVER




   if (!sendUdp(ttnServer, _TTNPORT, buff_up, build_index)) {
    return(-1);
   }
# endif

   yield();
   Udp.flush();

# ifdef _PROFILER
   if ((debug>=1) && (pdebug & P_RX)) {
    int32_t endTime = micros();
    String response = "^ receivePacket:: end="; printInt(endTime,response);
    response += ", start="; printInt(startTime, response);
    response += ", diff=" +String(endTime-startTime) + " uSec";
    mPrint(response);
   }
# endif

# ifdef _THINGSERVER

   if (!sendUdp(thingServer, _THINGPORT, buff_up, build_index)) {
    return(-2);
   }
# endif

# endif

# if _LOCALSERVER>=1
# 834 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_txRx.ino"
   int index=0;
   if ((index = inDecodes((char *)(LoraUp.payLoad+1))) >=0 ) {

    uint8_t DevAddr[4];
    DevAddr[0]= LoraUp.payLoad[4];
    DevAddr[1]= LoraUp.payLoad[3];
    DevAddr[2]= LoraUp.payLoad[2];
    DevAddr[3]= LoraUp.payLoad[1];


# if _DUSB>=1
    if ((debug>=2) && (pdebug & P_RX)) {
     Serial.print(F("UP receivePacket:: Ind="));
     Serial.print(index);
     Serial.print(F(", Len="));
     Serial.print(LoraUp.size);
     Serial.print(F(", A="));
     for (int i=0; i<4; i++) {
      if (DevAddr[i]<0x0F) Serial.print('0');
      Serial.print(DevAddr[i],HEX);

     }

     Serial.print(F(", Msg="));
     for (int i=0; (i<statr[0].datal) && (i<23); i++) {
      if (statr[0].data[i]<0x0F) Serial.print('0');
      Serial.print(statr[0].data[i],HEX);
      Serial.print(' ');
     }
    }
# endif

   }
# if _MONITOR>=1
   else if ((debug>=2) && (pdebug & P_RX)) {
     mPrint("receivePacket:: No Index");
   }
# endif
# endif


   LoraUp.size = 0;
   LoraUp.payLoad[0] = 0x00;

   return(build_index);
        }

 return(0);

}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
# 22 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
#if defined(_UDPROUTER)


#if defined(_TTNROUTER)
#error "Please make sure that either _UDPROUTER or _TTNROUTER are defined but not both"
#endif
# 45 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
bool connectUdp()
{

 bool ret = false;
 unsigned int localPort = _LOCUDPPORT;
# if _MONITOR>=1
 if (debug>=1) {
  mPrint("Local UDP port=" + String(localPort));
 }
# endif

 if (Udp.begin(localPort) == 1) {
# if _MONITOR>=1
  if (debug>=1) {
   mPrint("UDP Connection successful");
  }
# endif
  ret = true;
 }
 else{
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("Connection failed");
  }
# endif
 }
 return(ret);
}
# 98 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
int readUdp(int packetSize)
{


 if (WlanConnect(10) < 0) {
# if _MONITOR>=1
   mPrint("v readUdp:: ERROR connecting to WLAN");
# endif
  Udp.flush();
  return(-1);
 }

 if (packetSize > TX_BUFF_SIZE) {
# if _MONITOR>=1
   mPrint("v readUdp:: ERROR package of size: " + String(packetSize));
# endif
  Udp.flush();
  return(-1);
 }



 if (Udp.read(buff_down, packetSize) < packetSize) {
# if _MONITOR>=1
   mPrint("v readUdp:: Reading less chars");
# endif
  return(-1);
 }

 yield();


 remoteIpNo = Udp.remoteIP();


 remotePortNo = Udp.remotePort();

 if (remotePortNo == 123) {

# if _MONITOR>=1
  if (debug>=0) {
   mPrint("v readUdp:: NTP msg rcvd");
  }
# endif
  gwayConfig.ntpErr++;
  gwayConfig.ntpErrTime = now();
  Udp.flush();
  return(0);
 }



 else {


  protocol= buff_down[0];
  uint16_t token= buff_down[2]<<8 | buff_down[1];
  uint8_t ident= buff_down[3];


# if _MONITOR>=1
  if ((debug>=3) && (pdebug & P_TX)) {
   mPrint("v readUdp:: message ident="+String(ident));
  }
# endif


  switch (ident) {
# 178 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case PUSH_DATA:
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_RX)) {
    mPrint("v PUSH_DATA:: size "+String(packetSize)+" From "+String(remoteIpNo.toString()));
   }
# endif
   Udp.flush();
  break;
# 196 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case PUSH_ACK:
# if _MONITOR>=1
   if ((debug>=2) && (pdebug & P_TX)) {
    char res[128];
    sprintf(res, "v PUSH_ACK:: token=%u, size=%u, IP=%d.%d.%d.%d, port=%d, protocol=%u ",
     (buff_down[2]<<8 | buff_down[1]),
     packetSize,
     remoteIpNo[0], remoteIpNo[1], remoteIpNo[2],remoteIpNo[3],
     remotePortNo,
     protocol);
    mPrint(res);
   }
# endif

  break;
# 223 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case PULL_DATA:
# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_RX)) {
    mPrint("v PULL_DATA");
   }
# endif
   Udp.flush();
  break;
# 245 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case PULL_ACK:
# if _MONITOR>=1
   if ((debug>=2) && (pdebug & P_TX)) {
    char res[128];
    sprintf(res, "v PULL_ACK:: token=%u, size=%u, IP=%d.%d.%d.%d, port=%d, protocol=%u ",
     (buff_down[2]<<8 | buff_down[1]),
     packetSize,
     remoteIpNo[0], remoteIpNo[1], remoteIpNo[2],remoteIpNo[3],
     remotePortNo,
     protocol);
    mPrint(res);
   }
# endif

   yield();
   Udp.flush();

  break;
# 283 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case PULL_RESP:

   if (protocol==0x01) {
    token = 0;
    buff_down[2]=0;
    buff_down[1]=0;
   }


# ifdef _PROFILER
   if ((debug>=1) && (pdebug & P_TX)) {
    char res[128];
    sprintf(res, "v PULL_RESP:: token=%u, size=%u, IP=%d.%d.%d.%d, port=%d, prot=%u, secs=%lu",
     token,
     (uint16_t) LoraDown.fcnt,

     remoteIpNo[0], remoteIpNo[1], remoteIpNo[2], remoteIpNo[3],
     remotePortNo,
     protocol,
     (unsigned long) micros()/1000000
    );
    mPrint(res);
   }
# endif



   sendTime = micros();




   if (sendPacket(buff_down, packetSize) < 0) {
# if _MONITOR>=1
    if (debug>=0) {
     mPrint("v readUdp:: ERROR: PULL_RESP sendPacket failed");
    }
# endif
    Udp.flush();
    return(-1);
   }




   if (loraWait(&LoraDown) == 0) {
    _state=S_CAD;
    _event=1;
    break;
   }



   txLoraModem(&LoraDown);


   for (int m=(gwayConfig.maxStat -1); m>0; m--) statr[m]= statr[m-1];


# if _MONITOR>=1
# 362 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
# if _LOCALSERVER>=2
    uint8_t DevAddr[4];
    int index;


    if ((index = inDecodes((char *)(LoraDown.payLoad+1))) >= 0 ) {

     LoraDown.fcnt= LoraDown.payLoad[7]<<8 | LoraDown.payLoad[6];


     strncpy ((char *)statr[0].data, (char *)LoraDown.payLoad+9, LoraDown.size-9-4);

     if ((LoraDown.size-9-4<=0) || (LoraDown.size-9-4>=30)) {

# if _MONITOR>=1
      if (debug>=1) {
       mPrint("PULL_RESP:: WARNING size="+String(LoraDown.size-9-4));
      }
# endif
     }
     else {

     }

     DevAddr[0]= LoraDown.payLoad[4];
     DevAddr[1]= LoraDown.payLoad[3];
     DevAddr[2]= LoraDown.payLoad[2];
     DevAddr[3]= LoraDown.payLoad[1];

     statr[0].datal = encodePacket(
          (uint8_t *)(statr[0].data),
          LoraDown.size -9 -4,
          (uint16_t)LoraDown.fcnt,
          DevAddr,
          decodes[index].appKey,
          1
     );
    }
    else {
# if _MONITOR >= 1
     if ((debug>=1) && (pdebug & P_MAIN)) {
      String response ="v PULL_RESP:: index inDecodes not found, Addr=";
      response+=
       String(LoraDown.payLoad[4],HEX) + " " +
       String(LoraDown.payLoad[3],HEX) + " " +
       String(LoraDown.payLoad[2],HEX) + " " +
       String(LoraDown.payLoad[1],HEX);
      mPrint(response);
     }
# endif
    }
# elif _LOCALSERVER==1

    statr[0].datal = 0;
# else

# endif


   if ((debug>=1) && (pdebug & P_TX)) {

    String response = "v txLoraModem hi:: ";
    printDwn(&LoraDown, response);

    response += " datal=" + String(statr[0].datal);
    response += " data= [ ";
    for (int i=0; i< statr[0].datal; i++) {
     response += String(statr[0].data[i], HEX) + " ";
    }
    response += "]";
    mPrint(response);

    yield();

    response = "v txLoraModem lo:: ";

# if _LOCALSERVER>=2

     if (statr[0].datal>24) {
      mPrint("readUDP:: ERROR: statr.datal larger than 24");
      response+= ", statr[0].datal=" + String(statr[0].datal);
      statr[0].datal=24;
     }

     response+= "data=[ " ;

     if ((statr[0].datal < 0) || (statr[0].datal > 24)) {
      mPrint("ERROR datal<0");
      statr[0].datal=0;
     }
     else for (int i=0; i<statr[0].datal; i++) {
      response += String(statr[0].data[i],HEX) + " ";
     }

     response += "], addr=";
     printHex((IPAddress)DevAddr, ':', response);

     response += ", d_fcnt=" + String(LoraDown.fcnt);
# endif

    response += ", size=" + String(LoraDown.size);

    response += ", old=[ ";
    for(int i=0; i<LoraDown.size; i++) {
     printHexDigit(LoraDown.payLoad[i],response);
     response += " ";
    }
    response += "]";

    mPrint(response);
   }

# endif

   statr[0].time = now();
   statr[0].ch = gwayConfig.ch;
   statr[0].sf = LoraDown.sf;
   statr[0].upDown = 1;
   statr[0].node = (
     LoraDown.payLoad[1]<<24 |
     LoraDown.payLoad[2]<<16 |
     LoraDown.payLoad[3]<<8 |
     LoraDown.payLoad[4]
   );

   addSeen(listSeen, statr[0]);

# if RSSI>=1
    statr[0].rssi = _rssi - rssicorr;
# endif





   txDones=0;
   _state=S_TXDONE;
   _event=1;

   yield();

  break;
# 514 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
  case TX_ACK:

   if (protocol == 1) {
# if _MONITOR>=1
    if ((debug>=1) && (pdebug & P_TX)) {
     mPrint("^ TX_ACK:: readUdp: protocol version 1");



    }
# endif
    break;
   }

# if _MONITOR>=1
   if ((debug>=1) && (pdebug & P_TX)) {
    mPrint("^ TX_ACK:: readUDP: protocol version 2+");
   }
# endif
  break;


  default:
# if _GATEWAYMGT==1

    gateway_mgt(packetSize, buff_down);
# endif
# if _MONITOR>=1
    mPrint(", ERROR ident not recognized="+String(ident));
# endif
  break;
  }

# if _MONITOR>=1
  if ((debug>=3) && (pdebug & P_TX)) {
   String response= "v readUdp:: ident="+String(ident,HEX);
   response+= ", tmst=" + String(LoraDown.tmst);
   response+= ", imme=" + String(LoraDown.imme);
   response+= ", sf=" + String(LoraDown.sf);
   response+= ", freq=" + String(LoraDown.freq);
   if (debug>=3) {
    if (packetSize > 4) {
     response+= ", size=" + String(packetSize) + ", data=";
     buff_down[packetSize] = 0;
     response+=String((char *)(buff_down+4));
    }
   }
   mPrint(response);
  }
# endif


  return packetSize;
 }
}
# 586 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
int sendUdp(IPAddress server, int port, uint8_t *msg, uint16_t length)
{

 if (WlanConnect(3) < 0) {
# if _MONITOR>=1
  if (pdebug & P_MAIN) {
   mPrint("sendUdp: ERROR not connected to WiFi");
  }
# endif
  Udp.flush();
  return(0);
 }



# if _MONITOR>=1
 if ((debug>=2) && (pdebug & P_MAIN)) {
  mPrint("sendUdp: WlanConnect connected to="+WiFi.SSID()+". Server IP="+ String(WiFi.localIP().toString()) );
 }
# endif

 if (!Udp.beginPacket(server, (int) port)) {
# if _MONITOR>=1
  if ( debug>=0 ) {
   mPrint("M sendUdp:: ERROR Udp.beginPacket");
  }
# endif
  return(0);
 }

 yield();

 if (Udp.write((unsigned char *)msg, length) != length) {
# if _MONITOR>=1
  if ( debug>=0 ) {
   mPrint("sendUdp:: ERROR Udp write");
  }
# endif
  Udp.endPacket();
  return(0);
 }

 yield();

 if (!Udp.endPacket()) {
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("sendUdp:: ERROR Udp.endPacket");
  }
# endif
  return(0);
 }
 return(1);
}
# 656 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
void pullData()
{
# if _GWAYSCAN==1
  mPrint("pull_Data:: Called when _GWAYSCAN==1");
  return;
# endif

    uint8_t pullDataReq[13];
    int pullIndex = 0;

 uint8_t token_h = (uint8_t)rand();
    uint8_t token_l = (uint8_t)rand();


    pullDataReq[0] = protocol;
    pullDataReq[1] = token_l;
    pullDataReq[2] = token_h;
    pullDataReq[3] = PULL_DATA;


    pullDataReq[4] = MAC_array[0];
    pullDataReq[5] = MAC_array[1];
    pullDataReq[6] = MAC_array[2];
    pullDataReq[7] = 0xFF;
    pullDataReq[8] = 0xFF;
    pullDataReq[9] = MAC_array[3];
    pullDataReq[10] = MAC_array[4];
    pullDataReq[11] = MAC_array[5];

    pullDataReq[12] = 0;

    pullIndex = 12;

 uint8_t *pullPtr = pullDataReq;


# ifdef _TTNSERVER
  sendUdp(ttnServer, _TTNPORT, pullDataReq, pullIndex);
  yield();
# endif

# ifdef _THINGSERVER
  sendUdp(thingServer, _THINGPORT, pullDataReq, pullIndex);
# endif

# if _MONITOR>=1
 if (pullPtr != pullDataReq) {
  mPrint("pullPtr != pullDatReq");
 }

    if ((debug>=1) && (pdebug & P_RX)) {
  yield();
  mPrint("^ PULL_DATA:: token=" +String(token_h<<8 | token_l) +", len=" + String(pullIndex) );
  Serial.print("v Gateway EUI=");
  for (int i=0; i<pullIndex; i++) {
   Serial.print(pullDataReq[i],HEX);
   Serial.print(':');
  }
  Serial.println();
 }
#endif

 return;

}
# 730 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_udpSemtech.ino"
void sendStat()
{

    uint8_t status_report[STATUS_SIZE];
    char stat_timestamp[32];
 char clat[10]={0};
 char clon[10]={0};

    int stat_index=0;
 uint8_t token_h = (uint8_t)rand();
    uint8_t token_l = (uint8_t)rand();


    status_report[0] = protocol;
 status_report[1] = token_h;
    status_report[2] = token_l;
    status_report[3] = PUSH_DATA;


    status_report[4] = MAC_array[0];
    status_report[5] = MAC_array[1];
    status_report[6] = MAC_array[2];
    status_report[7] = 0xFF;
    status_report[8] = 0xFF;
    status_report[9] = MAC_array[3];
    status_report[10] = MAC_array[4];
    status_report[11] = MAC_array[5];

    stat_index = 12;


 sprintf(stat_timestamp, "%04d-%02d-%02d %02d:%02d:%02d CET", year(),month(),day(),hour(),minute(),second());


 ftoa(lat,clat,5);
 ftoa(lon,clon,5);


 yield();

    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index,
  "{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}",
  stat_timestamp, clat, clon, (int)alt, statc.msg_ttl, statc.msg_ok, statc.msg_down, 0, 0, 0, platform, email, description);

 yield();

    stat_index += j;
    status_report[stat_index] = 0;

# if _MONITOR>=1
    if ((debug>=2) && (pdebug & P_RX)) {
  mPrint("RX stat update: <"+String(stat_index)+"> "+String((char *)(status_report+12)) );
 }
# endif

 if (stat_index > STATUS_SIZE) {
# if _MONITOR>=1
   mPrint("sendStat:: ERROR buffer too big");
# endif
  return;
 }

#if _GWAYSCAN==0

# ifdef _TTNSERVER
  sendUdp(ttnServer, _TTNPORT, status_report, stat_index);
# endif

# ifdef _THINGSERVER
  yield();
  sendUdp(thingServer, _THINGPORT, status_report, stat_index);
# endif
#endif

 return;

}


#endif
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
# 34 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void printInt (uint32_t i, String & response)
{
 response +=(String(i/1000000) + "." + String(i%1000000));
}






#define printReg(x) {int i=readRegister(x); if(i<=0x0F) Serial.print('0'); Serial.print(i,HEX);}

void printRegs(struct LoraDown *LoraDown, String & response)
{
 response += "v FIFO(0x00)=0x" + String(readRegister(REG_FIFO),HEX);
 response += "v OPMODE(0x01)=0x" + String(readRegister(REG_OPMODE),HEX);
 response += "v FRF(0x06)=0x" + String(readRegister(REG_FRF_MSB),HEX);
 response += "     (0x07)=0x" + String(readRegister(REG_FRF_MID),HEX);
 response += "     (0x08)=0x" + String(readRegister(REG_FRF_LSB),HEX);

 response += "v PREAMBLE_MSB (0x20)=0x" + String(readRegister(REG_PREAMBLE_MSB),HEX);
 response += "v PREAMBLE_LSB (0x21)=0x" + String(readRegister(REG_PREAMBLE_LSB),HEX);

# if _DUSB>=1
 if (debug>=1) {

  Serial.print("v PAC                  (0x09)\t=0x"); printReg(REG_PAC); Serial.println();
  Serial.print("v PARAMP               (0x0A)\t=0x"); printReg(REG_PARAMP); Serial.println();
  Serial.print("v REG_OCP              (0x0B)\t=0x"); printReg(REG_OCP); Serial.println();
  Serial.print("v LNA                  (0x0C)\t=0x"); printReg(REG_LNA); Serial.println();
  Serial.print("v FIFO_ADDR_PTR        (0x0D)\t=0x"); printReg(REG_FIFO_ADDR_PTR); Serial.println();
  Serial.print("v FIFO_TX/RX_BASE_AD   (0x0E/0x0F)\t=0x"); printReg(REG_FIFO_TX_BASE_AD);
   Serial.print("/"); printReg(REG_FIFO_RX_BASE_AD); Serial.println();

  Serial.print("v FIFO_RX_CURRENT_ADDR (0x10)\t=0x"); printReg(REG_FIFO_RX_CURRENT_ADDR); Serial.println();






  Serial.print("v PREAMBLE_MSB         (0x20)\t=0x"); printReg(REG_PREAMBLE_MSB); Serial.println();
  Serial.print("v PREAMBLE_LSB         (0x21)\t=0x"); printReg(REG_PREAMBLE_LSB); Serial.println();
  Serial.print("v REG_PAYLOAD_LENGTH   (0x22)\t="); Serial.println(readRegister(REG_PAYLOAD_LENGTH));
  Serial.print("v MAX_PAYLOAD_LENGTH   (0x23)\t="); Serial.println(readRegister(REG_MAX_PAYLOAD_LENGTH));
  Serial.print("v REG_HOP_PERIOD       (0x24)\t="); Serial.println(readRegister(REG_HOP_PERIOD));
  Serial.print("v FIFO_RX_BYTE_ADDR_PTR(0x25)\t=0x"); printReg(REG_FIFO_RX_BYTE_ADDR_PTR); Serial.println();

  Serial.println("");
 }
# endif


 return;
}
# 102 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void printDwn(struct LoraDown *LoraDown, String & response)
{
 uint32_t i= LoraDown->tmst;
 uint32_t m= LoraDown->usec;

 response += "micr="; printInt(m, response);
 response += ", tmst="; printInt(i, response);

 response += ", wait=";
 if (i>m) {
  response += String(i-m);
 }
 else {
  response += "(";
  response += String(m-i);
  response += ")";
 }

 response += ", freq=" +String(LoraDown->freq);
 response += ", sf=" +String(LoraDown->sf);
 response += ", bw=" +String(LoraDown->bw);
 response += ", datr=" +String(LoraDown->datr);
 response += ", powe=" +String(LoraDown->powe);
 response += ", crc=" +String(LoraDown->crc);
 response += ", imme=" +String(LoraDown->imme);
 response += ", iiq=" +String(LoraDown->iiq, HEX);
 response += ", prea=" +String(LoraDown->prea);
 response += ", rfch=" +String(LoraDown->rfch);
 response += ", ncrc=" +String(LoraDown->ncrc);
 response += ", size=" +String(LoraDown->size);
 response += ", strict=" +String(_STRICT_1CH);

 response += ", a=";
 uint8_t DevAddr [4];
  DevAddr[0] = LoraDown->payLoad[4];
  DevAddr[1] = LoraDown->payLoad[3];
  DevAddr[2] = LoraDown->payLoad[2];
  DevAddr[3] = LoraDown->payLoad[1];
 printHex((IPAddress)DevAddr, ':', response);

 yield();

 return;
}
# 159 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void printIP(IPAddress ipa, const char sep, String & response)
{
 response += (String)ipa[0] + sep;
 response += (String)ipa[1] + sep;
 response += (String)ipa[2] + sep;
 response += (String)ipa[3];
}





void printHex(uint32_t hexa, const char sep, String & response)
{
# if _MONITOR>=1
 if ((debug>=0) && (hexa==0)) {
  mPrint("printHex:: hexa amount to convert is 0");
 }
# endif

 uint8_t * h = (uint8_t *)(& hexa);

 if (h[0]<016) response += '0'; response += String(h[0], HEX) + sep;
 if (h[1]<016) response += '0'; response += String(h[1], HEX) + sep;
 if (h[2]<016) response += '0'; response += String(h[2], HEX) + sep;
 if (h[3]<016) response += '0'; response += String(h[3], HEX) + sep;
}




void printHexDigit(uint8_t digit, String & response)
{

    if(digit < 0x10)
        response += '0';
    response += String(digit,HEX);
}
# 214 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void mPrint(String txt)
{

# if _DUSB>=1
 if (gwayConfig.dusbStat>=1) {
  Serial.println(txt);
 }
# endif

#if _MONITOR>=1
 time_t tt = now();

 monitor[iMoni].txt = "";
 stringTime(tt, monitor[iMoni].txt);

 monitor[iMoni].txt += "- " + String(txt);



 iMoni = (iMoni+1) % gwayConfig.maxMoni ;

#endif

 return;
}
# 253 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
int mStat(uint8_t intr, String & response)
{
# if _MONITOR>=1

 if (debug>=0) {

  response += "I=";

  if (intr & IRQ_LORA_RXTOUT_MASK) response += "RXTOUT ";
  if (intr & IRQ_LORA_RXDONE_MASK) response += "RXDONE ";
  if (intr & IRQ_LORA_CRCERR_MASK) response += "CRCERR ";
  if (intr & IRQ_LORA_HEADER_MASK) response += "HEADER ";
  if (intr & IRQ_LORA_TXDONE_MASK) response += "TXDONE ";
  if (intr & IRQ_LORA_CDDONE_MASK) response += "CDDONE ";
  if (intr & IRQ_LORA_FHSSCH_MASK) response += "FHSSCH ";
  if (intr & IRQ_LORA_CDDETD_MASK) response += "CDDETD ";

  if (intr == 0x00) response += "  --  ";

  response += ", CH=" + String(gwayConfig.ch);

  response += ", SF=" + String(sf);

  response += ", E=" + String(_event);

  response += ", S=";
  switch (_state) {
   case S_TXDONE:
    response += "S_TXDONE";
   break;
   case S_INIT:
    response += "S_INIT ";
   break;
   case S_SCAN:
    response += "S_SCAN ";
   break;
   case S_CAD:
    response += "S_CAD  ";
   break;
   case S_RX:
    response += "S_RX   ";
   break;
   case S_TX:
    response += "S_TX   ";
   break;
   default:
    response += " -- ";
  }
  response += ", eT=";
  response += String( micros() - eventTime );

  response += ", dT=";
  response += String( micros() - doneTime );
 }
# endif
 return(1);
}
# 324 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void ftoa(float f, char *val, int p)
{
 int j=1;
 int ival, fval;
 char b[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

 for (int i=0; i< p; i++) { j= j*10; }

 ival = (int) f;
 fval = (int) ((f- ival)*j);
 if (fval<0) fval = -fval;

 if ((f<0) && (ival == 0)) strcat(val, "-");
 strcat(val,itoa(ival,b,10));
 strcat(val,".");

 itoa(fval,b,10);
 for (unsigned int i=0; i<(p-strlen(b)); i++) {
  strcat(val,"0");
 }


 strcat(val,b);
}
# 358 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void printDigits(uint32_t digits)
{

    if(digits < 10)
        Serial.print(F("0"));
    Serial.print(digits);
}
# 381 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
static void stringTime(time_t t, String & response)
{

 if (t==0) { response += "--"; return; }




 time_t eTime = t;


 uint8_t _hour = hour(eTime);
 uint8_t _minute = minute(eTime);
 uint8_t _second = second(eTime);

 uint8_t _month = month(eTime);
 uint8_t _day = day(eTime);

 switch(weekday(eTime)) {
  case 1: response += "Sun "; break;
  case 2: response += "Mon "; break;
  case 3: response += "Tue "; break;
  case 4: response += "Wed "; break;
  case 5: response += "Thu "; break;
  case 6: response += "Fri "; break;
  case 7: response += "Sat "; break;
 }
 if (_day < 10) response += "0"; response += String(_day) + "-";
 if (_month < 10) response += "0"; response += String(_month) + "-";
 response += String(year(eTime)) + " ";

 if (_hour < 10) response += "0"; response += String(_hour) + ":";
 if (_minute < 10) response += "0"; response += String(_minute) + ":";
 if (_second < 10) response += "0"; response += String(_second);
}





int sendNtpRequest(IPAddress timeServerIP)
{
 const int NTP_PACKET_SIZE = 48;
 uint8_t packetBuffer[NTP_PACKET_SIZE];

 memset(packetBuffer, 0, NTP_PACKET_SIZE);

 packetBuffer[0] = 0b11100011;
 packetBuffer[1] = 0;
 packetBuffer[2] = 6;
 packetBuffer[3] = 0xEC;

 packetBuffer[12] = 49;
 packetBuffer[13] = 0x4E;
 packetBuffer[14] = 49;
 packetBuffer[15] = 52;


 if (!sendUdp( (IPAddress) timeServerIP, (int) 123, packetBuffer, NTP_PACKET_SIZE)) {
  gwayConfig.ntpErr++;
  gwayConfig.ntpErrTime = now();
  return(0);
 }
 return(1);

}
# 459 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
int getNtpTime(time_t *t)
{
 gwayConfig.ntps++;

    if (!sendNtpRequest(ntpServer))
 {
# if _MONITOR>=1
  if (debug>=0) {
   mPrint("utils:: ERROR getNtpTime: sendNtpRequest failed");
  }
# endif
  return(0);
 }

 const int NTP_PACKET_SIZE = 48;
 uint8_t packetBuffer[NTP_PACKET_SIZE];
 memset(packetBuffer, 0, NTP_PACKET_SIZE);

    uint32_t beginWait = millis();
 delay(10);
    while (millis() - beginWait < 1500)
 {
  int size = Udp.parsePacket();
  if ( size >= NTP_PACKET_SIZE ) {

   if (Udp.read(packetBuffer, NTP_PACKET_SIZE) < NTP_PACKET_SIZE) {
# if _MONITOR>=1
    if (debug>=0) {
     mPrint("getNtpTime:: ERROR packetsize too low");
    }
# endif
    break;
   }
   else {

    uint32_t secs;
    secs = packetBuffer[40] << 24;
    secs |= packetBuffer[41] << 16;
    secs |= packetBuffer[42] << 8;
    secs |= packetBuffer[43];


    *t = (time_t)(secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
    Udp.flush();
    return(1);
   }
   Udp.flush();
  }
  delay(100);
    }

 Udp.flush();



 gwayConfig.ntpErr++;
 gwayConfig.ntpErrTime = now();

# if _MONITOR>=1
 if ((debug>=3) && (pdebug & P_MAIN)) {
  mPrint("getNtpTime:: WARNING read time failed");
 }
# endif

 return(0);
}





#if _NTP_INTR==1
void setupTime()
{
  time_t t;
  getNtpTime(&t);
  setSyncProvider(t);
  setSyncInterval(_NTP_INTERVAL);
}
#endif
# 551 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
int SerialName(uint32_t a, String & response)
{
#if _TRUSTED_NODES>=1
 uint8_t * in = (uint8_t *)(& a);
 uint32_t id = ((in[0]<<24) | (in[1]<<16) | (in[2]<<8) | in[3]);

 for (unsigned int i=0; i< (sizeof(nodes)/sizeof(nodex)); i++) {

  if (id == nodes[i].id) {
# if _MONITOR>=1
   if ((debug>=3) && (pdebug & P_MAIN )) {
    mPrint("SerialName:: i="+String(i)+", Name="+String(nodes[i].nm)+". for node=0x"+String(nodes[i].id,HEX));
   }
# endif

   response += nodes[i].nm;
   return(i);
  }
 }
#endif

 return(-1);
}


#if _LOCALSERVER>=1
# 585 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
int inDecodes(char * id) {

 uint32_t ident = ((id[3]<<24) | (id[2]<<16) | (id[1]<<8) | id[0]);

 for (unsigned int i=0; i< (sizeof(decodes)/sizeof(codex)); i++) {
  if (ident == decodes[i].id) {
   return(i);
  }
 }
 return(-1);
}
#endif
# 610 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_utils.ino"
void die(String s)
{
# if _MONITOR>=1
 mPrint(s);
# endif

# if _DUSB>=1
 Serial.println(s);
 if (debug>=2) Serial.flush();
# endif

 delay(50);
 abort();
}






void gway_failed(const char *file, uint16_t line) {
#if _MONITOR>=1
 String response = "Program failed in file: ";
 response += String(file);
 response += ", line: ";
 response += String(line);
 mPrint(response);
#endif
}
# 1 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
# 39 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
#if _SERVER==1
# 67 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
boolean YesNo()
{
 boolean ret = false;
 String response = "";
 response += "<script>";

 response += "var ch = \"\"; ";
 response += "function ynDialog(s,y) {";
 response += "  try { adddlert(s); }";
 response += "  catch(err) {";
 response += "    ch  = \" \" + s + \".\\n\\n\"; ";
 response += "    ch += \"Click OK to continue,\\n\"; ";
 response += "    ch += \"or Cancel\\n\\n\"; ";
 response += "    if(!confirm(ch)) { ";
 response += "      javascript:window.location.reload(true);";
 response += "    } ";
 response += "    else { ";
 response += "      document.location.href = '/'+y; ";
 response += "    } ";
 response += "  }";
 response += "}";
 response += "</script>";
 server.sendContent(response);

 return(ret);
}





boolean GoOn()
{
 boolean ret = false;
 String response = "";
 response += "<script>";

 response += "var ch = \"\"; ";
 response += "function doContinue(s,y) {";
 response += "  try { adddlert(s); }";
 response += "  catch(err) {";
 response += "    ch  = \" \" + s + \".\\n\\n\"; ";
 response += "    ch += \"Click Continue to get back to the page\\n\"; ";
 response += "    alert(ch); ";
 response += "  }";
 response += "}";
 response += "</script>";
 server.sendContent(response);

 return(ret);
}
# 131 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
void wwwFile(String fn)
{
#if _STAT_LOG == 1

 if (!SPIFFS.exists(fn)) {
# if _MONITOR>=1
   mPrint("wwwFile:: ERROR: SPIFFS file not found="+fn);
# endif
  return;
 }
# if _MONITOR>=1
 else if (debug>=1) {
  mPrint("wwwFile:: SPIFFS Filesystem exists= " + String(fn));
 }
 uint16_t readFile = 0;
# endif

# if _MONITOR>=1

 File f = SPIFFS.open(fn, "r");


 while (f.available()) {

  String s=f.readStringUntil('\n');
  if (s.length() == 0) {
   Serial.print(F("wwwFile:: String length 0"));
   break;
  }
  server.sendContent(s.substring(12));
  server.sendContent("\n");

  readFile++;

  yield();
 }

 f.close();

# endif

#endif

 return;

}





void buttonDocu()
{

 String response = "";
 response += "<script>";

 response += "var txt = \"\";";
 response += "function showDocu() {";
 response += "  try { adddlert(\"Welcome,\"); }";
 response += "  catch(err) {";







    response += "	 window.open(	\"https://things4u.github.io/Projects/SingleChannelGateway/index.html\",\"_blank\" ) ";
 response += "  }";
 response += "}";

 response += "</script>";
 server.sendContent(response);

 return;
}
# 218 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
void buttonRegs()
{
 String response = "";
 uint8_t j, k;
 response += "<script>";

 response += "var txt = \"\";";
 response += "function showRegs() {";
 response += "  try { adddlert(\"Registers,\"); }";
 response += "  catch(err) {";

 response += "	txt = \"\"; ";
 response += "	txt += \"<div style='font-size:24px;'>\\n\"; ";

 for (int i=0; i< _REG_AMOUNT; i++) {
  response += "	txt += \"";
  response += registers[i].name;
  response += "  (0x";
  j= registers[i].regid;
  response += (j<16 ? "0" : "");
  response += String(j,HEX);
  response += ") = 0x";
  k = registers[i].regvalue;
  response += (k<16 ? "0" : "");
  response += String(k,HEX);
  response += " \\n \"; ";
 }

 response += "  	 txt += \"\\n\\n\"; ";
 response += "    txt += \"Click OK to read register value for reading,\\n\"; ";
 response += "    txt += \"or Cancel to return to the home page.\\n\\n\"; ";
 response += "    alert(txt); ";



   for (int i=0; i< _REG_AMOUNT; i++) {registers[i].regvalue= readRegister(registers[i].regid); }
 response += " txt += \"</div>\"; ";
 response += "  }";

 response += "}";
 response += "</script>";
 server.sendContent(response);

 return;
}






void buttonLog()
{
#if _STAT_LOG == 1

 int startFile = (gwayConfig.logFileNo > LOGFILEMAX ? (gwayConfig.logFileNo - LOGFILEMAX) : 0);

# if _MONITOR>=1
  mPrint("buttonLog:: gwayConfig.logFileNo="+String(gwayConfig.logFileNo)+", LOGFILEMAX="+String(LOGFILEMAX)+", startFile="+String(startFile)+", recs="+String(gwayConfig.logFileRec) );
# endif

 for (int i=startFile; i<= gwayConfig.logFileNo; i++ ) {
  String fn = "/log-" + String(i);
  wwwFile(fn);
 }

#endif

 return;
}







static void wwwButtons()
{
 String response = "";
 String mode = (gwayConfig.expert ? "Basic Mode" : "Expert Mode");
 String moni = (gwayConfig.monitor ? "Hide Monitor" : "Monitor ON");
 String seen = (gwayConfig.seen ? "Hide Seen" : "Last seen ON");

 YesNo();
 buttonDocu();

 buttonRegs();

 response += "<input type=\"button\" value=\"Documentation\" onclick=\"showDocu()\" >";

 response += "<input type=\"button\" value=\"Register\" onclick=\"showRegs()\" >";

# if _STAT_LOG == 1
 response += "<a href=\"LOG\" download><button type=\"button\">Log Files</button></a>";
# endif

 response += "<a href=\"EXPERT\"><button>"+ mode +"</button></a>";

# if _MONITOR>=1
 response += "<a href=\"MONITOR\"><button>"+ moni +"</button></a>";
# endif

# if _MAXSEEN>=1
 response += "<a href=\"SEEN\"><button>"+ seen +"</button></a>";
# endif

 server.sendContent(response);

 return;
}
# 346 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void setVariables(const char *cmd, const char *arg)
{


 if (strcmp(cmd, "DEBUG")==0) {
  if (atoi(arg) == 1) {
   debug = (debug+1)%4;
  }
  else if (atoi(arg) == -1) {
   debug = (debug+3)%4;
  }
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }

 if (strcmp(cmd, "CAD")==0) {
  gwayConfig.cad=(bool)atoi(arg);
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }

 if (strcmp(cmd, "HOP")==0) {
  gwayConfig.hop=(bool)atoi(arg);
  if (! gwayConfig.hop) {
   setFreq(freqs[gwayConfig.ch].upFreq);
   rxLoraModem();
   sf = SF7;
   cadScanner();
  }
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }



 if (strcmp(cmd, "DELAY")==0) {
  gwayConfig.txDelay+=atoi(arg)*1000;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }



 if (strcmp(cmd, "TRUSTED")==0) {
  gwayConfig.trusted=atoi(arg);
  if (atoi(arg) == 1) {
   gwayConfig.trusted = (gwayConfig.trusted +1)%4;
  }
  else if (atoi(arg) == -1) {
   gwayConfig.trusted = (gwayConfig.trusted -1)%4;
  }
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }



 if (strcmp(cmd, "SF")==0) {
  if (atoi(arg) == 1) {
   if (sf>=SF12) sf=SF7; else sf= (sf_t)((int)sf+1);
  }
  else if (atoi(arg) == -1) {
   if (sf<=SF7) sf=SF12; else sf= (sf_t)((int)sf-1);
  }
  rxLoraModem();
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }



 if (strcmp(cmd, "FREQ")==0) {
  uint8_t nf = sizeof(freqs)/ sizeof(freqs[0]);


  if (atoi(arg) == 1) {
   if (gwayConfig.ch==(nf-1)) gwayConfig.ch=0; else gwayConfig.ch++;
  }
  else if (atoi(arg) == -1) {
   Serial.println("down");
   if (gwayConfig.ch==0) gwayConfig.ch=(nf-1); else gwayConfig.ch--;
  }
  setFreq(freqs[gwayConfig.ch].upFreq);
  rxLoraModem();
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }

 if (strcmp(cmd, "GETTIME")==0) {
  Serial.println(F("gettime tbd"));
 }




 if (strcmp(cmd, "HELP")==0) { Serial.println(F("Display Help Topics")); }


# if _GATEWAYNODE==1
 if (strcmp(cmd, "NODE")==0) {
  gwayConfig.isNode =(bool)atoi(arg);
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }


 if (strcmp(cmd, "FCNT")==0) {
  LoraUp.fcnt=0;
  LoraDown.fcnt=0;
  rxLoraModem();
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }
 if (strcmp(cmd, "DCNT")==0) {
  LoraDown.fcnt=0;
  rxLoraModem();
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }
# endif



# if _WIFIMANAGER==1
 if (strcmp(cmd, "NEWSSID")==0) {


  WiFiManager wifiManager;
# if _MONITOR>=1
  if (!wifiManager.autoConnect()) {
   Serial.println("failed to connect and hit timeout");
   ESP.restart();
   delay(1000);
  }
# endif




  String ssid = String(AP_NAME) + "-" + String(ESP_getChipId(), HEX);

# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_MAIN)) {
   mPrint("Set Variables:: ssid="+ ssid );
  }
# endif


 }
# endif


# if _OTA==1
 if (strcmp(cmd, "UPDATE")==0) {
  if (atoi(arg) == 1) {
   updateOtaa();
   writeGwayCfg(_CONFIGFILE, &gwayConfig );
  }
 }
# endif

# if _REFRESH==1
 if (strcmp(cmd, "REFR")==0) {
  gwayConfig.refresh =(bool)atoi(arg);
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
 }
# endif

}







static void openWebPage()
{
 ++gwayConfig.views;
 String response="";

 server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
 server.sendHeader("Pragma", "no-cache");
 server.sendHeader("Expires", "-1");




 server.setContentLength(CONTENT_LENGTH_UNKNOWN);
 server.send(200, "text/html", "");
 String tt=""; printIP((IPAddress)WiFi.localIP(),'.',tt);

#if _REFRESH==1
 if (gwayConfig.refresh) {
  response += String() + "<!DOCTYPE HTML><HTML><HEAD><meta http-equiv='refresh' content='"+_WWW_INTERVAL+";http://";
  response += tt;
  response += "'><TITLE>1ch Gateway " + String(tt) + "</TITLE>";
 }
 else {
  response += String("<!DOCTYPE HTML><HTML><HEAD><TITLE>1ch Gateway " + String(tt) + "</TITLE>");
 }
#else
 response += String("<!DOCTYPE HTML><HTML><HEAD><TITLE>1ch Gateway " + String(tt) + "</TITLE>");
#endif
 response += "<META HTTP-EQUIV='CONTENT-TYPE' CONTENT='text/html; charset=UTF-8'>";
 response += "<META NAME='AUTHOR' CONTENT='M. Westenberg (mw1554@hotmail.com)'>";

 response += "<style>.thead {background-color:green; color:white;} ";
 response += ".cell {border: 1px solid black;}";
 response += ".config_table {max_width:100%; min-width:300px; width:98%; border:1px solid black; border-collapse:collapse;}";
 response += "</style></HEAD><BODY>";

 response +="<h1>ESP Gateway Config</h1>";

 response +="<p style='font-size:10px;'>";
 response +="Version: "; response+=VERSION;
 response +="<br>ESP alive since ";
 stringTime(startTime, response);

 response +=", Uptime: ";
 uint32_t secs = millis()/1000;
 uint16_t days = secs / 86400;
 uint8_t _hour = hour(secs);
 uint8_t _minute = minute(secs);
 uint8_t _second = second(secs);
 response += String(days) + "-";

 if (_hour < 10) response += "0"; response += String(_hour) + ":";
 if (_minute < 10) response += "0"; response += String(_minute) + ":";
 if (_second < 10) response += "0"; response += String(_second);

 response +="<br>Current time    ";
 stringTime(now(), response);
 response +="<br>";
 response +="</p>";

 server.sendContent(response);
}
# 583 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void gatewaySettings()
{
 String response="";
 String bg="";

 response +="<h2>Gateway Settings</h2>";

 response +="<table class=\"config_table\">";
 response +="<tr>";
 response +="<th class=\"thead\">Setting</th>";
 response +="<th colspan=\"2\" style=\"background-color: green; color: white; width:120px;\">Value</th>";
 response +="<th colspan=\"2\" style=\"background-color: green; color: white; width:100px;\">Set</th>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( gwayConfig.cad ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">CAD</td>";
 response +="<td colspan=\"2\" style=\"border: 1px solid black;"; response += bg; response += "\">";
 response += ( gwayConfig.cad ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"CAD=0\"><button>OFF</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"CAD=1\"><button>ON</button></a></td>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( gwayConfig.hop ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">HOP</td>";
 response +="<td colspan=\"2\" style=\"border: 1px solid black;"; response += bg; response += "\">";
 response += ( gwayConfig.hop ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"HOP=0\"><button>OFF</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"HOP=1\"><button>ON</button></a></td>";
 response +="</tr>";


 response +="<tr><td class=\"cell\">SF Setting</td><td class=\"cell\" colspan=\"2\">";
 if (gwayConfig.cad) {
  response += "AUTO</td>";
 }
 else {
  response += sf;
  response +="<td class=\"cell\"><a href=\"SF=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"SF=1\"><button>+</button></a></td>";
 }
 response +="</tr>";


 response +="<tr><td class=\"cell\">Channel</td>";
 response +="<td class=\"cell\" colspan=\"2\">";
 if (gwayConfig.hop) {
  response += "AUTO</td>";
 }
 else {
  response += String(gwayConfig.ch);
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"FREQ=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"FREQ=1\"><button>+</button></a></td>";
 }
 response +="</tr>";


#ifdef _TRUSTED_NODES
 response +="<tr><td class=\"cell\">Trusted Nodes</td><td class=\"cell\" colspan=\"2\">";
 response +=gwayConfig.trusted;
 response +="</td>";
 response +="<td class=\"cell\"><a href=\"TRUSTED=-1\"><button>-</button></a></td>";
 response +="<td class=\"cell\"><a href=\"TRUSTED=1\"><button>+</button></a></td>";
 response +="</tr>";
#endif



#if _MONITOR>=1
 response +="<tr><td class=\"cell\">Debug Level</td><td class=\"cell\" colspan=\"2\">";
 response +=debug;
 response +="</td>";
 response +="<td class=\"cell\"><a href=\"DEBUG=-1\"><button>-</button></a></td>";
 response +="<td class=\"cell\"><a href=\"DEBUG=1\"><button>+</button></a></td>";
 response +="</tr>";


 response +="<tr><td class=\"cell\">Debug pattern</td>";

 bg = ( (pdebug & P_SCAN) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=SCAN\">";
 response +="<button>SCN</button></a></td>";

 bg = ( (pdebug & P_CAD) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=CAD\">";
 response +="<button>CAD</button></a></td>";

 bg = ( (pdebug & P_RX) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=RX\">";
 response +="<button>RX</button></a></td>";

 bg = ( (pdebug & P_TX) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=TX\">";
 response +="<button>TX</button></a></td>";
 response += "</tr>";


 response +="<tr><td class=\"cell\"></td>";
 bg = ( (pdebug & P_PRE) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=PRE\">";
 response +="<button>PRE</button></a></td>";

 bg = ( (pdebug & P_MAIN) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=MAIN\">";
 response +="<button>MAI</button></a></td>";

 bg = ( (pdebug & P_GUI) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=GUI\">";
 response +="<button>GUI</button></a></td>";

 bg = ( (pdebug & P_RADIO) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=RADIO\">";
 response +="<button>RDIO</button></a></td>";
 response +="</tr>";
#endif


 bg = " background-color: ";
 bg += ( (gwayConfig.dusbStat == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">Usb Debug</td>";
 response +="<td class=\"cell\" colspan=\"2\" style=\"border: 1px solid black; " + bg + "\">";
 response += ( (gwayConfig.dusbStat == true) ? "ON" : "OFF" );
 response +="</td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"DUSB=0\"><button>OFF</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"DUSB=1\"><button>ON</button></a></td>";
 response +="</tr>";

#if _LOCALSERVER>=1

 bg = " background-color: ";
 bg += ( (gwayConfig.showdata == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">Show Node Data</td>";
 response +="<td class=\"cell\" colspan=\"2\" style=\"border: 1px solid black; " + bg + "\">";
 response += ( (gwayConfig.showdata == true) ? "ON" : "OFF" );
 response +="</td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SHOWDATA=0\"><button>OFF</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SHOWDATA=1\"><button>ON</button></a></td>";
 response +="</tr>";

#endif


#if _REFRESH==1
 bg = " background-color: ";
 bg += ( (gwayConfig.refresh == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">WWW Refresh</td>";
 response +="<td class=\"cell\" colspan=\"2\" style=\"border: 1px solid black; " + bg + "\">";
 response += ( (gwayConfig.refresh == 1) ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"REFR=0\"><button>OFF</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"REFR=1\"><button>ON</button></a></td>";
 response +="</tr>";
#endif

#if _GATEWAYNODE==1
 response +="<tr><td class=\"cell\">Framecounter Internal Sensor</td>";
 response +="<td class=\"cell\" colspan=\"2\">";
 response +=LoraUp.fcnt;
 response +="</td><td colspan=\"2\" style=\"border: 1px solid black;\">";
 response +="<button><a href=\"/FCNT\">RESET   </a></button></td>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( (gwayConfig.isNode == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">Gateway Node</td>";
 response +="<td class=\"cell\" colspan=\"2\"  style=\"border: 1px solid black;" + bg + "\">";
 response += ( (gwayConfig.isNode == true) ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"NODE=1\"><button>ON</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"NODE=0\"><button>OFF</button></a></td>";
 response +="</tr>";
#endif

#if _REPEATER>=0

 bg = " background-color: ";
 bg += ( _REPEATER==1 ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">Repeater</td>";
 response +="<td colspan=\"2\" style=\"border: 1px solid black;"; response += bg; response += "\">";
 response += ( _REPEATER==1 ? "ON" : "OFF" );


 response +="</tr>";
#endif


 response +="<tr><td class=\"cell\">Format SPIFFS</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+gwayConfig.formatCntr+"</td>";
 response +="<td style=\"width:30px;\" colspan=\"2\" class=\"cell\"><input type=\"button\" value=\"FORMAT\" onclick=\"ynDialog(\'Do you really want to format?\',\'FORMAT\')\" /></td></tr>";


#if _STATISTICS >= 1
 response +="<tr><td class=\"cell\">Statistics</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+statc.resets+"</td>";
 response +="<td style=\"width:30px;\" colspan=\"2\" class=\"cell\"><input type=\"button\" value=\"RESET   \" onclick=\"ynDialog(\'Do you really want to reset statistics?\',\'RESET\')\" /></td></tr>";


 response +="<tr><td class=\"cell\">Boots and Resets</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+gwayConfig.boots+"</td>";
 response +="<td style=\"width:30px;\" colspan=\"2\" class=\"cell\" ><input type=\"button\" value=\"BOOT    \" onclick=\"ynDialog(\'Do you want to reset boots?\',\'BOOT\')\" /></td></tr>";
#endif

 response +="</table>";

 server.sendContent(response);
}
# 813 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void statisticsData()
{
 String response="";



 response +="<h2>Package Statistics</h2>";
 response +="<table class=\"config_table\">";
 response +="<tr><th class=\"thead\">Counter</th>";
# if _STATISTICS == 3
  response +="<th class=\"thead\">C 0</th>";
  response +="<th class=\"thead\">C 1</th>";
  response +="<th class=\"thead\">C 2</th>";
# endif
 response +="<th class=\"thead\">Pkgs</th>";
 response +="<th class=\"thead\">Pkgs/hr</th>";
 response +="</tr>";




 response +="<tr><td class=\"cell\">Packages Downlink</td>";
# if _STATISTICS == 3
  response +="<td class=\"cell\">" + String(statc.msg_down_0) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_down_1) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_down_2) + "</td>";
# endif
 response += "<td class=\"cell\">" + String(statc.msg_down) + "</td>";
 response +="<td class=\"cell\"></td></tr>";

 response +="<tr><td class=\"cell\">Packages Uplink Total</td>";
# if _STATISTICS == 3
  response +="<td class=\"cell\">" + String(statc.msg_ttl_0) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_ttl_1) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_ttl_2) + "</td>";
# endif
 response +="<td class=\"cell\">" + String(statc.msg_ttl) + "</td>";
 response +="<td class=\"cell\">" + String((statc.msg_ttl*3600)/(now() - startTime)) + "</td></tr>";

# if _GATEWAYNODE==1
  response +="<tr><td class=\"cell\">Packages Internal Sensor</td>";
# if _STATISTICS == 3
   response +="<td class=\"cell\">" + String(statc.msg_sens_0) + "</td>";
   response +="<td class=\"cell\">" + String(statc.msg_sens_1) + "</td>";
   response +="<td class=\"cell\">" + String(statc.msg_sens_2) + "</td>";
# endif
  response +="<td class=\"cell\">" + String(statc.msg_sens) + "</td>";
  response +="<td class=\"cell\">" + String((statc.msg_sens*3600)/(now() - startTime)) + "</td></tr>";
# endif

 response +="<tr><td class=\"cell\">Packages Uplink OK </td>";
#if _STATISTICS == 3
  response +="<td class=\"cell\">" + String(statc.msg_ok_0) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_ok_1) + "</td>";
  response +="<td class=\"cell\">" + String(statc.msg_ok_2) + "</td>";
# endif
 response +="<td class=\"cell\">" + String(statc.msg_ok) + "</td>";
 response +="<td class=\"cell\">" + String((statc.msg_ok*3600)/(now() - startTime)) + "</td></tr>";



 #if _STATISTICS == 2
 response +="<tr><td class=\"cell\">SF7 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf7;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf7/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF8 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf8;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf8/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF9 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf9;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf9/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF10 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf10;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf10/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF11 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf11;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf11/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF12 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf12;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf12/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
# endif

# if _STATISTICS == 3
 response +="<tr><td class=\"cell\">SF7 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf7_0;
  response +="<td class=\"cell\">"; response +=statc.sf7_1;
  response +="<td class=\"cell\">"; response +=statc.sf7_2;
  response +="<td class=\"cell\">"; response +=statc.sf7;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf7/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF8 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf8_0;
  response +="<td class=\"cell\">"; response +=statc.sf8_1;
  response +="<td class=\"cell\">"; response +=statc.sf8_2;
  response +="<td class=\"cell\">"; response +=statc.sf8;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf8/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF9 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf9_0;
  response +="<td class=\"cell\">"; response +=statc.sf9_1;
  response +="<td class=\"cell\">"; response +=statc.sf9_2;
  response +="<td class=\"cell\">"; response +=statc.sf9;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf9/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF10 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf10_0;
  response +="<td class=\"cell\">"; response +=statc.sf10_1;
  response +="<td class=\"cell\">"; response +=statc.sf10_2;
  response +="<td class=\"cell\">"; response +=statc.sf10;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf10/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF11 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf11_0;
  response +="<td class=\"cell\">"; response +=statc.sf11_1;
  response +="<td class=\"cell\">"; response +=statc.sf11_2;
  response +="<td class=\"cell\">"; response +=statc.sf11;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf11/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF12 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf12_0;
  response +="<td class=\"cell\">"; response +=statc.sf12_1;
  response +="<td class=\"cell\">"; response +=statc.sf12_1;
  response +="<td class=\"cell\">"; response +=statc.sf12;
  response +="<td class=\"cell\">"; response += String(statc.msg_ttl>0 ? 100*statc.sf12/statc.msg_ttl : 0)+" %";
  response +="</td></tr>";
# endif

 response +="</table>";
 server.sendContent(response);
}
# 983 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void messageHistory()
{
#if _STATISTICS >= 1
 String response="";


 response += "<h2>Message History</h2>";
 response += "<table class=\"config_table\">";
 response += "<tr>";
 response += "<th class=\"thead\">Time</th>";
 response += "<th class=\"thead\" style=\"width: 20px;\">Up/Dwn</th>";
 response += "<th class=\"thead\">Node</th>";
#if _LOCALSERVER>=1
 if (gwayConfig.showdata) {
  response += "<th class=\"thead\">Data</th>";
 }
#endif
 response += "<th class=\"thead\" style=\"width: 20px;\">C</th>";
 response += "<th class=\"thead\">Freq</th>";
 response += "<th class=\"thead\" style=\"width: 40px;\">SF</th>";
 response += "<th class=\"thead\" style=\"width: 50px;\">pRSSI</th>";
#if RSSI==1
 if (debug => 1) {
  response += "<th class=\"thead\" style=\"width: 50px;\">RSSI</th>";
 }
#endif


 response += "</tr>";
 server.sendContent(response);


 for (int i=0; i< gwayConfig.maxStat; i++) {
  if (statr[i].sf == 0) break;

  response = "" + String();

  response += "<tr><td class=\"cell\">";
  stringTime(statr[i].time, response);
  response += "</td>";

  response += String() + "<td class=\"cell\">";
  response += String(statr[i].upDown ? "v" : "^");
  response += "</td>";

  response += String() + "<td class=\"cell\">";

#ifdef _TRUSTED_NODES
  switch (gwayConfig.trusted) {
   case 0:
    printHex(statr[i].node,' ',response);
    break;
   case 1:
    if (SerialName(statr[i].node, response) < 0) {
     printHex(statr[i].node,' ',response);
    };
    break;
   case 2:
    if (SerialName(statr[i].node, response) < 0) {
     continue;
    };
    break;
   case 3:
   default:
# if _MONITOR>=1
     mPrint("Unknow value for gwayConfig.trusted");
# endif
    break;
  }

#else
  printHex(statr[i].node,' ',response);
#endif

  response += "</td>";

#if _LOCALSERVER>=1
 if (gwayConfig.showdata) {
  response += String() + "<td class=\"cell\">";
  if (statr[i].datal>24) statr[i].datal=24;
  for (int j=0; j<statr[i].datal; j++) {
   if (statr[i].data[j] <0x10) response+= "0";
   response += String(statr[i].data[j],HEX) + " ";
  }
  response += "</td>";
 }
#endif


  response += String() + "<td class=\"cell\">" + statr[i].ch + "</td>";
  response += String() + "<td class=\"cell\">" + freqs[statr[i].ch].upFreq + "</td>";
  response += String() + "<td class=\"cell\">" + statr[i].sf + "</td>";

  response += String() + "<td class=\"cell\">" + statr[i].prssi + "</td>";
#if RSSI==1
  if (debug >= 2) {
   response += String() + "<td class=\"cell\">" + statr[i].rssi + "</td>";
  }
#endif
  response += "</tr>";
  server.sendContent(response);
 }

 server.sendContent("</table>");

#endif
}
# 1102 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void nodeHistory()
{
# if _MAXSEEN>=1
 if (gwayConfig.seen) {

  String response= "";

  response += "<h2>Node Last Seen History</h2>";
  response += "<table class=\"config_table\">";
  response += "<tr>";
  response += "<th class=\"thead\" style=\"width: 215px;\">Time</th>";
  response += "<th class=\"thead\" style=\"width: 20px;\">Up/Dwn</th>";
  response += "<th class=\"thead\">Node</th>";
  response += "<th class=\"thead\">Pkgs</th>";
  response += "<th class=\"thead\" style=\"width: 20px;\">C</th>";
  response += "<th class=\"thead\" style=\"width: 40px;\">SF</th>";

  response += "</tr>";
  server.sendContent(response);



  for (int i=0; i<gwayConfig.maxSeen; i++) {

   if (listSeen[i].idSeen == 0)
   {
# if _MONITOR>=1
    if ((debug>=2) && (pdebug & P_MAIN)) {
     mPrint("nodeHistory:: idSeen==0 for i="+String(i));
    }
# endif
    break;
   }
   response = "";

   response += String("<tr><td class=\"cell\">");
   stringTime((listSeen[i].timSeen), response);
   response += "</td>";

   response += String("<td class=\"cell\">");
   switch (listSeen[i].upDown)
   {
    case 0: response += String("^"); break;
    case 1: response += String("v"); break;
    default: mPrint("wwwServer.ino:: ERROR upDown");
   }
   response += "</td>";

   response += String() + "<td class=\"cell\">";
# ifdef _TRUSTED_NODES
    switch (gwayConfig.trusted) {
     case 0:
      printHex(listSeen[i].idSeen,' ',response);

      break;
     case 1:
      if (SerialName(listSeen[i].idSeen, response) < 0) {

       printHex(listSeen[i].idSeen,' ',response);
      };
      break;
     case 2:
      if (SerialName(listSeen[i].idSeen, response) < 0) {

       continue;

      };
      break;
     case 3:

     default:
# if _MONITOR>=1
       mPrint("Unknow value for gwayConfig.trusted");
# endif
     break;
    }
# else
    printHex(listSeen[i].idSeen,' ',response);
# endif

   response += "</td>";

   response += String() + "<td class=\"cell\">" + listSeen[i].cntSeen + "</td>";
   response += String() + "<td class=\"cell\">" + listSeen[i].chnSeen + "</td>";
   response += String() + "<td class=\"cell\">" + listSeen[i].sfSeen + "</td>";

   server.sendContent(response);

  }
  server.sendContent("</table>");
 }
# endif

 return;
}
# 1209 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
void monitorData()
{
# if _MONITOR>=1
 if (gwayConfig.monitor) {
  String response="";
  response +="<h2>Monitoring Console</h2>";

  response +="<table class=\"config_table\">";
  response +="<tr>";
  response +="<th class=\"thead\">Monitor Console</th>";
  response +="</tr>";

  for (int i= iMoni-1+gwayConfig.maxMoni; i>=iMoni; i--) {
   if (monitor[i % gwayConfig.maxMoni].txt == "-") {
    break;
   }
   response +="<tr><td class=\"cell\">" ;
   response += String(monitor[i % gwayConfig.maxMoni].txt);
   response += "</td></tr>";
  }

  response +="</table>";
  server.sendContent(response);
 }
# endif
}







static void wifiConfig()
{
 if (gwayConfig.expert) {
  String response="";
  response +="<h2>WiFi Config</h2>";

  response +="<table class=\"config_table\">";

  response +="<tr><th class=\"thead\">Parameter</th><th class=\"thead\">Value</th></tr>";

  response +="<tr><td class=\"cell\">WiFi Host Name</td><td class=\"cell\">";
# if defined(ESP32_ARCH)
  response +=WiFi.getHostname(); response+="</tr>";
# else
  response +=wifi_station_get_hostname(); response+="</tr>";
# endif

  response +="<tr><td class=\"cell\">WiFi Host SSID</td><td class=\"cell\">";
  response +=WiFi.SSID(); response+="</tr>";

  response +="<tr><td class=\"cell\">IP Address</td><td class=\"cell\">";
  printIP((IPAddress)WiFi.localIP(),'.',response);
  response +="</tr>";
  response +="<tr><td class=\"cell\">IP Gateway</td><td class=\"cell\">";
  printIP((IPAddress)WiFi.gatewayIP(),'.',response);
  response +="</tr>";
#if _GWAYSCAN==0
# ifdef _TTNSERVER
  response +="<tr><td class=\"cell\">NTP Server</td><td class=\"cell\">"; response+=NTP_TIMESERVER; response+="</tr>";
  response +="<tr><td class=\"cell\">LoRa Router</td><td class=\"cell\">"; response+=_TTNSERVER; response+="</tr>";
  response +="<tr><td class=\"cell\">LoRa Router IP</td><td class=\"cell\">";
  printIP((IPAddress)ttnServer,'.',response);
  response +="</tr>";
# endif
# ifdef _THINGSERVER
  response +="<tr><td class=\"cell\">LoRa Router 2</td><td class=\"cell\">"; response+=_THINGSERVER;
  response += String() + ":" + _THINGPORT + "</tr>";
  response +="<tr><td class=\"cell\">LoRa Router 2 IP</td><td class=\"cell\">";
  printIP((IPAddress)thingServer,'.',response);
  response +="</tr>";
# endif
#endif

  response +="</table>";

  server.sendContent(response);
 }
}







static void systemStatus()
{
 if (gwayConfig.expert) {
  String response="";
  response +="<h2>System Status</h2>";

  response +="<table class=\"config_table\">";
  response +="<tr>";
  response +="<th class=\"thead\">Parameter</th>";
  response +="<th class=\"thead\" style=\"width:120px;\">Value</th>";
  response +="<th colspan=\"2\" class=\"thead width:120px;\">Set</th>";
  response +="</tr>";

  response +="<tr><td style=\"border: 1px solid black;\">Gateway ID</td>";
  response +="<td class=\"cell\">";
  if (MAC_array[0]< 0x10) response +='0'; response +=String(MAC_array[0],HEX);
  if (MAC_array[1]< 0x10) response +='0'; response +=String(MAC_array[1],HEX);
  if (MAC_array[2]< 0x10) response +='0'; response +=String(MAC_array[2],HEX);
  response +="FFFF";
  if (MAC_array[3]< 0x10) response +='0'; response +=String(MAC_array[3],HEX);
  if (MAC_array[4]< 0x10) response +='0'; response +=String(MAC_array[4],HEX);
  if (MAC_array[5]< 0x10) response +='0'; response +=String(MAC_array[5],HEX);
  response+="</tr>";


  response +="<tr><td class=\"cell\">Free heap</td><td class=\"cell\">"; response+=ESP.getFreeHeap(); response+="</tr>";

# if !defined ESP32_ARCH
   response +="<tr><td class=\"cell\">ESP speed</td><td class=\"cell\">"; response+=ESP.getCpuFreqMHz();
   response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SPEED=80\"><button>80</button></a></td>";
   response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SPEED=160\"><button>160</button></a></td>";
   response+="</tr>";
   response +="<tr><td class=\"cell\">ESP Chip ID</td><td class=\"cell\">"; response+=ESP.getChipId(); response+="</tr>";
# endif
  response +="<tr><td class=\"cell\">OLED</td><td class=\"cell\">"; response+=_OLED; response+="</tr>";

# if _STATISTICS >= 1
   response +="<tr><td class=\"cell\">WiFi Setups</td><td class=\"cell\">"; response+=gwayConfig.wifis; response+="</tr>";
   response +="<tr><td class=\"cell\">WWW Views</td><td class=\"cell\">"; response+=gwayConfig.views; response+="</tr>";
# endif


  response +="<tr><td class=\"cell\">Time Correction (uSec)</td><td class=\"cell\">";
  response += gwayConfig.txDelay;
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"DELAY=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"DELAY=1\"><button>+</button></a></td>";
  response +="</tr>";


  response +="<tr><td class=\"cell\">Package Statistics List</td><td class=\"cell\">";
  response += gwayConfig.maxStat;
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"MAXSTAT=-5\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"MAXSTAT=5\"><button>+</button></a></td>";
  response +="</tr>";


  response +="<tr><td class=\"cell\">Last Seen List</td><td class=\"cell\">";
  response += gwayConfig.maxSeen;
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"MAXSEEN-5\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"MAXSEEN+5\"><button>+</button></a></td>";
  response +="</tr>";


# if _WIFIMANAGER==1
   response +="<tr><td><tr><td>";
   response +="Click <a href=\"/NEWSSID\">here</a> to reset accesspoint<br>";
   response +="</td><td></td></tr>";
# endif

# if _UPDFIRMWARE==1

   response +="<tr><td class=\"cell\">Update Firmware</td><td colspan=\"2\"></td>";
   response +="<td class=\"cell\" colspan=\"2\" class=\"cell\"><a href=\"/UPDATE=1\"><button>UPDATE</button></a></td></tr>";
# endif

  response +="</table>";
  server.sendContent(response);
 }
}







static void interruptData()
{
 if (gwayConfig.expert) {
  uint8_t flags = readRegister(REG_IRQ_FLAGS);
  uint8_t mask = readRegister(REG_IRQ_FLAGS_MASK);
  String response="";

  response +="<h2>System State and Interrupt</h2>";

  response +="<table class=\"config_table\">";
  response +="<tr>";
  response +="<th class=\"thead\">Parameter</th>";
  response +="<th class=\"thead\">Value</th>";
  response +="<th colspan=\"2\"  class=\"thead\">Set</th>";
  response +="</tr>";

  response +="<tr><td class=\"cell\">_state</td>";
  response +="<td class=\"cell\">";
  switch (_state) {
   case S_INIT: response +="INIT"; break;
   case S_SCAN: response +="SCAN"; break;
   case S_CAD: response +="CAD"; break;
   case S_RX: response +="RX"; break;
   case S_TX: response +="TX"; break;
   case S_TXDONE: response +="TXDONE"; break;
   default: response +="unknown"; break;
  }
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">_STRICT_1CH</td>";
  response +="<td class=\"cell\">" ;
  response += String(_STRICT_1CH);
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">flags (8 bits)</td>";
  response +="<td class=\"cell\">0x";
  if (flags <16) response += "0";
  response +=String(flags,HEX);
  response+="</td></tr>";


  response +="<tr><td class=\"cell\">mask (8 bits)</td>";
  response +="<td class=\"cell\">0x";
  if (mask <16) response += "0";
  response +=String(mask,HEX);
  response+="</td></tr>";

  response +="<tr><td class=\"cell\">Re-entrant cntr</td>";
  response +="<td class=\"cell\">";
  response += String(gwayConfig.reents);
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">ntp call cntr</td>";
  response +="<td class=\"cell\">";
  response += String(gwayConfig.ntps);
  response+="</td></tr>";

  response +="<tr><td class=\"cell\">ntpErr cntr</td>";
  response +="<td class=\"cell\">";
  response += String(gwayConfig.ntpErr);
  response +="</td>";
  response +="<td colspan=\"2\" style=\"border: 1px solid black;\">";
  stringTime(gwayConfig.ntpErrTime, response);
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">loraWait errors/success</td>";
  response +="<td class=\"cell\">";
  response += String(gwayConfig.waitErr);
  response +="</td>";
  response +="<td colspan=\"2\" style=\"border: 1px solid black;\">";
  response += String(gwayConfig.waitOk);
  response +="</td></tr>";

  response +="</tr>";

  response +="</table>";

  server.sendContent(response);
 }
}
# 1479 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
void setupWWW()
{
 server.begin();

 server.on("/", []() {
  sendWebPage("","");
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });







 server.on("/EXPERT", []() {
  server.sendHeader("Location", String("/"), true);
  gwayConfig.expert = bool(1 - (int) gwayConfig.expert) ;
  server.send( 302, "text/plain", "");
 });

# if _MONITOR>=1

 server.on("/MONITOR", []() {
  server.sendHeader("Location", String("/"), true);
  gwayConfig.monitor = bool(1 - (int) gwayConfig.monitor) ;
  server.send( 302, "text/plain", "");
 });
# endif


 server.on("/SEEN", []() {
  server.sendHeader("Location", String("/"), true);
  gwayConfig.seen = bool(1 - (int) gwayConfig.seen) ;
  server.send( 302, "text/plain", "");
 });





 server.on("/HELP", []() {
  sendWebPage("HELP","");
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/CAD=1", []() {
  gwayConfig.cad=(bool)1;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/CAD=0", []() {
  gwayConfig.cad=(bool)0;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/DEBUG=-1", []() {
  debug = (debug+3)%4;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _DUSB>=1 || _MONITOR>=1
  if ((debug>=1) && (pdebug & P_GUI)) {
   mPrint("DEBUG -1: config written");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });

 server.on("/DEBUG=1", []() {
  debug = (debug+1)%4;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
  if (pdebug & P_GUI) {
   mPrint("DEBUG +1: config written");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/FORMAT", []() {
  Serial.print(F("FORMAT ..."));
  msg_oLED("FORMAT");
  SPIFFS.format();
  initConfig(&gwayConfig);
  gwayConfig.formatCntr++;
  writeConfig(_CONFIGFILE, &gwayConfig);
  printSeen( _SEENFILE, listSeen);
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_MAIN )) {
   mPrint("www:: manual Format DONE");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/RESET", []() {
  mPrint("RESET");
  startTime= now() - 1;

  statc.msg_ttl = 0;
  statc.msg_ok = 0;
  statc.msg_down = 0;
  statc.msg_sens = 0;

#if _STATISTICS >= 3
  statc.msg_ttl_0 = 0;
  statc.msg_ttl_1 = 0;
  statc.msg_ttl_2 = 0;

  statc.msg_ok_0 = 0;
  statc.msg_ok_1 = 0;
  statc.msg_ok_2 = 0;

  statc.msg_down_0 = 0;
  statc.msg_down_1 = 0;
  statc.msg_down_2 = 0;

  statc.msg_sens_0 = 0;
  statc.msg_sens_1 = 0;
  statc.msg_sens_2 = 0;
#endif

# if _STATISTICS >= 1
  for (int i=0; i< gwayConfig.maxStat; i++) { statr[i].sf = 0; }
# if _STATISTICS >= 2
   statc.sf7 = 0;
   statc.sf8 = 0;
   statc.sf9 = 0;
   statc.sf10= 0;
   statc.sf11= 0;
   statc.sf12= 0;

   statc.resets= 0;

# if _STATISTICS >= 3
    statc.sf7_0 = 0; statc.sf7_1 = 0; statc.sf7_2 = 0;
    statc.sf8_0 = 0; statc.sf8_1 = 0; statc.sf8_2 = 0;
    statc.sf9_0 = 0; statc.sf9_1 = 0; statc.sf9_2 = 0;
    statc.sf10_0= 0; statc.sf10_1= 0; statc.sf10_2= 0;
    statc.sf11_0= 0; statc.sf11_1= 0; statc.sf11_2= 0;
    statc.sf12_0= 0; statc.sf12_1= 0; statc.sf12_2= 0;
# endif

# endif
# endif

  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  initSeen(listSeen);

  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/BOOT", []() {
  mPrint("BOOT");
# if _STATISTICS >= 2
   gwayConfig.wifis = 0;
   gwayConfig.views = 0;
   gwayConfig.ntpErr = 0;
   gwayConfig.ntpErrTime = 0;
   gwayConfig.ntps = 0;
# endif
  gwayConfig.boots = 0;
  gwayConfig.reents = 0;

  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
  if ((debug>=2) && (pdebug & P_GUI)) {
   mPrint("wwwServer:: BOOT: config written");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/REBOOT", []() {
  sendWebPage("","");
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
  ESP.restart();
 });


 server.on("/NEWSSID", []() {
  sendWebPage("NEWSSID","");
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });





 server.on("/PDEBUG=SCAN", []() {
  pdebug ^= P_SCAN;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=CAD", []() {
  pdebug ^= P_CAD;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=RX", []() {
  pdebug ^= P_RX;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=TX", []() {
  pdebug ^= P_TX;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=PRE", []() {
  pdebug ^= P_PRE;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=MAIN", []() {
  pdebug ^= P_MAIN;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=GUI", []() {
  pdebug ^= P_GUI;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/PDEBUG=RADIO", []() {
  pdebug ^= P_RADIO;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/DELAY=1", []() {
  gwayConfig.txDelay+=5000;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_GUI)) {
   mPrint("DELAY +, config written");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/DELAY=-1", []() {
  gwayConfig.txDelay-=5000;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
  if ((debug>=1) && (pdebug & P_GUI)) {
   mPrint("DELAY -, config written");
  }
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/TRUSTED=1", []() {
 gwayConfig.trusted = (gwayConfig.trusted +1)%4;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
   mPrint("TRUSTED +, config written");
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/TRUSTED=-1", []() {
  gwayConfig.trusted = (gwayConfig.trusted -1)%4;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
# if _MONITOR>=1
   mPrint("TRUSTED -, config written");
# endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/SF=1", []() {
  if (sf>=SF12) sf=SF7; else sf= (sf_t)((int)sf+1);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/SF=-1", []() {
  if (sf<=SF7) sf=SF12; else sf= (sf_t)((int)sf-1);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/FREQ=1", []() {
  uint8_t nf = sizeof(freqs)/sizeof(freqs[0]);
#if _DUSB>=2
  Serial.print("FREQ==1:: For freq[0] sizeof vector=");
  Serial.print(sizeof(freqs[0]));
  Serial.println();
#endif
  if (gwayConfig.ch==(nf-1)) gwayConfig.ch=0; else gwayConfig.ch++;
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/FREQ=-1", []() {
  uint8_t nf = sizeof(freqs)/sizeof(freqs[0]);
  if (gwayConfig.ch==0) gwayConfig.ch=(nf-1); else gwayConfig.ch--;
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/NODE=1", []() {
#if _GATEWAYNODE==1
  gwayConfig.isNode =(bool)1;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
#endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/NODE=0", []() {
#if _GATEWAYNODE==1
  gwayConfig.isNode =(bool)0;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
#endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });

#if _GATEWAYNODE==1

 server.on("/FCNT", []() {
  LoraUp.fcnt=0;
  rxLoraModem();
  writeGwayCfg(_CONFIGFILE, &gwayConfig );


  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
#endif


 server.on("/REFR=1", []() {
#if _REFRESH==1
  gwayConfig.refresh =1;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
#endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/REFR=0", []() {
#if _REFRESH==1
  gwayConfig.refresh =0;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
#endif
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/DUSB=1", []() {
  gwayConfig.dusbStat =1;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/DUSB=0", []() {
  gwayConfig.dusbStat =0;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });


 server.on("/SHOWDATA=1", []() {
  gwayConfig.showdata =1;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/SHOWDATA=0", []() {
  gwayConfig.showdata =0;
  writeGwayCfg(_CONFIGFILE, &gwayConfig );
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });



 server.on("/HOP=1", []() {
  gwayConfig.hop=true;
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/HOP=0", []() {
  gwayConfig.hop=false;
  setFreq(freqs[gwayConfig.ch].upFreq);
  rxLoraModem();
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });

#if !defined ESP32_ARCH

 server.on("/SPEED=80", []() {
  system_update_cpu_freq(80);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/SPEED=160", []() {
  system_update_cpu_freq(160);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
#endif

 server.on("/DOCU", []() {

  server.sendHeader("Location", String("/"), true);
  buttonDocu();
  server.send( 302, "text/plain", "");
 });


 server.on("/REGS", []() {
  buttonRegs();
  server.sendHeader("Location", String("/"), true);

  server.send( 302, "text/plain", "");
 });


 server.on("/LOG", []() {
  server.sendHeader("Location", String("/"), true);
# if _MONITOR>=1
   mPrint("LOG button");
# endif
  buttonLog();
  server.send( 302, "text/plain", "");
 });



 server.on("/UPDATE=1", []() {

  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });





 server.on("/MAXSTAT=-5", []() {
  if (gwayConfig.maxStat>5) {
   struct stat_t * oldStat = statr;
   gwayConfig.maxStat-=5;
   statr = (struct stat_t *) malloc(gwayConfig.maxStat * sizeof(struct stat_t));
   for (int i=0; i<gwayConfig.maxStat; i+=1) {
    statr[i]=oldStat[i];
   }
   free(oldStat);
  }
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/MAXSTAT=5", []() {
  struct stat_t * oldStat = statr;
  gwayConfig.maxStat+=5;
  statr = (struct stat_t *) malloc(gwayConfig.maxStat * sizeof(struct stat_t));
  for (int i=0; i<gwayConfig.maxStat; i+=1) {
   if (i<(gwayConfig.maxStat-5)) {
    statr[i]=oldStat[i];
   }
   else {
    statr[i].sf=0;
   }
  }
  free(oldStat);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });

 server.on("/MAXSEEN-5", []() {
  if (gwayConfig.maxSeen>5) {
   struct nodeSeen * oldSeen = listSeen;
   gwayConfig.maxSeen-=5;
   listSeen = (struct nodeSeen *) malloc(gwayConfig.maxSeen * sizeof(struct nodeSeen));
   for (int i=0; i<gwayConfig.maxSeen; i+=1) {
    listSeen[i]=oldSeen[i];
   }
   if (gwayConfig.maxSeen<iSeen) iSeen = gwayConfig.maxSeen;
   free(oldSeen);
  }
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });
 server.on("/MAXSEEN+5", []() {
  struct nodeSeen * oldSeen = listSeen;
  gwayConfig.maxSeen+=5;
  listSeen = (struct nodeSeen *) malloc(gwayConfig.maxSeen * sizeof(struct nodeSeen));
  for (int i=0; i<gwayConfig.maxSeen; i+=1) {
   if (i<(gwayConfig.maxSeen-5)) listSeen[i]=oldSeen[i];
   else listSeen[i].idSeen=0;
  }
  free(oldSeen);
  server.sendHeader("Location", String("/"), true);
  server.send( 302, "text/plain", "");
 });







# if _MONITOR>=1
  mPrint("WWW Server started on port " + String(_SERVERPORT) );
# endif

 return;
}
# 2037 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
void sendWebPage(const char *cmd, const char *arg)
{
 openWebPage(); yield();

 wwwButtons();

 setVariables(cmd,arg); yield();

 statisticsData(); yield();
 messageHistory(); yield();
 nodeHistory(); yield();
 monitorData(); yield();

 gatewaySettings(); yield();
 wifiConfig(); yield();
 systemStatus(); yield();
 interruptData(); yield();

 websiteFooter(); yield();

 server.client().stop();
}
# 2067 "/Users/sh/Documents/GitHub/ESP-1ch-Gateway-master 2/src/_wwwServer.ino"
static void websiteFooter()
{

 server.sendContent(String() + "<br><br /><p style='font-size:10px'>Click <a href=\"/HELP\">here</a> to explain Help and REST options</p><br>");
 server.sendContent(String() + "</BODY></HTML>");

 server.sendContent(""); yield();
}


#endif