// Boiler temperature controller.
// Improvement for an old cast iron boiler by adding temperature control and shutdown management.
// Setpoint temperature can be overriden by boiler control with restore to default timeout
// Sits on standard framework
// Has one DS temp sensor for boiler exit temperature
// Uses PCF8574 I2C for 8 bit 5v io
//
// The system has 
//  2 conventional single port valves
//  standard cylinder stat to HW valve
//  Tado with on/off relay for CH valve
//  Gas boiler with mechanical thermostat
//
// 240 v input board listens to 
//   cylinder stat volts
//   and tado relay volts
//   main gas valve (meaning flame on) for info and fault detect
// 4 output relays do:
//   interrupt the boiler stat (which is set at a higher temperature)
//   interrupt the HW stat to turn HW off
//   force on the HW valve
//   force on the CH valve
//
// The temerature monitor has a rate compensation to predict the actual temperature
// to mitigate over / undershoot
// Tighter temperature hysteresis - around 5C - with anti-cycle
// Shutdown to dump boiler heat to system before pump stop
//
// Temerature load and weather compensation is performed by the Boiler Control.
// build in sim for testing

const char* module = "boiler-temp";
const char* buildDate = __DATE__  "  "  __TIME__;

// ----------- start properties include 1 --------------------
#include <SPIFFS.h>
#include <ArduinoJson.h>
String propFile = "/props.properties";   // prop file name
#define PROPSSIZE 40
String propNames[PROPSSIZE];
int propNamesSz = 0;
#define BUFFLEN 200
char buffer[BUFFLEN];
int bufPtr = 0;
#include <esp_task_wdt.h>
#include <EEPROM.h>
// ----------- end properties include 1 --------------------

// ----------- start WiFi & Mqtt & Telnet include 1 ---------------------
#define HAVEWIFI 1
// sort of a DNS
#define DNSSIZE 20
#define SYNCHINTERVAL 30
// wifi, time, mqtt
#include <ESP32Time.h>
ESP32Time rtc;
#include <WiFi.h>
#include "ESPTelnet.h"
#include "EscapeCodes.h"
#include <PicoMQTT.h>
ESPTelnet telnet;
IPAddress ip;
PicoMQTT::Client mqttClient;

// ----------- end  WiFi & Mqtt & Telnet include 1 ---------------------

// ---------- start custom ------------------------
// -------- ds1820 ----------
// only have 1 device on the line
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#include "PCF8574.h"
PCF8574 PCF_01(0x20);
//const byte PFCADDRESS = 0x20;     // standard address
//PCF8574 PCF_01(PFCADDRESS);
// ---------- end custom ------------------------

// --------- standard properties ------------------
int logLevel = 2;
String logLevelN = "logLevel";
int eeWriteLimit = 100;
String eeWriteLimitN = "eeWriteLimit";
String wifiSsid = "<ssid>";
String wifiSsidN = "wifiSsid";        
String wifiPwd = "<pwd>";
String wifiPwdN = "wifiPwd";
byte wifiIp4 = 0;   // > 0 for fixed ip
String wifiIp4N = "wifiIp4";
byte mqttIp4 = 200;
String mqttIp4N = "mqttIp4";
int mqttPort = 1883;
String mqttPortN = "mqttPort";   
int telnetPort = 23;
String telnetPortN = "telnetport";   
String mqttId = "xx";          // is username-N and token xx/n/.. from unitId
String mqttIdN = "mqttId";   
int unitId  = 9;                  // uniquness of mqtt id
String unitIdN = "unitId";
int wdTimeout = 30;
String wdTimeoutN = "wdTimeout";
// generic way of setting property as 2 property setting operations
String propNameA = "";
String propNameN = "propName";
String propValue = "";
String propValueN = "propValue";
// these used to apply adjustment via props system
String restartN = "restart";
String writePropsN = "writeProps";

// ------- custom properties -----------
int dSetTemp = 60;       // default target off temp
String dSetTempN = "dSetTemp";
int hystTemp = 5;       //  set-x = target on temp
String hystTempN = "hystTemp";
int idleTemp = 35;      // cool to temp in shotdown
String idleTempN = "idleTemp";
int tempRate = 5;       // ajustment rate /sec
String tempRateN = "tempRate";
int antiCycleTime = 180;    // anti cycle min on to on
String antiCycleTimeN = "antiCycleTime";
bool simMode = false;
String simModeN = "simMode";
bool graphMode = false;
String graphModeN = "graphMode";
int controllerTimeout = 60;       // when resets defaults
String controllerTimeoutN = "contTimeout";


// ------- end custom properties -----------

// ----------- start properties include 2 --------------------

bool mountSpiffs()
{
   if(!SPIFFS.begin(true))
  {
    log(1, "SPIFFS Mount Failed");
    return false;
  }
  else
  {
    log(1, "SPIFFS formatted");
  }
  return true;
}

// checks a property name in json doc and keeps a list in propNames
bool checkProp(JsonDocument &doc, String propName, bool reportMissing)
{
  if (propNamesSz >= PROPSSIZE)
  {
    log(0, "!! props names limit");
  }
  else
  {
    propNames[propNamesSz++] = propName;
  }
  if (doc.containsKey(propName))
  {
    String val = doc[propName].as<String>();
    log(0, propName + "=" + val);
    return true;
  }
  if (reportMissing)
  {
    log(0, propName + " missing");
  }
  return false;
}

bool readProps()
{
  log(0, "Reading file " + propFile);
  File file = SPIFFS.open(propFile);
  if(!file || file.isDirectory())
  {
    log(0, "− failed to open file for reading");
    return false;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) 
  {
    log(0, "deserializeJson() failed: ");
    log(0, error.f_str());
    return false;
  }
  extractProps(doc, true);
  return true;
}

// writes/displays the props
bool writeProps(bool noWrite)
{
  JsonDocument doc;
  addProps(doc);
  String s;
  serializeJsonPretty(doc, s);
  s = s.substring(3,s.length()-2);
  log(0, s);
  if (noWrite)
  {
    return true;
  }
  log(0, "Writing file:" + propFile);
  File file = SPIFFS.open(propFile, FILE_WRITE);
  if(!file)
  {
    log(0, "− failed to open file for write");
    return false;
  }
  serializeJson(doc, file);
  file.close();
  return true;
}

// is expected to be form 'name=value' or 'name value' and can be a comma sep list
// name can be case insensitve match on first unique characters..
// converted to Json to update
void adjustProp(String s)
{
  String ss = s;
  while (true)
  {
    int p1 = ss.indexOf(',');
    if (p1 < 0)
    {
      adjustProp2(ss);
      return;
    }
    String s1 = ss.substring(0, p1);
    adjustProp2(s1);
    ss = ss.substring(p1+1);
  }
}
void adjustProp2(String s)
{
  int p1 = s.indexOf('=');
  if (p1 < 0)
  {
    p1 = s.indexOf(' ');
  }
  if (p1 < 0)
  {
    log(0, "no = or space found");
    return;
  }
  String p = s.substring(0,p1);
  String pl = p;
  pl.toLowerCase();
  String v = s.substring(p1+1);
  int ip;
  int m = 0;
  for (int ix = 0; ix < propNamesSz; ix++)
  {
    if (propNames[ix].length() >= pl.length())
    {
      String pn = propNames[ix].substring(0, pl.length());
      pn.toLowerCase();
      if (pl == pn)
      {
        if (m == 0)
        {
          ip = ix;
          m++;
        }
        else
        {
          if (m == 1)
          {
            log(0, "duplicate match " + p + " " + propNames[ip]);
          }
          m++;
          log(0, "duplicate match " + p + " " + propNames[ix]);
        }
      }
    }
  }
  if (m > 1)
  {
    return;
  }
  else if (m==0)
  {
    log(0, "no match " + p);
    return;
  }
  s = "{\"" + propNames[ip] + "\":\"" + v + "\"}";
 
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, s);
  if (error) 
  {
    log(0, "deserializeJson() failed: ");
    log(0, error.f_str());
    return;
  }
  extractProps(doc, false);
}

// logger
void log(int level, String s)
{
  if (level > logLevel) return;
  Serial.println(s);
  #if HAVEWIFI
  telnet.println(s);
  #endif
}
void log(int level)
{
  if (level > logLevel) return;
  Serial.println();
  #if HAVEWIFI
  telnet.println();
  #endif
}
void loga(int level, String s)
{
  if (level > logLevel) return;
  Serial.print(s);
  #if HAVEWIFI
  telnet.print(s);
  #endif
}

void checkSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
      case 0:
        break;
      case '\r':
        break;
      case '\n':
        buffer[bufPtr++] = 0;
        processCommandLine(String(buffer));
        bufPtr = 0;
        break;
      default:
        if (bufPtr < BUFFLEN -1)
        {
          buffer[bufPtr++] = c;
        }
        break;
    }
  } 
}
// for counting restarts - write to eprom
struct eeStruct
{
  unsigned int writes = 0;
  unsigned int wdtRestart = 0;
  unsigned int isrRestart = 0;
  unsigned int panicRestart = 0;
  unsigned int otherRestart = 0;
};

eeStruct eeData;

// for reliability stats from each restart
int getGatewayCount = 0;
int getWifiCount = 0;
int reConnWifiCount = 0;
unsigned long mqttDiscMs = 0;
int mqttConnCount = 0;
int mqttDiscCount = 0;
int mqttConnFailCount = 0;
int mqttConnTimeMs = 0;
int mqttSendCount = 0;
int mqttInCount = 0;

void eeDataReset()
{
  eeData.writes = 0;
  eeData.isrRestart = 0;
  eeData.wdtRestart = 0;
  eeData.panicRestart = 0;
  eeData.otherRestart = 0;
}

void eepromInit()
{
  int eeSize = sizeof(eeData);
  EEPROM.begin(eeSize);
  log(0, "ee size "+ String(eeSize));
}
void eepromWrite()
{
  eeData.writes++;
  if (eeData.writes > eeWriteLimit)
  {
    log(0, "eeprop Write limit..");
    return;
  }
  EEPROM.put(0, eeData);
  EEPROM.commit();
  log(0, "eewrite:" + String(eeData.writes));
}
void eepromRead()
{
  EEPROM.get(0, eeData);
  log(0, "eeWrites: " + String(eeData.writes));
}
void checkRestartReason()
{
  eepromRead();
  int resetReason = esp_reset_reason();
  log(0, "ResetReason: " + String(resetReason));
  switch (resetReason)
  {
    case ESP_RST_POWERON:
      return;// ok
    case ESP_RST_PANIC:
      eeData.panicRestart++;
      break;
    case ESP_RST_INT_WDT:
      eeData.isrRestart++;
      break;
    case ESP_RST_TASK_WDT:
      eeData.wdtRestart++;
      break;
    default:
      eeData.otherRestart++;
      break;
  }
  eepromWrite();
  logResetStats();
}

void logResetStats()
{
  log(0, "eeWrites: " + String(eeData.writes));
  log(0, "panic: " + String(eeData.panicRestart));
  log(0, "taskwd: " + String(eeData.wdtRestart));
  log(0, "irswd: " + String(eeData.isrRestart));
  log(0, "other: " + String(eeData.otherRestart));
  #if HAVEWIFI
  log(0, "getGateway: " + String(getGatewayCount));
  log(0, "getWifi: " + String(getWifiCount));
  log(0, "reconnWifi: " + String(reConnWifiCount));
  log(0, "mqttConn: " + String(mqttConnCount));
  log(0, "mqttConnT: " + String(mqttConnTimeMs));
  log(0, "mqttDisc: " + String(mqttDiscCount));
  log(0, "mqttFail: " + String(mqttConnFailCount));
  log(0, "mqttSend: " + String(mqttSendCount));
  log(0, "mqttIn: " + String(mqttInCount));
  log(0, "wfChannel: " + String(WiFi.channel()));
  log(0, "wfRSSI: " + String(WiFi.RSSI()));
  log(0, "wfPower: " + String(WiFi.getTxPower()));
  #endif
}
// ----------- end properties include 2 --------------------

// ----------- custom properties modify section  --------------------
// extract and add properties to json doc
// customize this for props expected and data types - watch with bools

void extractProps(JsonDocument &doc, bool reportMissing)
{
  propNamesSz = 0;
  log(0, "setting properties:");
  String propName;
  propName = logLevelN; if (checkProp(doc, propName, reportMissing)) logLevel = doc[propName].as<int>();
  propName = eeWriteLimitN; if (checkProp(doc, propName, reportMissing)) eeWriteLimit = doc[propName].as<int>();
  propName = wifiSsidN; if (checkProp(doc, propName, reportMissing)) wifiSsid = doc[propName].as<String>();
  propName = wifiPwdN;  if (checkProp(doc, propName, reportMissing)) wifiPwd = doc[propName].as<String>();
  propName = wifiIp4N;  if (checkProp(doc, propName, reportMissing)) wifiIp4 = doc[propName].as<byte>();
  propName = mqttPortN; if (checkProp(doc, propName, reportMissing)) mqttPort = doc[propName].as<int>();
  propName = mqttIp4N;  if (checkProp(doc, propName, reportMissing)) mqttIp4 = doc[propName].as<byte>();
  propName = telnetPortN;if (checkProp(doc, propName, reportMissing)) telnetPort = doc[propName].as<int>();
  propName = mqttIdN;   if (checkProp(doc, propName, reportMissing)) mqttId = doc[propName].as<String>();
  propName = unitIdN;   if (checkProp(doc, propName, reportMissing)) unitId = doc[propName].as<int>();
  propName = wdTimeoutN;if (checkProp(doc, propName, reportMissing)) wdTimeout = max(doc[propName].as<int>(),30);
  // these just for adjustment
  propName = restartN; if (checkProp(doc, propName, false)) ESP.restart();
  propName = writePropsN; if (checkProp(doc, propName, false)) writeProps(false);
  propName = propNameN; if (checkProp(doc, propName, false)) propNameA = doc[propName].as<String>();
  propName = propValueN;if (checkProp(doc, propName, false)) propValue = doc[propName].as<String>();  // picked up in checkState()

  // ----- start custom extract -----

  propName = dSetTempN;if (checkProp(doc, propName, reportMissing)) dSetTemp = doc[propName].as<int>();
  propName = hystTempN;if (checkProp(doc, propName, reportMissing)) hystTemp = doc[propName].as<int>();
  propName = idleTempN;if (checkProp(doc, propName, reportMissing)) idleTemp = doc[propName].as<int>();
  propName = tempRateN;if (checkProp(doc, propName, reportMissing)) tempRate = doc[propName].as<double>();
  propName = antiCycleTimeN;if (checkProp(doc, propName, reportMissing)) antiCycleTime = doc[propName].as<int>();
  propName = simModeN;if (checkProp(doc, propName, reportMissing)) simMode = doc[propName].as<int>();
  propName = graphModeN;if (checkProp(doc, propName, reportMissing)) graphMode = doc[propName].as<int>();
  propName = controllerTimeoutN;if (checkProp(doc, propName, reportMissing)) controllerTimeout = doc[propName].as<int>();

  // ----- end custom extract -----
}

// adds props for props write - customize
void addProps(JsonDocument &doc)
{
  doc[logLevelN] = logLevel;
  doc[eeWriteLimitN] = eeWriteLimit;
  doc[wifiSsidN] = wifiSsid;
  doc[wifiPwdN] = wifiPwd;
  doc[wifiIp4N] = wifiIp4;
  doc[mqttIp4N] = mqttIp4;
  doc[telnetPortN] = telnetPort;
  doc[mqttPortN] = mqttPort;
  doc[mqttIdN] = mqttId;
  doc[unitIdN] = unitId;
  doc[wdTimeoutN] = wdTimeout;

  // ----- start custom add -----
  doc[dSetTempN] = dSetTemp;
  doc[hystTempN] = hystTemp;
  doc[idleTempN] = idleTemp;
  doc[tempRateN] = tempRate;
  doc[antiCycleTimeN] = antiCycleTime; 
  doc[simModeN] = simMode;
  doc[graphModeN] = graphMode;
  doc[controllerTimeoutN] = controllerTimeout;
  // ----- end custom add -----
}

// custom modified section for props control and general commands
void processCommandLine(String cmdLine)
{
  if (cmdLine.length() == 0)
  {
    return;
  }
  
  switch (cmdLine[0])
  {
    case 'h':
    case '?':
      log(0, "v:version, w:writeprops, d:dispprops, l:loadprops p<prop>=<val>: change prop, r:restart");
      log(0, "s:showstats, z:zerostats, n:dns, 0,1,2:loglevel = " + String(logLevel));
      return;
    case 'w':
      writeProps(false);
      return;
    case 'd':
      writeProps(true);
      return;
    case 'l':
      readProps();
      return;
    case 'p':
      adjustProp(cmdLine.substring(1));
      return;
    case 'r':
      ESP.restart();
      return;
    case 'v':
      loga(0, module);
      loga(0, " ");
      log(0, buildDate);
      return;
    case 'z':
      eeDataReset();
      return;
    case 's':
      logResetStats();
      return;
    case 't':
      {
        tm now = rtc.getTimeStruct();
        log(0, "ESP time: " + dateTimeIso(now));
        return;
      }
    case 'n':
      logDns();
      break;
    case '0':
      logLevel = 0;
      log(0, " loglevel=" + String(logLevel));
      return;
    case '1':
      logLevel = 1;
      log(0, " loglevel=" + String(logLevel));
      return;
    case '2':
      logLevel = 2;
      log(0, " loglevel=" + String(logLevel));
      return;

  // ----- start custom cmd -----

  // ----- end custom cmd -----
    default:
      log(0, "????");
      return;
  }
}
// ----------- end custom properties modify section  --------------------

// ------------ start wifi and mqtt include section 2 ---------------
IPAddress localIp;
IPAddress gatewayIp;
IPAddress primaryDNSIp;
String mqttMoniker;

int recoveries = 0;
unsigned long seconds = 0;
unsigned long lastSecondMs = 0;
unsigned long startRetryDelay = 0;
int long retryDelayTime = 10;  // seconds
bool retryDelay = false;


// state engine
#define START 0
#define STARTGETGATEWAY 1
#define WAITGETGATEWAY 2
#define STARTCONNECTWIFI 3
#define WAITCONNECTWIFI 4
#define ALLOK 5

String stateS(int state)
{
  if (state == START) return "start";
  if (state == STARTGETGATEWAY) return "startgetgateway";
  if (state == WAITGETGATEWAY) return "waitgetgateway";
  if (state == STARTCONNECTWIFI) return "startconnectwifi";
  if (state == WAITCONNECTWIFI) return "waitconnectwifi";
  if (state == ALLOK) return "allok";
  return "????";
}
int state = START;

unsigned long startWaitWifi;

bool startGetGateway()
{
  getGatewayCount++;
  WiFi.disconnect();
  delay (500);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  log(1, "Start wifi dhcp" + wifiSsid + " " + wifiPwd);
  WiFi.begin(wifiSsid, wifiPwd);
  startWaitWifi = millis();
  return true;
}
bool startWifi()
{
  getWifiCount++;
  WiFi.disconnect();
  WiFi.setAutoReconnect(true);
  delay (500);
  unsigned long startWaitWifi = millis();
  WiFi.mode(WIFI_STA);
  if (wifiIp4 == 0)
  {
    log(1, "Start wifi dhcp: " + wifiSsid + " " + wifiPwd);
  }
  else
  {
    IPAddress subnet(255, 255, 0, 0);
    IPAddress fixedIp = localIp;
    fixedIp[3] = wifiIp4;
    if (!WiFi.config(fixedIp, gatewayIp, subnet, primaryDNSIp, primaryDNSIp)) 
    {
      log(1, "STA Failed to configure");
      return false;
    }
    log(1, "Start wifi fixip: " + wifiSsid + " " + wifiPwd);
  }
  WiFi.begin(wifiSsid, wifiPwd);
  return true;
}

int waitWifi()
{
  // 0=waiting, <0=fail, >0=ok
  unsigned long et = millis() - startWaitWifi;
  if (WiFi.status() == WL_CONNECTED)
  {
    localIp = WiFi.localIP();
    gatewayIp = WiFi.gatewayIP();
    primaryDNSIp = WiFi.dnsIP();
    log(1, "connected, t=" + String(et) + ", local=" + localIp.toString() + " gateway=" + gatewayIp.toString() + " dns=" + primaryDNSIp.toString());
    reConnWifiCount--;
    return 1;
  }
  
  if (et > 30000)
  {
    log(1, "... fail wifi connection timeout");
    return -1;
  }
  return 0;
}


// dns support
struct dnsIsh
{
  bool used = false;
  String name;
  String ip;
  int timeout;
};
dnsIsh dnsList[DNSSIZE];
unsigned long dnsVersion = 0;
unsigned long lastSynchTime = 0;

void logDns()
{
  log(0, "dns v=" + String(dnsVersion));
  for (int ix = 0; ix < DNSSIZE; ix++)
  {
    if (dnsList[ix].used && dnsList[ix].timeout > 0)
    {
      log(0, String(ix) + " " + dnsList[ix].name + " " + dnsList[ix].ip);
    }
  }
}

String dnsGetIp(String name)
{
  for (int ix = 0; ix < DNSSIZE; ix++)
  {
    if (dnsList[ix].used && dnsList[ix].name.startsWith(name))
    {
      return dnsList[ix].ip;
    }
  }
  return "";
}

// ESP32 Time
String formatd2(int i)
{
  if (i < 10)
  {
    return "0" + String(i);
  }
  return String(i);
}
String dateTimeIso(tm d)
{
  return String(d.tm_year+1900)+"-"+formatd2(d.tm_mon+1)+"-"+formatd2(d.tm_mday)+"T"+formatd2(d.tm_hour)+":"+formatd2(d.tm_min)+":"+formatd2(d.tm_sec);
}

// time and dns synch
void sendSynch()
{
  // will get updates if not in synch
  JsonDocument doc;
  doc["r"] = mqttMoniker + "/c/s";    // reply token
  doc["n"] = mqttId + String(unitId);
  doc["i"] = localIp.toString();
  doc["e"] = rtc.getEpoch();
  doc["v"] = dnsVersion;
  mqttSend("mb/s", doc);
}

void synchCheck()
{
  if (seconds - lastSynchTime > SYNCHINTERVAL/2)
  {
    lastSynchTime = seconds;
    sendSynch();
  }
}

void processSynch(JsonDocument &doc)
{
  unsigned long epoch = doc["e"].as<unsigned long>();
  if (epoch > 0)
  {
    rtc.setTime(epoch);
    tm now = rtc.getTimeStruct();
    log(2, "espTimeSet: " + dateTimeIso(now));
  }
  else
  {
    int timeAdjust = doc["t"].as<int>();
    if (timeAdjust != 0)
    {
      rtc.setTime(rtc.getEpoch() + timeAdjust);
      log(2, "espTimeAdjust: " + String(timeAdjust));
    }
  }
  long newDnsVersion = doc["v"].as<long>();
  if (newDnsVersion != 0)
  {
    dnsVersion  = newDnsVersion;
    log(2, "dns version: " + String(dnsVersion));
    for (int ix = 0; ix < DNSSIZE; ix++)
    {
      dnsList[ix].used = false;
    }
    for (int ix = 0; ix < DNSSIZE; ix++)
    {
      if (doc.containsKey("n" + String(ix)))
      {
        dnsList[ix].name = doc["n" + String(ix)].as<String>();
        dnsList[ix].ip = doc["i" + String(ix)].as<String>();
        dnsList[ix].used = true;
        dnsList[ix].timeout = 1;   // for consistency with dnsLog
        log(2, ".. " + dnsList[ix].name + " " + dnsList[ix].ip);
      }
      else
      {
        break;
      }
    }
  }
}



// ------------- mqtt section -----------------

// mqtt 
void setupMqttClient()
{
  IPAddress fixedIp = localIp;
  fixedIp[3] = mqttIp4;
  String server = fixedIp.toString();
  mqttClient.host = server;
  mqttClient.port = mqttPort;
  mqttMoniker = mqttId + "/" + String(unitId);
  mqttClient.client_id = mqttMoniker;
  String topic = mqttMoniker + "/c/#";
  mqttClient.subscribe(topic, &mqttMessageHandler);
  mqttSubscribeAdd();
  mqttClient.connected_callback = [] {mqttConnHandler();};
  mqttClient.disconnected_callback = [] {mqttDiscHandler();};
  mqttClient.connection_failure_callback = [] {mqttFailHandler();};
  mqttClient.begin();
}

// mqtt handlers
void mqttConnHandler()
{
  log(0, "MQTT connected: " + String(millis() - mqttDiscMs));
  sendSynch();
  mqttConnCount++;
}
void mqttDiscHandler()
{
  log(0, "MQTT disconnected");
  mqttDiscCount++;
  mqttDiscMs = millis();
}
void mqttFailHandler()
{
  log(0, "MQTT CONN FAIL");
  if (WiFi.isConnected())
  {
    mqttConnFailCount++;
  }
  mqttDiscMs = millis();
}
void mqttMessageHandler(const char * topicC, Stream & stream)
{
  mqttInCount++;
  String topic = String(topicC);
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, stream);
  if (error) 
  {
    log(2, "deserialize fail: " +  String(error.f_str()) + " " + topic);
    return;
  }
  if (logLevel >=2)
  {
    String s;
    serializeJson(doc, s);
    log(2, "in:" + topic + " " + s);
   }
  if (topic.endsWith("/p"))
  {   
    // its a property setting
    adjustProp(doc["p"].as<String>());
  }
  else if (topic.endsWith("/s"))
  {   
    // its a synch response message
    processSynch(doc);
  }
  else
  {
    handleIncoming(topic, doc);
  }
}

void mqttSend(String topic, JsonDocument &doc)
{
  if (logLevel >=2)
  {
    String s;
    serializeJson(doc, s);
    log(2, "out: " + topic + " " + s);
  }
  if (WiFi.isConnected() && mqttClient.connected())
  {
    // publish using begin_publish()/send() API
    auto publish = mqttClient.begin_publish(topic, measureJson(doc));
    serializeJson(doc, publish);
    publish.send();
    mqttSendCount++;
  }
}

// ------------ telnet --------------
void setupTelnet(int port) 
{  
  telnet.stop();
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onInputReceived(onTelnetInput);

  if (telnet.begin(port)) 
  {
    log(1, "telnet running");
  } 
  else 
  {
    log(1, "telnet start fail");
  }
}

void onTelnetConnect(String ip) 
{
  Serial.println("telnet connected");
  telnet.println("hello..");
}
void onTelnetDisconnect(String ip) 
{
  Serial.println("telnet disconnected");
}

void onTelnetReconnect(String ip) 
{
  Serial.println("telnet reconnected");
}

void onTelnetConnectionAttempt(String ip) 
{
  Serial.println("another telnet tried to connected - disconnecting");
  telnet.println("Another session trying. disconnecting you..");
  telnet.disconnectClient();
}

void onTelnetInput(String str) 
{
  processCommandLine(str);
}

void setRetryDelay()
{
  startRetryDelay = seconds;
  retryDelay = true;
  log(1, "retry delay....");
  recoveries++;
}
bool lastWifiState = false;
// state engine manager
void checkState()
{
  if (propValue != "")
  {
    adjustProp(propNameA + "=" + propValue);
    propValue = "";
  }  
  unsigned long nowMs = millis();
  while (nowMs - lastSecondMs > 1000)
  {
    seconds++;
    lastSecondMs+= 1000;
  }
  synchCheck();
  bool thisWifiState = WiFi.isConnected();
  if (thisWifiState != lastWifiState)
  {
    if (thisWifiState)
    {
      log(0, "WiFi Connected..");
      reConnWifiCount++;
    }
    else
    {
      log(0, "WiFi Disconnected..");
    }
    lastWifiState = thisWifiState;
  }
 
  if (retryDelay)
  {
    if (seconds - startRetryDelay < (retryDelayTime))
    {
      return; // retry wait
    }
    else
    {
      retryDelay = false;
    }
  }

  int res;
  switch (state)
  {
    case START:
      if (wifiIp4 == 0)
      {
        state = STARTCONNECTWIFI;    // dhcp ip
      }
      else
      {
        state = STARTGETGATEWAY;
      }
      return;
    case STARTGETGATEWAY:
      // only get gateway for fixed ip
      if (!startGetGateway())
      {
        setRetryDelay();
        return;
      }
      state = WAITGETGATEWAY;
      return;
    case WAITGETGATEWAY:
      res = waitWifi();
      if (res == 0)
      {
        return;
      }
      if (res < 0)
      {
        setRetryDelay();
        state = STARTGETGATEWAY;
        return;
      }
    case STARTCONNECTWIFI:
      if (!startWifi())
      {
        setRetryDelay();
        return;
      }
      state = WAITCONNECTWIFI;
      return;
    case WAITCONNECTWIFI:
      // mandatory we get connected once before proceeding
      res = waitWifi();
      if (res == 0)
      {
        return;
      }
      if (res < 0)
      {
        setRetryDelay();
        state = STARTCONNECTWIFI;
        return;
      }
      setupTelnet(telnetPort);
      setupMqttClient();
      state = ALLOK;

      return;

    case ALLOK:
      return;
  }
}
// ------------ end wifi and mqtt include section 2 ---------------

// ------------ start wifi and mqtt custom section 2 ---------------
void mqttSubscribeAdd()
{
  // use standard or custom handler
  // start subscribe add
 
  // end subscribe add
}
void handleIncoming(String topic, JsonDocument &doc)
{
  // start custom additional incoming
  if (topic.endsWith("/o"))
  {
    controllerOverrides(doc);
  }
  
  // end custom additional incoming
}
// ------------ end wifi and mqtt custom section 2 ---------------

// ---- start custom code --------------

//io pin mapping on PCF8574
// baked in that we are using top 4 input and lower 4 output
const byte CHONBIT = 0;         // force CH valve on
const byte HWONBIT = 1;         // force HW valve on
const byte HWOFFBIT  = 2;       // switch off from cylinder stat - hw off
const byte STATOFFBIT = 3;      // interrupts thermostat on boiler

const byte CHDEMANDBIT = 4;         // tado saying on
const byte HWDEMANDBIT = 5;         // hw stat sayimg on
const byte FLAMEONBIT = 6;          // main gas valve on so lit

#define IDLE 0
#define ANTICYCLE 2
#define HEATING 3
#define COOLING 4
#define SHUTDOWN 5

String tempStateS(int state)
{
  switch (state)
  {
    case IDLE: return "idle";
    case ANTICYCLE: return "anti";
    case HEATING: return "heat";
    case COOLING: return "cool";
    case SHUTDOWN: return "shut";
    default: return "??";
  }
}

int tempRequestWaitMs = 800;      // conversion time 12 bit
int valveOffDelay = 3;            // allow valve time to switch off
unsigned long requestTempMs;      // when conversion requested
bool tempRequested;               // in conversion wait
unsigned long lastOnTime;
unsigned long lastOffTime;
unsigned long boilerStatOffHoldStart;  
bool boilerStatOffHold;
bool pcfTestOk;   //!! to Do
byte seq;

// inputs
bool hwDemand;         
bool chDemand;
bool flameOn;
// from Boiler Contorl/ tado via mqtt
bool tadoAway;
int setTemp = 60;
// outputs
bool hwValveOn;        
bool chValveOn;
bool hwOff;
bool boilerStatOff;
// temp management
double actTemp = 50;
double prevTemp = 50;
double adjTemp = 50;
double ovrShoot = 0;
double undShoot = 0;
double tmpShoot = 0;
double shootHi = false;
double shootLo = false;

int controllerCountdown = 60;       // to time out signal from controller
int tempState = IDLE;
int dsCount;

// sim specific
double simArr[10];
double simTemp = 60;
int simArrSize = 10;

void simInit()
{
  tempState = HEATING;
  for (int ix = 0; ix < simArrSize; ix++)
  {
    simArr[ix] = simTemp;
  }
}

double doSim()
{
  switch (tempState)
  {
    case HEATING:
      simTemp += 0.2;
      break;
    default:
      if (simTemp > idleTemp - 3)
      {
        simTemp -= 0.2;
      }
      break;
  }
  for (int ix = simArrSize-1; ix > 0; ix--)
  {
    simArr[ix] = simArr[ix-1];
  }
  simArr[0] = simTemp;
  return simArr[simArrSize-1];
}

void restoreDefaults()
{
  setTemp = dSetTemp;
  tadoAway = false;
}

// scans deviced on the i2c bus
void ScanI2C()
{
  log(0, "I2C scanner. Scanning ...");
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      log(0, "Found: " + String(i));
    }
  }
}

String bitsS(byte b, int nBits)   // bit pattern
{
  String s;
  for (int ix = 0; ix < nBits; ix++)
  {
    s+= String(bitRead(b, ix)) + " ";
  }
  return s;
}

// for setting relays in one go
bool pcfSet8(byte pa, bool ba, byte pb, bool bb, byte pc, bool bc, byte pd, bool bd)
{
  byte pcf8Set = 0xff;
  if (ba) bitClear(pcf8Set, pa);  // remember relay inverse pulldown
  if (bb) bitClear(pcf8Set, pb);
  if (bc) bitClear(pcf8Set, pc);
  if (bd) bitClear(pcf8Set, pd);
 
  byte pcf8Get = PCF_01.read8();
  if ((pcf8Set & 0x0F) == (pcf8Get & 0x0F))
  {
    return true;
  }
  log(2, "Set PCF: " + bitsS(pcf8Set, 4));
  PCF_01.write8(pcf8Set);
  pcf8Get = PCF_01.read8();     // confirm
  if ((pcf8Set & 0x0F) == (pcf8Get & 0x0F))
  {
    log(2, " ok");
    return true;
  }
  log(2, " fail, read= " +  bitsS(pcf8Get, 4));
  return false;
}

// tests PFC responds on lower 4 bits used for relays
bool pcfTest()
{
  int failCount = 0;
  for (int ix = 0; ix < 4; ix++)
  {
    for (int ib = 0; ib < 2; ib++)
    {
      bool state = (ib == 0);  /// on then off 
      if (!pcfSet8(0, ix==0 && state, 1, ix==1 && state, 2, ix==2 && state, 3, ix==3 && state))
      {
        failCount++;
      }
      delay(100);
    }
  }
  return (failCount == 0);
}

String onOffS(bool b)
{
  if (b) return "on";
  return "..";
}

void tempSetup()
{
  simInit();
  restoreDefaults();
  sensors.begin();
  dsCount = sensors.getDeviceCount();
  log(0, "ds sensors = " + String(dsCount));
  
  // PFC control interface
  ScanI2C();
  PCF_01.begin(0xFF);  // set all bits on
  log(0, "pcs initial read " + bitsS(PCF_01.read8(), 8));
  pcfTestOk = pcfTest();    // check set bits respond
}

void controllerOverrides(JsonDocument doc)
{
  if (doc.containsKey("st"))
  {
    setTemp = doc["st"].as<int>();
  }
  if (doc.containsKey("aw"))
  {
    tadoAway = doc["aw"].as<int>();
  }
  controllerCountdown = controllerTimeout;
}


unsigned long lastTempMs = 0;
void tempLoop()
{
  unsigned long millisNow = millis();
  if (millisNow - lastTempMs >= 1000)
  {
    sensors.requestTemperatures(); 
    lastTempMs = millisNow;
    tempRequested = true;
    // do housekeeping
    controllerCountdown--;
    if (controllerCountdown <= 0)
    {
      // go back to default control
      restoreDefaults();
    }
  }
  if (tempRequested && millisNow - requestTempMs >= tempRequestWaitMs)
  {
    tempRequested = false;

    // getInputs
    byte getPcf8 = PCF_01.read8();
    hwDemand = bitRead(getPcf8, HWDEMANDBIT);
    chDemand = bitRead(getPcf8, CHDEMANDBIT);
    flameOn = bitRead(getPcf8, FLAMEONBIT);

    if (simMode)
    {
      actTemp = doSim();
    }
    else
    {
      actTemp = sensors.getTempCByIndex(0);
    }
   
    adjTemp = actTemp + ((actTemp - prevTemp) * tempRate);
    prevTemp = actTemp;

    if (shootHi)
    {
      if (actTemp > tmpShoot)
      {
        tmpShoot = actTemp;
      }
      else if (actTemp < tmpShoot - 1)
      {
        shootHi = false;
        ovrShoot = tmpShoot - setTemp;
      }
    }
    if (shootLo)
    {
      if (actTemp < tmpShoot)
      {
        tmpShoot = actTemp;
      }
      else if (actTemp > tmpShoot + 1)
      {
        shootLo = false;
        undShoot = setTemp - hystTemp - tmpShoot;
      }
    }

    if (hwDemand || chDemand)
    {
      // is demand
      switch (tempState)
      {
        case IDLE:
        case SHUTDOWN:
          if (adjTemp >= setTemp - hystTemp/2)
          {
            tempState = COOLING;
            lastOffTime = seconds;
          }
          else
          {
            tempState = HEATING;
            lastOnTime = seconds;
          }
          break;
      }
    }
    else
    {
      // no demand
      switch (tempState)
      {
        case ANTICYCLE:
        case HEATING:
        case COOLING:
          if (adjTemp > idleTemp)
          {
            tempState = SHUTDOWN;
          }
          else
          {
            tempState = IDLE;
          }
          break;
      }
    }
    
    switch (tempState)
    {
      case ANTICYCLE:
        {
          if (seconds - lastOnTime > antiCycleTime)
          {
            tempState = HEATING;
            lastOnTime = seconds;      // for anticycle
          }
        }
        break;
      case HEATING:
        {
          if (adjTemp >= setTemp)
          { 
            tempState = COOLING;
            lastOffTime = seconds;
            tmpShoot = actTemp;
            shootHi = true;
          }
        }
        break;
      case COOLING:
        {
          if (adjTemp <= (setTemp - hystTemp))
          {
            if (seconds - lastOnTime > antiCycleTime)
            {
              tempState = HEATING;
              lastOnTime = seconds;
              tmpShoot = actTemp;
              shootLo = true;
            }
            else
            {
              tempState = ANTICYCLE;
            }
          }
        }
        break;
      case SHUTDOWN:
        {
          if (adjTemp <= idleTemp)
          {
            tempState = IDLE;
          }
        }
        break;
      case IDLE:
        // do nothing

        break;

    }
    hwValveOn = false;
    chValveOn = false;
    boilerStatOff = false;
    switch (tempState)
    {
      case HEATING:
        break;
      case COOLING:
      case ANTICYCLE:
        boilerStatOff = true;
        break;
      case SHUTDOWN:
        hwValveOn = true;
        chValveOn = true;
        boilerStatOff = true;
        boilerStatOffHold = true;
        boilerStatOffHoldStart = seconds;
        break;
      default:
        break;
    }
    if (boilerStatOffHold)
    {
      if (seconds - boilerStatOffHoldStart < valveOffDelay)
      {
        boilerStatOff = true;
      }
    }

    hwOff = tadoAway;
    // set relays
    pcfSet8(HWONBIT, hwValveOn, CHONBIT, chValveOn, HWOFFBIT, hwOff, STATOFFBIT, boilerStatOff);

    // report
    JsonDocument doc;
    String s = tempStateS(tempState) + " ";
    switch (tempState)
    {
      case HEATING: s+= String(seconds - lastOnTime); break;
      case COOLING: s+= String(seconds - lastOffTime); break;
      case ANTICYCLE: s+= String(antiCycleTime - (seconds - lastOnTime)); break;
      case SHUTDOWN: s+= String(seconds - lastOffTime); break;
    }
    doc["ts"] = s;
    doc["hw"] = onOffS(hwDemand);
    doc["ch"] = onOffS(chDemand);
    doc["hv"] = onOffS(hwValveOn);
    doc["cv"] = onOffS(chValveOn);
    doc["so"] = onOffS(!boilerStatOff);
    doc["fo"] = onOffS(flameOn);
    doc["ho"] = onOffS(hwOff);
    doc["st"] = int(setTemp *10 + 0.5);
    doc["at"] = int(actTemp *10 + 0.5);
    doc["jt"] = int(adjTemp *10 + 0.5);
    doc["un"] = int(undShoot *10.0 + 0.5);
    doc["ov"] = int(ovrShoot *10.0 + 0.5);

    doc["seq"] = seq++;

    String topic = mqttMoniker + "/d";
    mqttSend(topic, doc);

    if (graphMode)
    {
      loga(1,String(setTemp));
      loga(1," ");
      loga(1,String(setTemp - hystTemp));
      loga(1," ");
      loga(1,String(actTemp));
      loga(1," ");
      log(1,String(adjTemp));
    }
  }
}

// ---- end custom code --------------

void setup()
{
  loga(1, module);
  loga(1, " ");
  log(1, buildDate);
  Serial.begin(115200);
  checkRestartReason();
  mountSpiffs();
  readProps();
  tempSetup();
  esp_task_wdt_config_t config;
  int wdt = max(wdTimeout*1000,2000);
  config.timeout_ms = max(wdTimeout*1000,2000);;
  config.idle_core_mask = 3; //both cores
  config.trigger_panic = true;
  esp_task_wdt_reconfigure(&config);
  esp_task_wdt_add(NULL);
}


void loop()
{
  esp_task_wdt_reset();
  telnet.loop();
  mqttClient.loop();
  checkSerial();
  checkState();
  tempLoop();

  delay(1);
}
