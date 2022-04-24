#include <main.h>

//====State variables====
// Global variables used by program. Indicate various parameters and states

double temperature; //Temperature of target: kiln, hotplate, boiler etc.
double cjTemperature; //Temperature of cold junction. Also ambient temperature
double housingTemperature; //Temperature of housing

double Setpoint, Input, Output; //PID. Define Variables we'll be connecting to
unsigned long windowStartTime; //PID Relay Time Window

time_t prevDisplay = 0; // when the digital clock was displayed

uint8_t primaryFault; //Fault of primary temperature sensor if any
bool tcFault; // Is thermocouple fault
bool rangeFault; //Is temperature range fault
uint8_t tcFaultCount;

bool heaterOn = false;
bool programStart = false;
bool programRunning = false;
bool programEnd = false;
char programFileName[64];

time_t progStartTime;
time_t progEndTime;
uint16_t programFullDuration = 0; //Full programm duration in minutes


//====Config variables====

max31856_thermocoupletype_t thermocoupleType = MAX31856_TCTYPE_K;
//PID. Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
unsigned int timeWindow = 1000; //SSR PWM Period, ms

char ssid[64]; 
char pass[64]; 

// NTP Servers:
char NTP_Server1[64];
char NTP_Server2[64];
int timeZone = 3; 


float tcHighLimit = 1050.0f;
float tcLowLimit = 10.0f;
int8_t cjHighLimit = 50;
int8_t cjLowLimit = 10;


int WiFi_Mode; // WiFi Mode: 0 - Connect to AP; 1 - Connect to AP, be AP if failed; 2 - Be only AP; 3 - No WiFi
int LocalJS; // 0 - Use CDN; 1 - Use local js libraries
int Auth;// 0 - no HTTP auth; 1 - Use http auth

char Auth_name[64];
char Auth_pass[64];

int GTM_TimeOffset;
char Initial_Time[8]; //hh:mm
char Initial_Date[11]; //yyyy-mm-dd
//P.S. ORDER MATTERS. Why? Because it doesn't works in reverse, fuck it


int ThermalRunaway = 0; //How long wait for temperature to rise before considering an error


//====Private variables====

unsigned int localPort = 8888;
const char progDir[] = "/progs/";
uint16_t prgArrayLength; //Length of programm array
prgStep progArray[100]; //Array of programm steps. 100 steps max

const char* PARAM_MESSAGE = "message";
const char* PARAM_TEMP = "temp";

//Objects
Adafruit_MAX31856 PrimaryTempSensor = Adafruit_MAX31856(MAXCS);
PID HeaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
WiFiUDP Udp;
StaticJsonDocument<768> doc;
AsyncWebServer server(80);

/*----------------------MAIN CODE-------------------------*/
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  LoadSettings(SPIFFS);
  SetupWifi();
  SetupNTPSync();
  SetupMax31856();
  SetupServer();
  ClearLogs(SPIFFS);
  xTaskCreate(T_UpdateTemperature,"Temp update",8192,NULL,1,NULL);
  xTaskCreate(T_ProgramLoop,"Control loop",8192,NULL,1,NULL);
  //LoadProgram("test.txt");
  //StartProgram();
  //xTaskCreate(T_DisplayTime,"Debug time display",8192,NULL,1,NULL);
}
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
}

//Temperature measurement related functions
void SetupMax31856(){
  pinMode(DRDY_PIN, INPUT);

  PrimaryTempSensor.begin();
  delay(10);
  PrimaryTempSensor.setThermocoupleType(thermocoupleType);
  delay(100);
  debugLogSerial("[ADDONS] Init max31856");
  Serial.print("Thermocouple type: ");
  switch (PrimaryTempSensor.getThermocoupleType() ) {
    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }
  PrimaryTempSensor.setConversionMode(MAX31856_CONTINUOUS);
  debugLogSerial("[ADDONS] Set max31856 mode to continious");
  PrimaryTempSensor.setTempFaultThreshholds(tcLowLimit,tcHighLimit);
  PrimaryTempSensor.setColdJunctionFaultThreshholds(cjLowLimit,cjHighLimit);
  delay(1000);
  primaryFault = PrimaryTempSensor.readFault();
  debugLogSerial("[ADDONS] Fault %X", primaryFault);
    if (primaryFault) {
      if (primaryFault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (primaryFault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (primaryFault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (primaryFault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (primaryFault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (primaryFault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (primaryFault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (primaryFault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
  }

}

//Setup and initialize PID
void SetupPID(){
  HeaterPID.SetOutputLimits(0,timeWindow);
  HeaterPID.SetSampleTime(timeWindow);
  HeaterPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  Setpoint = 0;

}

void SetupSSR(){
  pinMode(SSR_PIN,Output);
  digitalWrite(SSR_PIN,LOW);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  debugLogSerial("[TIME]Transmit NTP Request");
  WiFi.hostByName(NTP_Server1, ntpServerIP);
  debugLogSerial("[TIME] %s : %s", NTP_Server1, ntpServerIP.toString().c_str());
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      debugLogSerial("[TIME] Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  debugLogSerial("[TIME] No NTP Response. Trying another server");
  debugLogSerial("[TIME]Transmit NTP Request");
  WiFi.hostByName(NTP_Server2, ntpServerIP);
  debugLogSerial("[TIME] %s : %s", NTP_Server2, ntpServerIP.toString().c_str());
  sendNTPpacket(ntpServerIP);
  beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      debugLogSerial("[TIME] Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  debugLogSerial("[TIME] No NTP Response");
  return 0; // return 0 if unable to get the time
  //Add a SEPARATE function for initial setup. If we cant sync with NTP - DO NOT ATTEMPT TO SYNC AT ALL
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void SetupWifi(){
  debugLogSerial("Connecting to %s", ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debugLogSerial(".");
  }

  debugLogSerial("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
}

void SetupNTPSync(){
  Serial.println("TimeNTP Example");
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.remotePort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(48*SECS_PER_HOUR);
}

// FS Related Functions
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.print("SIZE: ");
    Serial.println(file.size());
    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    Serial.println(" ");
    file.close();
}

void readProgram(fs::FS &fs, const char * path){
  uint8_t fSize = 0;
  uint8_t stringAmount = 0;
  prgStep tempStep;
  char progName[64];


  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path);
  if(!file || file.isDirectory()){
      Serial.println("- failed to open file for reading");
      return;
  }
  fSize = file.size();
  //Serial.print("File size: ");
  //Serial.println(fSize);
  if (fSize == 0){
    file.close();
    Serial.println("File empty");
    return;
  }
  std::vector<String> v;
  while (file.available()) {
    v.push_back(file.readStringUntil('\n'));
  }

  //Serial.print("Program name ");
  //Serial.println(v[0].substring(1));
  prgArrayLength = v.size()-1;
  Serial.println(prgArrayLength);

  for (int i=1; i<v.size();i++){
    int firstDec = v[i].indexOf(":");
    int secondDec = v[i].indexOf(":",firstDec+1);
    tempStep.temp=v[i].substring(0,firstDec).toFloat();
    tempStep.rise=v[i].substring(firstDec+1,secondDec+1).toInt();
    if (v[i].indexOf("#") != -1){
      int commentStart = v[i].indexOf("#");
      tempStep.hold=v[i].substring(secondDec+1,commentStart+1).toInt();
    }
    else {
      tempStep.hold=v[i].substring(secondDec+1).toInt();
    }

    progArray[i-1].temp = tempStep.temp;
    progArray[i-1].rise = tempStep.rise;
    progArray[i-1].hold = tempStep.hold;


  }

  file.close();

}

void LoadProgram(const char *progname){
  strcpy(programFileName,progname);
  debugLogSerial("[PROG] Loading program %s", programFileName);
  char progpath[70];
  strcpy(progpath,progDir);
  strcat(progpath,programFileName); //Create programm path
  readProgram(SPIFFS,progpath); //Read programm from file to array
  programFullDuration = 0;
  for (int i=0;i<prgArrayLength;i++){
    programFullDuration += progArray[i].rise;
    programFullDuration += progArray[i].hold;
  }
  debugLogSerial("[PROG] Programm duration %u minutes",programFullDuration);

}

void StartProgram(){
  programStart = true;
}

void AbortProgram(){
  programStart = false;
  programRunning = false;
  programEnd = false;
  heaterOn = false;
  strcpy(programFileName,"");
}

//Linear interpolation y(x) between two points x0,y0 and x1,y1
double lerp(double x0, double x1, double y0, double y1, double x){
  return (y0*(x1-x)+y1*(x-x0))/(x1-x0);
}

void HandleSensorFault(){
  if (rangeFault || tcFault) {
    if (programRunning){
      AbortProgram();
      debugLogSerial("[PROG] Program aborted! Reason: range fault %d, sensor fault %d", rangeFault, tcFault);
    } else {
      debugLogSerial("[PROG] Sensor fault but no program to abort. Reason: range fault %d, sensor fault %d", rangeFault, tcFault);
    }
  }
}

void LoadSettings(fs::FS &fs){

  File confFile = fs.open("/etc/config.txt");
  if(!confFile || confFile.isDirectory()) {
    debugLogSerial("Failed to open config file");
    return;
  }

  DeserializationError error = deserializeJson(doc, confFile);
  if (error) {
    debugLogSerial("deserializeJson() failed: %c",error.c_str());
    return;
  }

  strcpy(ssid,doc["WiFi_SSID"]);
  strcpy(pass,doc["WiFi_Pass"]);
  //Serial.println(ssid);
  //Serial.println(pass);
  WiFi_Mode = doc["WiFi_Mode"];

  LocalJS = doc["Local_JS"].as<int>();
  Auth = doc["Auth"];
  strcpy(Auth_name,doc["Auth_name"]);
  strcpy(Auth_pass,doc["Auth_pass"]);

  strcpy(NTP_Server1, doc["NTP_Server1"]);
  strcpy(NTP_Server2, doc["NTP_Server2"]);
  timeZone = doc["GTM_TimeOffset"]; 
  const char* Initial_Date2 = doc["Initial_Date"];
  memcpy(Initial_Date, Initial_Date2, 11);  
  strcpy(Initial_Time,doc["Initial_Time"]);
  


  Kp = doc["PID_Kp"];
  Ki = doc["PID_Ki"];
  Kd = doc["PID_Kd"];

  tcLowLimit = doc["Min_Temp"].as<float>();
  tcHighLimit = doc["Max_Temp"].as<float>();
  cjLowLimit = doc["Min_Dev_Temp"].as<int8_t>();
  cjHighLimit = doc["Max_Dev_Temp"].as<int8_t>();
  ThermalRunaway = doc["Thermal_Runaway"];

  char TC_Type = doc["TC_Type"].as<char>();
  switch (TC_Type)
  {
  case 'K':
    thermocoupleType = MAX31856_TCTYPE_K;
    break;
  case 'N':
    thermocoupleType = MAX31856_TCTYPE_N;
    break;
  case 'J':
    thermocoupleType = MAX31856_TCTYPE_J;
    break;
  case 'S':
    thermocoupleType = MAX31856_TCTYPE_S;
    break;
  case 'T':
    thermocoupleType = MAX31856_TCTYPE_T;
    break;
  case 'E':
    thermocoupleType = MAX31856_TCTYPE_E;
    break;
  case 'R':
    thermocoupleType = MAX31856_TCTYPE_R;
    break;
  default:
    break;
  }

  confFile.close();


  debugLogSerial("Json readed from settings: ");
  serializeJsonPretty(doc,Serial);
  debugLogSerial("");




}

void SaveSettings(fs::FS &fs){
  File confFile = fs.open("/etc/config.txt");
  if(!confFile || confFile.isDirectory()) {
    debugLogSerial("Failed to open config file");
    return;
  }
  confFile.close();
  fs.remove("/etc/config.txt");
  File confFile1 = fs.open("/etc/config.txt",FILE_WRITE,true);
  
  StaticJsonDocument<512> doct;
  doct["WiFi_SSID"] = ssid;
  doct["WiFi_Pass"] = pass;
  doct["WiFi_Mode"] = WiFi_Mode;
  doct["Local_JS"] = LocalJS;
  doct["Auth"] = Auth;
  doct["Auth_name"] = Auth_name;
  doct["Auth_pass"] = Auth_pass;
  doct["NTP_Server1"] = NTP_Server1;
  doct["NTP_Server2"] = NTP_Server2;
  doct["GTM_TimeOffset"] = timeZone;
  doct["Initial_Date"] = Initial_Date;
  doct["Initial_Time"] = Initial_Time;
  doct["PID_Kp"] = Kp;
  doct["PID_Ki"] = Ki;
  doct["PID_Kd"] = Kd;
  doct["Min_Temp"] = tcLowLimit;
  doct["Max_Temp"] = tcHighLimit;
  doct["Min_Dev_Temp"] = cjLowLimit;
  doct["Max_Dev_Temp"] = cjHighLimit;
  doct["Thermal_Runaway"] = ThermalRunaway;

  switch (thermocoupleType) {
    case MAX31856_TCTYPE_B: doct["TC_Type"] = "B"; break;
    case MAX31856_TCTYPE_E: doct["TC_Type"] = "E"; break;
    case MAX31856_TCTYPE_J: doct["TC_Type"] = "J"; break;
    case MAX31856_TCTYPE_K: doct["TC_Type"] = "K"; break;
    case MAX31856_TCTYPE_N: doct["TC_Type"] = "N"; break;
    case MAX31856_TCTYPE_R: doct["TC_Type"] = "R"; break;
    case MAX31856_TCTYPE_S: doct["TC_Type"] = "S"; break;
    case MAX31856_TCTYPE_T: doct["TC_Type"] = "T"; break;
    default: Serial.println("Unknown"); break;
  }

  debugLogSerial("Json formed from settings: ");
  serializeJsonPretty(doct,Serial);
  serializeJsonPretty(doct,confFile1);
  confFile1.close();
}

void CreateLogFile(fs::FS &fs, time_t starttime){
  char logfilepath[128];
  char tempstring[64];
  strcpy(logfilepath,"/logs/");
  sprintf(tempstring,"%i",starttime);
  strcat(logfilepath,tempstring);
  strcat(logfilepath,".txt");
  //Serial.println(starttime);
  debugLogSerial("%s",logfilepath);
  File logfile = fs.open(logfilepath,"w",true);
  logfile.close();
}

void WriteLog(fs::FS &fs, time_t starttime, time_t currenttime){
  char logfilepath[128];
  char tempstring[64];
  strcpy(logfilepath,"/logs/");
  sprintf(tempstring,"%i",starttime);
  strcat(logfilepath,tempstring);
  strcat(logfilepath,".txt");
  debugLogSerial("%s",logfilepath);
  File logfile = fs.open(logfilepath,"a");
  logfile.printf("%i, %f \n",currenttime,temperature);
  logfile.flush();
  logfile.close();

}

void ClearLogs(fs::FS &fs){
  char logfilepath[128];
  char tempstring[64];
  uint8_t famount = 0;

  Serial.printf("Listing directory: /logs\r\n");

    File root = fs.open("/logs");

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
            famount++;
        }
        file = root.openNextFile();
    }
    root.close();
    file.close();

    if (famount > LOG_FILE_LIMIT) {
      root = fs.open("/logs");
      file = root.openNextFile();
      for (int i = 0; i<LOG_FILE_LIMIT;i++){
        strcpy(logfilepath,file.path());
        file = root.openNextFile();
        fs.remove(logfilepath);
      }
    }

}

void SetupServer(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } 
        if (request->hasParam(PARAM_TEMP)) {
          message = temperature;
        }
        else {
            message = "No message sent";
        }
        request->send(200, "text/plain", message);
    });

    server.on("/getstate", HTTP_GET, [] (AsyncWebServerRequest *request){
      String resp;
      if (request -> hasParam("temperature")){
        resp = temperature;
      }
      else if (request -> hasParam("cj_temperature")){
        resp = cjTemperature;
      }
      else if (request -> hasParam("setpoint")){
        resp = Setpoint;
      }
      else if (request -> hasParam("fault")){
        resp = primaryFault;
      }
      else {
        resp = "No param specified";
      }
      request->send(200,"text/plain",resp);
    });
    server.on("/program",HTTP_GET,[] (AsyncWebServerRequest *request){
      String resp;
      if (request ->hasParam("state")){ //0 - no program running, 1 - program running, 
        if (programRunning || programStart){
          resp = "1";
        } else if(strcmp(programFileName,"") != 0){
          resp = "2";
        }
        else {
          resp = "0";
        }
      } else if(request -> hasParam("proglist")){
        resp = ProgramList(SPIFFS);
        Serial.println(resp);
      } else {
        resp = "No param specified";
      }
      request->send(200,"text/plain",resp);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(NotFound);

    server.begin();

}

void NotFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

String ProgramList(fs::FS &fs){
    String response; 
    response = String();
    File root = fs.open("/progs");
    if(!root){
        Serial.println("- failed to open directory");
        return "Error";
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return "Error";
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            //Serial.print("  DIR : ");
            //Serial.println(file.name());
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
            response += file.name();
            response += "\t";
            response += file.size();
            response += "\r\n";
        }
        file = root.openNextFile();
    }
    return response;
}





//====RTOS tasks====
//Update temperature
void T_UpdateTemperature(void *params){
  for(;;){
    primaryFault = PrimaryTempSensor.readFault();
    if (primaryFault) {
      if (primaryFault & MAX31856_FAULT_CJRANGE) {
        debugLogSerial("[MAX31856] Cold Junction Range Fault");
        rangeFault = true;
      }
      if (primaryFault & MAX31856_FAULT_TCRANGE) {
        debugLogSerial("[MAX31856] Thermocouple Range Fault");
        rangeFault = true;
      }
      if (primaryFault & MAX31856_FAULT_CJHIGH)  debugLogSerial("[MAX31856] Cold Junction High Fault");
      if (primaryFault & MAX31856_FAULT_CJLOW)   debugLogSerial("[MAX31856] Cold Junction Low Fault");
      if (primaryFault & MAX31856_FAULT_TCHIGH)  debugLogSerial("[MAX31856] Thermocouple High Fault");
      if (primaryFault & MAX31856_FAULT_TCLOW)   debugLogSerial("[MAX31856] Thermocouple Low Fault");
      if (primaryFault & MAX31856_FAULT_OVUV)    debugLogSerial("[MAX31856] Over/Under Voltage Fault");
      if (primaryFault & MAX31856_FAULT_OPEN)    
      {
        debugLogSerial("[MAX31856]Thermocouple Open Fault");
        tcFaultCount++;
      }
    } else {
      if (tcFaultCount>0) tcFaultCount--; //Decrease fault count if we have no error
      rangeFault = false;
    }

    tcFault = (tcFaultCount>TC_ERROR_TRESHOLD) ? true : false;

    if (!digitalRead(DRDY_PIN)) {
      temperature = PrimaryTempSensor.readThermocoupleTemperature();
      cjTemperature = PrimaryTempSensor.readCJTemperature();
      //debugLogSerial("[MAX31856] Temperature sensor A readout: Internal temp = %.1f \t Last temp = %.1f \t",cjTemperature,temperature);
    }

    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

//PID control relay
void T_ControlHeater(void *params){
  for(;;){
    if (heaterOn){
      Input = temperature;
      HeaterPID.Compute();

      /************************************************
       * turn the output pin on/off based on pid output
       ************************************************/
      if (millis() - windowStartTime > timeWindow)
      { //time to shift the Relay Window
        windowStartTime += timeWindow;
      }
      if (Output > millis() - windowStartTime) digitalWrite(SSR_PIN, HIGH);
      else digitalWrite(SSR_PIN, LOW);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

// Programm loop
void T_ProgramLoop(void *params){
  
  time_t stepStartTime = 0;
  time_t stepRiseEndTime = 0;
  time_t stepHoldEndTime = 0;
  time_t stepRisePartEndTime = 0;
  uint16_t discStep = 1;
  double startTemp = 0;
  time_t t;
  time_t tempt;
  time_t tempt2;
  uint8_t currentStep = 0; //Current step of programm
  uint16_t tau = 0;

  for (;;){
    //Code to start program
    if (programStart){
      debugLogSerial("[PROG] Starting program %s",programFileName);
      t = now();
      progStartTime = t;
      debugLogSerial("[PROG] Programm start time: %i:%i:%i %i.%i.%i",hour(t),minute(t),second(t),day(t),month(t),year(t));
      progEndTime = progStartTime + programFullDuration*SECS_PER_MIN;
      t = progEndTime;
      debugLogSerial("[PROG] Programm end time: %i:%i:%i %i.%i.%i",hour(t),minute(t),second(t),day(t),month(t),year(t));
      startTemp = temperature;
      debugLogSerial("[PROG] Program start temperature: %f C",startTemp);
      CreateLogFile(SPIFFS,progStartTime);
      t = now();
      currentStep = 0;
      stepStartTime = t;
      tempt = t;
      tempt2 = t;
      stepRiseEndTime = t+progArray[0].rise*SECS_PER_MIN;
      stepHoldEndTime = stepRiseEndTime + progArray[0].hold*SECS_PER_MIN;
      tau = discStep*progArray[currentStep].rise/RISE_DISCR_STEP;
      stepRisePartEndTime = t+tau*SECS_PER_MIN;
      programStart = false;
      programRunning = true;
    }
    //Loop code
    if (programRunning){
      t=now();
      if (t<stepRiseEndTime){
        tau = discStep*progArray[currentStep].rise/RISE_DISCR_STEP;
        if (t<stepRisePartEndTime)
        {
          Setpoint = lerp(0,progArray[currentStep].rise,startTemp,progArray[currentStep].temp,tau);
        } 
        else 
        {
          discStep++;
          tau = discStep*progArray[currentStep].rise/RISE_DISCR_STEP;
          stepRisePartEndTime += SECS_PER_MIN*tau/discStep;
        }
      }

      if (t > stepRiseEndTime)
      {
        //do hold temperature
        if (Setpoint != progArray[currentStep].temp){
          Setpoint = progArray[currentStep].temp;
        }
      }
      if (t > stepHoldEndTime){
        //proceed to next step
        startTemp = progArray[currentStep].temp;
        discStep = 1;
        currentStep++;
        stepRiseEndTime = t+progArray[currentStep].rise*SECS_PER_MIN;
        stepHoldEndTime = stepRiseEndTime + progArray[currentStep].hold*SECS_PER_MIN;
        stepRisePartEndTime = t+discStep*progArray[currentStep].rise/RISE_DISCR_STEP;
      }

      //if program is ended
      if (t>progEndTime){
        programRunning = false;
        programEnd = true;
      }

      //debug message display
      if (t - tempt >= 30){
        tempt = t;
        debugLogSerial("[PROG], %f, %i:%i:%i",Setpoint, hour(t),minute(t),second(t));
      }
      //write log
      if (t-tempt2 >= LOG_PERIOD) {
        WriteLog(SPIFFS,progStartTime,t);
        tempt2 = t;
      }



      //debugLogSerial("[PROG], %f, %i:%i:%i",Setpoint, hour(t),minute(t),second(t));

    }
    //End program code
    if (programEnd){
      programEnd = false;
      debugLogSerial("[PROG] Program ended");
      char logfilepath[128];
      char tempstring[64];
      strcpy(logfilepath,"/logs/");
      sprintf(tempstring,"%i",progStartTime);
      strcat(logfilepath,tempstring);
      strcat(logfilepath,".txt");
      readFile(SPIFFS,logfilepath);

    }

    HandleSensorFault();
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

