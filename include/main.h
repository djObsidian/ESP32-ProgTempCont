#include <Arduino.h>
#include <Adafruit_MAX31856.h>
#include <TimeLib.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <SPIFFS.h>
#include <log.h>
#include <vector>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define MAXCS  27
#define DRDY_PIN 16

#define SSR_PIN 17

#define DEBUG_SERIAL true

#define FORMAT_SPIFFS_IF_FAILED true

#define RISE_DISCR_T_STEP 5 //Rise time discretisation step duration, minutes
#define RISE_DISCR_STEP 10

#define LOG_PERIOD 30 //Logging period in seconds
#define LOG_FILE_LIMIT 10 //Maximum amount of log files. After exeded old logs are deleted

#define TC_ERROR_TRESHOLD 5 //How many times we should encounter error in a row before we are sure that that's not a missread

typedef struct{
    double temp;
    uint16_t rise;
    uint16_t hold;
} prgStep;



//Temperature measurement functions
void SetupMax31856();

//Temperature measurement task
void T_UpdateTemperature(void *params);

//PID setup function
void SetupPID();

void SetupSSR();

//PID control task
void T_ControlHeater(void *params);

//time related functions
time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
void SetupNTPSync();
void T_DisplayTime(void *params);

//network
void SetupWifi();

//filesystem related functions
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void readFile(fs::FS &fs, const char * path);
void readProgram(fs::FS &fs, const char * path);

//settings functions
void LoadSettings(fs::FS &fs);
void SaveSettings(fs::FS &fs);

//programm control related functions
void LoadProgram(const char *progname);
void StartProgram();
void AbortProgram();

void T_ProgramLoop(void *params);

//logging
void CreateLogFile(fs::FS &fs, time_t starttime);
void WriteLog(fs::FS &fs, time_t starttime);
void ClearLogs(fs::FS &fs);

//helpers
double lerp(double x0, double x1, double y0, double y1, double x);
