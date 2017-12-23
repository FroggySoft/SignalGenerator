/*
 *  This sketch creates a WiFi access point and provide a web server on it.
 *
 * The webserver shows the signal type, frequency and duty-cycle
 *    
 *    NodeMCU 1.0 (ESP-12E module)
 *    
 *    AD9833 waveform generator
 *    
 *    Clock on D1
 *    Data  on D3
 *    Fsync on D4
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <DNSServer.h>

#define ESP8266_LED   2
#define LedPin  ESP8266_LED
#define LEDON   0
#define LEDOFF  1
#define ClockPin  D1
#define DataPin   D3
#define SyncPin   D4

enum WAVE_TYPE {
  WAVE_OFF,
  WAVE_SINE,
  WAVE_TRIANGLE,
  WAVE_SQUARE
};

#define REG_FREQ1   0x8000
#define REG_FREQ0   0x4000
#define REG_PHASE0  0xC000
#define REG_PHASE1  0xE000

#define REG_B28     0x2000
#define REG_HLB     0x1000
#define REG_FSEL    0x0800
#define REG_PSEL    0x0400
#define REG_PINSW   0x0200
#define REG_RESET   0x0100
#define REG_SLEEP1  0x0080
#define REG_SLEEP12 0x0040
#define REG_OPBITEN 0x0020
#define REG_SIGNPIB 0x0010
#define REG_DIV2    0x0008
#define REG_MODE    0x0002

#define SIGN_OUTPUT_MASK (REG_OPBITEN | REG_SIGNPIB | REG_DIV2 | REG_MODE)

bool prevLedState;
unsigned int mCounter = 0;

WAVE_TYPE  mType = WAVE_OFF ;
long  mFrequency = 10000;
bool  mOn = false;
uint16_t mControlRegister = 0;
uint32_t m_frequency_mhz = 25;

/* Set these to your desired credentials. */
const char *ssid = "SignalGenerator";
const char *password = "";
const byte DNS_PORT = 53;
const byte WEB_PORT = 80;

DNSServer        mDnsServer;
ESP8266WebServer mWebServer(WEB_PORT);


void setup()
{
  delay(1000);
  // declare the ledPin as an OUTPUT:
  pinMode(LedPin, OUTPUT);
  prevLedState = LEDOFF;
  digitalWrite(LedPin,prevLedState); 

  pinMode(ClockPin, OUTPUT);
  pinMode(DataPin, OUTPUT);
  pinMode(SyncPin, OUTPUT);

  digitalWrite(ClockPin,1); 
  digitalWrite(SyncPin,1); 

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress mIPAddress = WiFi.softAPIP();
  mDnsServer.start(DNS_PORT, "*", mIPAddress);

  // replay to all requests with same HTML
  mWebServer.onNotFound([]()
  {
    mWebServer.send(200, "text/html", GetHtmlReply());
  });
 
  mWebServer.on("/", handleRoot);
  mWebServer.on("/on", handleOn);
  mWebServer.on("/off", handleOff);
  mWebServer.on("/freq", handleFreq);
  mWebServer.on("/wave", handleWave);
  mWebServer.begin();

  resetDevice();
}

void loop()
{
  mDnsServer.processNextRequest();
  mWebServer.handleClient();
  delay(100);
}

void resetDevice()
{
  mControlRegister = REG_RESET | REG_B28;
  writeRegister(mControlRegister);
  setFrequency(mFrequency);
  setPhaseWord(0);
  mControlRegister = REG_B28;
  writeRegister(mControlRegister);
}

void setDeviceOn(bool aOn)
{
  mOn = aOn;
  if(mOn)
  {
    mControlRegister &= ~(REG_SLEEP1 | REG_SLEEP12);
  }
  else
  {
    mControlRegister |= (REG_SLEEP1 | REG_SLEEP12);  
  }
  writeRegister(mControlRegister);
}

void setWaveType(WAVE_TYPE aType)
{
  mType = aType;
  switch(mType)
  {
    case WAVE_SINE:
      mControlRegister &= ~REG_OPBITEN;
      mControlRegister &= ~REG_MODE;
      mControlRegister &= ~REG_DIV2;
      break;
    case WAVE_TRIANGLE:
      mControlRegister &= ~REG_OPBITEN;
      mControlRegister |= REG_MODE;
      mControlRegister &= ~REG_DIV2;
      break;
    case WAVE_SQUARE:
      mControlRegister &= ~REG_MODE;
      mControlRegister |= REG_OPBITEN;
      mControlRegister |= REG_DIV2;
      break;
  }
  writeRegister(mControlRegister);
}

uint32_t computeFrequencyWord(uint32_t aFrequency)
{
  // This is a manual expansion of (frequency * 2^28) / m_frequency_mhz
  // Since it doesn't require 64 bit multiplies or divides, it results in
  // substantially smaller code sizes.
  uint32_t lval = ((aFrequency & 0xFF) << 22) / (15625l * m_frequency_mhz);
  uint32_t mval = ((aFrequency & 0xFF00) << 14) / (15625l * m_frequency_mhz);
  uint32_t hval = ((aFrequency & 0xFF0000) << 6) / (15625l * m_frequency_mhz);
  return (hval << 16) + (mval << 8) + lval;
}

void setFrequency(long int aFrequency)
{
  mFrequency = aFrequency;
  uint32_t lFrequency = computeFrequencyWord(aFrequency);
  writeRegister(REG_FREQ0 | (lFrequency & 0x3FFF));
  writeRegister(REG_FREQ0 | ((lFrequency >> 14) & 0x3FFF));
}

void setPhaseWord(uint32_t phase)
{
  writeRegister(REG_PHASE0 | (phase & 0x0FFF));
}


/**************************************************************************/
/*
    @brief  Writes 16-bits to the generator
*/
/**************************************************************************/
void writeRegister(uint16_t aValue)
{
  // start tarnsmission
  digitalWrite(SyncPin,0); 

  uint16_t lByte = aValue;
  for (int i=0; i<16; i++)
  {
    bool lBit = lByte&0x8000;
    digitalWrite(DataPin, lBit);
    digitalWrite(ClockPin,0); 
    lByte <<= 1;
    digitalWrite(ClockPin,1); 
  }
  digitalWrite(SyncPin,1); 
}

/******************************
 * Handles for webrequests
 */
void handleRoot()
{
  mWebServer.send(200, "text/html", GetHtmlReply());
}


String GetHtmlReply()
{
  String lReply = "<html><Head>";
  lReply += "<title>Signal Generator</title></Head>";
  lReply += "<h1 align=center>Waveform signal generator</h1><br><hr><br>";

  lReply += "<table border=0 align=center>";
  
  lReply += "<tr><td><table border=0 align=center>";
  lReply += "<tr>";
  lReply += "<td align=center><button  width=100 height=60 type=\"button\" ";
  if (mOn)
  {
    lReply += "style=\"background-color:blue; color:white\" ";
  }
  lReply += "onclick=\"location.href='/on'\">On</button></td>";
  lReply += "<td align=center><button width=100 height=60 type=\"button\" ";
  if (!mOn)
  {
    lReply += "style=\"background-color:blue; color:white\" ";
  }
  lReply += "onclick=\"location.href='/off'\">Off</button></td>";
  lReply += "<td></td>";
  lReply += "</tr></table></td></tr>";
  
  lReply += "<tr><td><table border=0 align=center>";
  lReply += "<tr>";
  lReply += "<td align=center><button width=100 height=60 type=\"button\" ";
  if (mType==WAVE_SINE)
  {
    lReply += "style=\"background-color:blue; color:white\" ";    
  }
  lReply += "onclick=\"location.href='/wave?type=sine'\">Sine</button></td>";
  lReply += "<td align=center><button  width=100 height=60 type=\"button\" ";
  if (mType==WAVE_TRIANGLE)
  {
    lReply += "style=\"background-color:blue; color:white\" ";    
  }
  lReply += "onclick=\"location.href='/wave?type=triangle'\">Triangle</button></td>";
  lReply += "<td align=center><button  width=100 height=60 type=\"button\" ";
  if (mType==WAVE_SQUARE)
  {
    lReply += "style=\"background-color:blue; color:white\" ";    
  }
  lReply += "onclick=\"location.href='/wave?type=square'\">Square</button></td>";
  lReply += "</tr>";

  lReply += "</tr></table></td></tr>";
  
  lReply += "<tr><td><table border=0 align=center>";
  lReply += "<tr><td align=center colspan=2>";
  lReply += "<input type=\"number\" id=\"freq\" value=\"" + String(mFrequency) + "\"></td>";
  lReply += "<td><button onclick=\"sendFreq()\">Set</button></td></tr>";
 
  //lReply += "<tr><td><b>Duty-cycle:</b></td><td>"+String(mDuty) + " % </td></tr>";
  lReply += "</tr></table></td></tr>";

  lReply += "</table><br><br><br><br>";

  lReply += "<script>function sendFreq() {";
  lReply += "var x = document.getElementById(\"freq\").value;";
  lReply += "location.href='/freq?value='+x;}";
  lReply += "</script>";

  lReply += "</body></html>";

  return lReply;
}

void handleOn()
{
  setDeviceOn(true);
  mWebServer.send(200, "text/html", GetHtmlReply());
}
void handleOff()
{
  setDeviceOn(false);
  mWebServer.send(200, "text/html", GetHtmlReply());
}
void handleFreq()
{
  String lFreqStr = mWebServer.arg(0);
  unsigned int lFreq = lFreqStr.toInt();
  setFrequency(lFreq);
  mWebServer.send(200, "text/html", GetHtmlReply());
}
void handleWave()
{
  String lWaveStr = mWebServer.arg(0);
  if( lWaveStr.indexOf("sine")!=-1 ) setWaveType(WAVE_SINE);
  if( lWaveStr.indexOf("triangle")!=-1 ) setWaveType(WAVE_TRIANGLE);
  if( lWaveStr.indexOf("square")!=-1 ) setWaveType(WAVE_SQUARE);
  mWebServer.send(200, "text/html", GetHtmlReply());
}

