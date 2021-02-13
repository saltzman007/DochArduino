//ANZEIGEN HEATING ETC
//Github

#include <Arduino.h>

#include <Wire.h> //wire = 2Wire = i2c!
#include <LiquidCrystal_I2C.h>

#define _SMARTDEBUG
#include <smartdebug.h>
#include "MAX6675.h"
#include "Adafruit_MAX31865.h"

//PINOUT
//dont use 1 2 and serial!!

const int GasHahn = 21;
const int ZuendPin = 23;
const int PumpenSoftwarePWM = 26;

const int TempSpiCS = 4;
const int TempSpiDI = 13;

const int i2cSDA = 21; //not in code because this are esp32 adruino wire.cpp defaults
const int i2cSCL = 22; //not in code because this are esp32 adruino wire.cpp defaults

int BurningCheckDO = 32;
int BurningCheckCS = 5;
int BurningCheckCLK = 33;

//PININ
const int UrinSensorInteruptPin = 16;
const int TempSensor = 33;
const int AnalogPlus = 34;  //Pulldown 10K
const int AnalogMinus = 35; //Pulldown 10K

unsigned long ZuendZeitpunkt = 0; //0: Kein Zündvorgang
volatile unsigned long UrinSensorHeartbeat = 0;

int TempIst = 0;
const int MaxZuendZeit = 10000;

//Die ideale Tempereratur liegt zwischen TempMin und TempMax
//Bei TempError wird die Maschine wegen Überhitzung gestoppt

//Steigende Temperatur: Es wird geheizt bis TempMax
//Fallende TempIdeal: Es wird ab TempIdeal geheizt
//Ab TempMin kann genebelt werden

int TempMin = 240;
int TempIdeal = 260;
int TempMax = 280;
const int TempError = 300;

int UrinPumpStufe = 0;
const int GasMengeMin = 600;
bool GasHahnAuf = false;

int ErrorState = 0; //1: UrinLow; 2: GasLow; 4: Zuenden erfolglos

LiquidCrystal_I2C lcd(0x27, 16, 2); //  x27 i2c address lcdColumns, lcdRows
String Line1 = "Vesuv3000";
String Line2 = "Starting";

MAX6675 Flammdetektor(BurningCheckCLK, BurningCheckCS, BurningCheckDO);
Adafruit_MAX31865 TemperaturSensor = Adafruit_MAX31865(TempSpiCS, TempSpiDI, BurningCheckDO, BurningCheckCLK);

void Display()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(Line1);
  lcd.setCursor(0, 1);
  lcd.print(Line2);
}

void InteruptUrinSensor()
{
  UrinSensorHeartbeat = millis();
}

bool BinIchDran(unsigned long waitTime, unsigned long *p_oldTime)
{
  unsigned long millisecs = millis();

  if (*p_oldTime + waitTime < millisecs)
  {
    *p_oldTime = millisecs;
    return true;
  }

  return false;
}

inline boolean IsBurning()
{
  static bool result = false;

  const unsigned long waitTime = 300; // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  static unsigned long oldTime = 0;

  if (!BinIchDran(waitTime, &oldTime)) //if time too short return last return
    return result;

  int temp = (int)Flammdetektor.readCelsius();
  DEBUG_PRINTLN_VALUE("Flammdetektor: ", temp);

  if (temp > 100)
    result = true;
  else
    result = false;

  return result;
}

void GasHahnSchalten(bool auf)
{
  digitalWrite(GasHahn, auf);
  GasHahnAuf = auf;
}

void Zuenden()
{
  DEBUG_PRINTLN("Zuenden.");
  GasHahnSchalten(true);
  DEBUG_PRINTLN("Gashahn ist offen");

  digitalWrite(ZuendPin, 1);
  ZuendZeitpunkt = millis();
}

void Zuendkontrolle()
{
  const unsigned long waitTime = 500;
  static unsigned long oldTime = 0;

  if (!BinIchDran(waitTime, &oldTime))
    return;

  if (ZuendZeitpunkt == 0) //es wurde nicht gezündet
    return;

  if (IsBurning())
  {
    DEBUG_PRINTLN("Burning ");
    digitalWrite(ZuendPin, 0);
    ZuendZeitpunkt = 0;
    return;
  }

  if (millis() - ZuendZeitpunkt < MaxZuendZeit)
    return;

  DEBUG_PRINTLN("Zuenden erfolglos");
  ErrorState = 4;

  Line2 = "no ignition";
}

void ErrorAction()
{
  Line1 = "Error";
  DEBUG_PRINTLN_VALUE("Errorstate: ", ErrorState);

  GasHahnSchalten(false);
  digitalWrite(ZuendPin, 0);
  UrinPumpStufe = 0;
}

#ifdef _SMARTDEBUG
void WriteFault(Adafruit_MAX31865 max)
{
  uint8_t fault = max.readFault();
  if (fault)
  {
    DEBUG_PRINTLN_VALUE("Adafruit_MAX31865 Fault ", fault);

    if (fault & MAX31865_FAULT_HIGHTHRESH)
    {
      DEBUG_PRINTLN("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH)
    {
      DEBUG_PRINTLN("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW)
    {
      DEBUG_PRINTLN("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH)
    {
      DEBUG_PRINTLN("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW)
    {
      DEBUG_PRINTLN("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV)
    {
      DEBUG_PRINTLN("Under/Over voltage");
    }
    max.clearFault();
  }
}
#else
void WriteFault(Adafruit_MAX31865 max)
{
}
#endif

void UrinSoftwarePWM()
{
  if(UrinPumpStufe == 0)
    return;

  static unsigned long oldTime = 0;
  int UrinPumpFrequenz = 2 * UrinPumpStufe;

  if (BinIchDran(1000 / UrinPumpFrequenz, &oldTime))
  {
    if (TempIst < TempMin)
    {
      UrinPumpStufe = 0;
      return;
    }

    digitalWrite(PumpenSoftwarePWM, 1);
    delay(10);
    digitalWrite(PumpenSoftwarePWM, 0);
  }
}

void UrinRunningCheck()
{
  static unsigned long oldTime = 0;
  const unsigned long waitTime = 1000;

  if (BinIchDran(waitTime, &oldTime))
  {
    if (UrinPumpStufe == 0)
      return;

    if (millis() - UrinSensorHeartbeat > 5000)
    {
      Line2 = "no fluid running";
      ErrorState = 1;
    }
  }
}

void ReadTemp()
{
  const unsigned long waitTime = 500;
  static unsigned long oldTime = 0;

  if (BinIchDran(waitTime, &oldTime))
  {
    const float RREF = 4300.0; //pt100 <-> pt 1000
    TempIst = (int)TemperaturSensor.temperature(1000, RREF); //1000 == Ohm bei 0Grad
    WriteFault(TemperaturSensor);

    DEBUG_PRINTLN_VALUE("TEMP Ist: ", TempIst);
  }
}

void ShowHeatStatus()
{
  const unsigned long waitTime = 500;
  static unsigned long oldTime = 0;

  if (BinIchDran(waitTime, &oldTime))
  {
    if ((TempIst >= TempMin))
      Line2 = "hot";
    else
      Line2 = "cold";
  }
}

void TemperaturSteuerung()
{
  const unsigned long waitTime = 500;
  static unsigned long oldTime = 0;

  if (BinIchDran(waitTime, &oldTime))
  {
    if (TempIst < TempIdeal)
    {
      if (!IsBurning())
      {
        Zuenden();
      }
    }
    if (TempIst > TempError)
    {
      ErrorState = 8;
      Line2 = "Too hot.";
    }

    if (TempIst > TempMax)
    {
      GasHahnSchalten(false);
    }
  }
}

void GasKontrolle()
{
  const unsigned long waitTime = 500;
  static unsigned long oldTime = 0;

  if (!BinIchDran(waitTime, &oldTime))
    return;

  if (ZuendZeitpunkt != 0) //zuending hat seine eigene Kontrolle
    return;

  if(!GasHahnAuf || IsBurning())
    return;

  DEBUG_PRINTLN("Gas auf ohne Flamme");
  ErrorState = 16;
  Line2 = "flame extinct";
}

void CheckPlusAnalogMinus()
{
  const unsigned long waitTime = 300;
  static unsigned long oldTime = 0;

  if (BinIchDran(waitTime, &oldTime))
  {

    if (digitalRead(AnalogPlus) & (UrinPumpStufe < 9) & (TempIst >= TempMin))
    {
      ++UrinPumpStufe;
      UrinSensorHeartbeat = millis();
    }

    if (digitalRead(AnalogMinus) & (UrinPumpStufe > 0))
      --UrinPumpStufe;

    char str[17];
    sprintf(str, "Fluidlevel  %d", UrinPumpStufe);
    Line1 = str;

    DEBUG_PRINTLN_VALUE("URIN - PumpStufe: ", UrinPumpStufe);
  }
}

void setup()
{
  lcd.init();
  lcd.backlight();

  DEBUG_INIT(115200); // Initialisierung der seriellen Schnittstelle

  pinMode(AnalogPlus, INPUT);
  pinMode(AnalogMinus, INPUT);
  pinMode(PumpenSoftwarePWM, OUTPUT);
  pinMode(GasHahn, OUTPUT);
  pinMode(ZuendPin, OUTPUT);
  digitalWrite(GasHahn, 0);

  pinMode(UrinSensorInteruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(UrinSensorInteruptPin), InteruptUrinSensor, FALLING);
  TemperaturSensor.begin(MAX31865_2WIRE);

  DEBUG_PRINTLN("Setup finished.");
}

void loop()
{
  Display();

  if (ErrorState == 0)
  {
    ReadTemp();
    UrinSoftwarePWM();
    UrinRunningCheck();
    ShowHeatStatus();
    TemperaturSteuerung();
    Zuendkontrolle();
    GasKontrolle();
    CheckPlusAnalogMinus();
  }
  else
  {
    ErrorAction();
  }
}