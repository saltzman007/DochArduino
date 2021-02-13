
#ifdef _SMARTDEBUG
  #define DEBUG_INIT(speed) Serial.begin(speed)
  #define DEBUG_PRINT(txt) Serial.print(txt)
  #define DEBUG_PRINTLN(txt) Serial.println(txt)
  #define DEBUG_PRINTLN_VALUE(txt, val) Serial.print(txt); Serial.println(val)
  #define DEBUG_DELAY(ms) delay(ms)
#else
  #define DEBUG_INIT(speed)
  #define DEBUG_PRINT(txt)
  #define DEBUG_PRINTLN(txt)
  #define DEBUG_PRINTLN_VALUE(txt, val)
  #define DEBUG_DELAY(ms)
#endif

