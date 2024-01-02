#include <arduino.h>

#include "MyLog.h"

#ifdef LOG_ENABLED
static bool serial_initialized = false;
#endif

void initialize_serial() {
  #ifdef LOG_ENABLED

      Serial.begin(19200);
      delay(500);
      serial_initialized = true;

  #endif
};

void my_log(int level, const char* format, ...) {

  #ifdef LOG_ENABLED
    if (!serial_initialized)
      initialize_serial();

    if (level >= LOG_LEVEL)
    {
      char buffer[256];
      va_list args;
      va_start(args, format);
      vsprintf(buffer, format, args);
      va_end(args);

      char ts[12];
      sprintf(ts, "[%u] ", (unsigned int)millis());
      Serial.print(ts);
  
      Serial.println(buffer);
    }
  #endif
};
