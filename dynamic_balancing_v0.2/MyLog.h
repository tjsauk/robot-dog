#ifndef _MyLog_h
#define _MyLog_h

#include "myconfig.h"

void initialize_serial();

#define MYLOG_DEBUG 1
#define MYLOG_INFO 2
#define MYLOG_WARNING 3

void my_log(int level, const char* format, ...);

#endif // _MyLog_h
