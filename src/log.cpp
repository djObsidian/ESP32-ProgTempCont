#include <log.h>

void debugLogSerial(const char *fmt, ...){
    char *message;
    va_list args;
    size_t initialLen;
    size_t len;

    initialLen = strlen(fmt);

    message = new char[initialLen + 1];

    va_start(args, fmt);
    len = vsnprintf(message, initialLen + 1, fmt, args);
    if (len > initialLen) {
        delete[] message;
        message = new char[len + 1];

        vsnprintf(message, len + 1, fmt, args);
    }
    va_end(args);

    Serial.println(message);
}