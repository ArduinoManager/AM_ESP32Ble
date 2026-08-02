#ifndef AM_CACHEITEM_H
#define AM_CACHEITEM_H

#include <stdlib.h>
#include <string.h>

#include "AM_ESP32Ble.h"

enum AM_ValueType
{
    AM_INT,
    AM_LONG,
    AM_UNSIGNED_LONG,
    AM_FLOAT,
    AM_TEXT
};

class AM_CacheItem
{

private:
    char name[VARIABLELEN];
    AM_ValueType type;

    union
    {
        int intValue;
        long longValue;
        unsigned long unsignedlongValue;
        float floatValue;
    };

    char *textValue = nullptr;

public:
    AM_CacheItem();
    ~AM_CacheItem();

    char *getName();
    void setName(const char *name);

    void setType(AM_ValueType type);

    int getIntValue();
    long getLongValue();
    long getUnsignedLongValue();
    float getFloatValue();
    char *getStringValue();
    void setValue(int value);
    void setValue(long value);
    void setValue(unsigned long value);
    void setValue(float value);
    void setValue(const char *value);

    void print(void);
};

#endif