#include "AM_CacheItem.h"

#include <stdlib.h>
#include <string.h>

AM_CacheItem::AM_CacheItem()
{
    name[0] = '\0';
}

AM_CacheItem::~AM_CacheItem()
{
    if (this->textValue != nullptr)
    {
        free(this->textValue);
        this->textValue = nullptr;
    }
}

char *AM_CacheItem::getName()
{
    return name;
}

void AM_CacheItem::setName(const char *namex)
{
    strcpy(this->name, namex);
}

void AM_CacheItem::setType(AM_ValueType type)
{
    this->type = type;
}

int AM_CacheItem::getIntValue()
{
    return this->intValue;
}

long AM_CacheItem::getLongValue()
{
    return this->longValue;
}

long AM_CacheItem::getUnsignedLongValue()
{
    return this->unsignedlongValue;
}

float AM_CacheItem::getFloatValue()
{
    return this->floatValue;
}

char *AM_CacheItem::getStringValue()
{
    return this->textValue;
}

void AM_CacheItem::setValue(int value)
{
    this->intValue = value;
}

void AM_CacheItem::setValue(long value)
{
    this->longValue = value;
}

void AM_CacheItem::setValue(unsigned long value)
{
    this->unsignedlongValue = value;
}

void AM_CacheItem::setValue(float value)
{
    this->floatValue = value;
}

void AM_CacheItem::setValue(const char *value)
{
    if (this->textValue == nullptr)
    {
        this->textValue = (char *)malloc(strlen(value) + 1);
        if (this->textValue == nullptr)
        {
            return;
        }
    }
    else
    {
        char *p = (char *)realloc(this->textValue, strlen(value) + 1);
        if (p == nullptr)
        {
            return;
        }
        this->textValue = p;
    }

    strcpy(this->textValue, value);
}

void AM_CacheItem::print(void)
{
    switch (type)
    {
    case AM_TEXT:
        printf("name: %s : %s\n", name, textValue);
        break;

    default:
        printf("name: %s : ??\n", name);
        break;
    }
}