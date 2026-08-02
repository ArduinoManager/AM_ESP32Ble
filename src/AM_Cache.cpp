#include "AM_Cache.h"

AM_Cache::AM_Cache(/* args */)
{
    this->last_used = 0;
}

AM_Cache::~AM_Cache()
{
}

int AM_Cache::find(const char *name)
{
    for (int i = 0; i < last_used; i++)
    {
        if (strlen(cache[i].getName()) != 0 && strcmp(cache[i].getName(), name) == 0)
        {
            return i;
        }
    }

    return -1;
}

void AM_Cache::add(const char *name, int value)
{
    AM_CacheItem item;

    item.setName(name);
    item.setType(AM_INT);
    item.setValue(value);

    cache[this->last_used++] = item;
}

void AM_Cache::add(const char *name, long value)
{
    AM_CacheItem item;

    item.setName(name);
    item.setType(AM_LONG);
    item.setValue(value);

    cache[this->last_used++] = item;
}

void AM_Cache::add(const char *name, unsigned long value)
{
    AM_CacheItem item;

    item.setName(name);
    item.setType(AM_UNSIGNED_LONG);
    item.setValue(value);

    cache[this->last_used++] = item;
}

void AM_Cache::add(const char *name, float value)
{
    AM_CacheItem item;

    item.setName(name);
    item.setType(AM_FLOAT);
    item.setValue(value);

    cache[this->last_used++] = item;
}

void AM_Cache::add(const char *name, const char *value)
{
    int next = this->last_used++;

    cache[next].setName(name);
    cache[next].setType(AM_TEXT);
    cache[next].setValue(value);
}

bool AM_Cache::value_updated(const char *name, int value)
{
    int variable_idx;

    variable_idx = find(name);

    if (variable_idx == -1)
    {
        add(name, value);
        return true;
    }

    int current_value = cache[variable_idx].getIntValue();

    if (current_value == value)
    {
        return false;
    }

    cache[variable_idx].setValue(value);

    return true;
}

bool AM_Cache::value_updated(const char *name, long value)
{
    int variable_idx;

    variable_idx = find(name);

    if (variable_idx == -1)
    {
        add(name, value);
        return true;
    }

    long current_value = cache[variable_idx].getLongValue();

    if (current_value == value)
    {
        return false;
    }

    cache[variable_idx].setValue(value);

    return true;
}

bool AM_Cache::value_updated(const char *name, unsigned long value)
{
    int variable_idx;

    variable_idx = find(name);

    if (variable_idx == -1)
    {
        add(name, value);
        return true;
    }

    unsigned long current_value = cache[variable_idx].getUnsignedLongValue();

    if (current_value == value)
    {
        return false;
    }

    cache[variable_idx].setValue(value);

    return true;
}

bool AM_Cache::value_updated(const char *name, float value)
{
    int variable_idx;

    variable_idx = find(name);

    if (variable_idx == -1)
    {
        add(name, value);
        return true;
    }

    float current_value = cache[variable_idx].getFloatValue();

    if (abs(current_value - value) <= 1e-5)
    {
        return false;
    }

    cache[variable_idx].setValue(value);

    return true;
}

bool AM_Cache::value_updated(const char *name, const char *value)
{
    int variable_idx;

    variable_idx = find(name);

    if (variable_idx == -1)
    {
        // printf("\t\tVariable %s not found\n", name);
        add(name, value);
        return true;
    }

    char *current_value = cache[variable_idx].getStringValue();

    if (strcmp(current_value, value) == 0)
    {
        return false;
    }

    cache[variable_idx].setValue(value);

    return true;
}

void AM_Cache::clear(void)
{
    this->last_used = 0;
    for (int i = 0; i < MAX_VARIABLES; i++)
    {
        cache[i].setName(nullptr);
        cache[i].setValue((char *)nullptr);
    }
}