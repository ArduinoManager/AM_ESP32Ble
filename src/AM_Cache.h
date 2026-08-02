#ifndef AM_CACHE_H
#define AM_CACHE_H

#include "AM_CacheItem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MAX_VARIABLES 50


class AM_Cache
{
private:
    AM_CacheItem cache[MAX_VARIABLES];
    uint8_t last_used;

public:
    AM_Cache(/* args */);
    ~AM_Cache();

    int find(const char *name);

    void add(const char *name, int value);
    void add(const char *name, long value);
    void add(const char *name, unsigned long value);
    void add(const char *name, float value);
    void add(const char *name, const char  *value);

    bool value_updated(const char *name, int value);
    bool value_updated(const char *name, long value);
    bool value_updated(const char *name, unsigned long value);
    bool value_updated(const char *name, float value);
    bool value_updated(const char *name, const char  *value);

    void clear(void);
};

#endif