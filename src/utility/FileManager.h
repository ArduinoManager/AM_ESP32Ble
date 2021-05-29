
#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <SD.h>

class FileManager {

  public:
    void deleteFile(String &fileName);
    bool append(String &fileName, uint8_t *byte, size_t size);
    bool read(String &fileName, uint8_t position, uint8_t *byte, size_t size);
    bool update(String &fileName, uint8_t position, uint8_t *byte, size_t size);
    bool copy(String &sourcefileName, String &destinationfileName);
    bool copy(const char *sourcefileName, String &destinationfileName);
    bool copy(String &sourcefileName, const char *destinationfileName);
    bool remove(String &fileName, uint8_t position, size_t size);
    int find(String &fileName, uint8_t *byte, size_t size, bool (*check)(uint8_t *pRecord, void *pData), void *pData);
};

#endif
