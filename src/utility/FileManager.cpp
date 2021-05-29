#include "FileManager.h"


bool FileManager::append(String &fileName, uint8_t *byte, size_t size) {

  File f = SD.open(fileName, FILE_WRITE);

  if (f) {

    uint8_t ret = f.write(byte, size);
    f.close();

    return (ret == size);
  }

Serial.print("Error opening file"); Serial.print(fileName);

  return false;
}

void FileManager::deleteFile(String &fileName) {
  SD.remove(fileName);
}

bool FileManager::read(String &fileName, uint8_t position, uint8_t *byte, size_t size) {

  File f = SD.open(fileName, FILE_READ);

  if (f) {

    bool ok;

    ok = f.seek(position * size);

    //    Serial.print("Seek result "); Serial.println(ok);
    //    Serial.print("Size "); Serial.println(f.size());
    //    Serial.print("Position "); Serial.println(f.position());

    if (ok)
      ok = f.read(byte, size);

    f.close();

    return ok;
  }

  return false;
}

bool FileManager::update(String &fileName, uint8_t position, uint8_t *byte, size_t size) {
  
  File f = SD.open(fileName, FILE_WRITE);

  if (f) {

    bool ok;

    ok = f.seek(position * size);

    //    Serial.print("Seek result "); Serial.println(ok);
    //    Serial.print("Size "); Serial.println(f.size());
    //    Serial.print("Position "); Serial.println(f.position());

    if (ok)
      ok = f.write(byte, size);

    f.close();

    return ok;
  }

  return false;
}

bool FileManager::copy(String &sourcefileName, const char *destinationfileName) {

  String d = String(destinationfileName);
  return copy(sourcefileName, d);
}

bool FileManager::copy(const char *sourcefileName, String &destinationfileName) {

  String d = String(sourcefileName);
  return copy(d, destinationfileName);
}

bool FileManager::copy(String &sourcefileName, String &destinationfileName) {

  uint8_t buffer[64];

  if (SD.exists(destinationfileName)) {

    boolean ret = SD.remove(destinationfileName);
    if (!ret)
      return false;
  }

  File sourceFile = SD.open(sourcefileName, FILE_READ);

  if (sourceFile) {

    File destinationFile = SD.open(destinationfileName, FILE_WRITE);

    if (!destinationFile) {

      sourceFile.close();
      return false;
    }

    uint8_t ret = sourceFile.read(buffer, 64);

    while (ret > 0) {

      destinationFile.write(buffer, ret);
      ret = sourceFile.read(buffer, 64);
    }

    destinationFile.close();
    sourceFile.close();
    return true;
  }

  return false;
}

bool FileManager::remove(String &fileName, uint8_t position, size_t size) {

  File sourceFile = SD.open(fileName, FILE_READ);
  File tmpFile = SD.open("/tmp.txt", FILE_WRITE);

  if (sourceFile) {

    if (!tmpFile) {

      sourceFile.close();
      return false;
    }

    uint16_t length = sourceFile.size();
    uint8_t  buffer[size];

    //Serial.print("\t\tNumber of records "); Serial.println(length / size);

    for (int i = 0; i < length / size; i++) {

      if (i != position) {

        //Serial.print("\t\tWrite record "); Serial.println(i);

        bool ok = sourceFile.seek(i * size);

        if (!ok) {

          sourceFile.close();
          tmpFile.close();
          return false;
        }

        ok = sourceFile.read(buffer, size);
        if (!ok) {

          sourceFile.close();
          tmpFile.close();
          return false;
        }

        ok = tmpFile.write(buffer, size);
        if (!ok) {

          sourceFile.close();
          tmpFile.close();
          return false;
        }

      }
    }

    //Serial.print("\t\tFinal number of records "); Serial.println(tmpFile.size() / size);

    sourceFile.close();
    tmpFile.flush();
    tmpFile.close();

    deleteFile(fileName);

    if (!copy("/tmp.txt", fileName))
      return false;

    SD.remove("/tmp.txt");

    return true;
  }

  return false;
}

int FileManager::find(String &fileName, uint8_t *byte, size_t size, bool (*check)(uint8_t *pRecord, void *pData), void *pData) {

  File f = SD.open(fileName, FILE_READ);

  if (f) {

    bool ok;
    uint16_t length = f.size();

    for (int i = 0; i < length / size; i++) {

      ok = f.seek(i * size);
      if (!ok) {

        f.close();
        return -1;
      }

      ok = f.read(byte, size);
      if (!ok) {

        f.close();
        return -1;
      }

      ok = (*check)(byte,pData);

      if (ok) {

        f.close();
        return i;
      }
    }

    f.close();
  }

  return -1;
}

