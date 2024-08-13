/*
   AMController, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Author: Fabrizio Boco - fabboco@gmail.com

   All rights reserved

*/
#include "AM_ESP32Ble.h"
#include <BLE2902.h>

#ifdef ALARMS_SUPPORT

#ifdef DEBUG
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
static  const uint8_t 	 monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // API starts months from 1, this array starts from 0
#endif 

bool check(uint8_t *pRecord, void *pData) {

  Alarm a;

  memcpy(&a, pRecord, sizeof(a));

  if (strcmp(a.id, (char *)pData) == 0)
    return true;

  return false;
}

#endif


AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)
)
{
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;
  _sync = false;

  _connected = false;
  _connectionChanged = false;
}

#if defined(ALARMS_SUPPORT)

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*processAlarms)(char *alarm),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)
) : AMController(doWork, doSync, processIncomingMessages, processOutgoingMessages, deviceConnected, deviceDisconnected)
{
  _alarmFile = "/ALRME32B.TXT";

  _processAlarms = processAlarms;
  _lastAlarmCheck = 0;
	_startTime = 0;
}
#endif

void AMController::begin(const char *deviceName) {

  // Initialise the Bluefruit module
#ifdef DEBUG
  Serial.println("Initializing BLE Services");
#endif

  BLEDevice::init(deviceName);

  _pServer = BLEDevice::createServer();
  _pServerCallbacks = new ConnectionCallbacks(this);
  _pServer->setCallbacks(_pServerCallbacks);

  _pService = _pServer->createService(SERVICE_UUID);
  _pCharacteristic = _pService->createCharacteristic(
                       CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_READ |
                       BLECharacteristic::PROPERTY_WRITE |
                       BLECharacteristic::PROPERTY_NOTIFY
                     );

  _pCharacteristic->setCallbacks(new ReadWriteCallbacks(this));
  _pCharacteristic->addDescriptor(new BLE2902());
  
  _pService->start();
  
  // Battery Level
  _pBLService = _pServer->createService(BLEUUID((uint16_t)0x180F));
    
  _pBLCharacteristic = _pBLService->createCharacteristic(
                       BLEUUID((uint16_t)0x2A19),
                       BLECharacteristic::PROPERTY_READ |
                       BLECharacteristic::PROPERTY_NOTIFY
                     );
  
  _pBLService->addCharacteristic(_pBLCharacteristic);


  BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
  BatteryLevelDescriptor.setValue("Percentage 0 - 100");  
  _pBLCharacteristic->addDescriptor(&BatteryLevelDescriptor);
  
	_pBLService->start();

  // Advertising

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  // Setup the advertising packet(s)
#ifdef DEBUG
  Serial.println("Advertising started");
#endif

}

void AMController::loop() {
  this->loop(0);
}

void AMController::loop(unsigned long _delay) {

  if (_connectionChanged) {

    _connectionChanged = false;

    if (_connected) {

      _connected = true;
      if (_deviceConnected != NULL)
        _deviceConnected();
    }
    else {

      _connected = false;

      if (_deviceDisconnected != NULL)
        _deviceDisconnected();
        
      _pServer->startAdvertising();
    }
  }

  if (_dataAvailable) {
    _dataAvailable = false;
    processIncomingData();
  }

  if (_sync) {
    _sync = false;
    _doSync();
  }

#ifdef ALARMS_SUPPORT

  if (_processAlarms != NULL) {
    unsigned long now = this->now();

    if ( (now - _lastAlarmCheck) > ALARM_CHECK_INTERVAL) {
      _lastAlarmCheck = now;
      this->checkAndFireAlarms();
    }
  }

#endif

  _doWork();

  if (_connected)
    _processOutgoingMessages();

  delay(_delay);
}


void AMController::processIncomingData() {

  char      _variable[VARIABLELEN + 1];
  char      _value[VALUELEN + 1];
  bool      _var = true;
  uint8_t   _idx = 0;
  int	   		lastPound = -1;

  _variable[0] = '\0';
  _value[0] = '\0';

  uint8_t l = strlen(_remainBuffer);

  //Serial.print("Full buffer before >"); Serial.print(_remainBuffer); Serial.println("<");

  for (uint8_t i = 0; i < l; i++) {

    if (_var) {
      if (_remainBuffer[i] != '=') {
        _variable[_idx++] = _remainBuffer[i];
      }
      else {
        _variable[_idx] = '\0';
        _var = false;
        _idx = 0;
      }
    }
    else {
      if (_remainBuffer[i] == '#') {
        lastPound = i;
      }
      if (_remainBuffer[i] != '#') {
        _value[_idx++] = _remainBuffer[i];
      }
      else {
        _value[_idx] = '\0';
        _var = true;
        _idx = 0;

        if (strlen(_value) > 0 && strcmp(_variable, "Sync") == 0) {
          _sync = true;
        }
#ifdef ALARMS_SUPPORT
        else if (strcmp(_variable, "$AlarmId$") == 0 && strlen(_value) > 0) {
Serial.print("AlarmId "); Serial.print(_value);
          strcpy(_alarmId, _value);
        } else if (strcmp(_variable, "$AlarmT$") == 0 && strlen(_value) > 0) {
Serial.print("AlarmT "); Serial.print(_value);
          _alarmTime = atol(_value);
        }
        else if (strcmp(_variable, "$AlarmR$") == 0 && strlen(_value) > 0) {
Serial.print("AlarmR "); Serial.print(_value);
          if (_alarmTime == 0)
            this->removeAlarm(_alarmId);
          else
            this->createUpdateAlarm(_alarmId, _alarmTime, atoi(_value));
        }
#endif
#ifdef SD_SUPPORT
        else if (strlen(_variable) > 0 && strcmp(_variable, "SD") == 0) {
          File root;
          File entry;
#ifdef DEBUG
          Serial.println("List of Files");
#endif
          root = SD.open("/");
          if (!root) {
#ifdef DEBUG          
            Serial.println("Cannot open root dir");
#endif            
          }          
          root.rewindDirectory();
          entry =  root.openNextFile();
          if (!entry) {
#ifdef DEBUG          
            Serial.println("Cannot open first file");
#endif
          }
          while (entry) {          
            if (!entry.isDirectory()) {
	            String name = entry.name();
#ifdef DEBUG	            
              Serial.println(name);
#endif              
              this->writeTxtMessage("SD", name.c_str());
            }
            entry.close();
            entry = root.openNextFile();
          }
          root.close();
          this->writeTxtMessage("SD", "$EFL$");
#ifdef DEBUG          
          Serial.println("File list sent");
#endif          
        }
        else if (strlen(_variable) > 0 && strcmp(_variable, "$SDDL$") == 0) {
#ifdef DEBUG        
          Serial.print("File: "); Serial.println(_value);
#endif          
          String fileName = String(_value);
          fileName = "/" + fileName;
          File entry = SD.open(fileName.c_str(), FILE_READ);

          if (entry) {
#ifdef DEBUG          
            Serial.println("File Opened");
#endif            
            unsigned long n = 0;
            uint8_t buffer[64];
						this->writeTxtMessage("SD", "$C$");

            while (entry.available()) {
              n = entry.read(buffer, sizeof(buffer));
              writeBuffer(buffer, n * sizeof(uint8_t));
            }
            entry.close();
#ifdef DEBUG            
            Serial.println("File completed");
#endif            
            this->writeTxtMessage("SD", "$E$");
#ifdef DEBUG            
            Serial.println("End Sent");
#endif            
          }
        }
#endif
        if (strlen(_variable) > 0 && strlen(_value) > 0) {
#ifdef ALARMS_SUPPORT
          if (strcmp(_variable, "$Time$") == 0) {
            Serial.print("Setting time at value: "); Serial.println(atol(_value));
            _startTime = atol(_value) - millis() / 1000;
#ifdef DEBUG            
            Serial.print("Time Synchronized "); this->printTime(now()); Serial.println();
#endif            
          }
          else
#endif
#ifdef SDLOGGEDATAGRAPH_SUPPORT
            if (strlen(_variable) > 0 && strcmp(_variable, "$SDLogData$") == 0) {
              Serial.print("Logged data request for: "); Serial.println(_value);
              sdSendLogData(_value);
            }
            else
#endif
            {
              // Process incoming messages
#ifdef DEBUG                
              Serial.print("process "); Serial.print(_variable); Serial.print(" -> "); Serial.println(_value);
#endif              
              _processIncomingMessages(_variable, _value);
            }
        }
      }
    }
  }

  if (lastPound == l - 1) {
    _remainBuffer[0] = '\0';
  }
  else if (lastPound > 0) {
    char tmp[128];
    strcpy(tmp, &_remainBuffer[lastPound + 1]);
    strcpy(_remainBuffer, tmp);
  }

#ifdef DEBUG 
  Serial.print("Full buffer after  >"); Serial.print(_remainBuffer); Serial.println("<");
#endif  
}

void AMController::writeMessage(const char *variable, int value) {
char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%d#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  delay(WRITE_DELAY);
}

void AMController::writeMessage(const char *variable, float value) {
	char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%.3f#", variable, value);
  _pCharacteristic->setValue((uint8_t *)&buffer, strlen(buffer));
  _pCharacteristic->notify();
  delay(WRITE_DELAY);
}

void AMController::writeTripleMessage(const char *variable, float vX, float vY, float vZ) {
	char buffer[VARIABLELEN + VALUELEN + 3];

	if (!_connected) {
    return;
  }
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%.2f:%.2f:%.2f#", variable, vX, vY, vZ);
  writeBuffer((uint8_t *)&buffer, strlen(buffer)*sizeof(char));
  delay(WRITE_DELAY);
}

void AMController::writeTxtMessage(const char *variable, const char *value) {
 char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%s#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  delay(WRITE_DELAY);
}


/**
	Can send a buffer longer than 20 bytes
**/
void AMController::writeBuffer(uint8_t *buffer, int l) {
  uint8_t buffer1[22];

  if (!_connected) {
    return;
  }

  uint8_t idx = 0;

  while (idx < l) {

    uint8_t this_block_size = min(20, l - idx);
    memset(&buffer1, '\0', 22);
    memcpy(&buffer1, buffer + idx, this_block_size);

#ifdef DEBUG
    //Serial.print("Sending >"); Serial.print((char *)buffer1); Serial.print("<"); Serial.println();
#endif

    _pCharacteristic->setValue((uint8_t *)&buffer1, 20);
    _pCharacteristic->notify();
    delay(WRITE_DELAY);

    idx += this_block_size;
  }
}

void AMController::updateBatteryLevel(uint8_t level) {

  if (!_connected) {
    return;
  }
  
  #ifdef DEBUG    
  	Serial.print("Updating battery level to "); Serial.println(level);
	#endif
  
	_pBLCharacteristic->setValue(&level, 1);
	_pBLCharacteristic->notify();

	 delay(WRITE_DELAY);
	 delay(WRITE_DELAY);
}

void AMController::setDeviceName(const char *deviceName) {

#ifdef DEBUG    
  Serial.print("Setting new name "); Serial.println(deviceName);
#endif
	this->begin(deviceName);
}

void AMController::log(const char *msg) {

  this->writeTxtMessage("$D$", msg);
}

void AMController::log(int msg) {

  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$D$", buffer);
}

void AMController::logLn(const char *msg) {

  this->writeTxtMessage("$DLN$", msg);
}

void AMController::logLn(int msg) {

  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(long msg) {

  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(unsigned long msg) {

  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(float msg) {

  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {

  boolean previousValue = digitalRead(pin);

  digitalWrite(pin, value);
  delay(ms);
  digitalWrite(pin, previousValue);
}


#ifdef ALARMS_SUPPORT

#ifdef DEBUG

void AMController::printTime(unsigned long time) {

  int seconds;
  int minutes;
  int hours;
  int Wday;
  long Year;
  int Month;
  int Day;

  this->breakTime(time, &seconds, &minutes, &hours, &Wday, &Year, &Month, &Day);

  Serial.print(Day);
  Serial.print("/");
  Serial.print(Month);
  Serial.print("/");
  Serial.print(Year);
  Serial.print(" ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.print(seconds);
}

void AMController::breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day) {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from 1970 !!!

  unsigned long year;
  uint8_t month, monthLength;
  unsigned long days;

  *seconds = time % 60;
  time /= 60; // now it is minutes
  *minutes = time % 60;
  time /= 60; // now it is hours
  *hours = time % 24;
  time /= 24; // now it is days
  *Wday = ((time + 4) % 7) + 1;  // Sunday is day 1

  year = 0;
  days = 0;
  while ((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  *Year = year + 1970; // year is offset from 1970

  days -= LEAP_YEAR(year) ? 366 : 365;
  time -= days; // now it is days in this year, starting at 0

  days = 0;
  month = 0;
  monthLength = 0;
  for (month = 0; month < 12; month++) {
    if (month == 1) { // february
      if (LEAP_YEAR(year)) {
        monthLength = 29;
      }
      else {
        monthLength = 28;
      }
    }
    else {
      monthLength = monthDays[month];
    }

    if (time >= monthLength) {
      time -= monthLength;
    }
    else {
      break;
    }
  }
  *Month = month + 1;  // jan is month 1
  *Day = time + 1;     // day of month
}

#endif

unsigned long AMController::now() {
  if (_startTime == 0) {
  	// Time never synchronized 
  	return 0;
  }
  unsigned long now = _startTime + millis() / 1000;
  return now;
}

void AMController::createUpdateAlarm(char *id, unsigned long time, bool repeat) {

  FileManager fileManager;
  Alarm     a;
  int     pos;

  pos = fileManager.find(_alarmFile, (uint8_t*)&a, sizeof(a), &check, id);

  if (pos > -1) {

    a.time = time;
    a.repeat = repeat;

    fileManager.update(_alarmFile, pos, (uint8_t *)&a, sizeof(a));

#ifdef DEBUG
    dumpAlarms();
#endif
    return;
  }

  strcpy(a.id, id);
  a.time = time;
  a.repeat = repeat;

  fileManager.append(_alarmFile, (uint8_t *)&a, sizeof(a));

#ifdef DEBUG
  dumpAlarms();
#endif
}

void AMController::removeAlarm(char *id) {

  FileManager fileManager;
  Alarm     a;
  int     pos;
  pos = fileManager.find(_alarmFile, (uint8_t*)&a, sizeof(a), &check, id);

  if (pos > -1) {

    fileManager.remove(_alarmFile, pos, sizeof(a));
  }

#ifdef DEBUG
  dumpAlarms();
#endif
}


#ifdef DEBUG
void AMController::dumpAlarms() {

  Serial.println("\t----Dump Alarms -----");

  FileManager fileManager;

  for (int i = 0; i < MAX_ALARMS; i++) {

    Alarm a;

    if (!fileManager.read(_alarmFile, i, (uint8_t *)&a, sizeof(a)))
      return;

    time_t rawtime = a.time;
    struct tm *timeinfo = gmtime(&rawtime);

    Serial.print("\tId: "); Serial.print(a.id);
    Serial.print(" Time: "); Serial.print(timeinfo, "%A, %B %d %Y %H:%M:%S GMT");
    Serial.print(" Repeat: "); Serial.println(a.repeat);
  }
}
#endif

void AMController::checkAndFireAlarms() {

  FileManager fileManager;
  unsigned long now = this->now();

#ifdef DEBUG
  Serial.print("checkAndFireAlarms ");
  this->printTime(now);
  Serial.println();
  this->dumpAlarms();
#endif

  for (int i = 0; i < MAX_ALARMS; i++) {

    Alarm a;

    if (!fileManager.read(_alarmFile, i, (uint8_t *)&a, sizeof(a)))
      return;



    if (a.time <= now) {

#ifdef DEBUG
      Serial.print("Firing "); Serial.println(a.id);
#endif
      // First character of id is A and has to be removed
      _processAlarms(a.id);

      if (a.repeat) {

        a.time += 86400; // Scheduled again tomorrow

        fileManager.update(_alarmFile, i, (uint8_t *)&a, sizeof(a));
#ifdef DEBUG
        Serial.print("Alarm rescheduled at ");
        //this->printTime(a.time);
        Serial.println();
#endif
      }
      else {
        //     Alarm removed

        fileManager.remove(_alarmFile, i, sizeof(a));
#ifdef DEBUG
        this->dumpAlarms();
#endif
      }

    }
  }
}

#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT

void AMController::sdLogLabels(const char *variable, const char *label1) {

  this->sdLogLabels(variable, label1, NULL, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2) {

  this->sdLogLabels(variable, label1, label2, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3) {

  this->sdLogLabels(variable, label1, label2, label3, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4) {

  this->sdLogLabels(variable, label1, label2, label3, label4, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4, const char *label5) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile)
  {

    if (dataFile.size() > 0) {

#ifdef DEBUG
      Serial.print("No Labels required for "); Serial.println(variable);
#endif
      dataFile.close();
      return;
    }

    dataFile.print("-");
    dataFile.print(";");
    dataFile.print(label1);
    dataFile.print(";");

    if (label2 != NULL)
      dataFile.print(label2);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label3 != NULL)
      dataFile.print(label3);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label4 != NULL)
      dataFile.print(label4);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label5 != NULL)
      dataFile.println(label5);
    else
      dataFile.println("-");

    dataFile.flush();
    dataFile.close();
  } else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}


void AMController::sdLog(const char *variable, unsigned long time, float v1) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);

    dataFile.print(";-;-;-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);

    dataFile.print(";-;-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);

    dataFile.print(";-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);

    dataFile.println(";-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4, float v5) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_APPEND);

  if (dataFile && time > 0)
  {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);
    dataFile.print(";");

    dataFile.println(v5);

    dataFile.println();

    dataFile.flush();
    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening"); Serial.println(variable);
#endif
  }
}

void AMController::sdSendLogData(const char *variable) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer);

  if (dataFile) {

    char c;
    char buffer[128];
    int i = 0;

    dataFile.seek(0);

    while ( dataFile.available() ) {

      c = dataFile.read();

      if (c == '\n') {

        buffer[i++] = '\0';
#ifdef DEBUG
        Serial.println(buffer);
#endif
        this->writeTxtMessage(variable, buffer);

        i = 0;
      }
      else
        buffer[i++] = c;
    }

#ifdef DEBUG
    Serial.println("All data sent");
#endif

    dataFile.close();
  }
  else {
#ifdef DEBUG
    Serial.print("Error opening "); Serial.println(variable);
#endif
  }

  this->writeTxtMessage(variable, "");
}

// Size in bytes
uint16_t AMController::sdFileSize(const char *variable) {

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer, FILE_READ);

  if (dataFile) {

#ifdef DEBUG
    Serial.print("Size of "); Serial.print(variable); Serial.print(" :"); Serial.println(dataFile.size());
#endif

    return dataFile.size();
  }

  return -1;
}

void AMController::sdPurgeLogData(const char *variable) {

  noInterrupts();

  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  SD.remove(fileNameBuffer);

  interrupts();
}

#endif

////////////////////////////////////////////////////

void AMController::notifyConnected() {

  _connectionChanged = true;
  _connected = true;
}

void AMController::notifyDisconnected() {

  _connectionChanged = true;
  _connected = false;
}


void AMController::dataAvailable(String data) {
  strcat(_remainBuffer, data.c_str());
  _dataAvailable = true;
}
