/*
   Skeleton Arduino Manager for iPad / iPhone / Mac

   A simple test program to show the Arduino Manager
   features.

   Author: Fabrizio Boco - fabboco@gmail.com

   Version: 1.0

   02/22/2026

   All rights reserved

*/

/*
   AMController libraries, example sketches (The Software) and the related documentation (The Documentation) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under authors copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an AS IS basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <ESP32Servo.h>
#include "AM_ESP32Ble.h"


#if defined(ALARMS_SUPPORT)
AMController amController(&doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &processAlarms, &deviceConnected, &deviceDisconnected);
#else
AMController amController(&doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &deviceConnected, &deviceDisconnected);
#endif


#define SD_SELECT 17


void setup() {

  Serial.begin(115200);
  Serial.println("AMControllerESP32BleExample");
  Serial.println("----------------------------");

  amController.begin("AMManager");


#if (defined(SD_SUPPORT))
  Serial.println("Initializing SD card...");

  delay(1000);

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_SELECT)) {
    Serial.println("Card failed, or not present");
  }
  else {
    Serial.println("Card initialized");
  }
#endif


  /**

     Other initializations

  */


  Serial.println("**** Advertising ****");
  Serial.println("Ready");
}


void loop() {
  amController.loop();
}

/**
  This function is called periodically and its equivalent to the standard loop() function
*/
void doWork() {
}

/**
  This function is called when the ios device connects and needs to initialize the position of switches and knobs
*/
void doSync () {
}

/**
  This function is called when a new message is received from the iOS device
*/
void processIncomingMessages(char *variable, char *value) {
  //Serial.print("in: "); Serial.print(variable); Serial.print(" "); Serial.println(value);
}

/**
  This function is called periodically and messages can be sent to the iOS device
*/
void processOutgoingMessages() {
}

#if defined(ALARMS_SUPPORT)
/**

  This function is called when a Alarm is fired

*/
void processAlarms(char *alarm) {
}
#endif

/**
  This function is called when the iOS device connects
*/
void deviceConnected () {
  Serial.println("* Device connected");
}

/**
  This function is called when the iOS device disconnects
*/
void deviceDisconnected () {
  Serial.println("* Device disconnected");
}

/**
  Additional functions
**/
float getVoltage(int pin) {

  return (analogRead(pin) * 3.3 / 2048.0 * 1);   // converting from a 0 to 2048 digital range into voltage
}