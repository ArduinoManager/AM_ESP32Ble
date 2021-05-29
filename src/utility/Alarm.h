
#ifndef ALARM_H
#define ALARM_H

class Alarm {

public:
    char   			id[12];
	unsigned long 	time;
	bool			repeat;

  public:

    void setId(char *id) {

      strncpy(this->id, id, 12);
      id[11] = '\0';
    }

    void setTime(unsigned long time) {

      this->time = time;
    }

    unsigned long getTime() {

      return this->time;
    }


    void setRepeat(bool repeat) {

      this->repeat = repeat;
    }

	bool getRepeat() {

      return this->repeat;
    }
    
    void dump(HardwareSerial& serial) {

      serial.print("Id: "); serial.print(this->id);
      serial.print(" time: "); serial.print(this->time);
      serial.print(" Repeat: "); serial.println(this->repeat);
    };
};

#endif
