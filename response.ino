#include <Arduino.h>

#include "TTS.h"
#include "english.h"
#include "sound.h"

#ifndef _TTS_H_
#define _TTS_H_

class TTS {
  public:

    TTS(int pin);

    /**
     * speaks a string of (english) text
     */
    void sayText(const char *text);

    /**
     * speaks a string of phonemes
     */
    void sayPhonemes(const char *phonemes);

    /**
     * sets the pitch; higher values: lower pitch
     */
    void setPitch(byte pitch) { defaultPitch = pitch; }

    /**
     * gets the pitch
     */
    byte getPitch(void) { return defaultPitch; }

  private:
    byte defaultPitch;
    int pin;
};

#endif

//when called, lights up LED
void response1(){


}
//when called, initiates warning
void response2(){
sayText("")
}

//when called, initiates warning

//Arduino Uno sketch-sending serial text to spo-512

#define txpin 13
#define rxpin 11
softwareserial speak (rxpin, txpin);

void setup()  {
speak.begin(9600);     //set serial speed
delay (1000);

speak.printin();   //setup spo-512
cr)
delay(500);
}


void response3(){
// say are you crazy?
speak.printin();("[V14][S4][E2]are you crayzee?");
delay (3000);
}
