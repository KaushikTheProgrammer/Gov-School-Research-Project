#include <SD.h>
#include <SPI.h>
#include <Audio.h>

void setup() {
  // debug output at 9600 baud
  Serial.begin(9600);

  // setup SD-card
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println(" failed!");
    while(true);
  }
  Serial.println(" done.");
  // hi-speed SPI transfers

  // 44100kHz stereo => 88200 sample rate
  // 100 mSec of prebuffering.
  Audio.begin(88200, 100);
}

void playSound(int sound) {
  // open wave file from sdcard
  
  //please look back at the road
  if (sound==1){
  File myFile = SD.open("test.wav");
  }
  
  //you are distracted. please look back at the road
  else if (sound ==2){
  File myFile = SD.open("test.wav");
  }
  
  //Initiating computer override in 5...4...3...2...1
  else {
  File myFile = SD.open("test.wav");
  }
  
  if (!myFile) {
    // if the file didn't open, print an error and stop
    Serial.println("error opening test.wav");
    while (true);
  }

  const int S = 1024; // Number of samples to read in block
  short buffer[S];

  Serial.print("Playing");
  // until the file is not finished
  while (myFile.available()) {
    // read from the file into buffer
    myFile.read(buffer, sizeof(buffer));

    // Prepare samples
    int volume = 1024;
    Audio.prepare(buffer, S, volume);
    // Feed samples to audio
    Audio.write(buffer, S);

   
  }
  myFile.close();

  
  while (true) ;
}
