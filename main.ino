/*  Musical Marvin Project Source Code File
 *  Project: Georgia Tech ECE 3872 (Fall 2020) Electronic Sculpture Project
 *  Team: Fall 2020 ECE 3872 Section C Team 1
 *  Team Members: Maya Kelly, Shayna Seidel, Ruben Quiros, Zechuan Ding
 *  Author: Zechuan Ding
 *  Original Creation Date: October 7, 2020
 *  Latest Updated Date: November 5, 2020
 *  IDE: Arduino 1.8.13
 *  Hardware Platform: Arduino Mega 2560
 */
// ============================= Libraries =============================
#include <ServoTimer2.h>
#include <toneAC.h>
#include <Vector.h>
#include <SandTimer.h>
#include <Adafruit_VL53L0X.h>
#include "pitches.h"

#define UP 1500
#define DOWN 1700
#define LEFT 1200
#define MIDLEFT 1400
#define MIDDLE 1500
#define MIDRIGHT 1600
#define RIGHT 1800


// ============================= Pin Numbers =============================
const int resetButton = 19;
const int powerButton = 18;
const int stopButton = 16;
const int recordButton = 17;
const int playRecordButton = 14;
const int playLiveButton = 15;
const int analogPin = A0;


// ============================= Global Objects and Variables =============================
ServoTimer2 servo1;                             // Lower servo
ServoTimer2 servo2;                             // Upper servo
Adafruit_VL53L0X lox = Adafruit_VL53L0X();      // ToF sensor

int storage_array[80];                          // Use to initialize Vector objects
Vector<int> melody(storage_array);

int note_list[] = {NOTE_A2, NOTE_B2, NOTE_C3, NOTE_D3, NOTE_E3, NOTE_F3, NOTE_G3, NOTE_GS3};
//int m[] = { 262, 196, 196, 220, 196, 0, 247, 262 };                               // FOR DEBUG USE
int m[] = {NOTE_GS3, NOTE_A2,NOTE_F3,  NOTE_G3, NOTE_B2, NOTE_C3,NOTE_D3,NOTE_E3};  // FOR DEBUG USE
int thisNote = NOTE_A2;
int volume = 10;
const int interval = 50;
SandTimer my_timer;

enum State_enum {OFF, REST, RECORD, PLAY_RECORD, PLAY_LIVE};
uint8_t state = REST;
uint8_t state_before = OFF;

// ============================= RGBLED Class =============================
class RGBLED{
  public:
    RGBLED(int Rpin, int Gpin, int Bpin){
      // Set pin numbers
      redPin = Rpin;
      greenPin = Gpin;
      bluePin = Bpin;
      LEDwrite(0, 0, 0);
    }
    
    void LEDwrite(int r, int g, int b){
      // Light Common-cathode RGB LED according to input rgb values
      analogWrite(redPin, r);
      analogWrite(greenPin, g);
      analogWrite(bluePin, b);
    }
  private:
    int redPin;
    int greenPin;
    int bluePin;
};

// Create RGBLED objects
RGBLED led1(5, 4, 3);
RGBLED led2(8, 7, 6);


// ============================= servo_set Class =============================
class servo_set{
  /*
   * servo1 ranges from 750 to 2250 for 0 to 180 degree
   * servo2 ranges from 1500 to 2250 for 90 to 180 degree
   * DO NOT WRITE ANY NUMBER BELOW 1500 TO servo2!
   */
  public:
    servo_set(){
      p2i = p2f = UP;
      p1i = p1f = MIDLEFT;
    }
    void head_reset(){
      // Reset position of robot head
      p2i = p2f = UP;
      p1i = p1f = MIDLEFT;
      servo1.write(p1i);        
      servo2.write(p2i);
      delay(200);
    }

    void servo_move(){
      int p2_increment = (p2f - p2i)/100;
      int p1_increment = (p1f - p1i)/100;
      if (!p1_increment && !p2_increment){
        delay(800);
      }
      else{
        for (int i = 1; i <= 100; i++){
            servo1.write(p1f + i * p1_increment);
            servo2.write(p2i + i * p2_increment);
            delay(8);
        }
      }
    }
    
    void head_movement(){
      // Move robot head to certain position according to note pitch
      // Feel free to change values but be careful not to write any number below 1500 to servo2
      if (thisNote == note_list[0]){
        p1f = LEFT;
        p2f = DOWN;
        servo_move();
        p1i = LEFT;
        p2i = DOWN;
      }
      else if (thisNote == note_list[1]){
        p1f = LEFT;
        p2f = UP;
        servo_move();
        p1i = LEFT;
        p2i = UP;
      }
      else if (thisNote == note_list[2]){
        p1f = MIDLEFT;
        p2f = DOWN;
        servo_move();
        p1i = MIDLEFT;
        p2i = DOWN;
      }
      else if (thisNote == note_list[3]){
        p1f = MIDLEFT;
        p2f = UP;
        servo_move();
        p1i = MIDLEFT;
        p2i = UP;
      }
      
      else if (thisNote == note_list[4]){
        p1f = MIDRIGHT;
        p2f = DOWN;    
        servo_move(); //servo2.write(DOWN);
        p1i = MIDRIGHT;
        p2i = DOWN;
      }
      else if (thisNote == note_list[5]){
        p1f = MIDRIGHT;
        p2f = UP;
        servo_move(); //servo2.write(UP);
        p1i = MIDRIGHT;
        p2i = UP;
      }
      else if (thisNote == note_list[6]){
        p1f = RIGHT;
        p2f = DOWN;   
        servo_move(); //servo2.write(DOWN);
        p1i = RIGHT;
        p2i = DOWN;
      }
      else if (thisNote == note_list[7]){
        p1f = RIGHT;
        p2f = UP;    
        servo_move(); //servo2.write(UP);
        p1i = RIGHT;
        p2i = UP;
      }
      
    }

  private:
    int p1i, p1f, p2i, p2f;
    // initial position and final position of two servos per move
};

servo_set head; // Create servo_set object


// ============================= Music Functions =============================

int generateNote(int distance){
  // Generate one note
  static int out_tone;
  if (distance < 2*interval){
    out_tone = note_list[0];        // 0~100
  }
  else if (distance < 3*interval){
    out_tone = note_list[1];        // 100~150
  }
  else if (distance < 4*interval){
    out_tone = note_list[2];        // 150~200
  }
  else if (distance < 5*interval){
    out_tone = note_list[3];        // 200~250
  }
  else if (distance < 6*interval){
    out_tone = note_list[4];        // 250~300
  }
  else if (distance < 7*interval){
    out_tone = note_list[5];        // 300~350
  }
  else if (distance < 8*interval){
    out_tone = note_list[6];        // 350~400
  }
  else{
    out_tone = note_list[7];        // 400~
  }
  return out_tone;
}


// ============================= Function Headers =============================

void state_machine_run();

void resetting(){
  // ISR when reset button is pressed
  if (state != REST){
    state_before = state;
    head.head_reset();
  }
  state = REST;
}


// ============================= setup() =============================
void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  // Setting up ToF sensor
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  
  
  servo1.attach(9);
  servo2.attach(10);
  pinMode(resetButton, INPUT_PULLUP);
  pinMode(powerButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(recordButton, INPUT_PULLUP);
  pinMode(playRecordButton, INPUT_PULLUP);
  pinMode(playLiveButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(resetButton), resetting, FALLING);
  
  do{
   }while(digitalRead(powerButton)); // Do nothing and wait for power button to be pressed
  Serial.println("Hi there!");

  if(!digitalRead(recordButton))
    state_before = RECORD;
  if(!digitalRead(playRecordButton))
    state_before = PLAY_RECORD;
  if(!digitalRead(playLiveButton))
    state_before = PLAY_LIVE;
}


// ============================= loop() =============================
void loop() {
  
  state_machine_run();
 
  delay(10);

}


// ============================= Function Bodies =============================
void state_machine_run() 
{
  VL53L0X_RangingMeasurementData_t measure;
  static int distance = interval;
  static int t1;
  if (state == REST){
    led1.LEDwrite(127, 0, 0);
    led2.LEDwrite(127, 0, 0);

     my_timer.reset();
     my_timer.start(3000);
     while(!digitalRead(resetButton)){
      if (my_timer.finished()){
        melody.clear();
        break;
      }
     }
    
    if(!digitalRead(stopButton)){
      state_before = REST;
    }
    if(!digitalRead(recordButton) && (state_before != RECORD))
      state = RECORD;
    if(!digitalRead(playRecordButton) && (state_before != PLAY_RECORD))
    {
      state = PLAY_RECORD;
    }
    if(!digitalRead(playLiveButton) && (state_before != PLAY_LIVE))
      state = PLAY_LIVE;
  }
  
  else if (state == RECORD){
    led1.LEDwrite(0, 0, 127);
    led2.LEDwrite(0, 0, 127);
    int index = melody.size();
    int up_range = (index < 10) ? 10: 80;
    
    while(index < up_range && state == RECORD){
      Serial.print("Reading a measurement... ");
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
      if (measure.RangeStatus != 4){ // phase failures have incorrect data
        distance = measure.RangeMilliMeter;
        melody.push_back(generateNote(distance));
        
      } else {
        melody.push_back(note_list[7]);
      }
      delay(500);
      led2.LEDwrite(0, 0, 0);
      delay(500);
      led2.LEDwrite(0, 0, 127);
      index++;
      if(!digitalRead(stopButton)){
        state_before = REST;
        state = REST;
        head.head_reset();
      }
    }
    
    if(!digitalRead(stopButton)){
      state_before = REST;
      state = REST;
      head.head_reset();
    }
    if(!digitalRead(playRecordButton))
        state = PLAY_RECORD;
    if(!digitalRead(playLiveButton))
      state = PLAY_LIVE;
  }

  else if (state == PLAY_RECORD){
    led1.LEDwrite(0, 127, 0);
    led2.LEDwrite(0, 127, 0);

    int note_index = 0;
    while (note_index < melody.size() && (state == PLAY_RECORD)) {
      thisNote = melody[note_index];
      volume = round(((float)analogRead(analogPin)/1023) * 10);
      toneAC(thisNote,volume, 980, true); 
      head.head_movement();
      
      delay(200); // Wait the remaining 0.2s while the tone plays in the background
      note_index++;

      if(!digitalRead(stopButton)){
        state_before = REST;
        state = REST;
        head.head_reset();
      }
      if(!digitalRead(recordButton))
        state = RECORD;
      if(!digitalRead(playLiveButton))
        state = PLAY_LIVE;
    }
    
    if(!digitalRead(stopButton)){
      state_before = REST;
      state = REST;
      head.head_reset();
    }
    if(!digitalRead(recordButton))
      state = RECORD;
    if(!digitalRead(playLiveButton))
      state = PLAY_LIVE;
  }

  else if (state == PLAY_LIVE){
    led1.LEDwrite(127, 0, 127);
    led2.LEDwrite(127, 0, 127);

    while (state == PLAY_LIVE){
      //Serial.print("Reading a measurement... ");
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
      if (measure.RangeStatus != 4){ // phase failures have incorrect data
        distance = measure.RangeMilliMeter;
        delay(10);
        distance += measure.RangeMilliMeter;
        delay(10);
        distance += measure.RangeMilliMeter;
        distance = distance / 3; // Averaging
        if (distance < 1000){
          thisNote = generateNote(distance);
          Serial.println(distance);
          // Only play notes and move head when not out of range
          volume = round(((float)analogRead(analogPin)/1023) * 10);
          toneAC(thisNote,volume, 980, true); 
          head.head_movement();
          delay(180); // Wait the remaining 0.1s while the tone plays in the background
        }
      }
      else{
        Serial.println("Out of Range");
        delay(100);
      }
      
      if(!digitalRead(stopButton)){
        state_before = REST;
        state = REST;
        head.head_reset();
      }
      if(!digitalRead(recordButton))
        state = RECORD;
      if(!digitalRead(playRecordButton))
        state = PLAY_RECORD;
    }
  }
  
}
