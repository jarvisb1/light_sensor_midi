#include "MIDIUSB.h"
#include "PitchToNote.h"

//Uncomment the line below to enable the serial port for debugging. Comment it out to disable the serial output. NOTE: MIDI will not output in debugging mode because the serial port interferes with it.
//#define SERIAL_DEBUG (1)

#define SENSOR_VALUE_THRESHOLD (300) //The phototransistor seems to produce values in the 350+ range when covered up
#define LOOP_SLEEP_MS (200) // Milliseconds to sleep/delay at the end of each loop iteration.
#define NUM_SENSORS (1)
#define NUM_POS_ACTIONS (10)
#define NUM_NEG_ACTIONS (10)
#define DEFAULT_PITCH (pitchC3)
#define DEFAULT_VELOCITY (100)

const byte sensor_pins[NUM_SENSORS] = {0};
int velocity = DEFAULT_VELOCITY;
byte pitch = DEFAULT_PITCH;
byte channel = 0; //MIDI channel to output on. I'm not sure what happens if you change this.

byte pos_actions[NUM_POS_ACTIONS] = {60, 62, 64, 66, 68, 70, 72, 74, 76, 78};
byte neg_actions[NUM_NEG_ACTIONS] = {48, 46, 44, 42, 40, 38, 36, 34, 32, 30};
byte pos_idx = 0;
byte neg_idx = 0;

int sensor_vals[NUM_SENSORS];
int prev_sensors_covered = 0;
int curr_sensors_covered = 0;

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  }
  Serial.println("Sensor debug mode (no MIDI will be sent)"); 
#endif

  //Initialize the sensor values to a known value (0)
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_vals[i] = 0;
  }
}

void read_values() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_vals[i] = analogRead(sensor_pins[i]);
#ifdef SERIAL_DEBUG
    Serial.print("Pin "); Serial.print(sensor_pins[i]); Serial.print(" sensor read: "); Serial.println(sensor_vals[i]);
#endif
  }
}

int num_sensors_covered() {
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor_vals[i] > SENSOR_VALUE_THRESHOLD) {
      count += 1;
    }
  }
  return count;
}

void play_midi_note() {
#ifndef SERIAL_DEBUG //Prevents this following from being executed if in serial debug mode
  midiEventPacket_t noteOn = {0x09, (byte)(0x90 | channel), pitch, (byte)velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();

  delay(50);

  midiEventPacket_t noteOff = {0x08, (byte)(0x80 | channel), pitch, (byte)velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();

#else
  Serial.print("MIDI packet velocity: "); Serial.print((byte)velocity); Serial.print(" , pitch: "); Serial.println((byte)pitch);
#endif
}

void loop() {
  read_values();
  curr_sensors_covered = num_sensors_covered();
#ifdef SERIAL_DEBUG
  Serial.print("Previous num covered: "); Serial.print(prev_sensors_covered); Serial.print(". Current num covered: "); Serial.println(curr_sensors_covered);
#endif

  if (curr_sensors_covered > prev_sensors_covered) {
#ifdef SERIAL_DEBUG
    Serial.println("Sending positive action");
#endif

    pitch = pos_actions[pos_idx];
    pos_idx = (pos_idx + 1) % NUM_POS_ACTIONS;
    play_midi_note();
  } 
  else if (curr_sensors_covered < prev_sensors_covered) {
#ifdef SERIAL_DEBUG
    Serial.println("Sending negative action");
#endif
    pitch = neg_actions[neg_idx];
    neg_idx = (neg_idx + 1) % NUM_NEG_ACTIONS;
    play_midi_note();
  }
  //Note that there is no final else here, because if the number of sensors covered is unchanged we should do nothing

  prev_sensors_covered = curr_sensors_covered;
  delay(LOOP_SLEEP_MS);
}

