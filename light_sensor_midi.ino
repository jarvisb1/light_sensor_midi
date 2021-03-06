#include "MIDIUSB.h"
#include "PitchToNote.h"

//Uncomment the line below to enable the serial port for debugging. Comment it out to disable the serial output. NOTE: MIDI will not output in debugging mode because the serial port interferes with it.
//#define SERIAL_DEBUG (1)

#define LOOP_SLEEP_MS (200) // Milliseconds to sleep/delay at the end of each loop iteration.
#define NUM_SENSORS (4)
#define NUM_POS_ACTIONS (5)
#define NUM_NEG_ACTIONS (4)
#define DEFAULT_PITCH (pitchC3)
#define DEFAULT_VELOCITY (100)

const byte sensor_pins[NUM_SENSORS] = {0, 1, 2, 3};
const int sensor_thresholds[NUM_SENSORS] = {70, 70, 70, 70}; //Used in case the sensors read different values for the same light level

int velocity = DEFAULT_VELOCITY;
byte pitch = DEFAULT_PITCH;
byte channel = 0; //MIDI channel to output on. I'm not sure what happens if you change this.

byte pos_actions[NUM_POS_ACTIONS] = {60, 62, 64, 66, 68};
byte neg_actions[NUM_NEG_ACTIONS] = {48, 46, 44, 42};
byte pos_idx = 0;
byte neg_idx = 0;

int sensor_vals[NUM_SENSORS];
int prev_sensors_active = 0;
int curr_sensors_active = 0;

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
    //pinMode ( sensor_pins[i], INPUT_PULLUP );
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

int num_sensors_active() {
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor_vals[i] < sensor_thresholds[i]) {
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
  curr_sensors_active = num_sensors_active();
#ifdef SERIAL_DEBUG
  Serial.print("Previous num covered: "); Serial.print(prev_sensors_active); Serial.print(". Current num covered: "); Serial.println(curr_sensors_active);
#endif

  if (curr_sensors_active > prev_sensors_active) {
#ifdef SERIAL_DEBUG
    Serial.println("Sending positive action");
#endif

    pitch = pos_actions[pos_idx];
    pos_idx = (pos_idx + 1) % NUM_POS_ACTIONS;
    play_midi_note();
  } 
  else if (curr_sensors_active < prev_sensors_active) {
#ifdef SERIAL_DEBUG
    Serial.println("Sending negative action");
#endif
    pitch = neg_actions[neg_idx];
    neg_idx = (neg_idx + 1) % NUM_NEG_ACTIONS;
    play_midi_note();
  }
  //Note that there is no final else here, because if the number of sensors covered is unchanged we should do nothing

  prev_sensors_active = curr_sensors_active;
  delay(LOOP_SLEEP_MS);
}

