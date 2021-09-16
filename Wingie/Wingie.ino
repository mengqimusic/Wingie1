#include <I2Cdev.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "AC101.h"
#include "TCA6424A.h"
#include <Wire.h>
#include "Wingie.h"
#include "WiFi.h"

#define BASE_NOTE 48
#define SDA1 21
#define SCL1 22

#define MODE_NUM 3 // +1
#define BAR_MODE 0
#define INT_MODE 1
#define REQ_MODE 2
#define POLY_MODE 3

#define POLY_MODE_NOTE_ADD_L 12
#define POLY_MODE_NOTE_ADD_R 24

Wingie dsp(44100, 32);
AC101 ac;
TCA6424A tca;

int volume = 0;
const int lOctPin[2] = {13, 14};
const int rOctPin[2] = {23, 19};
const int routePin[2] = {4, 5}, sourcePin = 18, interruptPin = 15;
const int potPin[3] = {34, 36, 39};
const int sources[2] = {0x2020, 0x0408}; // MIC, LINE
const float inputGainFactor[2] = {2., 1.};

volatile bool keyChanged = 0;
bool source, key[2][12], keyPrev[2][12], firstPress[2] = {true, true};
bool routeButtonPressed[2], routeChanging[2] = {false, false}, sourceChanged = false, sourceChanged2 = false, volumeRaise = false;
bool muteStatus[2][9];
int note[2], octPrev[2], route[2] = {0, 0}, allKeys[2] = {0, 0}, currentPoly[2] = {0, 0};
byte routeButtonBuffer[2] = {255, 255};

// for Tap Sequencer
bool trig[2] = {false, false}, trigged[2] = {false, false}, threshChanged[2] = {false, false};
int seq[2][12], seqLen[2] = {0, 0}, playHeadPos[2] = {0, 0}, writeHeadPos[2] = {0, 0};

unsigned long currentMillis, tcaReadMillis = 0, sourceChangedMillis = 0, startupMillis = 0;

bool startup = true; // startup

void setup() {
  Wire.begin(SDA1, SCL1, 400000);
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_NULL);
  btStop();

  for (int i = 0; i < 2; i++) {
    pinMode(lOctPin[i], INPUT);
    pinMode(rOctPin[i], INPUT);
    pinMode(routePin[i], INPUT);
  }
  pinMode(sourcePin, INPUT);
  pinMode(interruptPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPin), keyChange, FALLING);

  // initialize device
  Serial.println("Initializing TCA6424A...");
  tca.initialize();

  // verify connection
  Serial.println("TCA6424A : Connection Testing...");

  if (tca.testConnection()) {
    Serial.println("TCA6424A : Connection Successful!");
    tca.setAllPolarity(0, 0, 0);
  }
  else
    Serial.println("TCA6424A : Connection Failed :(");

  Serial.println("AC101 : Connecting...");

  while (not ac.begin()) {
    Serial.println("AC101 : Failed! Trying...");
    delay(100);
  }
  Serial.println("AC101 : Connection OK!");

  ac.SetVolumeHeadphone(volume);
  ac.SetVolumeSpeaker(0);

  source = !digitalRead(sourcePin);
  acWriteReg(ADC_SRC, sources[source]);
  dsp.setParamValue("/Wingie/input_gain_factor", inputGainFactor[source]);

  //ac.DumpRegisters();

  dsp.start();
  dsp.setParamValue("resonator_input_gain", 0.1);
  dsp.setParamValue("resonator_output_gain", 0.4);
  dsp.setParamValue("route0", 0);
  dsp.setParamValue("route1", 0);

  int oct[2];
  oct[0] = -!digitalRead(lOctPin[0]) + !digitalRead(lOctPin[1]);
  oct[1] = -!digitalRead(rOctPin[0]) + !digitalRead(rOctPin[1]);

  uint32_t b = 0;
  for (int i = 0; i < 3; i++) b |= tca.readBank(i) << (i * 8);

  for (int kb = 0; kb < 2; kb++) {
    for (int i = 0; i < 12; i++) {
      key[kb][i] = b >> (i * 2 + kb) & B00000001;
      keyPrev[kb][i] = key[kb][i];
    }
  }

  dsp.setParamValue("note0", BASE_NOTE + oct[0] * 12);
  dsp.setParamValue("note1", BASE_NOTE + oct[1] * 12 + 12);
  dsp.setParamValue("left_threshold", 0.4165);
  dsp.setParamValue("right_threshold", 0.4165);

  dsp.setParamValue("/Wingie/left/poly_note_0", 0 + BASE_NOTE + POLY_MODE_NOTE_ADD_L);
  dsp.setParamValue("/Wingie/left/poly_note_1", 4 + BASE_NOTE + POLY_MODE_NOTE_ADD_L);
  dsp.setParamValue("/Wingie/left/poly_note_2", 7 + BASE_NOTE + POLY_MODE_NOTE_ADD_L);
  dsp.setParamValue("/Wingie/right/poly_note_0", 0 + BASE_NOTE + POLY_MODE_NOTE_ADD_R);
  dsp.setParamValue("/Wingie/right/poly_note_1", 4 + BASE_NOTE + POLY_MODE_NOTE_ADD_R);
  dsp.setParamValue("/Wingie/right/poly_note_2", 7 + BASE_NOTE + POLY_MODE_NOTE_ADD_R);
}

void loop() {
  interrupts();
  currentMillis = millis();


  //
  // startup
  //
  if (startup) {
    if (currentMillis - startupMillis > 50) {
      startupMillis = currentMillis;
      if (volume < 63) {
        volume += 1;
        ac.SetVolumeHeadphone(volume);
      }
      else startup = false;
    }
  }


  //
  // Interface Reading
  //
  float Mix = (1. - analogRead(potPin[0]) / 4095.);
  dsp.setParamValue("mix", Mix);

  float Decay = (1. - analogRead(potPin[1]) / 4095.) * 9.9 + 0.1;
  Decay = fscale(0.1, 10., 0.1, 10., Decay, -3.25);
  dsp.setParamValue("/Wingie/left/decay", Decay);
  dsp.setParamValue("/Wingie/right/decay", Decay);

  float Volume = (1. - analogRead(potPin[2]) / 4095.);
  dsp.setParamValue("input_gain", Volume);

  bool tmp = !digitalRead(sourcePin);


  //
  // source change
  //
  if (tmp != source) {
    source = tmp;
    sourceChanged = true;
    ac.SetVolumeHeadphone(0);
    dsp.setParamValue("/Wingie/left/mode_changed", 1);
    dsp.setParamValue("/Wingie/right/mode_changed", 1);
    sourceChangedMillis = currentMillis;
  }

  if (sourceChanged) {
    if (currentMillis - sourceChangedMillis > 5) {
      sourceChanged = false;
      sourceChanged2 = true;
      acWriteReg(ADC_SRC, sources[source]);
      dsp.setParamValue("/Wingie/input_gain_factor", inputGainFactor[source]);
      sourceChangedMillis = currentMillis;
    }
  }

  if (sourceChanged2) {
    if (currentMillis - sourceChangedMillis > 50) {
      sourceChanged2 = false;
      ac.SetVolumeHeadphone(volume);
      dsp.setParamValue("/Wingie/left/mode_changed", 0);
      dsp.setParamValue("/Wingie/right/mode_changed", 0);
    }
  }


  //
  // oct change
  //
  noInterrupts();

  int oct[2];
  oct[0] = -!digitalRead(lOctPin[0]) + !digitalRead(lOctPin[1]);
  oct[1] = -!digitalRead(rOctPin[0]) + !digitalRead(rOctPin[1]);

  for (int i = 0; i < 2; i++) {
    if (octPrev[i] != oct[i]) {
      octPrev[i] = oct[i];
      switch (i) {
        case 0 :
          dsp.setParamValue("note0", note[0] + BASE_NOTE + oct[0] * 12);
          break;
        case 1 :
          dsp.setParamValue("note1", note[1] + BASE_NOTE + oct[1] * 12 + 12);
          break;
      }
    }
  }

  //
  // Route Change
  //
  for (int kb = 0; kb < 2; kb++) {
    for (int i = 0; i < 8; i++) bitWrite(routeButtonBuffer[kb], i, digitalRead(routePin[kb]));

    if (routeButtonBuffer[kb] == 0) routeButtonPressed[kb] = true;
    else if (routeButtonBuffer[kb] == 255) {
      if (routeButtonPressed[kb] && !threshChanged[kb]) routeChanging[kb] = true;
      threshChanged[kb] = false;
      routeButtonPressed[kb] = false;
      if (!kb) dsp.setParamValue("/Wingie/left/mode_changed", 0);
      if (kb) dsp.setParamValue("/Wingie/right/mode_changed", 0);
    }

    if (routeChanging[kb]) {
      routeChanging[kb] = false;
      if (route[kb] < MODE_NUM) route[kb] += 1;
      else route[kb] = 0;
      if (!kb) {
        dsp.setParamValue("route0", route[kb]);
        dsp.setParamValue("/Wingie/left/mode_changed", 1);
      }
      if (kb) {
        dsp.setParamValue("route1", route[kb]);
        dsp.setParamValue("/Wingie/right/mode_changed", 1);
      }

      if (route[kb] != REQ_MODE) {
        for (int i = 0; i < 9; i++) {
          muteStatus[kb][i] = false;
          char buff[100];
          if (!kb) snprintf(buff, sizeof(buff), "/Wingie/left/mute_%d", i);
          else snprintf(buff, sizeof(buff), "/Wingie/right/mute_%d", i);
          const std::string str = buff;
          dsp.setParamValue(str, false);
        }
      }

    }
  }


  //
  // Note Change
  //
  if (keyChanged || currentMillis - tcaReadMillis > 100) {
    keyChanged = 0;
    tcaReadMillis = currentMillis;

    uint32_t b = 0;
    for (int i = 0; i < 3; i++) b |= tca.readBank(i) << (i * 8);

    for (int kb = 0; kb < 2; kb++) {
      for (int i = 0; i < 12; i++) {
        bool tmp = b >> (i * 2 + kb) & B00000001;
        key[kb][i] = tmp;
        bitWrite(allKeys[kb], i, !tmp);

        if (key[kb][i] != keyPrev[kb][i]) {
          if (!key[kb][i]) {

            if (routeButtonPressed[kb]) { // Changle threshold
              threshChanged[kb] = true;
              float thresh = 0.0833 * i + 0.0833;
              if (!kb) {
                dsp.setParamValue("left_threshold", thresh);
              };
              if (kb) {
                dsp.setParamValue("right_threshold", thresh);
              };
            }
            else {
              //if (!kb) dsp.setParamValue("/Wingie/left/mode_changed", 1);
              //if (kb) dsp.setParamValue("/Wingie/right/mode_changed", 1);

              if (firstPress[kb]) {
                firstPress[kb] = false;

                if (route[kb] != POLY_MODE && route[kb] != REQ_MODE) {
                  note[kb] = i;
                  seq[kb][0] = i;
                  seqLen[kb] = 0;
                  playHeadPos[kb] = 0;
                  writeHeadPos[kb] = 0;
                  if (!kb) dsp.setParamValue("note0", note[kb] + BASE_NOTE + oct[kb] * 12);
                  if (kb) dsp.setParamValue("note1", note[kb] + BASE_NOTE + oct[kb] * 12 + 12);
                }
              }
              else { // Not First Press
                if (route[kb] != POLY_MODE && route[kb] != REQ_MODE) {
                  note[kb] = i;
                  writeHeadPos[kb] += 1;
                  seqLen[kb] += 1;
                  seq[kb][writeHeadPos[kb]] = i;
                  if (!kb) dsp.setParamValue("note0", note[kb] + BASE_NOTE + oct[kb] * 12);
                  if (kb) dsp.setParamValue("note1", note[kb] + BASE_NOTE + oct[kb] * 12 + 12);
                }
              }

              if (route[kb] == REQ_MODE) {
                if (i < 4 || i > 6) {
                  int key;
                  if (i > 6) key = i - 3;
                  else key = i;
                  muteStatus[kb][key] = !muteStatus[kb][key];
                  char buff[100];
                  if (!kb) snprintf(buff, sizeof(buff), "/Wingie/left/mute_%d", key);
                  else snprintf(buff, sizeof(buff), "/Wingie/right/mute_%d", key);
                  const std::string str = buff;
                  dsp.setParamValue(str, muteStatus[kb][key]);
                }
              }

              if (route[kb] == POLY_MODE) {
                if (currentPoly[kb] == 0) {
                  currentPoly[kb] = 1;
                  if (!kb) dsp.setParamValue("/Wingie/left/poly_note_0", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_L);
                  if (kb) dsp.setParamValue("/Wingie/right/poly_note_0", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_R);
                }
                else if (currentPoly[kb] == 1) {
                  currentPoly[kb] = 2;
                  if (!kb) dsp.setParamValue("/Wingie/left/poly_note_1", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_L);
                  if (kb) dsp.setParamValue("/Wingie/right/poly_note_1", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_R);
                }
                else if (currentPoly[kb] == 2) {
                  currentPoly[kb] = 0;
                  if (!kb) dsp.setParamValue("/Wingie/left/poly_note_2", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_L);
                  if (kb) dsp.setParamValue("/Wingie/right/poly_note_2", i + BASE_NOTE + oct[kb] * 12 + POLY_MODE_NOTE_ADD_R);
                }
              }

            } // !routeButtonPressed[kb]
          } // Key Press Action End

          else { // Key Release Action Start
            if (!allKeys[kb]) firstPress[kb] = true;
            //if (!kb) dsp.setParamValue("/Wingie/left/mode_changed", 0);
            //if (kb) dsp.setParamValue("/Wingie/right/mode_changed", 0);
          } // Key Release Action End

        }
        keyPrev[kb][i] = key[kb][i];
      }
    }


    //
    // Harmony Mute in Mode 5
    //
    //    if (route[0] == REQ_MODE) {
    //      dsp.setParamValue("/Wingie/left/mute_0", !key[0][0]);
    //      dsp.setParamValue("/Wingie/left/mute_1", !key[0][1]);
    //      dsp.setParamValue("/Wingie/left/mute_2", !key[0][2]);
    //      dsp.setParamValue("/Wingie/left/mute_3", !key[0][3]);
    //      dsp.setParamValue("/Wingie/left/mute_4", !key[0][7]);
    //      dsp.setParamValue("/Wingie/left/mute_5", !key[0][8]);
    //      dsp.setParamValue("/Wingie/left/mute_6", !key[0][9]);
    //      dsp.setParamValue("/Wingie/left/mute_7", !key[0][10]);
    //      dsp.setParamValue("/Wingie/left/mute_8", !key[0][11]);
    //    }
    //    if (route[1] == REQ_MODE) {
    //      dsp.setParamValue("/Wingie/right/mute_0", !key[1][0]);
    //      dsp.setParamValue("/Wingie/right/mute_1", !key[1][1]);
    //      dsp.setParamValue("/Wingie/right/mute_2", !key[1][2]);
    //      dsp.setParamValue("/Wingie/right/mute_3", !key[1][3]);
    //      dsp.setParamValue("/Wingie/right/mute_4", !key[1][7]);
    //      dsp.setParamValue("/Wingie/right/mute_5", !key[1][8]);
    //      dsp.setParamValue("/Wingie/right/mute_6", !key[1][9]);
    //      dsp.setParamValue("/Wingie/right/mute_7", !key[1][10]);
    //      dsp.setParamValue("/Wingie/right/mute_8", !key[1][11]);
    //    }
  }

  //
  // Tap Sequencer
  //
  trig[0] = dsp.getParamValue("/Wingie/left_trig");
  trig[1] = dsp.getParamValue("/Wingie/right_trig");

  for (int kb = 0; kb < 2; kb++) {
    if (seqLen[kb]) {
      if (trig[kb] && !trigged[kb]) {
        trigged[kb] = true;
        if (playHeadPos[kb] < seqLen[kb]) playHeadPos[kb] += 1;
        else playHeadPos[kb] = 0;
        note[kb] = seq[kb][playHeadPos[kb]];
        if (!kb) dsp.setParamValue("note0", note[kb] + BASE_NOTE + oct[kb] * 12);
        if (kb) dsp.setParamValue("note1", note[kb] + BASE_NOTE + oct[kb] * 12 + 12);
        if (!kb) dsp.setParamValue("/Wingie/left/mode_changed", 1);
        if (kb) dsp.setParamValue("/Wingie/right/mode_changed", 1);
      }
    }
    if (!trig[kb] && trigged[kb]) {
      trigged[kb] = false;
      if (!kb) dsp.setParamValue("/Wingie/left/mode_changed", 0);
      if (kb) dsp.setParamValue("/Wingie/right/mode_changed", 0);
    }
  }

}

void keyChange() {
  keyChanged = 1;
}
