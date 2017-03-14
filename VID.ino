#include <DueTimer.h>

const byte A_PHASE = 2;
const byte B_PHASE = 1;
const byte C_PHASE = 0;

const byte A_PHASE_PIN = 11;
const byte B_PHASE_PIN = 12;
const byte C_PHASE_PIN = 13;
const byte ROTOR_SENSOR_PIN = 30;
const byte M_PIN = A0;

byte phase;
int phasePeriod;

int u;

int M;
int valueMpin;

double w;
int firstRotorSensorTime, lastRotorSensorTime;

void nextPhase(){  
  disableAllPhasePin();
  phase++;
  switch(phase){
    case A_PHASE:
      analogWrite(A_PHASE_PIN, u);
      break;
    case B_PHASE:
      analogWrite(B_PHASE_PIN, u);
      break;
    case C_PHASE:
      analogWrite(C_PHASE_PIN, u);
      break;
    default:
      phase = -1;
      break;  
  }
  
  Timer4.setPeriod(phasePeriod).start();
}

void disableAllPhasePin(){
  analogWrite(A_PHASE_PIN, 0);
  analogWrite(B_PHASE_PIN, 0);
  analogWrite(C_PHASE_PIN, 0);
}

void rotorSensorInterrupt(){
  lastRotorSensorTime = millis();
  int rotorSensorPeriod = lastRotorSensorTime - firstRotorSensorTime;
  Serial.println(rotorSensorPeriod);
  firstRotorSensorTime = lastRotorSensorTime;
}

void setup() {  
  Serial.begin(9600);
  
  pinMode(A_PHASE_PIN, OUTPUT);
  pinMode(B_PHASE_PIN, OUTPUT);
  pinMode(C_PHASE_PIN, OUTPUT);

  phase = A_PHASE;
  u = 255;
  phasePeriod = 50000;
  Timer4.attachInterrupt(nextPhase).start(phasePeriod);

  firstRotorSensorTime = millis();
  attachInterrupt(ROTOR_SENSOR_PIN, rotorSensorInterrupt, FALLING);
}

void loop() {
  valueMpin = analogRead(M_PIN);
  phasePeriod = map(valueMpin, 0, 1023, 500, 50000);
}
