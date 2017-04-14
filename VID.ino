#include <DueTimer.h>
#include <math.h>

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

int Ud;
double T;
double microT;

double M;
int valueMpin;

double w;
int firstRotorSensorTime, lastRotorSensorTime;

const double Lm = 0.000145;
const double Ls = 0.000144;
const double Lr = 0.000146;
const double Tr = 0.00073;
const double Rr = 0.2;
const double Rs = 2.4; 
const double sigma = 0.7;
const double tau = 0.000001;
const double e = 0.1;

double psi;

double I0[2];
double u[2][12];

boolean isServeU;

void nextPhase(){ 
  disableAllPhasePin(); 
  if (isServeU){   
    Timer4.setPeriod(phasePeriod - microT).start();
    isServeU = false;
  }
  else{    
    phase++;
    switch(phase){
      case A_PHASE:
        analogWrite(A_PHASE_PIN, Ud);
        phase = -1;
        break;
      case B_PHASE:
        analogWrite(B_PHASE_PIN, Ud);
        break;
      case C_PHASE:
        analogWrite(C_PHASE_PIN, Ud);
        break;
    }

    isServeU = true;
    Timer4.setPeriod(microT).start();
  }
}

void disableAllPhasePin(){
  analogWrite(A_PHASE_PIN, 0);
  analogWrite(B_PHASE_PIN, 0);
  analogWrite(C_PHASE_PIN, 0);
}

void rotorSensorInterrupt(){
  lastRotorSensorTime = millis();
  double rotorSensorPeriodMilli = lastRotorSensorTime - firstRotorSensorTime;
  double rotorSensorPeriod = rotorSensorPeriodMilli / 1000;
  //w = 2 * M_PI / rotorSensorPeriod;
  firstRotorSensorTime = lastRotorSensorTime;

  w = 1.5;
//  double analogM = analogRead(M_PIN);
//  M = analogM / 1000000;
  M = 0.00001;
  CalculateUdT();
}

void CalculateUdT()
{
  psi = GetEcsponentPsi();

  double nu0[2][2];
  nu0[0][0] = psi;
  nu0[0][1] = psi + e;
  
  double v = -tau * GetDifPsi(psi) / e;
  v = v*(nu0[0][1] - nu0[0][0]);
  
  nu0[1][0] = v + nu0[0][0];
  nu0[1][1] = v + nu0[0][1];  

  I0[0] = min(nu0[0][0],min(nu0[0][1],min(nu0[1][0],nu0[1][1])));
  I0[1] = max(nu0[0][0],max(nu0[0][1],max(nu0[1][0],nu0[1][1])));

  u[0][5] = nu0[1][0];
  u[0][6] = nu0[1][1];
  T = tau;

  SearchLastStep();
  
//  microT = 1000000000 * T;
//  
//  double UdValue = GetUd();
//  Ud = GetAnalogUd(UdValue);
}

double GetEcsponentPsi()
{
  double psi1 = sigma * Lr * Ls * Tr * w + Lm * Lm * Tr * w - sigma * Lr * Ls - Lm * Lm;
  double psi2 = sqrt(-6 * Rr * psi1 * M * Rs * Tr) * Lr;
  return psi2 / (3 * Rr * psi1);
}

double GetDifPsi(double p)
{
  double psi1 = -(Lm * Lm + Lr * Ls * Tr * sigma * (2 + Lm * w)) / (2 * Lr * Ls * sigma * Tr);
  double psi2 = p * (psi1 - Lr * M / (3 * p * p * Rr * Tr));

  double psi3 = Lm * Lm / (Lr * Ls * sigma * Tr) - w + 2 / 3 * Lr * M / (Lm * p * p * Rr * Tr);
  double psi4 = (1 / 4.0 * psi3 * psi3 + Lm * Lm * w / (Lr * Ls * sigma * Tr)) * (p * p);

  double psi5 = 2 / 3.0 * (Lr * M) / Rr * (Rs / (Ls * sigma * Tr) + 1 / (Tr * Tr));

  double psi6 = sqrt(psi4 + psi5);

  return psi2 + psi6;
}

void SearchLastStep()
{
  bool doNextStepByTime;
  do
  {
    doNextStepByTime = false;

    int i = 5;
    double probability = GetCurrentProbability(i, GetDifPsi(psi));
    double sideProbability;
    if (probability > 0)
    {
      i = 6;
      do
      {
          sideProbability = GetUpProbability(i, GetDifPsi(psi + e * (i - 5)));
          i++;
      } while (i < 11 && sideProbability > 0);
      if (i < 11)
      {
          i = 5;
          do
          {
              sideProbability = GetDownProbability(i, GetDifPsi(psi + e * (i - 5)));
              i--;
          } while (i > 0 && sideProbability > 0);
          if (i > 0)
          {
              u[0][5] = u[1][5];
              u[0][6] = u[1][6];
              T += tau;
              doNextStepByTime = true;
          }
      }
    }
  }while (doNextStepByTime);
}

double GetUpProbability(int i, double p)
{
  double v = tau * p / e;

  u[1][i + 1] = (u[0][i] - u[1][i]*(1 - v))/v;
  u[0][i + 1] = (u[0][i] - u[1][i]) / v + u[0][i];

  double maxU = max(u[0][i], max(u[1][i], max(u[0][i + 1], u[1][i + 1])));
  double minU = min(u[0][i], min(u[1][i], min(u[0][i + 1], u[1][i + 1])));

  return GetProbability(minU, maxU);
}

double GetDownProbability(int i, double p)
{
  double v = tau * p / e;

  u[0][i - 1] = (u[1][i] - u[0][i]*(1 - v))/v;
  u[1][i - 1] = (u[1][i]*(1 + v) - u[0][i])/v;

  double maxU = max(u[0][i], max(u[1][i], max(u[0][i - 1], u[1][i - 1])));
  double minU = min(u[0][i], min(u[1][i], min(u[0][i - 1], u[1][i - 1])));

  return GetProbability(minU, maxU);
}

double GetCurrentProbability(int i, double p)
{
  double v = tau * p / e;

  u[1][i] = v * (u[0][i] - u[0][i + 1]) + u[0][i];
  u[1][i + 1] = v * (u[0][i] - u[0][i + 1]) + u[0][i + 1];

  double maxU = max(u[0][i], max(u[1][i], max(u[0][i + 1], u[1][i + 1])));
  double minU = min(u[0][i], min(u[1][i], min(u[0][i + 1], u[1][i + 1])));

  return GetProbability(minU, maxU);
}

double GetProbability(double minU, double maxU)
{
  if (I0[1] <= minU || I0[0] >= maxU) return 0;
  if (I0[1] <= maxU && I0[0] >= minU) return 1;
  if (I0[1] >= maxU && I0[0] >= minU) return (maxU - I0[0]) / (I0[1] - I0[0]);

  if (I0[1] <= maxU && I0[0] <= minU) return (I0[1] - minU) / (I0[1] - I0[0]);
  if (I0[1] >= maxU && minU >= I0[0]) return (maxU - minU) / (I0[1] - I0[0]);

  return 0;
}

double GetId()
{
  double id1 = -Lm * psi / (2 * Lr * Ls * sigma);
  double id2 = Tr * w * psi / 2.0;
  double id3 = -Lr / (3 * Lm * psi) * M / Rr;
  double id4 = Lm * psi / (Lr * Ls * sigma) - Tr * w * psi + 2 / 3 * Lr * M / (Lm * psi * Rr);
  double id5 = 0.25 * id4 * id4;
  double id6 = 2 * Lr * M / (3 * Lm * Lm * Rr) * (Rs * Tr / (Ls * sigma) + 1);
  double id7 = Tr * w * psi * psi / (Lr * Ls * sigma);

  return id1 + id2 + id3 + sqrt(id5 + id6 + id7);
}

double GetUd()
{
  double Id = GetId();

  double u1 = -Lm / (2 * Lr * Ls * sigma);
  double u2 = Tr * w / 2.0;
  double u3 = Lr * M / (3 * Lm * psi * psi * Rr);
  double u4 = Lm * psi / (Lr * Ls * sigma) - Tr * w * psi + 2 * Lr * M / (3 * Lm * psi * Rr);
  double u5 = Lm / (Ls * Lr * sigma) - Tr * w - 2 * Lr * M / (3 * Lm * psi * psi * Rr);
  double u6 = 4 * Tr * w * psi / (Lr * Ls * sigma);
  double u7 = u4 * u5 + u6;
  double u8 = 8 * Lr * M / (3 * Lm * Lm * Rr) * (Rs * Tr / (Ls * sigma) + 1);
  double u9 = 4 * Tr * w * psi * psi / (Lr * Ls * sigma);
  double u10 = 2 * Tr * sqrt(u4 * u4 + u8 + u9);
  double u11 = sigma * Ls * (u1 + u2 + u3 + u7 / u10) * (-psi + Lm * Id) + Rs * Id;
  double u12 = -2 / 3.0 * sigma * Lr * Ls * M / Rr * (w / (Lm * psi) - Id / (Tr * psi * psi));
  double u13 = -Lm / (Lr * Tr) * (psi - Lm * Id);

  return u11 + u12 + u13;
}

int GetAnalogUd(double udValue)
{
  int maxU = 12;
  int maxAnalogU = 255;
  if (udValue > maxU){
    return maxAnalogU;
  }
  return map((int)udValue, 0, maxU, 0, maxAnalogU);
}

void setup() {  
  Serial.begin(9600);
  
  pinMode(A_PHASE_PIN, OUTPUT);
  pinMode(B_PHASE_PIN, OUTPUT);
  pinMode(C_PHASE_PIN, OUTPUT);

  phase = -1;
  Ud = GetAnalogUd(9);
  isServeU = false;
  phasePeriod = 232710;
  microT = 153985;
  Timer4.attachInterrupt(nextPhase).start(phasePeriod);

  firstRotorSensorTime = millis();
  attachInterrupt(ROTOR_SENSOR_PIN, rotorSensorInterrupt, FALLING);
}

void loop() {
}
