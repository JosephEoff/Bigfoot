#include <EEPROM.h>
#include <TimerOne.h>
#include <AutoPID.h>
#include <pidautotuner.h>
//https://github.com/jackw01/arduino-pid-autotuner


#define PEDAL A0
#define KICKOVERPOINT A1
#define PEDAL_MINIMUM A2
#define FIXEDSPEED A3
#define MAXSPEED A4

#define PWM_PIN 9
#define PULSE_PIN 3
#define BUTTON_PIN 4
#define period 50 // 50 microseconds = 20kHz


int pedalValue = 0;
float pedalAverage = 0.0;
int pedalMinimum = 1023;
int kickover = 1023;
int fixedspeed = 0;
int maxspeed = 0;
int pwmValue = 0;
double StitchesPerMinute = 0;
double TargetSPM = 0;
double FixedSPM = 0;
double MaxSPM = 0;
bool stopped = true;
bool startmotor = false;

volatile byte pulseCounter = 0;
volatile unsigned long pulseTime[2];
volatile bool gotPulse = false;

bool stillSpinning = true;

// Sewing machine parameters:
#define STUCKMOTORTIMEOUT_SECONDS 5
#define SPMERROR 5000 //Outrageous number of stitches per minute calculated.
#define HANDWHEEL_DIAMETER 135
#define TACHOWHEEL_DIAMETER 40
// Use the same unit for both measurements.  I used millimeters.
#define PULSESPERROTATION  20  //Number of slots in the tachometer slotted wheel.
#define SLOWESTSPEED 45 //lowest speed possible in stitches per minute.  Applies to the fixed speed knob as well as the variable range.
#define FIXEDSPEED_MAX 200 //maximum stitches per minute for the fixed speed knob
#define FASTESTSPEED 1000 //Stitches per minute.  Absolute maximum speed allowed.
#define PEDALHYSTERESIS 30 //ADC counts difference between the stop position and the sto position of the pedal.
#define DKP 4.94  //Replace with struct and flash storage after autotune
#define DKI 0.38 //Replace with struct and flash storage after autotune
#define DKD 62.75 //Replace with struct and flash storage after autotune
#define OUTPUT_MIN 10 //at 20KHz PWM, the Arduino PWM doesn't work below about 10 - the output just stays low.
#define OUTPUT_MAX 1023
#define PIDLoop_milliseconds 5
const float stitchToRPMRatio = (float)TACHOWHEEL_DIAMETER / (float)HANDWHEEL_DIAMETER;
const float pulseToRPMRatio = 1.0 / (float)PULSESPERROTATION;
unsigned long microsecondsStuckMotorTimeout = STUCKMOTORTIMEOUT_SECONDS * 1000000;
unsigned long microsocondsOfLastPulse = 0;

const String Signature = "Adelheid";

const int EEPROM_Address = 0;

double PWMValue = 0;

AutoPID MotorPID(&StitchesPerMinute, &TargetSPM, &PWMValue, OUTPUT_MIN, OUTPUT_MAX, DKP, DKI, DKD);

int buttonPressCounter = 0;

struct PID_Parameters{
  double KP;
  double KI;
  double KD;
  String Signature;
};

PID_Parameters Parameters = {0,0,0,""};

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  Timer1.initialize(period);
  Timer1.pwm(PWM_PIN, 0);
  pinMode(PULSE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), tachometer_ISR, RISING);
  MotorPID.setTimeStep(PIDLoop_milliseconds);
  //MotorPID.setBangBang(50);
  pinMode(LED_BUILTIN, OUTPUT);
  LoadPIDParametersFromEEPROM();
}

void  tachometer_ISR() {
  if (gotPulse) {
    return;
  }
  pulseTime[pulseCounter] = micros();
  if (pulseCounter == 1) {
    gotPulse = true;
    pulseCounter = 0;
  }
  else {
    pulseCounter += 1;
  }
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressCounter += 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
     buttonPressCounter = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  if (buttonPressCounter > 10) {
    buttonPressCounter = 0;
    DoAutoTune();
  }
  pedalMinimum = analogRead(PEDAL_MINIMUM);
  pedalValue = analogRead(PEDAL);
  kickover = analogRead(KICKOVERPOINT);
  maxspeed = analogRead(MAXSPEED);
  fixedspeed = analogRead(FIXEDSPEED);
  FixedSPM = mapf((float)fixedspeed, 0.0, 1023.0, (float)SLOWESTSPEED, (float)FIXEDSPEED_MAX);
  MaxSPM = mapf((float)maxspeed, 0.0, 1023.0, (float)SLOWESTSPEED, (float)FASTESTSPEED);
  pedalAverage = pedalAverage + (((float)pedalValue - pedalAverage)*0.02);
  
  if ((int)pedalAverage <= (pedalMinimum)) {
    //Stop.  Pedal is not pressed.
    MotorPID.stop();
    MotorPID.reset();
    pwmValue = 0;
    TargetSPM = 0;
    Timer1.pwm(PWM_PIN, pwmValue);
    stopped = true;
    startmotor = false;
    return;
  }
  if ((int)pedalAverage >=  pedalMinimum + PEDALHYSTERESIS){
    startmotor = true;
  }
  if (!startmotor){
    return;
  }
  if ((int)pedalAverage <= kickover) {
    TargetSPM = FixedSPM;
  }
  else {
    TargetSPM = mapf(pedalAverage, (float)kickover, 1023.0, FixedSPM, MaxSPM);
  }
  if (stopped) {
    stopped = false;
    //slap the motor and make it turn.
    SlapMotor();
    microsocondsOfLastPulse = micros();
  }
  UpdateSPM();

  if (stillSpinning) {
    MotorPID.run();
    Timer1.pwm(PWM_PIN, (int)PWMValue);
  }
  else {
    //Motor stuck, shutoff.
    Timer1.pwm(PWM_PIN, 0);
  }
}

void UpdateSPM() {
  if (gotPulse) {
    CalculateStitchesPerMinute();
    gotPulse = false;
    stillSpinning = true;
    microsocondsOfLastPulse = micros();
  }

  //Stuck motor detection
  if (micros() - microsocondsOfLastPulse >= microsecondsStuckMotorTimeout) {
    stillSpinning = false;
  }
}

void SlapMotor(){
    Timer1.pwm(PWM_PIN, 1023);
    delay(15);
}

void CalculateStitchesPerMinute() {
  // Convert pulse time to pulses per second = 1 second / (pulse time * pulses per rotatation)
  // Convert pulses per second to RPM = pulses per second * 60
  // Convert RPM to stitches per minute = RPM * ratio
  float pulsespersecond = (float)(1000000.0 / (pulseTime[1] - pulseTime[0]));
  float pulsesperminute = pulsespersecond * 60.0;
  float RPM = pulsesperminute * pulseToRPMRatio;
  float SPM = RPM * stitchToRPMRatio;
  if (SPM >= SPMERROR) {
    //Outrageous number of stitches per minute calculated.
    //This happens when a slot on the tachometer wobbles back and forth in front of the LED.
    //It happens mostly at very low speed.
    //Ignore outrageous numbers.
    //Slap the motor to get past the edge condition.
    SlapMotor();
    return;
  }
  StitchesPerMinute = SPM;
}

void DoAutoTune() {
  PIDAutotuner tuner = PIDAutotuner();
  long loopInterval = 1000 * PIDLoop_milliseconds;
  tuner.setTargetInputValue(200); //Target 200 stitches per minute
  tuner.setLoopInterval(loopInterval);
  tuner.setOutputRange(0, 1023);
  //PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot, PIDAutotuner::ZNModeNoOvershoot
  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
  tuner.startTuningLoop(micros());

  // Run a loop until tuner.isFinished() returns true
  long microseconds;
  while (!tuner.isFinished()) {

    long prevMicroseconds = microseconds;
    microseconds = micros();
    UpdateSPM();
    double output = tuner.tunePID(StitchesPerMinute, microseconds);
    Timer1.pwm(PWM_PIN, (int)output);
    while (micros() - microseconds < loopInterval) delayMicroseconds(1);
  }
  Timer1.pwm(PWM_PIN,0);

  // Get PID gains - set your PID controller's gains to these
  Parameters.KP = tuner.getKp();
  Parameters.KI = tuner.getKi();
  Parameters.KD = tuner.getKd();
  SavePIDParametersToEEPROM();
  Serial.print("KP: ");
  Serial.println(Parameters.KP);
  Serial.print("KI: ");
  Serial.println(Parameters.KI);
  Serial.print("KD: ");
  Serial.println(Parameters.KD);
 
}

void LoadPIDParametersFromEEPROM(){
  EEPROM.get(EEPROM_Address, Parameters);
  if (Parameters.Signature != Signature){
    //Set default values and save to EEPROM
    Parameters.KP = DKP;
    Parameters.KI = DKI;
    Parameters.KD = DKD;
    SavePIDParametersToEEPROM();
  }
  MotorPID.setGains(Parameters.KP, Parameters.KI, Parameters.KP);
}

void SavePIDParametersToEEPROM(){
  Parameters.Signature = Signature;
  EEPROM.put(EEPROM_Address, Parameters);
  Serial.println("Saved PID parameters.");
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
