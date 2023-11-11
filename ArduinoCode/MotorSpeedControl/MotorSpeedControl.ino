#include <TimerOne.h>

#define PEDAL A0
#define KICKOVERPOINT A1
#define PEDAL_MINIMUM A2
#define SLOWSPEED A3
#define MAXSPEED A4

#define PWM_PIN 9
#define PULSE_PIN 3
#define period 50 // 50 microseconds = 20kHz
int pedalValue=0;
int pedalMinimum =1023;
int kickover = 1023;
int slowspeed = 0;
int maxspeed = 0;
int pwmValue = 0;


void setup() {
  Serial.begin(1000000);
  pinMode(PWM_PIN, OUTPUT);
  Timer1.initialize(period);
  Timer1.pwm(PWM_PIN,0);
  pinMode (PULSE_PIN, INPUT);
  
}

void loop() {

  pedalMinimum = analogRead(PEDAL_MINIMUM);  
  pedalValue = analogRead(PEDAL);
  kickover = analogRead(KICKOVERPOINT);
  slowspeed = analogRead(SLOWSPEED);
  maxspeed = analogRead(MAXSPEED);
   
 
  if (pedalValue<=pedalMinimum){
    //Stop.  Pedal is not pressed.
    pwmValue = 0;
  }
  else if (pedalValue<=kickover){
    pwmValue = slowspeed;    
  }
  else {
    pwmValue = map(pedalValue,pedalMinimum, 1023, slowspeed, maxspeed);
  }
  
  Timer1.pwm(PWM_PIN, pwmValue);

  delay(5);
}
