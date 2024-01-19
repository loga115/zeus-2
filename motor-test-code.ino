
/* PID
 *  P: fix the error 
 *  I: integration of error, correct based on previous and current error
 *  smoothen the error correction, i needs to be corrected based on the condition of the value and ki needs to change based on bot frame
 *  D: rate of change of P&I, kd corrects for p&i
 *  p: speed+ accuracy- 
 *  i: integral used to account for p accuracy
 *  
 *  use the arduino time inputs
 *  
 */

/* PINS
 *  pwm pins: 3,5,6,9,10,11
      pwm input pins: 10,11
      direction pins : 
      Left: A0, A1
      Right: A3, A4
      standby pins: A2
      sensor pins:
      from left to right, d12, d9, d8, d7, d6, d5, d4, d3, d2
      sensor emitter pin: d2  
  
*/  
#include <QTRSensors.h>
QTRSensors qtr;

   // pwm 5,6 have lower freq so dont use them for pwm
int lpins[] = {A0, A1};
int rpins[] = {A3, A4};
int lp = 10, rp = 11;
uint8_t ir1 = 12, ir2 = 9, ir3 = 8, ir4 = 7, ir5 = 6, ir6 = 5, ir7 = 4, ir8 = 3;
int emitter=2;
int stby = A2;
int kp = 0, ki = 0, kd = 0;
uint8_t pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
int an_ir[8];
int dig_ir[8];  
int pushbutton_pin = A5;
int lpwm = 0;
int rpwm = 0;
uint16_t sensorData[8]; // qtr sensors documentation says readCalibrated returns data in uint16_t


void calib() {
  motorForward(lpins,lp, HIGH);
  motorReverse(rpins,rp, HIGH); // spin
  for (int i=0; i<100; i++) {
    qtr.calibrate();
    delay(40);
  }
  motorStop(lpins, lp, 10);
  motorStop(rpins, rp, 10);
}



void setup() {
  
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
   pinMode(3,OUTPUT);
   pinMode(A0,OUTPUT);
   pinMode(A1,OUTPUT);
   qtr.setTypeRC();
   qtr.setSensorPins(pins, 8);
   qtr.setEmitterPin((uint8_t) emitter);
   calib();
}
// stop
// qtr library

void motorStop(int *pins, int pwm_pin) {
  digitalWrite(pins[0], LOW);
  digitalWrite(pins[1], LOW);
  analogWrite(pwm_pin, 10);
}

void motorForward(int *pins, int pwm_pin, int pwm_value) {
  digitalWrite(pins[0], HIGH);
  digitalWrite(pins[1], LOW);
  analogWrite(pwm_pin, pwm_value);
}

void motorReverse(int *pins, int pwm_pin, int pwm_value) {
  digitalWrite(pins[0], LOW); 
  digitalWrite(pins[1], HIGH);
  digitalWrite(pwm_pin, pwm_value);
}


void writeMotors(int pwm) {
  motorForward(lpins,lp, pwm);
  motorForward(rpins, rp, pwm);
}

float getPID(float error)
{
  static float reset,prev_error;
  static unsigned long int prev_time;
  //Error is the difference between the postion of the bot and the position we want it to be
  unsigned long current_time=millis();
  double del_time=current_time-prev_time;                   //Steady state error
  reset += (error-prev_error)*del_time;                     //Reset-The small errors that get accumulated over time *reset gets added over time , hence global variable
  float rate_error= (error-prev_error)/del_time;    
  //float rate_error = error-prev_error;//Rate of change of error
  float pid=kp*(error) + ki*(reset) + kd*(rate_error);      //Calculate PID value
  
  if(pid!=0){
  //Serial.print(error);
  //Serial.print(" ");
  //Serial.print(rate_error);
  //Serial.print(" ");
  //Serial.print(pid);
  //Serial.print("\n");
  }
  prev_error=error;
  prev_time=current_time;

  return pid;
}



void loop() {
  digitalWrite(stby, HIGH);
  // put your main code here, to run repeatedly:
  motorForward(lpins, lp, HIGH);
  delay(500);
  motorStop(lpins, lp);
  delay(500);
  motorReverse(lpins, lp, HIGH);
  delay(500);
  motorStop(lpins, lp);
  delay(500);
  motorForward(rpins, rp, HIGH);
  delay(500);
  motorStop(rpins, rp);
  delay(500);
  motorReverse(rpins, rp, HIGH);
  delay(500);
  motorStop(rpins, rp);
  delay(500);
  
}