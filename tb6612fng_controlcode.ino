
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
      direction pins : a0-a3
      standby pins: A4
      sensor pins:
      from left to right, d12, d9, d8, d7, d6, d5, d4, d3, d2
      sensor emitter pin: d2  
  
*/  
   // pwm 5,6 have lower freq so dont use them for pwm
int lpins[] = {A0, A1};
int rpins[] = {A2, A3};
int ir1 = 12;
int ir2 = 9;
int ir3 = 8;
int ir4 = 7;
int ir5 = 6;
int ir6 = 5;
int ir7 = 4;
int ir8 = 3;
int emitter=2;
int stby = A4;
int kp = 0, ki = 0, kd = 0;
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
int an_ir[8];
int dig_ir[8];

void setup() {
  
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
   pinMode(3,OUTPUT);
}
// stop
// qtr library

void motorStop(int *pins, int pwm) {
  digitalWrite(pins[0], LOW);
  digitalWrite(pins[1], LOW);
  analogWrite(pwm, 230);
}

void motorForward(int *pins, int pwm) {
  digitalWrite(pins[0], HIGH);
  digitalWrite(pins[1], LOW);
  analogWrite(pwm, 230);
}

void motorReverse(int in1, int in2, int pwm) {
  digitalWrite(pins[0], LOW); 
  digitalWrite(pins[1], HIGH);
  digitalWrite(pwm, 230);
}


float getPID(float error)
{
  static float reset,prev_error;
  static unsigned long int prev_time;
  //Error is the difference between the postion of the bot and the position we want it to be
  unsigned long current_time=millis();
  double del_time=current_time-prev_time;                   //Steady state error
  reset += (error-prev_error)*del_time;                                 //Reset-The small errors that get accumulated over time *reset gets added over time , hence global variable
  float rate_error= (error-prev_error)/del_time;    
  //float rate_error = error-prev_error;//Rate of change of error
  float pid=kp*(error) + ki*(reset) + kd*(rate_error);     //Calculate PID value
  
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
  delay(200);
  motorForward(r1,r2,lp);
  delay(3000);
  motorStop(r1,r2,rp);
  delay(200);
  motorReverse(r1,r2,rp);
  delay(3000);
  motorStop(r1,r2,rp);
  /*
  delay(200);
  motorForward(l1, l2, lp);
  delay(5000);
  motorStop(l1,l2,lp);
  delay(200);
  motorReverse(l1,l2,lp);
  delay(5000);
  motorStop(l1,l2,lp);
  */
}
