
//
// Train_2019A_Template
// 2019-04-15 : BDK 
//
#define deltaT_ms 40 // digital control frame rate
//
#define pingTrigPin 4 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 5 // ping sensor echo pin (input to Arduino)
#define motorPin 6  // PWM from motor
//


void setup(void)
{
  //
  // set pin states
  //
  pinMode(pingTrigPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  setTrainMotorPWM(0);
  //
  // Initialize serial communication
  // Use high baud rate to allow rapid execution
  //
  Serial.begin(57600);
  Serial.println("Train Control Running");
}

////////////////////////////////////////////////////////////
// start main loop
////////////////////////////////////////////////////////////
void loop(void)
{
  //
  // wait for fixed frame rate
  //
  static unsigned long millisLast = 0;
  word deltaT_ms_actual;
  while (millis() - millisLast < deltaT_ms) {  }
  deltaT_ms_actual = millis() - millisLast;
  millisLast = millis();
  //Serial.print(deltaT_ms_actual); Serial.print(" ");
  //
  // Get the ping distance to the train ahead
  //
  double d_Ping_cm = 0.0;
  d_Ping_cm = getPingDistance_cm();
  Serial.print(d_Ping_cm); Serial.print(" ");
  //
  // do the control law calculations -- MUST BE IN SUBROUTINE!
  //
  byte train_control_pwm=0;
  train_control_pwm = train_control_law(d_Ping_cm);
  //Serial.print(train_control_pwm); Serial.print(" ");
  //
  // send the PWM command to the motor
  //
  setTrainMotorPWM(train_control_pwm);
  //
  // new line for serial terminal
  //
  Serial.println();
}
////////////////////////////////////////////////////////////
// end main loop
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Ping Sensor Input
////////////////////////////////////////////////////////////
double getPingDistance_cm()
{
  //
  // 3000 us timeout to prevent large delay when no train detected
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // compute distance
  // use the speed of sound (google)
  // and the echo_time measurement
  //
  double d_cm = 0.017*echo_time;
  //
  // return the computed distance
  //
  return d_cm;
}

////////////////////////////////////////////////////////////
// Train Motor Output
////////////////////////////////////////////////////////////
void setTrainMotorPWM(byte pwm_command)
{
  analogWrite(motorPin, pwm_command);
  delay(2); // this was put here for a reason, but I forget
  return;
}
////////////////////////////////////////////////////////////
// end of Train Motor Output
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// train control law
////////////////////////////////////////////////////////////
byte train_control_law(double y_sensor)
{
  byte train_control_pwm;
  double Ki=2, lead=0; //Control parameters
  static double last_time=0.0, err=0.0, errlast=0.0, lead_last=0;
  static double total=0.75, u=0.75, ulast=0.75, yd=7.5, ydlast=7.5; //Warm Start
  double current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
  double delta_time = current_time - last_time; //delta time interval 
  last_time=current_time; //Update last_time
  if (y_sensor <= 5) return 0; //Stop condition
  if (delta_time >= deltaT_ms){    
    yd = (0.001*deltaT_ms*10*ulast + ydlast*0.4)/(0.001*deltaT_ms + 0.4); //Low Pass Filter
    if(yd>10) yd=10; else if (yd<5) yd=5; //Capping the filter
    err = y_sensor-yd; //Error calculation
    total += err; //For the integral control
    if(Ki*total*0.001*deltaT_ms>1) total=1/0.08; else if(total<0) total=0; //Capping integral control
    lead = (3*0.3731*(err-errlast) + 3*(0.001*deltaT_ms) + (0.0268*lead_last))/(0.001*deltaT_ms + 0.0268); //Lead compensator equation
    if(lead>1) lead=1; else if(lead<0) lead=0; //Capping lead compensator
    u = Ki*total*0.001*deltaT_ms + lead;  //Compute PID control
    if(u>1) u=1; else if(u<0) u=0; //Capping total input
    errlast = err; ulast=u; ydlast=yd; lead_last=lead; //Updating the values for next iteration 
  }
  double train_control_percent =  u*100; //multiply to make percent 
  train_control_pwm = constrain(2.55 * train_control_percent,0,255); // scale percent into PWM byte
  return train_control_pwm;
}
////////////////////////////////////////////////////////////
// end of train control law
////////////////////////////////////////////////////////////
