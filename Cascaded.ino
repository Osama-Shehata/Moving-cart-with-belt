#include <TimerOne.h>
#include <PID_v1.h>

#define Limit_Switch_pin 13
#define Encoder_output_A 2 // pin2 of the Arduino
#define Encoder_output_B 3 // pin 3 of the Arduino
#define PWMPin 11
int IN2 = 5;
int IN1 = 6;

int b, a;
double Count_pulses = 0;
double PWM;
double Setpoint = 0;
double Distance = 0, pos_d=0, velocity_i=0;
double pos_d_prev = 0, velocity, Distance_prev=0;

int flag = 0;
/**/
double cm_pulses = (0.02918989223);
/**/
double velocity_read = 0;
/**/

double Kp_pos=5, Ki_pos=0, Kd_pos=0;
double Kp_vel=2.0*0.4*0.1, Ki_vel=450.0*0.4*0.1, Kd_vel=0;
            
//double Kp_pos_low=Kp_pos*0.1, Ki_pos_low=Ki_pos*0.1, Kd_pos_low=0;
//double Kp_vel_low=Kp_vel*0.1, Ki_vel_low=Ki_vel*0.1, Kd_vel_low=0;

/**/
PID PIDPosition(&Distance, &velocity, &Setpoint, Kp_pos, Ki_pos, Kd_pos, DIRECT);
PID PIDVelocity(&velocity_read, &PWM, &velocity, Kp_vel, Ki_vel, Kd_vel, DIRECT);



long prevT_i;
long prevTA_i, prevTB_i;

void setup() {
  Serial.begin(115200); // activates the serial communication
  pinMode(Limit_Switch_pin, INPUT_PULLUP);
    pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input
    pinMode(Encoder_output_B,INPUT); 
    pinMode(PWMPin, OUTPUT);   // sets the Encoder_output_B pin as the input
    pinMode(IN1,OUTPUT); // sets the Encoder_output_A pin as the input
    pinMode(IN2,OUTPUT); 
//    attachInterrupt(digitalPinToInterrupt(Encoder_output_A),DC_Motor_Encoder,RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_output_A), DC_Motor_EncoderA, CHANGE); 

    Timer1.initialize(5000); // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(dataAqu); // attaches callback() as a timer overflow interrupt}
    
    //turn the PID on
    PIDPosition.SetMode(AUTOMATIC);
    PIDPosition.SetOutputLimits(-7, 7);
    PIDVelocity.SetMode(AUTOMATIC);
    PIDVelocity.SetOutputLimits(-255, 255);
    prevT_i =micros();
    prevTA_i =micros();
    prevTB_i =micros();
    PIDPosition.SetSampleTime(4.5);
    PIDVelocity.SetSampleTime(4.5);

}

void loop() {

  if (digitalRead(Limit_Switch_pin) == LOW && flag == 0){
    Setpoint = 0;
    Count_pulses = 0;
    flag = 1;
  }
  if (digitalRead(Limit_Switch_pin) == LOW && flag == 1){
    Setpoint = 1.5;
    flag = 0;
  }

  Serial.print(digitalRead(Limit_Switch_pin));
  Serial.print(',');
 Serial.print(PWM);
 Serial.print(',');
 Serial.print(velocity_read);
 Serial.print(',');
 Serial.println(Distance);

 if (Setpoint == 0){
  Serial.println("Enter your distance.");  
    while (Serial.available() == 0)
    { //Wait for user input  
      }
  Setpoint = (double)Serial.parseFloat();  
   }
  
  
}

void dataAqu(){
  
  int puls = 0, vel = 0;
  
  noInterrupts(); // disable interrupts temporarily while reading
  puls = Count_pulses;
  vel = velocity_i;
  interrupts(); 
  
  Distance = puls * cm_pulses;
  velocity_read = vel * cm_pulses * (((Distance-Distance_prev) == 0) ? 0: 1);
  
  PIDVelocity.SetTunings(Kp_vel, abs(PWM) == 255 ? 0: Ki_vel, Kd_vel);
  PIDPosition.Compute();
  PIDVelocity.Compute();
  setMotor(PWM, PWMPin);
  
  
  Distance_prev = Distance;
  
 
}

void DC_Motor_Encoder() {
  b=digitalRead(Encoder_output_B);
  int increment = 0;
  if(b>0){
   increment = 1;
   Count_pulses++;
  }
  else{
    increment = -1;
    Count_pulses--;
  }
  pos_d = pos_d + increment;
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/(1.0e6);
  velocity_i = increment/deltaT;
  prevT_i = currT;

  
}
void DC_Motor_EncoderA() {
  b = digitalRead(Encoder_output_B);

  a = digitalRead(Encoder_output_A);
  int increment = 0;
  if (a == LOW){
    if(b == HIGH){
     increment = 1;
     Count_pulses++;
    }
    else{
      increment = -1;
      Count_pulses--;
    }
  } else {
    if(b == LOW){
     increment = 1;
     Count_pulses++;
    }
    else{
      increment = -1;
      Count_pulses--;
    }
  }
  
  pos_d = pos_d + increment;
  long currT = micros();
  float deltaT = ((float) (currT - prevTA_i))/(1.0e6);
  velocity_i = increment/deltaT;
  prevTA_i = currT;  
}

void DC_Motor_EncoderB() {
  a = digitalRead(Encoder_output_A);

  b = digitalRead(Encoder_output_B);
  
  int increment = 0;
  if (b == HIGH){
    if(a == HIGH){
     increment = 1;
     Count_pulses++;
    }
    else{
      increment = -1;
      Count_pulses--;
    }
  } else {
    if(a == LOW){
     increment = 1;
     Count_pulses++;
    }
    else{
      increment = -1;
      Count_pulses--;
    }
  }
  
  pos_d = pos_d + increment;
  long currT = micros();
  float deltaT = ((float) (currT - prevTB_i))/(1.0e6);
  velocity_i = increment/deltaT;
  prevTB_i = currT;

}

void setMotor(double pwmVal, int pwmpin){

 
  
  if(pwmVal > 0){
    if (pwmVal > 70){
      analogWrite(pwmpin,pwmVal);
    } else{
      analogWrite(pwmpin,0);
    }
    IN2 = 6;
    IN1 = 5; 
    
  }
  else{
    IN2 = 5;
    IN1 = 6; 
    if (pwmVal < -70){
    analogWrite(pwmpin,-pwmVal);
    } else {
      analogWrite(pwmpin,0);
    }
    
  }
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  
}
