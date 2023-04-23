#include <TimerOne.h>
#include <PID_v1.h>


#define Limit_Switch_pin 13
#define Encoder_output_A 2 // pin2 of the Arduino
#define Encoder_output_B 3 // pin 3 of the Arduino
#define PWMPin 11
int IN2 = 6;
int IN1 = 5;

int b, a;
double Count_pulses = 0;
double PWM = -255;
double Setpoint = -15;
double Distance = 0, pos_d=0, velocity_i=0;
double pos_d_prev = 0, velocity, velocity2, Distance_prev = 0;
int flag = 0;

double cm_pulses = (0.02918989223);


long prevTA_i, prevTB_i; 
void setup() {
  Serial.begin(115200); // activates the serial communication
    pinMode(Limit_Switch_pin, INPUT_PULLUP);
    pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input
    pinMode(Encoder_output_B,INPUT); 
    pinMode(PWMPin, OUTPUT);   // sets the Encoder_output_B pin as the input
    pinMode(IN1,OUTPUT); // sets the Encoder_output_A pin as the input
    pinMode(IN2,OUTPUT); 
//    attachInterrupt(digitalPinToInterrupt(Encoder_output_A),DC_Motor_EncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_output_A), DC_Motor_EncoderA, CHANGE); 
//    attachInterrupt(digitalPinToInterrupt(Encoder_output_B), DC_Motor_EncoderB, CHANGE); 
    Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(dataAqu); // attaches callback() as a timer overflow interrupt}
  
     //turn the PID on
  
    prevTA_i =micros();
    prevTB_i =micros();
}

void loop() {
  
//  if (digitalRead(Limit_Switch_pin) == LOW && flag == 0){
//    Setpoint = 0;
//    Count_pulses = 0;
//    flag = 1;
//  }
//  if (digitalRead(Limit_Switch_pin) == LOW && flag == 1){
//    Setpoint = 80;
//    flag = 0;
//  }
//  if (Setpoint == 0){
//  Serial.println("Enter your distance.");  
//    while (Serial.available() == 0)
//    { //Wait for user input  
//      }
//    Setpoint = (double)Serial.parseFloat();  
//   }
    
  
  // put your main code here, to run repeatedly:

   
   
  
}

void dataAqu(){
  int puls = 0, vel = 0;
  
  noInterrupts(); // disable interrupts temporarily while reading
  puls = Count_pulses;
  vel = velocity_i;
  interrupts(); 
  
  Distance = puls * cm_pulses;
  velocity = vel * cm_pulses;
  if (Distance < Setpoint && flag == 0){
      PWM = -PWM;
      flag = 1;
    }
    else if (Distance > 0 && flag == 1){
      PWM = -PWM;
      flag = 0;
  }

  setMotor(PWM, PWMPin);

  Serial.print(PWM);
  Serial.print(',');
  Serial.print(velocity);
  Serial.print(',');
  Serial.println(Distance);
  
   
 
}

void DC_Motor_EncoderA() {
  b = digitalRead(Encoder_output_B);

  a = digitalRead(Encoder_output_A);
  int increment = 0;
  if (a == HIGH){
    if(b == LOW){
     increment = 1;
     Count_pulses++;
    }
    else{
      increment = -1;
      Count_pulses--;
    }
  } else {
    if(b == HIGH){
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
    if (pwmVal > 80){
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
    if (pwmVal < -80){
    analogWrite(pwmpin,-pwmVal);
    } else {
      analogWrite(pwmpin,0);
    }
    
  }
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  
}
