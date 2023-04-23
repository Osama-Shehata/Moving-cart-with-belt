/*
 * The purpose of this code is to count the ouput pulses or 
 * the encoder outputs as you rotate the Motor shaft. You can run the 
 * same code on the Arduino Uno, Arduino Nano, Arduino Mega, etc.
 */
#define Encoder_output_A 3 // pin2 of the Arduino
#define Encoder_output_B 2 // pin 3 of the Arduino
#define PWM 7 // pin 7 of the Arduino
#define IN1 5 // pin 6 of the Arduino
#define IN2 6 // pin 5 of the Arduino

#include "TimerOne.h"

// these two pins has the hardware interrupts as well. 
int b;
int Count_pulses = 0;

void setup() {
Serial.begin(9600); // activates the serial communication
pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input
pinMode(Encoder_output_B,INPUT); // sets the Encoder_output_B pin as the input

attachInterrupt(digitalPinToInterrupt(Encoder_output_A),DC_Motor_Encoder,RISING);

Timer1.initialize(1000000);         // initialize timer1, and set a 1/2 second period
Timer1.attachInterrupt(dataAqu);  // attaches callback() as a timer overflow interrupt
}

void loop() {
//  int dir = -1;
//  int pwr = 255;
//  setMotor(dir,pwr,PWM,IN1,IN2);
//  Serial.print("Encoder_output_A = ");
//  Serial.println(digitalRead(Encoder_output_A));
//  Serial.print("Encoder_output_B = ");
//  Serial.println(digitalRead(Encoder_output_B));
//  

//   Set the motor speed and direction
  int dir = 1;
  int pwr = 255;
  setMotor(dir,pwr,PWM,IN1,IN2);
  Serial.print("Count_pulses = ");
  Serial.println(Count_pulses);
  Serial.print("Position_cm = ");
  Serial.println(Count_pulses*0.0248982912937347);
  Serial.print("\n");
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void dataAqu()
{
  Serial.println(Count_pulses);
  //Serial.write(13);
  //Serial.write(10);
}

void DC_Motor_Encoder(){
  b = digitalRead(Encoder_output_B);
  if(b > 0){
    Count_pulses++;
  }
  else{
    Count_pulses--;
  }
}
