#include <TimerOne.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
MPU6050 mpu(Wire);
Servo servo ;
#define water 49

#define ENCA1 18
#define ENCA2 19
#define ENCB1 2
#define ENCB2 3

#define PWMA 4
#define IN1A 5
#define IN2A 6

#define PWMB 44
#define IN1B 46
#define IN2B 45


#define PWMAD 13
#define IN1AD 12
#define IN2AD 10

#define PWMBD 8
#define IN1BD 7
#define IN2BD 9


// ECHO pin, needs to be a pin that supports interrupts!
#define ULTRASONIC_PIN_INPUT 22     //A
#define ULTRASONIC_PIN_INPUT_2 23   //B
#define ULTRASONIC_PIN_INPUT_AD 53  //AD
#define ULTRASONIC_PIN_INPUT_BD 15  //BD



// TRIG pin, can be any output pin
#define ULTRASONIC_PIN_OUTPUT 33     //A
#define ULTRASONIC_PIN_OUTPUT_2 34   //B
#define ULTRASONIC_PIN_OUTPUT_AD 52  //AD
#define ULTRASONIC_PIN_OUTPUT_BD 16  //BD


// update interval, make sure to keep it above 20ms
#define ULTRASONIC_TIMER_US 50000

bool flag_pwm = false;
bool flag_heading = false;

int desired_heading;
int current_heading;
int heading_threshold = 10;
int heading_error;

int counterZiad2 = 0;
int counterZiad = 0;
// globals
long prevT = 0;
long prevT2 = 0;
int posPrev = 0;
int posPrev2 = 0;

volatile int pos_i = 0;
volatile int pos_y = 0;
volatile float velocity_i = 0;
volatile float velocity_y = 0;
volatile long prevT_i = 0;
volatile long prevT_y = 0;

float eprev=0;
float yprev =0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;


float eintegral = 0;
float yintegral = 0;
float ederivative =0;
float yderivative =0;

int counter=0;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  
  servo.attach(36); 
  pinMode(water, OUTPUT);

  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  pinMode(PWMAD, OUTPUT);
  pinMode(IN1AD, OUTPUT);
  pinMode(IN2AD, OUTPUT);
  pinMode(PWMBD, OUTPUT);
  pinMode(IN1BD, OUTPUT);
  pinMode(IN2BD, OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);
  digitalWrite(water,LOW);
  servo.write(90);
  pinMode(ULTRASONIC_PIN_INPUT, INPUT_PULLUP);
  pinMode(ULTRASONIC_PIN_OUTPUT, OUTPUT);
  pinMode(ULTRASONIC_PIN_INPUT_2, INPUT_PULLUP);
  pinMode(ULTRASONIC_PIN_OUTPUT_2, OUTPUT);
  Wire.begin();
  byte status = mpu.begin();
  delay(500);
  mpu.calcOffsets();  // gyro and accelero
  digitalWrite(water,LOW);
    servo.write(55);
    delay(200);
    servo.write(130);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1),readEncoder2,RISING);
}

long duration;
long distance;
long duration_2;
long distance_2;
bool flag_start = false;
int avoids=4;

void loop() {
  // read the position and velocity
  int pos = 0;
  int pos2 = 0;
  //float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  pos2 = pos_y;
  //velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float deltaT2 = ((float) (currT-prevT2))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  float velocity2 = (pos2 - posPrev2)/deltaT2;
  posPrev = pos;
  prevT = currT;
  posPrev2 = pos2;
  prevT2 = currT;

  // Convert count/s to RPM
  float v1 = velocity1/400.0*60.0;
  float v2 = velocity2/400.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 80;
  float vt2 = 80;

  // Compute the control signal u
  float kp = 1;
  float ki = 0;
  float kd =0 ;

  float kp2 = 1;
  float ki2 = 0;
  float kd2 =0;
  
  float e = vt-v1Filt;
  float y = vt2-v2Filt;

  eintegral = eintegral + e*deltaT;
  yintegral = yintegral + y*deltaT2;

  
  ederivative = (e-eprev)/(deltaT);
  yderivative = (y-yprev)/(deltaT2);
  
  float u = kp*e + ki*eintegral+kd*ederivative;
  float z = kp2*y + ki2*yintegral+kd2*yderivative;

  // Set the motor speed and direction
  if (counterZiad<=5000 && counterZiad2 <=5000)
  {
  int dir = 1;
  if (u<0)
  {
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255)
  {
    pwr = 255;
  }
 //setMotor(dir,pwr,PWMA,IN1A,IN2A);
  setMotor(dir,pwr,PWMA,IN1A,IN2A);
  setMotorB(dir2,pwr2,PWMB,IN1B,IN2B);
  int dir2 = 1;
   if (z<0)
   {
    dir2 = -1;
   }
  int pwr2 = (int) fabs(z);
  if(pwr2 > 255)
  {
    pwr2 = 255;
  }
  //setMotor(dir,pwr,PWMA,IN1A,IN2A);
  //setMotorB(dir2,pwr2,PWMB,IN1B,IN2B);
  
  }

  else
  {
    stopmotor ();
  
  }
 eprev = e;

////////////////////////////////////////
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(vt2);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.print(y);
  Serial.println();
  counter++;
  char c;
  if (Serial.available() > 0) {
    if (flag_heading == false) {
      mpu.update();
      desired_heading = mpu.getAngleZ();
      flag_heading = true;
    }
    c = Serial.read();
    //c='1';
    switch (c) {
      case '1':

      Serial.println("ana da5lt");
        if (flag_start == false) {
          startmotor();
          flag_start = true;
        }
        if (counter < 40) {
          //noInterrupts();
          ultrasonic1();
          delay(50);
          ultrasonic2();
          //interrupts(); 

          if (distance_2 == 0 || distance == 0 ) {
            //flag_pwm = false;
                avoids=4;
                startmotor();                            
              if (avoids>3){
                startmotor();
                calculateHeadingError();
                correctHeading();
              //flag_pwm = true;
            }
            }
            else if (distance_2 <= 50 || distance <= 50){
            //noInterrupts();
            avoids=0;
            flag_pwm = false;
            stopmotor2();
            ultrasonic1();
            delay(50);
            ultrasonic2();
            
            if(distance_2 <= 50 || distance <= 50){
            for(int i=0;i<=2;i++){
            stopmotor();
            backwards_right();
            stopmotor();
            forwards_left();
            avoids+=1;
              //break;
            }
            }
            //stopmotor();
            //backwards_right();
            //stopmotor();
            //forwards_left();
            //avoids+=1;
            //interrupts(); 
            }
            
          else {
              //flag_pwm = false;
              avoids=4;
              startmotor();
              if (avoids>3){
                startmotor();
                calculateHeadingError();
                correctHeading();
            }
          } 


        }
        
         else if (counter > 40) {
           //stopmotor();
          flag_heading = false;
          if (flag_heading == false) {
            mpu.update();
            desired_heading = mpu.getAngleZ()-90;
            flag_heading = true;
          }
          //noInterrupts();
          ultrasonic1();
          delay(50);
          ultrasonic2();
          //interrupts(); 
          if (distance_2 == 0 || distance == 0 ) {
            flag_pwm = false;
                avoids=4;
              if (avoids>3){
                startmotor();
                calculateHeadingError();
                correctHeading();
              flag_pwm = true;
            }
            }
            else if (distance_2 <= 50 || distance <= 50){
            //noInterrupts();
            avoids=0;
            flag_pwm = false;
            stopmotor();
            backwards_right();
            stopmotor();
            forwards_left();
            avoids+=1;
            //interrupts(); 
            }
          else {
              flag_pwm = false;
              avoids=4;
              if (avoids>3){
                startmotor();
                calculateHeadingError();
                correctHeading();
            }
          } 
          }
        
        break;

      case '2':
            Serial.println("ana da5lt 2");
        break;

      case 'T':
      Serial.println("ana da5lt T");
      servo.write(130);
      //stopmotor2();
      servo.write(55);
        break;

      case 'F':
            Serial.println("ana da5lt f");
            stopmotor2();
      break;
    }
  }
}
void calculateHeadingError() {
  // Read the current heading
  mpu.update();
  //mpu.getAngleZ();
  current_heading = mpu.getAngleZ();

  // Calculate the heading error
  heading_error = current_heading - desired_heading;
  if (heading_error > 180) {
    heading_error -= 360;
  }
  if (heading_error < -180) {
    heading_error += 360;
  }
  //Serial.print("heading_error:");
  //Serial.println(heading_error);
}

void correctHeading() {
  // Turn the vehicle until it is facing the correct
  // direction
  if (heading_error < -heading_threshold) {
    //while (heading_error < -heading_threshold) {
    //goRight();
    correct_heading_GOLEFT();
    Serial.println("GOLEFT");
    delay(4);
    //calculateHeadingError();
    //}
  } else if (heading_error > heading_threshold) {
    //while (heading_error > heading_threshold) {
    //goLeft();
    correct_heading_GORIGHT();
    Serial.println("GORIGHT");
    delay(4);
    //calculateHeadingError();
    //}
  }
}
void ultrasonic1() {

  // Clears the trigPin
  digitalWrite(ULTRASONIC_PIN_OUTPUT, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ULTRASONIC_PIN_OUTPUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN_OUTPUT, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ULTRASONIC_PIN_INPUT, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}
void ultrasonic2() {
  digitalWrite(ULTRASONIC_PIN_OUTPUT_2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ULTRASONIC_PIN_OUTPUT_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN_OUTPUT_2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_2 = pulseIn(ULTRASONIC_PIN_INPUT_2, HIGH);
  // Calculating the distance
  distance_2 = duration_2 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance_2: ");
  Serial.println(distance_2);
}
void startmotor() {

  if (flag_pwm == false) {
    //Serial.println("start motor function");
    analogWrite(PWMA, 255);
    analogWrite(PWMAD, 255);
    analogWrite(PWMB, 255);
    analogWrite(PWMBD, 255);
    digitalWrite(IN1A, HIGH);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN1AD, HIGH);
    digitalWrite(IN2AD, LOW);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, HIGH);
    digitalWrite(IN1BD, LOW);
    digitalWrite(IN2BD, HIGH);
    flag_pwm = true;
    delay(150);
  }

  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, HIGH);
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 255);
  analogWrite(PWMBD, 255);
  //Serial.println("y");
  //delay(1000);
}

void stopmotor() {
  //Serial.println("stop motor function");
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMBD, 0);
  analogWrite(PWMAD, 0);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, HIGH);
  delay(100);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, LOW);
  delay(1000);
}
void stopmotor2() {
  //Serial.println("stop motor function");
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMBD, 0);
  analogWrite(PWMAD, 0);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, HIGH);
  delay(100);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, LOW);
  delay(4000);
}

void turnmotor() {
  //Serial.println("start motor function");
  analogWrite(PWMA, 255);  // Motor speed
  analogWrite(PWMB, 255);  // Motor speed
  analogWrite(PWMBD, 255);
  analogWrite(PWMAD, 255);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, LOW);
  //Serial.println("y");
  delay(100);
  //turnback();
  move_backwards();
}
void turnback() {
  //Serial.println("start motor function");
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 255);  // Motor speed
  analogWrite(PWMBD, 255);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, HIGH);
  delay(2000);
}
void move_backwards() {
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 20);  // Motor speed
  analogWrite(PWMBD, 20);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, LOW);
  delay(100);
}
void backwards_right() {
  //          Serial.println("backwards_right");
  analogWrite(PWMA, 20);
  analogWrite(PWMAD, 20);
  analogWrite(PWMB, 255);  // Motor speed
  analogWrite(PWMBD, 255);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, LOW);
  delay(1000);
}
void forwards_left() {
  //Serial.println("forwards_left");
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 20);  // Motor speed
  analogWrite(PWMBD, 20);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, HIGH);
  delay(1000);
}
void correct_heading_GORIGHT() {
  analogWrite(PWMA, 20);
  analogWrite(PWMAD, 20);
  analogWrite(PWMB, 255);
  analogWrite(PWMBD, 255);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, HIGH);
  delay(500);
}
void correct_heading_GOLEFT() {
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 20);
  analogWrite(PWMBD, 20);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, HIGH);
  delay(500);
}

void backwards_left() {
  //          Serial.println("backwards_right");
  analogWrite(PWMA, 255);
  analogWrite(PWMAD, 255);
  analogWrite(PWMB, 20);  // Motor speed
  analogWrite(PWMBD, 20);
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, HIGH);
  digitalWrite(IN1AD, LOW);
  digitalWrite(IN2AD, HIGH);
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);
  digitalWrite(IN1BD, HIGH);
  digitalWrite(IN2BD, LOW);
  delay(1200);
}
void forwards_right() {
  //Serial.println("forwards_left");
  analogWrite(PWMA, 20);
  analogWrite(PWMAD, 20);
  analogWrite(PWMB, 255);  // Motor speed
  analogWrite(PWMBD, 255);
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
  digitalWrite(IN1AD, HIGH);
  digitalWrite(IN2AD, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, HIGH);
  digitalWrite(IN1BD, LOW);
  digitalWrite(IN2BD, HIGH);
  delay(1000);
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  //noInterrupts();
   analogWrite(pwm,pwmVal); // Motor speed
   analogWrite(PWMAD,pwmVal); // set the motor speed with analogWrite

  if(dir == 1)
  { 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite (IN1AD,HIGH);
    digitalWrite (IN2AD,LOW);
  }
  else if(dir == -1)
  {
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite (IN1AD,LOW);
    digitalWrite (IN2AD,HIGH);
  }
  //interrupts();
}

void setMotorB(int dir, int pwmVal, int pwm, int in1, int in2)
{
  //noInterrupts();
   analogWrite(pwm,pwmVal); // Motor speed
   analogWrite(PWMBD,pwmVal);

  if(dir == 1)
  { 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite (IN1BD,HIGH);
    digitalWrite (IN2BD,LOW);
  }
  else if(dir == -1)
  {
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite (IN1BD,LOW);
    digitalWrite (IN2BD,HIGH);
    
  }
  //interrupts();
}

void readEncoder()
{
  // Read encoder A when ENCA rises
counterZiad +=1;
  int b = digitalRead(ENCA2);
  int increment = 0;
  if(b>0)
  {
    // If B is high, increment forward
    increment = 1;
  }
  else
  {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

}

void readEncoder2()
{
  // Read encoder B when ENCA rises
  counterZiad2 +=1;
  int b = digitalRead(ENCB2);
  int increment = 0;
  if(b>0)
  {
    // If B is high, increment forward
    increment = 1;
  }
  else
  {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_y = pos_y + increment;

}