#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 RSpeed_msg;
std_msgs::Float32 LSpeed_msg;
ros::Publisher RSpeed_pub("/right_speed", &RSpeed_msg);
ros::Publisher LSpeed_pub("/left_speed", &LSpeed_msg);
/***pin defination******************************************/
// Encoder
#define interruptL 0
#define interruptR 1
#define encoderLB 2
#define encoderRB 3
#define encoderLA 4
#define encoderRA 7
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENA = 5;
const int ENB = 6;

/***argument defination*************************************/
// Encoder
byte preEncoderLA, preEncoderRA;
//int pulseL = 0, pulseR = 0; //1025 for pid controller
double pulseL = 0, pulseR = 0;
double durationR = 0, durationL = 0;
double abs_pulseL, abs_pulseR;
boolean dirL, dirR;

//Motor control
double pwmR = 0;
double pwmL = 0;
int pwmR_buf;
int pwmL_buf;
boolean mutexR=false;
boolean mutexL=false;
int R_dir;  //0 for stop, 1 for forward, 2 for backward
int L_dir;  //0 for stop, 1 for forward, 2 for backward

//PID controller
double val_outputL, val_outputR;
//double Kp_R=0.6, Ki_R=10, Kd_R=0.05;  //1025 Best
//double Kp_L=0.6, Ki_L=10, Kd_L=0.05;  //1025 Best

//double Kp_R=0.8, Ki_R=0.9, Kd_R=0;
//double Kp_L=1.2, Ki_L=1, Kd_L=0;  
double Kp_R=5, Ki_R=0.7, Kd_R=0;
double Kp_L=5, Ki_L=1, Kd_L=0;  
boolean resultR, resultL;
PID pidR(&abs_pulseR, &val_outputR, &pwmR, Kp_R, Ki_R, Kd_R, DIRECT);
PID pidL(&abs_pulseL, &val_outputL, &pwmL, Kp_L, Ki_L, Kd_L, DIRECT);

double current_timeR = 0;
double current_timeL = 0;
/***function declaration***********************************/
void EncoderInit();
void EncoderL();
void EncoderR();
void MotorInit();
void MoveMotor();

/***ROS callback function************************/
void MotorR_Cb( const std_msgs::Int32& msg){
  pwmR_buf=msg.data;
  if(pwmR_buf > 0){
    R_dir = 1;
  }
  if(pwmR_buf < 0){
    R_dir = 2;
    pwmR_buf = abs(pwmR_buf);
  }
  if(pwmR_buf == 0){
    R_dir = 0;
  }
  mutexR = true;
}

void MotorL_Cb( const std_msgs::Int32& msg){
  pwmL_buf=msg.data;
  if(pwmL_buf >= 0){
    L_dir = 1;
  }
  if(pwmL_buf < 0){
    L_dir = 2;
    pwmL_buf = abs(pwmL_buf);
  }
  if(pwmL_buf == 0){
    L_dir = 0;
  }
  mutexL = true;
}

/***ROS parameter************************/
ros::Subscriber<std_msgs::Int32> subR("/MotorR", MotorR_Cb );
ros::Subscriber<std_msgs::Int32> subL("/MotorL", MotorL_Cb );


void setup() {
    nh.initNode();
    nh.subscribe(subR);
    nh.subscribe(subL);
    nh.advertise(RSpeed_pub);
    nh.advertise(LSpeed_pub);
//    Serial.begin(9600);//Initialize the serial port
    EncoderInit();//Initialize the module
    MotorInit();
    pidR.SetMode(AUTOMATIC);
    pidL.SetMode(AUTOMATIC);
    pidR.SetSampleTime(50);
    pidL.SetSampleTime(50);
    delay(100);
}

void loop() {
//    Serial.print("Left pulse = ");
//    Serial.print(val_output);
//    Serial.print(", Right pulse = ");
//    Serial.println(pulseR);
//    pwmR = 100;
//    pwmL = 100;
    if(mutexR && mutexL){
      pwmR = (double)pwmR_buf;
      pwmL = (double)pwmL_buf;
      mutexR = false;
      mutexL = false;
    }
    

    abs_pulseL=abs(pulseL);
    abs_pulseR=abs(pulseR);
    resultL=pidL.Compute();//PID conversion is complete and returns 1
    resultR=pidR.Compute();//PID conversion is complete and returns 1
    if(resultR && resultL){
      MoveMotor();
      abs_pulseL = 0;
      pulseR = 0; //Count clear, wait for the next count
      abs_pulseR = 0;
      pulseL = 0; //Count clear, wait for the next count
    }
    nh.spinOnce();
    int light = analogRead(A0);
    //light_msg.data = light;
    RSpeed_msg.data = (durationR*0.01064)/25;
    LSpeed_msg.data = (durationL*0.01064)/25;
    RSpeed_pub.publish(&RSpeed_msg);
    LSpeed_pub.publish(&LSpeed_msg);
    /*Serial.print("Left pulse = ");
    Serial.print(durationL);
    Serial.print(", Right pulse = ");
    Serial.println(durationR);*/
    durationL = 0;
    durationR = 0;
    delay(40);
}

void EncoderInit(){
    dirL = false;
    dirR = true; //default -> Forward
    pinMode(encoderLA,INPUT);
    pinMode(encoderLB,INPUT);
    pinMode(encoderRA,INPUT);
    pinMode(encoderRB,INPUT);
    attachInterrupt(interruptL, EncoderL, CHANGE);
    attachInterrupt(interruptR, EncoderR, CHANGE);  
}

void EncoderL(){
    int Lstate = digitalRead(encoderLA);
    if((preEncoderLA == LOW) && Lstate==HIGH)
    {
      byte val = digitalRead(encoderLB);
      if(val == LOW && dirL)
      {
        dirL = false; //Reverse
      }
      else if(val == HIGH && !dirL)
      {
        dirL = true;  //Forward
      }
    }
    preEncoderLA = Lstate;
  
    if(!dirL){
      pulseL++;
      durationL--;
    }
    else{
      pulseL++;
      durationL++;
    }
    current_timeL = millis();
}

void EncoderR(){
    int Rstate = digitalRead(encoderRA);
    if((preEncoderLA == LOW) && Rstate==HIGH)
    {
      byte val = digitalRead(encoderRB);
      if(val == LOW && dirR)
      {
        dirR = false; //Reverse
      }
      else if(val == HIGH && !dirR)
      {
        dirR = true;  //Forward
      }
    }
    preEncoderRA = Rstate;
  
    if(!dirR){
      pulseR++;
      durationR++;
    }
    else{
      pulseR++;
      durationR--;
    }
    current_timeR = millis();
}

void MotorInit(){
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT);
    pinMode(ENB,OUTPUT);
}

void MoveMotor(){
    if(R_dir == 1 && L_dir == 1){   //Forward
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);

      analogWrite(ENA,abs(val_outputL));
      analogWrite(ENB,abs(val_outputR));

    }

    if(R_dir == 2 && L_dir == 2){   //Backward
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);

      analogWrite(ENA,abs(val_outputL));
      analogWrite(ENB,abs(val_outputR));
    }

    if(R_dir == 1 && L_dir == 2){   //Left Spin
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);

      analogWrite(ENA,abs(val_outputL));
      analogWrite(ENB,abs(val_outputR));
    }

    if(R_dir == 2 && L_dir == 1){   //Right Spin
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);

      analogWrite(ENA,abs(val_outputL));
      analogWrite(ENB,abs(val_outputR));
    }

    if(R_dir == 0 && L_dir == 0){   //Stop
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,LOW);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);

      analogWrite(ENA,abs(val_outputL));
      analogWrite(ENB,abs(val_outputR));
    }
}
