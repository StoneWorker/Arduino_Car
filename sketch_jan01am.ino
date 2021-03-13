#include <Servo.h>  //加入含有舵机控制库的头文件

Servo myServo;  //舵机

#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4

int eventNumber =0;

const int interruptButton =  2;
const int leftLed =  11;
const int MidLed =  12;
const int rightLed =  10;
const int superInput =  A1; //超声波控制引脚

const int superOutput =  A0; //超声波控制引脚
const int servoPin = A5 ; //舵机信号控制引脚,需要有PWM数字引脚，或者模拟引脚
const int leftRedLed =  A3;
const int midGreenLed =  A2;
const int rightRedLed =  13;
const int pinBuzzer = 3;
const int leftPWM = 5;
const int rightPWM = 6;
const int leftMotorsInput1 = 7;
const int leftMotorsInput2 = 4;
const int rightMotorsInput1 = 9;
const int rightMotorsInput2 = 8;

const int speedCar = 200;   // Vout=120/255*7.4=3.48V

void setup() {
 // put your setup code here, to run once:
 //串口初始化
 //Serial.begin(9600); 

 pinMode(interruptButton, INPUT);
 pinMode(leftLed, INPUT);
 pinMode(MidLed, INPUT);
 pinMode(rightLed, INPUT);
 pinMode(superInput, INPUT);   //超声波控制引脚初始化
 
 pinMode(superOutput, OUTPUT);   //超声波控制引脚初始化
 pinMode(leftRedLed, OUTPUT);
 pinMode(midGreenLed, OUTPUT);
 pinMode(rightRedLed, OUTPUT);
 pinMode(pinBuzzer, OUTPUT);
 pinMode(leftPWM, OUTPUT);
 pinMode(rightPWM, OUTPUT);
 pinMode(leftMotorsInput1, OUTPUT);
 pinMode(leftMotorsInput2, OUTPUT);
 pinMode(rightMotorsInput1, OUTPUT);
 pinMode(rightMotorsInput2, OUTPUT); 

 attachInterrupt(digitalPinToInterrupt(interruptButton), changeEventNumber, RISING);
 myServo.attach(servoPin);  //舵机初始化  
 myServo.write(80);         //write()函数输出的PWM即为舵机专用的20ms为周期的PWM波,不同的高电平时间给出不同的角度,0.5ms0度,1ms45度,1.5ms90度,2.5ms180度
}

void loop() {
  // put your main code here, to run repeatedly: 
  if(eventNumber==0) {
    allStop();
  }  
  if(eventNumber==1) {
    trace();
  }
  if(eventNumber==2){
    superMove();
  }  
}

void changeEventNumber (){    
  eventNumber++;
  if(eventNumber==3){
    eventNumber=0;
  }
}

void alarm (){  
 for(int i=255;i<256;i++)       //用循环的方式将频率从200HZ/255/700HZ 增加到 260HZ/800HZ
  {
    tone(pinBuzzer,i);          //在3号端口输出频率 
    delay(5);                   //该频率维持5毫秒
  }
}

void motorRun(int cmd,int value)
{
  analogWrite(leftPWM, value);  //设置PWM输出，即设置速度
  analogWrite(rightPWM, value);
  switch(cmd){
     case BACKWARD:
      digitalWrite(leftMotorsInput1, HIGH);
      digitalWrite(leftMotorsInput2, LOW);
      digitalWrite(rightMotorsInput1, HIGH);
      digitalWrite(rightMotorsInput2, LOW);
      break;
     case FORWARD:
      digitalWrite(leftMotorsInput1, LOW);
      digitalWrite(leftMotorsInput2, HIGH);
      digitalWrite(rightMotorsInput1, LOW);
      digitalWrite(rightMotorsInput2, HIGH);
      break;
     case TURNLEFT:
      digitalWrite(leftMotorsInput1, HIGH);
      digitalWrite(leftMotorsInput2, LOW);
      digitalWrite(rightMotorsInput1, LOW);
      digitalWrite(rightMotorsInput2, HIGH);
      break;
     case TURNRIGHT:
      digitalWrite(leftMotorsInput1, LOW);
      digitalWrite(leftMotorsInput2, HIGH);
      digitalWrite(rightMotorsInput1, HIGH);
      digitalWrite(rightMotorsInput2, LOW);
      break;
     default:
      digitalWrite(leftMotorsInput1, LOW);
      digitalWrite(leftMotorsInput2, LOW);
      digitalWrite(rightMotorsInput1, LOW);
      digitalWrite(rightMotorsInput2, LOW);
  }
}

int getDistance()
{
  digitalWrite(superOutput, LOW); // 使发出发出超声波信号接口低电平2μs
  delayMicroseconds(2);
  digitalWrite(superOutput, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
  delayMicroseconds(10);
  digitalWrite(superOutput, LOW); // 保持发出超声波信号接口低电平
  int distance = pulseIn(superInput, HIGH)/58; // 读出脉冲时间,将脉冲时间转化为距离（单位：厘米）
  return distance;
}

void allStop(){
    digitalWrite(rightRedLed, LOW );
    digitalWrite(leftRedLed, LOW);
    digitalWrite(midGreenLed, LOW); 
    noTone(pinBuzzer);
    motorRun(STOP, 0);
}

void trace ()
{
  int data[3];
  data[0] = digitalRead(leftLed);
  data[1] = digitalRead(MidLed);
  data[2] = digitalRead(rightLed);

  if(data[0])
  {
    digitalWrite(leftRedLed, HIGH); 
  }
  else
  {
    digitalWrite(leftRedLed, LOW); 
  }
  if(data[1])
  {
    digitalWrite(midGreenLed, HIGH);    
  }
  else
  {
    digitalWrite(midGreenLed, LOW); 
  }
  if(data[2])
  {
    digitalWrite(rightRedLed, HIGH);  
  }
  else
  {
    digitalWrite(rightRedLed, LOW); 
  }
  
  if(!data[1])
  {
    alarm ();    
  }
  else 
  {
    noTone(pinBuzzer);
  }
  
  if(data[0] && !data[1])  //左边检测到黑线
  {
    motorRun(TURNLEFT, speedCar);
  }
  if(data[2] && !data[1])  //右边检测到黑线
  {
    motorRun(TURNRIGHT, speedCar);
  }
  if(data[1] && !data[0] && !data[2])  //左右都没有检测到黑线
  {
    motorRun(FORWARD, speedCar);
  }
  if(!data[0] && !data[1] && !data[2])  //左中右都没有检测到黑线
  {
    delay(100);
    motorRun(BACKWARD, speedCar);
  }
  if(data[1] && data[0] && data[2])  //左中右都检测到黑线
  {
    motorRun(STOP, 0);
  }
}

void superMove()
{
  int dis[3];           //距离
  dis[1]=getDistance(); //中间距离
  if(dis[1]<30)
  {
    motorRun(STOP,0);
    myServo.write(20);
    delay(300);
    dis[0]=getDistance(); //右边距离
    myServo.write(140);
    delay(600);
    dis[2]=getDistance(); //左边距离    
    myServo.write(80);
    
    if(dis[0]<dis[2] && dis[1]<dis[2]) //右边距离障碍的距离比左边近
    {
      //左转
      digitalWrite(rightRedLed, LOW);
      digitalWrite(leftRedLed, HIGH);
      digitalWrite(midGreenLed, LOW); 
      alarm (); 
      motorRun(TURNLEFT,speedCar);
      delay(600);
    }
    if(dis[2]<dis[0] && dis[1]<dis[0])            //右边距离障碍的距离比左边远
    {
      //右转
      digitalWrite(rightRedLed, HIGH);
      digitalWrite(leftRedLed, LOW);
      digitalWrite(midGreenLed, LOW); 
      alarm (); 
      motorRun(TURNRIGHT,speedCar);
      delay(600);
    }
    if(dis[1]>dis[2] && dis[1]>dis[0])            //右边距离障碍的距离比左边远
    {
      //后退
      digitalWrite(rightRedLed, LOW);
      digitalWrite(leftRedLed, LOW);
      digitalWrite(midGreenLed, LOW); 
      alarm (); 
      motorRun(BACKWARD,speedCar);
      delay(600);
    }
  }
  else
  {
    digitalWrite(rightRedLed, LOW );
    digitalWrite(leftRedLed, LOW);
    digitalWrite(midGreenLed, HIGH); 
    noTone(pinBuzzer);
    motorRun(FORWARD,speedCar);
  }  
}
