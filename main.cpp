#include "Timer.h"
#include <Servo.h>//红是电源正极;白色或者橙色是信号级;黑色或者棕色的,是负极电源

#define TempEN 7//红外使能

#define serv1 2//下舵机
#define serv2 3//上舵机
#define left 6//底板左电机
#define right 5//底板右电机

#define leftdircon1 8
#define leftdircon2 9//左电机方向控制
#define rightdircon1 10
#define rightdircon2 11//右电机方向控制

#define leftsignal1 0 //黑
#define upsignal1 1 //灰
#define leftsignal2 2 //红
#define upsignal2 3 //橙

int left1,left2,up1,up2;//四路红外数据接收

Servo servo1,servo2;//

Timer dataread;//接收数据定时器
Timer followme;//定时检测跟随移动
Timer poschang;//舵机没动停止跟随

int pos1 = 90;//中间位置
int pos2 = 0;//正向，走线方便，但是不能再向下
int diva = 10;//红外数据比较裕量
int mot = 0;//转向标志位
int running = 1;//运行标志。1表示停止状态，4表示前进，7表示后退，5表示左转，6表示右转
int lastrunning = 1;//保存转向前的运行状态，前进或者后退；跟随移动状态防止跟随被中止
int receivedata = 0;//保存蓝牙接收到的数据
int leftspeed = 150;
int rightspeed = 150;//初速
int poschange = 0;//舵机运动标志位
int timecount = 100;//跟随运动中断程序计数值
int go = 0;//跟随运动标志位。1表示后退，2表示前进
int leftspeedtemp1;//表示补偿后的电机速度，给同样量电机左右好像转速不同
int leftspeedtemp2;

void trans1(int posi);//左右舵机控制程序
void trans2(int posj);//上下舵机控制程序
void distancevary(int temp1,int temp2);//红外数据比较程序
void datareceive(void);//定时中断接收蓝牙数据程序
void followmyhand(void);//定时中断跟随控制程序
void poschangevalue(void);//定时中断清舵机运动标志位程序，可能用处不大

void setup()
{
  pinMode(TempEN,OUTPUT);
  pinMode(leftdircon1,OUTPUT);
  pinMode(leftdircon2,OUTPUT);
  pinMode(rightdircon1,OUTPUT);
  pinMode(rightdircon2,OUTPUT);
  
  digitalWrite(TempEN,HIGH);//使能红外感应
  
  servo1.attach(serv1);//左右
  servo2.attach(serv2);//上下
  
  dataread.every(10,datareceive);
  followme.every(200,followmyhand);
  poschang.every(1000,poschangevalue);//初始化定时器中断程序
  
  Serial.begin(9600);
}

void loop()
{
  dataread.update(); //每10ms?查询一次数据缓冲区
  followme.update();//
  poschang.update();//开启定时器中断
  switch(running)//蓝牙操纵时不进行红外检测
  {
    case 0://手机蓝牙程序重力感应模式下停止发送0
    case 1://手机蓝牙程序按键模式下停止发送1
    if(lastrunning==4||lastrunning==7){
      analogWrite(left,1);
      analogWrite(right,1);
    }//从蓝牙控制到跟随移动状态,判断语句防止跟随移动被停止
    left1=analogRead(leftsignal1);
    delay(1);
    left2=analogRead(leftsignal2);
    delay(1);//读取红外数据
    distancevary(left2,left1);//比较
    trans1(mot); //执行左右位置
    
    up2=analogRead(upsignal2);
    delay(1);
    up1=analogRead(upsignal1);
    delay(1);
    if(pos2>30) up2+=30;
 // Serial.println(up2);测试用
    distancevary(up1,up2);
    trans2(mot);//上下位置
    if(left1>570||left2>570||up1>570||up2>570) go = 1;//后退所用红外数据很可能需要再调节，难以确定
    if(left1>100&&left2>100&&up1>100&&up2>100&&left1<200&&left2<200&&up1<200&&up2<200) go = 2;//同上
    lastrunning = 0;//清除蓝牙控制标志位
    break;
  
    case 7:
    digitalWrite(leftdircon1,HIGH);
    digitalWrite(leftdircon2,LOW);
    digitalWrite(rightdircon2,HIGH);
    digitalWrite(rightdircon1,LOW);
    lastrunning = running;//刷新前运行标志，以用于转向后恢复
    leftspeedtemp1=leftspeed+5;//进行补偿
    analogWrite(left,leftspeedtemp1);
    analogWrite(right,rightspeed);
    break;
  
    case 6:                 
    analogWrite(right,1);
    analogWrite(left,200);//转向速度未最大，可按需重设
    delay(50);//延时保证转向效果，延时时间可能需要按需重设
    break;//右转
    
    case 5:
    analogWrite(left,1);
    analogWrite(right,200);
    delay(50);//同上
    break;//左转
  
    case 4:
    digitalWrite(leftdircon2,HIGH);
    digitalWrite(leftdircon1,LOW);
    digitalWrite(rightdircon1,HIGH);
    digitalWrite(rightdircon2,LOW);//反向
    lastrunning = running;
    leftspeedtemp2 = leftspeed+20;
    analogWrite(left,leftspeedtemp2);
    analogWrite(right,rightspeed);
    
    default:break;
  } 
 if(running==5||running==6){
    analogWrite(left,leftspeed);
    analogWrite(right,rightspeed);
    running = lastrunning;
}//转向完成恢复前一运行状态
}

void trans1(int posi)
{
   if(posi==1&&pos1>0){
    pos1--;
    poschange = 1;
  }
  if(posi==2&&pos1<180){
    pos1++;
    poschange = 1;
  }//舵机范围0-180°，角度改变，标志位置位
  servo1.write(pos1);//进行转向
  mot=0;//清标志位
}

void trans2(int posi)
{
   if(posi==2&&pos2>0){
    pos2--;
    poschange = 1;
  }
  if(posi==1&&pos2<120){
    pos2++;
    poschange = 1;
  }//舵机范围0-120°，可按需重设，其余同上
  servo2.write(pos2);
  mot=0;
}

/*该程序必要性存疑，实际数据测算繁琐，但又未找到良好办法解决，
故区间划分相当随意。若有良好办法，请删除该程序。若有合理数据，
请重设区间*/
void distancevary(int temp1,int temp2)
{
  if(temp1<400&&temp2<400) diva=10;
  if(temp1>650&&temp2>650){ 
   diva=80;
 }//
else{
  diva=40;
}
if(temp1>temp2+diva) mot=1;
if(temp1<temp2-diva) mot=2;
}

void datareceive()
{
  if(Serial.available()>0){//蓝牙有发送数据再进行读取
    receivedata = Serial.read();//读取数据
    if(receivedata>125&&receivedata<255){
      leftspeed = rightspeed = receivedata;
      analogWrite(left,leftspeed);
      analogWrite(right,rightspeed);//适应手机软件的调速功能
    }
    else{
      running = receivedata;//获取运行状态标志
    }
  }
  while(Serial.read()>0) ;//清串口缓存，实际必要性存疑，若有疑问，尽量不用。
}

void followmyhand(void)
{
  if(poschange==1&&go==1&&timecount>1){
    /*需要后退标志位置位，舵机运动标志位置位，跟随运动中断计数值
    大于4同时成立才能进行跟随运动控制*/
    digitalWrite(leftdircon1,HIGH);
    digitalWrite(leftdircon2,LOW);
    digitalWrite(rightdircon2,HIGH);
    digitalWrite(rightdircon1,LOW); 
    analogWrite(left,150);
    analogWrite(right,150);//运行速度可能需要补偿和重设
    go = 0;//清标志位
    timecount = 0;//清计数值
  }
  if(poschange==1&&go==2&&timecount>1){
    digitalWrite(leftdircon2,HIGH);
    digitalWrite(leftdircon1,LOW);
    digitalWrite(rightdircon1,HIGH);
    digitalWrite(rightdircon2,LOW);
    analogWrite(left,150);
    analogWrite(right,150);
    go = 0;
    timecount = 0;
  }//同上
  timecount++;
  /*timecount这个变量的设置是为了满足实时性和持续性的双重要求。
  实时性要求中断检测的时间够短，持续性要求跟随移动的距离够长。
  因为在中断中添加延时是极为不适合的，所以增加了此标志位来解决
  这个问题。要求改变可重设或删除。*/
  if(timecount==2){
    analogWrite(left,1);
    analogWrite(right,1);
  }
}

void poschangevalue(void)
{
  poschange = 0;
}