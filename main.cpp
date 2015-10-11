#include "Timer.h"
#include <Servo.h>//���ǵ�Դ����;��ɫ���߳�ɫ���źż�;��ɫ������ɫ��,�Ǹ�����Դ

#define TempEN 7//����ʹ��

#define serv1 2//�¶��
#define serv2 3//�϶��
#define left 6//�װ�����
#define right 5//�װ��ҵ��

#define leftdircon1 8
#define leftdircon2 9//�����������
#define rightdircon1 10
#define rightdircon2 11//�ҵ���������

#define leftsignal1 0 //��
#define upsignal1 1 //��
#define leftsignal2 2 //��
#define upsignal2 3 //��

int left1,left2,up1,up2;//��·�������ݽ���

Servo servo1,servo2;//

Timer dataread;//�������ݶ�ʱ��
Timer followme;//��ʱ�������ƶ�
Timer poschang;//���û��ֹͣ����

int pos1 = 90;//�м�λ��
int pos2 = 0;//�������߷��㣬���ǲ���������
int diva = 10;//�������ݱȽ�ԣ��
int mot = 0;//ת���־λ
int running = 1;//���б�־��1��ʾֹͣ״̬��4��ʾǰ����7��ʾ���ˣ�5��ʾ��ת��6��ʾ��ת
int lastrunning = 1;//����ת��ǰ������״̬��ǰ�����ߺ��ˣ������ƶ�״̬��ֹ���汻��ֹ
int receivedata = 0;//�����������յ�������
int leftspeed = 150;
int rightspeed = 150;//����
int poschange = 0;//����˶���־λ
int timecount = 100;//�����˶��жϳ������ֵ
int go = 0;//�����˶���־λ��1��ʾ���ˣ�2��ʾǰ��
int leftspeedtemp1;//��ʾ������ĵ���ٶȣ���ͬ����������Һ���ת�ٲ�ͬ
int leftspeedtemp2;

void trans1(int posi);//���Ҷ�����Ƴ���
void trans2(int posj);//���¶�����Ƴ���
void distancevary(int temp1,int temp2);//�������ݱȽϳ���
void datareceive(void);//��ʱ�жϽ����������ݳ���
void followmyhand(void);//��ʱ�жϸ�����Ƴ���
void poschangevalue(void);//��ʱ�ж������˶���־λ���򣬿����ô�����

void setup()
{
  pinMode(TempEN,OUTPUT);
  pinMode(leftdircon1,OUTPUT);
  pinMode(leftdircon2,OUTPUT);
  pinMode(rightdircon1,OUTPUT);
  pinMode(rightdircon2,OUTPUT);
  
  digitalWrite(TempEN,HIGH);//ʹ�ܺ����Ӧ
  
  servo1.attach(serv1);//����
  servo2.attach(serv2);//����
  
  dataread.every(10,datareceive);
  followme.every(200,followmyhand);
  poschang.every(1000,poschangevalue);//��ʼ����ʱ���жϳ���
  
  Serial.begin(9600);
}

void loop()
{
  dataread.update(); //ÿ10ms?��ѯһ�����ݻ�����
  followme.update();//
  poschang.update();//������ʱ���ж�
  switch(running)//��������ʱ�����к�����
  {
    case 0://�ֻ���������������Ӧģʽ��ֹͣ����0
    case 1://�ֻ��������򰴼�ģʽ��ֹͣ����1
    if(lastrunning==4||lastrunning==7){
      analogWrite(left,1);
      analogWrite(right,1);
    }//���������Ƶ������ƶ�״̬,�ж�����ֹ�����ƶ���ֹͣ
    left1=analogRead(leftsignal1);
    delay(1);
    left2=analogRead(leftsignal2);
    delay(1);//��ȡ��������
    distancevary(left2,left1);//�Ƚ�
    trans1(mot); //ִ������λ��
    
    up2=analogRead(upsignal2);
    delay(1);
    up1=analogRead(upsignal1);
    delay(1);
    if(pos2>30) up2+=30;
 // Serial.println(up2);������
    distancevary(up1,up2);
    trans2(mot);//����λ��
    if(left1>570||left2>570||up1>570||up2>570) go = 1;//�������ú������ݺܿ�����Ҫ�ٵ��ڣ�����ȷ��
    if(left1>100&&left2>100&&up1>100&&up2>100&&left1<200&&left2<200&&up1<200&&up2<200) go = 2;//ͬ��
    lastrunning = 0;//����������Ʊ�־λ
    break;
  
    case 7:
    digitalWrite(leftdircon1,HIGH);
    digitalWrite(leftdircon2,LOW);
    digitalWrite(rightdircon2,HIGH);
    digitalWrite(rightdircon1,LOW);
    lastrunning = running;//ˢ��ǰ���б�־��������ת���ָ�
    leftspeedtemp1=leftspeed+5;//���в���
    analogWrite(left,leftspeedtemp1);
    analogWrite(right,rightspeed);
    break;
  
    case 6:                 
    analogWrite(right,1);
    analogWrite(left,200);//ת���ٶ�δ��󣬿ɰ�������
    delay(50);//��ʱ��֤ת��Ч������ʱʱ�������Ҫ��������
    break;//��ת
    
    case 5:
    analogWrite(left,1);
    analogWrite(right,200);
    delay(50);//ͬ��
    break;//��ת
  
    case 4:
    digitalWrite(leftdircon2,HIGH);
    digitalWrite(leftdircon1,LOW);
    digitalWrite(rightdircon1,HIGH);
    digitalWrite(rightdircon2,LOW);//����
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
}//ת����ɻָ�ǰһ����״̬
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
  }//�����Χ0-180�㣬�Ƕȸı䣬��־λ��λ
  servo1.write(pos1);//����ת��
  mot=0;//���־λ
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
  }//�����Χ0-120�㣬�ɰ������裬����ͬ��
  servo2.write(pos2);
  mot=0;
}

/*�ó����Ҫ�Դ��ɣ�ʵ�����ݲ��㷱��������δ�ҵ����ð취�����
�����仮���൱���⡣�������ð취����ɾ���ó������к������ݣ�
����������*/
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
  if(Serial.available()>0){//�����з��������ٽ��ж�ȡ
    receivedata = Serial.read();//��ȡ����
    if(receivedata>125&&receivedata<255){
      leftspeed = rightspeed = receivedata;
      analogWrite(left,leftspeed);
      analogWrite(right,rightspeed);//��Ӧ�ֻ�����ĵ��ٹ���
    }
    else{
      running = receivedata;//��ȡ����״̬��־
    }
  }
  while(Serial.read()>0) ;//�崮�ڻ��棬ʵ�ʱ�Ҫ�Դ��ɣ��������ʣ��������á�
}

void followmyhand(void)
{
  if(poschange==1&&go==1&&timecount>1){
    /*��Ҫ���˱�־λ��λ������˶���־λ��λ�������˶��жϼ���ֵ
    ����4ͬʱ�������ܽ��и����˶�����*/
    digitalWrite(leftdircon1,HIGH);
    digitalWrite(leftdircon2,LOW);
    digitalWrite(rightdircon2,HIGH);
    digitalWrite(rightdircon1,LOW); 
    analogWrite(left,150);
    analogWrite(right,150);//�����ٶȿ�����Ҫ����������
    go = 0;//���־λ
    timecount = 0;//�����ֵ
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
  }//ͬ��
  timecount++;
  /*timecount���������������Ϊ������ʵʱ�Ժͳ����Ե�˫��Ҫ��
  ʵʱ��Ҫ���жϼ���ʱ�乻�̣�������Ҫ������ƶ��ľ��빻����
  ��Ϊ���ж��������ʱ�Ǽ�Ϊ���ʺϵģ����������˴˱�־λ�����
  ������⡣Ҫ��ı�������ɾ����*/
  if(timecount==2){
    analogWrite(left,1);
    analogWrite(right,1);
  }
}

void poschangevalue(void)
{
  poschange = 0;
}