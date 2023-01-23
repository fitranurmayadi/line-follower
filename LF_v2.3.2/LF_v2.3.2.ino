//_____________ FITRA NURMAYADI
//_____________ 150402098

#include <EEPROM.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(36,38,40,42,44,46);//pin lcd

#define bawah     digitalRead(25) //pin button
#define atas      digitalRead(27) //pin button
#define batal     digitalRead(29) //pin button
#define ok        digitalRead(31) //pin button
#define inki     digitalRead(19)
#define inka     digitalRead(21)


#define buzzer    23 //buzzer
#define Lpwm  5 //pin pwm motor
#define Rpwm  9 //pin pwm motor
#define Ldir  3 //pin arah/direction
#define Rdir  7 //pin arah/direction
const int Low =  50;
const int Mid = 100;
const int High = 150;

unsigned char B[8]={
  EEPROM.read(0),
  EEPROM.read(1),
  EEPROM.read(2),
  EEPROM.read(3),
  EEPROM.read(4),
  EEPROM.read(5),
  EEPROM.read(6),
  EEPROM.read(7)}; //konstanta nilai pembanding sensor
unsigned char gki=EEPROM.read(8);//generator pwm
unsigned char gka=EEPROM.read(9);//
  
unsigned char kp=EEPROM.read(10);//konstanta P
unsigned char ki=EEPROM.read(11);//konstanta I
unsigned char kd=EEPROM.read(12);//konstanta D
unsigned char ts=EEPROM.read(13);//konstanta waktu sampling analog
unsigned char mv=EEPROM.read(14);//konstanta kecepatan maksimal

char data[33];

unsigned int counter=0;
unsigned int countertengah=0;
unsigned int counterkanan=0;
unsigned int counterkiri=0;

float tF, pid;
unsigned int a[10];
unsigned int c[10];
unsigned int s[10];
unsigned char ss[10];//untuk sensor
unsigned int dd0,dd1;//untuk arah motor
unsigned int dat,dut,sayap,sensor,x,yoo,yon;

int ski=0;
int ska=0;
int t;
int T=0;

int P,I,D,kpF,kiF,kdF,posisi,error,last_error,mvF,Min,m1,m2,gas,hasil;
int Z=0;
int crule=0;
int finish=0;

  
  
//*************************SCAN GARIS**************************//
  void scan(){

  a[0]=analogRead(A0);
  a[1]=analogRead(A2);
  a[2]=analogRead(A4);
  a[3]=analogRead(A6);
  a[4]=analogRead(A8);
  a[5]=analogRead(A10);
  a[6]=analogRead(A12);
  a[7]=analogRead(A14);

  c[0]=B[0];
  c[1]=B[1];
  c[2]=B[2];
  c[3]=B[3];
  c[4]=B[4];
  c[5]=B[5];
  c[6]=B[6];
  c[7]=B[7];
  
  lcd.setCursor(6,0);
  if(inki==HIGH) {s[8]=1;lcd.print("1");} else{s[8]=0;lcd.print("0");}
  if(a[0]>c[0])  {s[0]=1;lcd.print("1");} else if(a[0]<c[0]){s[0]=0;lcd.print("0");} 
  if(a[1]>c[1])  {s[1]=1;lcd.print("1");} else if(a[1]<c[1]){s[1]=0;lcd.print("0");} 
  if(a[2]>c[2])  {s[2]=1;lcd.print("1");} else if(a[2]<c[2]){s[2]=0;lcd.print("0");} 
  if(a[3]>c[3])  {s[3]=1;t=1;lcd.print("1");} else if(a[3]<c[3]){s[3]=0;lcd.print("0");} 
  if(a[4]>c[4])  {s[4]=1;t=1;lcd.print("1");} else if(a[4]<c[4]){s[4]=0;lcd.print("0");} 
  if(a[5]>c[5])  {s[5]=1;lcd.print("1");} else if(a[5]<c[5]){s[5]=0;lcd.print("0");} 
  if(a[6]>c[6])  {s[6]=1;lcd.print("1");} else if(a[6]<c[6]){s[6]=0;lcd.print("0");} 
  if(a[7]>c[7])  {s[7]=1;lcd.print("1");} else if(a[7]<c[7]){s[7]=0;lcd.print("0");} 
  if(inka==HIGH) {s[9]=1;lcd.print("1");} else{s[9]=0;lcd.print("0");}
  
  dat=(s[0]*128)+(s[1]*64)+(s[2]*32)+(s[3]*16)+(s[4]*8)+(s[5]*4)+(s[6]*2)+(s[7]*1);
  sensor=dat;
  dut=(s[8]*2)+(s[9]*1);
  sayap=dut;
  yoo = dat;
  yon = dat;
  yon &= 0b11000011;
  yoo &= 0b01011010;

  if(yoo==0b00011010){T=1;}
  if(yoo==0b01011010){T=1;}
  if(yoo==0b01011000){T=1;}
  
  }
//*********************************SELESAI SCAN************************************

//*********************************MENU PILIHAN************************************
void menu(){
  menu0:
  delay(100); lcd.clear();  
  lcd.setCursor(0,0); lcd.print(">>>>>>>><<<<<<<<");
  lcd.setCursor(0,1); lcd.print("SETUP  ||  START");
  if(ok==0){goto menu1;}
  if(batal==0){goto start;}
  if(atas==0){goto start;}
  if(bawah==0){goto start;}
  goto menu0;
  
  menu1:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print("ATUR   ||  SAVE");
  if(atas==0){goto save;}
  if(bawah==0){goto save;}
  if(ok==0){ goto menu2;}
  if(batal==0){ goto menu0;}
  goto menu1;
  
  menu2:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print("SET PID       <>");
  if(atas==0){goto menu6;}
  if(bawah==0){goto menu3;}
  if(ok==0){ goto set_kp;}
  if(batal==0){ goto menu1;}
  goto menu2;
  
  menu3:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print("SET KECEPATAN <>");
  if(atas==0){goto menu2;}
  if(bawah==0){goto menu4;}
  if(ok==0){ goto set_MAXcepat;}
  if(batal==0){ goto menu1;}
  goto menu3;
  
  menu4:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print("SET ADC       <>");
  if(atas==0){goto menu3;}
  if(bawah==0){goto menu5;} 
  if(ok==0){ delay(50); lcd.clear(); goto sensor;}
  if(batal==0){ goto menu1;}
  goto menu4;

  menu5:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print("RESET EPPROM  <>");
  if(atas==0){goto menu4;}
  if(bawah==0){goto menu6;}
  if(ok==0){kp=ki=kd=mv=ts=B[0]=B[1]=B[2]=B[3]=B[4]=B[5]=B[6]=B[7]=0;lcd.setCursor(0,0); lcd.print("Reset Sukses"); delay(500);lcd.clear();}
  if(batal==0){ goto menu1;}
  goto menu5;

  menu6:
  delay(100); lcd.clear();  lcd.setCursor(0,0); lcd.print(">>.CEK SENSOR.<<");
  if(atas==0){goto menu5;}
  if(bawah==0){goto menu2;}
  if(ok==0){goto cek_sensor; }
  if(batal==0){ goto menu1;}
  goto menu6;

  //**************************SET PID*********************************//
  set_kp:
  delay(100); lcd.clear();  lcd.setCursor(0,0); sprintf(data,"kp:%d",kp); lcd.print(data);
  if(atas==0){delay(20); if(kp<255){kp++;}}
  if(bawah==0){delay(20); if(kp>0){kp--;}}
  if(ok==0){EEPROM.update(10,kp);delay(21);goto set_ki;}
  if(batal==0){ goto menu2;}
  goto set_kp;
  
  set_ki:
  delay(100); lcd.clear();  lcd.setCursor(0,0); sprintf(data,"ki:%d",ki); lcd.print(data);
  if(atas==0){delay(20); if(ki<255){ki++;}}
  if(bawah==0){delay(20); if(ki>0){ki--;}}
  if(ok==0){EEPROM.update(11,ki);delay(21);goto set_kd;}
  if(batal==0){ goto set_kp;}
  goto set_ki;

  set_kd:
  delay(100); lcd.clear();  lcd.setCursor(0,0); sprintf(data,"kd:%d",kd); lcd.print(data);
  if(atas==0){delay(20); if(kd<255){kd++;}}
  if(bawah==0){delay(20); if(kd>0){kd--;}}
  if(ok==0){EEPROM.update(12,kd);delay(21);goto set_ts;}
  if(batal==0){ goto set_ki;}
  goto set_kd;

  set_ts:
  delay(100); lcd.clear();  lcd.setCursor(0,0); sprintf(data,"Ts:%d",ts); lcd.print(data);
  if(atas==0){delay(10); if(ts<250){ts+=5;}}
  if(bawah==0){delay(10); if(ts>0){ts-=5;}}
  if(ok==0){EEPROM.update(13,ts);delay(21);goto set_kp;}
  if(batal==0){ goto set_kd;}
  goto set_ts;
  //******************************SET KECEPATAN*********************//
  set_MAXcepat:
  delay(100); lcd.clear();  lcd.setCursor(0,0); sprintf(data,"Speed:%d",mv); lcd.print(data);
  if(atas==0){delay(50); if(mv<255){mv+=5;}}
  if(bawah==0){delay(50); if(mv>0){mv-=5;}}
  if(batal==0){EEPROM.update(14,mv);delay(21);goto menu3;}
  goto set_MAXcepat;

  cek_sensor:
  delay(100); lcd.clear();
  scan();
  lcd.setCursor(0,0); sprintf(data,"%d|",a[0]);lcd.print(data);
  lcd.setCursor(4,0); sprintf(data,"%d|",a[1]);lcd.print(data);
  lcd.setCursor(8,0); sprintf(data,"%d|",a[2]);lcd.print(data);
  lcd.setCursor(12,0); sprintf(data,"%d|",a[3]);lcd.print(data);
  lcd.setCursor(0,1); sprintf(data,"%d|",a[4]);lcd.print(data);
  lcd.setCursor(4,1); sprintf(data,"%d|",a[5]);lcd.print(data);
  lcd.setCursor(8,1); sprintf(data,"%d|",a[6]);lcd.print(data);
  lcd.setCursor(12,1); sprintf(data,"%d|",a[7]);lcd.print(data);
  
  if(ok==0){goto menu6;}
  
  
  goto cek_sensor;

  sensor:
  delay(100);
  lcd.clear();
  lcd.setCursor(0,1); lcd.print(" ADC ");
  scan();
  if(ok==0){x=0; goto set_adc;}
  if(batal==0){ goto menu4;}
  goto sensor;

  set_adc:
  delay(100); lcd.clear(); lcd.setCursor(0,0); sprintf(data,"ADC %d = %3d, %3d",x,a[x],B[x]); lcd.print(data);
  a[x];
  if(ok==0){ delay(50); B[x]=a[x]/4; EEPROM.update(x,B[x]);}
  if(batal==0){ delay(50); lcd.clear(); goto sensor;}
  if(bawah==0){ delay(50); if(x<7){x++;}}
  if(atas==0){ delay(50); if(x>0){x--;}}
  goto set_adc;



  save:
  delay(50);
  lcd.setCursor(0,0); lcd.print("MENYIMPAN....");
  EEPROM.update(0, B[0]);lcd.setCursor(11,0);lcd.print("  0%");delay(12);
  EEPROM.update(1, B[1]);lcd.setCursor(11,0);lcd.print("  9%");delay(12);
  EEPROM.update(2, B[2]);lcd.setCursor(11,0);lcd.print(" 12%");delay(12);
  EEPROM.update(3, B[3]);lcd.setCursor(11,0);lcd.print(" 17%");delay(12);
  EEPROM.update(4, B[4]);lcd.setCursor(11,0);lcd.print(" 21%");delay(12);
  EEPROM.update(5, B[5]);lcd.setCursor(11,0);lcd.print(" 27%");delay(12);
  EEPROM.update(6, B[6]);lcd.setCursor(11,0);lcd.print(" 32%");delay(12);
  EEPROM.update(7, B[7]);lcd.setCursor(11,0);lcd.print(" 38%");delay(12);
  EEPROM.update(8, gka);lcd.setCursor(11,0);lcd.print(" 45%");delay(12);
  EEPROM.update(9, gki);lcd.setCursor(11,0);lcd.print(" 56%");delay(12);
  EEPROM.update(10, kp);lcd.setCursor(11,0);lcd.print(" 64%");delay(12);
  EEPROM.update(11, ki);lcd.setCursor(11,0);lcd.print(" 72%");delay(12);
  EEPROM.update(12, kd);lcd.setCursor(11,0);lcd.print(" 81%");delay(12);
  EEPROM.update(13, ts);lcd.setCursor(11,0);lcd.print(" 95%");delay(12);
  EEPROM.update(14, mv);lcd.setCursor(11,0);lcd.print("100%");delay(12);
  lcd.setCursor(0,0); lcd.print("SELESAI....");delay(20);
  goto menu0;
  
  start:
  delay(50);lcd.clear();
}

//********************************TULIS KE EEPROM**********************//


void load(){
  delay(50);
  lcd.clear();
  lcd.setCursor(0,0);lcd.print("MENGUNDUH.......");
  kpF=kp;
  lcd.setCursor(11,0);lcd.print(" 0%");delay(12);
  kiF=ki;
  lcd.setCursor(11,0);lcd.print(" 9%");delay(12);
  kdF=(float)kd/(float)2;
  lcd.setCursor(11,0);lcd.print(" 17%");delay(12);
  tF=(float)ts*(float)4;
  lcd.setCursor(11,0);lcd.print(" 21%");delay(12);
  mvF=mv;
  lcd.setCursor(11,0);lcd.print(" 32%");delay(12);
  Min=0-mv;
  lcd.setCursor(11,0);lcd.print(" 45%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 54%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 64%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 72%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 83%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 91%");delay(12);
  lcd.setCursor(11,0);lcd.print(" 98%");delay(12);
  lcd.setCursor(11,0);lcd.print("100%");delay(12);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("PID");
  lcd.setCursor(0,1); sprintf(data, "K = %3d %3d 3%d", kpF,kiF,kdF); lcd.print(data); 
  delay(200); 
  lcd.clear();
  lcd.setCursor(0,0); sprintf(data, "KECEPATAN  = %3d", mv);lcd.print(data);
  lcd.setCursor(0,1); sprintf(data, "WAKTU SAMP = %3d",(int)tF); lcd.print(data); 
  delay(200); 
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("MENGUNDUH...100%"); delay(100);
  delay(50);
  }


void motor(unsigned char d0, unsigned char d1,unsigned char L, unsigned char R){
  
  digitalWrite(Ldir, d0);
  digitalWrite(Rdir, d1);
  analogWrite(Lpwm, L);
  analogWrite(Rpwm, R);
}

void motormati(){
  while(1){
  digitalWrite(Ldir, 0);
  digitalWrite(Rdir, 0);
  analogWrite(Lpwm, 0);
  analogWrite(Rpwm, 0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SISTEM BERHENTI");
  digitalWrite(23, HIGH);
  delay(500);
  digitalWrite(23, LOW);
  delay(1000);
  }
  }

void interupsi(){
  detachInterrupt(digitalPinToInterrupt(19));
  detachInterrupt(digitalPinToInterrupt(21));
  if(T==1){
  counter++;
  }
  T=0;
  attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
  attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);
  }



void PID(){
  digitalWrite(23,0);  
  if (sensor==0b11100111){Z=1;}
  if (sensor==0b00011000){ posisi=0; t=1;}   
  else if (sensor==0b00111100){posisi=0;}
  else if (sensor==0b01111110){posisi=0;}
  else if (sensor==0b01111111){posisi=0;}
  else if (sensor==0b11111110){posisi=0;}

  else if (sensor==0b00001000){ posisi=-1;}
  else if (sensor==0b00010000){ posisi=1;}
  else if (sensor==0b00011100){ posisi=-1;}
  else if (sensor==0b00111000){ posisi=1;}
  
  else if (sensor==0b00001100){ posisi=-2;}
  else if (sensor==0b00110000){ posisi=2;}

  else if (sensor==0b00000100){ posisi=-3;}
  else if (sensor==0b00100000){ posisi=3;}
  else if (sensor==0b00001110){ posisi=-3;}
  else if (sensor==0b01110000){ posisi=3;}
  
  else if (sensor==0b00000110){ posisi=-4;}
  else if (sensor==0b01100000){ posisi=4;}

  else if (sensor==0b00000010){ posisi=-5;}
  else if (sensor==0b01000000){ posisi=5;}
  else if (sensor==0b00000111){ posisi=-5;}
  else if (sensor==0b11100000){ posisi=5;}
  else if (sensor==0b00011110){ posisi=-5;}
  else if (sensor==0b01111000){ posisi=5;}
  
  else if (sensor==0b00000011){ posisi=-6;}
  else if (sensor==0b11000000){ posisi=6;}
  else if (sensor==0b00111110){ posisi=-6;}
  else if (sensor==0b01111100){ posisi=6;}
  else if (sensor==0b00001111){ posisi=-6;}
  else if (sensor==0b11110000){ posisi=6;}
  
  else if (sensor==0b00000001){ posisi=-7;}
  else if (sensor==0b10000000){ posisi=7;}
  else if (sensor==0b00011111){ posisi=-7;}
  else if (sensor==0b11111000){ posisi=7;}
  
  if (sensor==0b00000000 && sayap==0b10){posisi=9;} //int kiri
  else if (sensor==0b00000000 && sayap==0b01){posisi=-9;}//int kanan
  else if (sensor==0b00000000 && sayap==0b00)
  {
    if (posisi>0 && posisi<9){ posisi=8;}
   else if (posisi<0 && posisi>-9){posisi=-8;} 
   if (posisi>8) {posisi=10;}
    else if (posisi<-8){posisi=-10;}
    }

  error = (kpF*posisi);
  P = error;
  I = kiF*(error+last_error);
  D = kdF*(error-last_error);
  last_error = error;
  pid = (float)P + (float)I+(float)D;
  hasil = (int)pid;

  //*****motorkiri
  m1 = mvF - hasil;
  if(m1 > mvF){ m1 = mvF;}
  if(m1 < Min){ m1 = Min;}
  if(m1 > 0){ dd0 = 0;}
  if(m1 < 0){ dd0 = 1;gas= 0-m1;m1=gas;}
  if(error==0){ m1=mvF;}
  //*****motorkanan
  m2 = mvF + hasil;
  if(m2 > mvF){ m2 = mvF;}
  if(m2 < Min){ m2 = Min;}
  if(m2 > 0){ dd1 = 0;}
  if(m2 < 0){ dd1 = 1;gas= 0-m2;m2=gas;}
  if(error==0){ m2=mvF;}

  motor(dd0,dd1,m1,m2);
}

//************************     AYO GERAK   *******************************

void rule(){
  while(crule==0){
  digitalWrite(23, HIGH);
  delay(100); lcd.clear();  
  lcd.setCursor(0,0); lcd.print("KIRI       KANAN");
  lcd.setCursor(0,1); lcd.print("<<     GO     >>");
  if(ok==0){ delay(50); digitalWrite(23, LOW);lcd.clear();kiriRule();crule=1;}
  if(batal==0){ delay(50); digitalWrite(23, LOW);lcd.clear();kiriRule();crule=1;}
  if(atas==0){ delay(50); digitalWrite(23, LOW);lcd.clear(); kananRule();crule=1;}
  if(bawah==0){ delay(50); digitalWrite(23, LOW);lcd.clear();kananRule();crule=1;}
  }
}


void setup() {
  //Serial.begin(9600);
  lcd.begin(16,2);
  pinMode(19,INPUT);  
  pinMode(21,INPUT);
  pinMode(25,INPUT);
  pinMode(27,INPUT);
  pinMode(28,INPUT);
  pinMode(31,INPUT);
  
  pinMode(buzzer,OUTPUT);
//  pinMode(generator1,OUTPUT);
//  pinMode(generator2,OUTPUT);
  
  pinMode(Lpwm,OUTPUT);
  pinMode(Ldir,OUTPUT);
  pinMode(Rpwm,OUTPUT);
  pinMode(Rdir,OUTPUT);
  menu();
  load();
  }
  
void loop(){
 rule();
}



/******************************************************************************************************************/

void kananRule(){//*******KANAN  
  attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
  attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);
  counter=0;
  Z=0;
  
  while(1){
      digitalWrite(23,0);
      lcd.setCursor(0,1); sprintf( data, "L%3d R%3d P%3d", m1,m2,posisi); lcd.print(data);
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
   
   if(counter==2){
        detachInterrupt(digitalPinToInterrupt(19));
        detachInterrupt(digitalPinToInterrupt(21));
        scan();
        while(yon != 0b11000000){
        lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);  
          scan();
          digitalWrite(23, HIGH);
          motor(0,0,0,80);  
        }
        counter++;
        mvF=mv;
        digitalWrite(23, LOW);
        attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
        attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
      } 
  
  else if(counter==7){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    while(yon != 0b00000011){
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
      digitalWrite(23, HIGH);
      motor(0,0,80,0);  
    }
    digitalWrite(23, LOW);
    counter++;
    attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
    attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
  }
  
  else if(counter==9){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    while(yon != 0b00000011){
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
      digitalWrite(23, HIGH);
      motor(0,0,80,0);
    }
    digitalWrite(23, LOW);
    counter++;
    posisi==-7;
    attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
    attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
  }

  else if(counter==11){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
    while(yon != 0b11000000){
      scan();
      digitalWrite(23, HIGH);
      motor(0,0,0,80);  
    }
    digitalWrite(23, LOW);
    counter++;
    }
  
  if(counter==12){
    fkanan();
  }
  
  PID(); 
  }

}

void kiriRule(){//*******KIRI
  attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
  attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);
  counter=0;
  Z=0;
  
  while(1){
      digitalWrite(23,0);
      lcd.setCursor(0,1); sprintf( data, "L%3d R%3d P%3d", m1,m2,posisi); lcd.print(data);
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
   
   if(counter==2){
        detachInterrupt(digitalPinToInterrupt(19));
        detachInterrupt(digitalPinToInterrupt(21));
        scan();
        while(yon != 0b00000011){
        lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);  
          scan();
          digitalWrite(23, HIGH);
          motor(0,0,80,0);  
        }
        counter++;
        mvF=mv;
        digitalWrite(23, LOW);
        attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
        attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
      } 
  
  else if(counter==7){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    while(yon != 0b11000000){
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
      digitalWrite(23, HIGH);
      motor(0,0,0,80);  
    }
    digitalWrite(23, LOW);
    counter++;
    attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
    attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
  }
  
  else if(counter==9){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    while(yon != 0b11000000){
      scan();
      lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
      digitalWrite(23, HIGH);
      motor(0,0,0,80);
    }
    digitalWrite(23, LOW);
    counter++;
    posisi==-7;
    attachInterrupt(digitalPinToInterrupt(19),interupsi, RISING);
    attachInterrupt(digitalPinToInterrupt(21),interupsi, RISING);  
  }

  else if(counter==11){
    detachInterrupt(digitalPinToInterrupt(19));
    detachInterrupt(digitalPinToInterrupt(21));
    scan();
    lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
    while(yon != 0b00000011){
      scan();
      digitalWrite(23, HIGH);
      motor(0,0,80,0);  
    }
    digitalWrite(23, LOW);
    counter++;
    }
    
  if(counter==12){
    fkiri();
  }
  PID();
  }
}

void fkanan(){
  detachInterrupt(digitalPinToInterrupt(19));
  detachInterrupt(digitalPinToInterrupt(21));
    
  finish=0;
  Z=0;
  while(1){
  scan();
  lcd.setCursor(0,0); sprintf(data,"%d %d",counter,finish); lcd.print(data);
  
  if(Z==1 && sensor==0b11111111){
    scan();
    lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
    while(yon != 0b11000000){
      scan();
      digitalWrite(23, HIGH);
      motor(0,0,0,80);  
    }
    digitalWrite(23, LOW);
    finish++;  
  }
  
  if(finish==1 && sensor==0b11111111){
    digitalWrite(23, HIGH);
    delay(2000);
    motormati();
  }
  PID();
  }
}

void fkiri(){
  detachInterrupt(digitalPinToInterrupt(19));
  detachInterrupt(digitalPinToInterrupt(21));
    
  finish=0;
  Z=0;
  while(1){
  scan();
  lcd.setCursor(0,0); sprintf(data,"%d %d",counter,finish); lcd.print(data);
  if(Z==1  && sensor==0b11111111){
    scan();
    lcd.setCursor(0,0); sprintf(data,"%d",counter); lcd.print(data);
    while(yon != 0b00000011){
      scan();
      digitalWrite(23, HIGH);
      motor(0,0,80,0);  
    }
    digitalWrite(23, LOW);
    finish++;  
    }
  
  if(finish==1 && sensor==0b11111111){
    digitalWrite(23, HIGH);
    delay(2000);
    motormati();
    }

  PID();
  }
}
