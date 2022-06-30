//general_code
//input:no of steps from python program
#define m1_DIR 2
#define m1_STP 3
#define m2_DIR 4
#define m2_STP 5
#define m3_DIR 6
#define m3_STP 7
#define m4_DIR 8
#define m4_STP 9
#define relaypin 12

int stepsPerRevolution = 6400;
int r2b1,r3b1;
int no,high,low,mid,no1,no2,no3,sum,type;
int datafromuser[7];
// int pin = 0;

void direction(int dir) //if d1 == -1,then direction is low,if 1,then direction is high
{ if (dir == 1)
  { digitalWrite(m1_DIR, LOW);
    digitalWrite(m2_DIR, LOW); 
    digitalWrite(m3_DIR, LOW);}
  if (dir == 2)
  { digitalWrite(m1_DIR, LOW);
    digitalWrite(m2_DIR, LOW); 
    digitalWrite(m3_DIR, HIGH);}
  if (dir == 3)
  { digitalWrite(m1_DIR, LOW);
    digitalWrite(m2_DIR, HIGH); 
    digitalWrite(m3_DIR, LOW);}
  if (dir == 4)
  { digitalWrite(m1_DIR, LOW);
    digitalWrite(m2_DIR, HIGH); 
    digitalWrite(m3_DIR, HIGH);}
  if (dir == 5)
  { digitalWrite(m1_DIR, HIGH);
    digitalWrite(m2_DIR, LOW); 
    digitalWrite(m3_DIR, LOW);}
  if (dir == 6)
  { digitalWrite(m1_DIR, HIGH);
    digitalWrite(m2_DIR, LOW); 
    digitalWrite(m3_DIR, HIGH);}
  if (dir == 7)
  { digitalWrite(m1_DIR, HIGH);
    digitalWrite(m2_DIR, HIGH); 
    digitalWrite(m3_DIR, LOW);}
  if (dir == 8)
  { digitalWrite(m1_DIR, HIGH);
    digitalWrite(m2_DIR, HIGH); 
    digitalWrite(m3_DIR, HIGH);}
}

void pulse(int mo,int del)
{ if(mo == 1)  
  { digitalWrite(m1_STP, HIGH);
    delayMicroseconds(del);
    digitalWrite(m1_STP, LOW);
    delayMicroseconds(del);
  }

  if(mo == 2)
  { digitalWrite(m2_STP, HIGH);
    delayMicroseconds(del);
    digitalWrite(m2_STP, LOW);
    delayMicroseconds(del);
  }

  if(mo == 3)
  { digitalWrite(m3_STP, HIGH);
    delayMicroseconds(del);
    digitalWrite(m3_STP, LOW);
    delayMicroseconds(del);
  }  
}
void move(int a,int b,int c,int p,int q,int r,int gap)
{ sum = a + b + c;

  while(sum > 0)
  { if (a == 0)
    { r2b1 = 1;
      r3b1 = 1;
    } 

    if (a > 0)
    { r2b1 = b/a;
      r3b1 = c/a;  
      pulse(p,gap);
      a = a - 1;
    }    

    if (b > 0)
    { for (int i = 0;i < r2b1;i++)
      {pulse(q,gap);
       b = b - 1;
      } 
    }  

    if (c > 0)
    { for (int j = 0;j < r3b1;j++)
      {pulse(r,gap);
       c = c - 1;
      } 
    }   

    sum = a + b + c;
  }
}
void conveyor()
{
 digitalWrite(m4_DIR, HIGH);
 for(int c =0;c < 8800;c++)
  {  digitalWrite(m4_STP, HIGH);
     delayMicroseconds(500);
     digitalWrite(m4_STP, LOW);
     delayMicroseconds(500);
  }   
}
// void suction()
// {
//   if (pin == 0)
//   {pin = 1;
//    digitalWrite(relaypin,HIGH);}
//   else if (pin == 1)
//   {pin = 0;
//   digitalWrite(relaypin,LOW);}

// }
void setup() {
  // put your setup code here, to run once:
  pinMode(m1_STP, OUTPUT);
  pinMode(m1_DIR, OUTPUT);  
  pinMode(m2_STP, OUTPUT);
  pinMode(m2_DIR, OUTPUT);
  pinMode(m3_STP, OUTPUT);
  pinMode(m3_DIR, OUTPUT);
  pinMode(m4_STP, OUTPUT);
  pinMode(m4_DIR, OUTPUT);
  pinMode(relaypin, OUTPUT);
  digitalWrite(relaypin,HIGH);  
  Serial.begin(9600);
  //direction(1);
  //move(533,533,533,2,1,3,1000);     
}

void loop() {

  // put your main code here, to run repeatedly:
  mid = 0;
  high = 0;
  low = 0;
  no = 0;
  no2 = 0;
  no1 = 0;
  no3 = 0;
  datafromuser[0] = 0;
  datafromuser[1] = 0;
  datafromuser[2] = 0;
  datafromuser[3] = 0;
  datafromuser[4] = 0;
  datafromuser[5] = 0;
  datafromuser[6] = 0;
    
  if(Serial.available())
  { String serialread = Serial.readStringUntil('+');
    type = serialread.toInt();
    if(type == 901)                       //move delta
    { for(int l = 0;l < 7;l++)
      {serialread = Serial.readStringUntil('+');
      datafromuser[l]=serialread.toInt();
      }

      mid = datafromuser[0];              //the number  of steps  and the order of motor are received from python program  
      high = datafromuser[1];
      low = datafromuser[2];
      no = datafromuser[3];
      no2 = datafromuser[4];
      no1 = datafromuser[5];
      no3 = datafromuser[6];
      direction(no);
      move(mid,high,low,no2,no1,no3,1000);
    } 
    if(type == 902)                       //move conveyor
    {conveyor();
    }
    if(type == 903)                       //suction cup on
    {digitalWrite(relaypin,LOW);
    }
    if(type == 904)                       //suction cup on
    {digitalWrite(relaypin,HIGH);
    }
  }
    
  Serial.end();
  Serial.begin(9600);                   
  delay(500);
}