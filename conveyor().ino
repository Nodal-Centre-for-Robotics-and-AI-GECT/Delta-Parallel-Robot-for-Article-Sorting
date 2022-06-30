#define m4_DIR 8
#define m4_STP 9 

const int stepsPerRevolution = 3200;

void conveyor()   //done using delay ,can be done used millis() but current might be a constraint.
{     digitalWrite(m4_DIR, HIGH);
      
      for (int i = 0;i < stepsPerRevolution ; i++) 
      {   digitalWrite(m4_STP, HIGH);
          delayMicroseconds(500);
          digitalWrite(m4_STP, LOW);
          delayMicroseconds(500);
      }
}

void setup() 
{
  // put your setup code here, to run once:
 pinMode(m4_STP, OUTPUT);
 pinMode(m4_DIR, OUTPUT);
}
void loop() 
{ 
  // put your main code here, to run repeatedly:

  //moving the conveyor:

  conveyor();
   
}