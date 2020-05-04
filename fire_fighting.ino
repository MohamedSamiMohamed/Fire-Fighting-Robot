#define trigPin 1
#define steps 32
#define forwardEchoPin 0
#define interruptPin 2
#define blinkPin 11
int MotorDir;
int rightMotorStep=0;
int leftMotorStep=0;
int fanMotorStep=0;
float duration;
float distance;
float distance2;
int pinaya;
bool ledState;
int counter;
unsigned long time;
unsigned long previous;
unsigned long current=micros();
unsigned long startFireHigh;
unsigned long currentMillis;
unsigned long previousMillis;
bool first=false;
bool move_work;
bool fan_work;
void setup() {
pinMode(interruptPin,INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(interruptPin),recordHigh,RISING);
for(int i=3;i<=10;i++)
pinMode(i,OUTPUT);
pinMode(A2,OUTPUT);
pinMode(A1,INPUT_PULLUP);
pinMode(A3,OUTPUT);
pinMode(A4,OUTPUT);
pinMode(A5,OUTPUT);
pinMode(trigPin,OUTPUT);
pinMode(forwardEchoPin,INPUT);
pinMode(blinkPin,OUTPUT);
pinMode(12,INPUT);
pinMode(A0,INPUT);
Serial.begin(9600); 
digitalWrite( trigPin, LOW ); 
previous=0;
pinaya=0;
MotorDir=1;
previousMillis=0;
ledState=true;
move_work=true;
fan_work=false;
}

void loop() {
  //here the fire detection not happened so we need to stop fan motor and turn on movement motors 
  if(digitalRead(2)==0){
    fan_work=false;
    move_work=true;
    }  
  move_work=analogRead(A1);
  digitalWrite(11,ledState);
  //blinking the led every second
  currentMillis=millis();
  if((unsigned)(currentMillis-previousMillis)>=1000){
    ledState=!ledState;
    digitalWrite(11,ledState);
    previousMillis=currentMillis;
    }
    
  bool mainFlag=false;
  //doing one step to each motor of the movement motors
  move_stepper(9,10,7,8,10,1,fanMotorStep,(fan_work&&(!move_work))); 
  move_stepper(5,6,3,4,0,MotorDir,leftMotorStep,(move_work&&(!fan_work)));
  move_stepper(A4,A5,A2,A3,0,MotorDir,rightMotorStep,(move_work&&(!fan_work)));

//trig the pins of ultraSonic sensors
   current=micros();
   if(first==false)
    if((unsigned long)(current-previous)>=2){
    digitalWrite( trigPin, HIGH );
    previous=micros();
    first=true;
    }
    //wait 10 microseconds where trigPin high
       current=micros();
    if((unsigned long)(current-previous)>=10){
      mainFlag=true;
     digitalWrite( trigPin, LOW );
      previous=micros();
      first=false;
    }

  duration = 0;
  if(mainFlag){
    //get the distance between the forward ultrasonic and the obstacle 
  distance =checkUltra(mainFlag,pinaya,duration);
  if(distance>100){
    if(pinaya==0){
      MotorDir=1;
      }
      //here distance will be greater than 100 only at the right sensor and his pin isn't 5 but it's just indicator and has no effect
      // i treated this pin in the logic of the code as A0
      //we must turn the robot to the right by rotating it two revolutions to the right
    if(pinaya==5){
      for(int i=0;i<2*steps;i++)
       move_stepper(A4,A5,A2,A3,50,MotorDir,rightMotorStep,(move_work&&(!fan_work)));
       //then w set the pin to be checked to 0 as the forward ultrasonic
       pinaya=0;
       MotorDir=1;
       }
       //here is the same as above but to the left
       else if(pinaya==12){
       for(int i=0;i<2*steps;i++)
       move_stepper(5,6,3,4,50,MotorDir,leftMotorStep,(move_work&&(!fan_work)));
               pinaya=0;
               MotorDir=1;
        }
    }
    //if the measured distance from the forward ultrasonic was less than 100 cm
    //then check the right ultrasonic
    if(distance<=100&&pinaya==0){
     pinaya=5;
  }
  //check the left ultrasonic
    else if(distance<=100&&pinaya==5){
    pinaya=12;
    }
    //make the robot go backward if all directions has obstacles
    else if(distance<=100&&pinaya==12){
    pinaya=0;
    MotorDir=0;
      }
  }
  //here we check about the fire detection sensor if it's high we calculate the period which this sensor being high
  //if this period more than 0.5 sec then it's a fire and turn on the fan work and turn off the move_work
if(digitalRead(2)==1){
  currentMillis=millis();
  if((currentMillis-startFireHigh)>=500){
    fan_work=true;
    move_work=false;
    //turn on the fan
    move_stepper(9,10,7,8,0,1,fanMotorStep,fan_work);    
    }
    else{
      move_work=true;
      fan_work=false;
      }
  }

 time=millis();
 while(millis()<time+50){
  
  }  
        }
  

//this function calculate the distance between ultraSonic sensor and an obstacle 
float checkUltra(bool flag,int echoPin,float duration){
  bool condition=false;

while(flag){
    if(echoPin==0){
   condition=PIND&1;
    }
    else if(echoPin==12){
      condition=PINB&16;
      }
      else{
        condition=PINC&1;
        }
    if(condition){
      time = micros();
      break;
      }
  }
  while(flag){
      if(echoPin==0){
   condition=PIND&1;
    }
    else if(echoPin==12){
      condition=PINB&16;
      }
      else{
        condition=PINC&1;
        }
    if(condition==0){
       duration=micros()-time;
        break;
        }
    }
    return(( duration/2 ) * 0.0344);

  }  
//this function called by the interrupt to record the time which the switch get high at.
void recordHigh(){
  startFireHigh=millis();
  }

//this function to control the motors of the robot , it takes the pins and the delay between every step and the other ,firection 0 counterClockWise,1 ClockWise, and the order of the current step
//also take a boolean variable to indicate that this motor should work now or not
  void move_stepper(int pin1,int pin2,int pin3,int pin4,int step_delay,int motor_direction,int&this_step,bool motor_work){
  if((motor_work==false)){
    return;
    }
  if(motor_direction==0){
    int temp1=pin3;
    pin3=pin4;
    pin4=temp1;
    int temp2=pin1;
    pin1=pin2;
    pin2=temp2;
  }
  switch(this_step%4){
    case 0: 
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
    digitalWrite(pin3,LOW);
    digitalWrite(pin4,HIGH);  
    this_step++; 
    break;
        case 1: 
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,HIGH);
    digitalWrite(pin3,LOW);
    digitalWrite(pin4,LOW);  
    this_step++;
    break;
        case 2: 
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,LOW);  
    this_step++;
    break;
        case 3: 
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,LOW);
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,HIGH);  
    this_step=0;
    break;
    }
 time=millis();
 while(millis()<time+step_delay){
  
  }  
  
  }

