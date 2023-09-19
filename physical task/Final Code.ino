
#define R_motor_A 4
#define R_motor_B 7
#define L_motor_A 2
#define L_motor_B 3

#define R_motor_pwm 6
#define L_motor_pwm 5

#include <NewPing.h>

#define TRIGGER_PIN_FL A0   // Front Left Trigger Pin
#define ECHO_PIN_FL    A1   // Front Left Echo Pin
#define TRIGGER_PIN_L  A2    // Left Side Trigger Pin
#define ECHO_PIN_L     A3    // Left Side Echo Pin

#define MAX_DISTANCE   500  // Maximum distance for the ultrasonic sensors

#define R_motor_A 4
#define R_motor_B 7
#define L_motor_A 2
#define L_motor_B 3

#define R_motor_pwm 6
#define L_motor_pwm 5

NewPing sonarFL(TRIGGER_PIN_FL, ECHO_PIN_FL, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);


float p_wall , d_wall , error_wall ;
float i_wall = 0 ;
float prev_error_wall = 0 ;
float KP_wall = 2.5 ;  //3.5
float KI_wall = 0 ;
float KD_wall = 5 ; //6

int setpoint = 15; 
int max_speed_wall = 150 ;
int motor_mid_speed_wall = 100 ;
int left_motor_speed_wall = motor_mid_speed_wall ;
int right_motor_speed_wall = motor_mid_speed_wall ;

int IR_val[6] = {0,0,0,0,0,0} ;
int IR_weight[6] = {-20,-10,-5,5,10,20};

int max_speed = 150 ;
int motor_mid_speed = 100 ;
int left_motor_speed = motor_mid_speed ;
int right_motor_speed = motor_mid_speed ;

int x=0;

float p , d , error ;
float i = 0 ;
float prev_error = 0 ;
float KP = 3.5 ;
float KI = 0 ;
float KD = 10 ;

int a,b,c,e,f,g ;

  
void setup() {
  Serial.begin(9600) ;
  pinMode(A4, INPUT) ;
  pinMode(12, INPUT) ;
  pinMode(11, INPUT) ;   
  pinMode(10, INPUT) ;
  pinMode(9, INPUT) ;
  pinMode(8, INPUT) ;


  pinMode(2, OUTPUT) ;
  pinMode(3, OUTPUT) ;
  pinMode(4, OUTPUT) ;
  pinMode(7, OUTPUT) ;
  pinMode(5, OUTPUT) ;
  pinMode(6, OUTPUT) ;

  set_motor_speed()  ;
  forward() ;
  delay(700) ;
  
}

void loop() {

  while(1){
//if x=2 end of speeding stage, now wall following
  
  IR_val[0] = digitalRead(A4) ;                        //right most
  IR_val[1] = digitalRead(12) ;
  IR_val[2] = digitalRead(11) ;     
  IR_val[3] = digitalRead(10) ;
  IR_val[4] = digitalRead(9) ;
  IR_val[5] = digitalRead(8) ;                     // left most

   


    if ( (IR_val[2] == 0 || IR_val[3] == 0) && IR_val[4] == 0 && IR_val[5] == 0 ){ 
  //if ( IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){           //  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){ - all white - so can be a T junction, 4 way junction or end of maze
      stop() ;                                                                          // also checked left turn only   - may have to put or for val[2],val[3]
   
    delay(250);
    forward();
    analogWrite(R_motor_pwm, 120) ;
    analogWrite(L_motor_pwm, 120) ;
    delay(400);

    stop();//
    delay(200); ///// checking    delay(500)
    

  a= digitalRead(A4) ;               //right
  b = digitalRead(12) ;  
  c = digitalRead(11) ;    
  e = digitalRead(10) ;
  f = digitalRead(9) ;
  g = digitalRead(8) ;               //left

  delay(100);

  if (a==0 && b==0 && c==0 && e==0 && f==0 && g==0){   //if (a==0 && b==0 && c==0 && e==0 && f==0 && g==0){
    x = x+1;    //(x = 1 means end of exploration)
    stop();
    
  if (x ==1) {    // have to u turn

    delay(1000);
    //right 90

  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, LOW) ;
  analogWrite(R_motor_pwm, 100) ;
  

  delay(1000);

  stop();
  delay(200);

  //left (- 90)

  
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
  analogWrite(L_motor_pwm, 100) ;

  delay(1200);          // CHANGED THIS  1300

  forward();
     analogWrite(R_motor_pwm, 120) ;
    analogWrite(L_motor_pwm, 120) ;
  delay(200);  

       
//     left();
//    analogWrite(R_motor_pwm, 100) ;
//    analogWrite(L_motor_pwm, 100) ;
//    delay(1100);                  //CHECK HOW LONG FOR approximately 180 DEGREES
//    stop();
//     delay(200);
//     forward();

    //else{
      //forward();
    //}
    
    
  }

  if (x==2){
    forward();
    delay(1000);
    break;
  }
    
 
  }
  else {
     left();
     analogWrite(R_motor_pwm, 120) ;
     analogWrite(L_motor_pwm, 120) ;
     delay(420);                  //CHECK HOW LONG FOR approximately 90 DEGREES
     stop();
     delay(100);
     forward();
   
    }
  }


 else if (IR_val[0]==1 && IR_val[1]==1 && IR_val[2]==1 && IR_val[3]==1 && IR_val[4]==1 && IR_val[5]==1){     //u turn
    stop();
    delay(300);
    
//    forward();
//    analogWrite(R_motor_pwm, 120) ;
//    analogWrite(L_motor_pwm, 120) ;
//    delay(200);
//    stop();
//    delay(200);
//    left();
//    analogWrite(R_motor_pwm, 100) ;
//    analogWrite(L_motor_pwm, 100) ;
//    delay(1100);                  //CHECK HOW LONG FOR approximately 180 DEGREES

//NEW U TURN

//right 90

  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, LOW) ;
  analogWrite(R_motor_pwm, 100) ;
  

  delay(1000);

  stop();
  delay(200);

  //left (- 90)

  
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
  analogWrite(L_motor_pwm, 100) ;

  delay(1200);
//
//  forward();
//     analogWrite(R_motor_pwm, 120) ;
//    analogWrite(L_motor_pwm, 120) ;
//  delay(200);





    stop();
     delay(200);
     forward();
    
  }

  //for right

  else if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[5]==1){
    stop();
    delay(200);
    forward();
    analogWrite(R_motor_pwm, 120) ;
    analogWrite(L_motor_pwm, 120) ;
    delay(400);

    stop();//
    delay(200); ///// checking   delay(1000);
    

  a= digitalRead(A4) ;               //right
  b = digitalRead(12) ;  
  c = digitalRead(11) ;    
  e = digitalRead(10) ;
  f = digitalRead(9) ;
  g = digitalRead(8) ;               //left

  delay(100); // added now to check

   if (a==1 && b==1 && c==1 && e==1 && f==1 && g==1){
     right();
     analogWrite(R_motor_pwm, 120) ;
     analogWrite(L_motor_pwm, 120) ;
     delay(400);                  //CHECK HOW LONG FOR approximately 90 DEGREES
     stop();
     delay(100);
     forward();
    
   }
else{
  forward();
}
  
    
    
  }




      

  PID() ;
  set_motor_speed() ;
 
  
}


// now wall following

while(true){


delay(50);
  int distanceFL = sonarFL.ping_cm();
  int distanceL = sonarL.ping_cm();
  Serial.print(distanceFL);
  Serial.println("cm ");
  Serial.print(distanceL);
  Serial.println("cm");
 

  if (distanceFL==0 || distanceFL>60 ){
    distanceFL=60;
  }
  if (distanceFL < 30){
    left_motor_speed_wall =120;
    right_motor_speed_wall =70; 

    set_motor_speed_wall();   
    turnRight();
    delay(200);
    //PID();
    forward();
    
    //delay(200);
   // forward() ;
  }
   
  
  
  Serial.print(error_wall);
  Serial.print("  ");
  Serial.print(left_motor_speed_wall);
  Serial.print("  ");
  Serial.print(right_motor_speed_wall);
  Serial.print("  ");
  delay(100) ; 
  PID_wall() ;
  set_motor_speed_wall() ;


}
}

void forward() {
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
}

void reverse(){
    digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, HIGH) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
  
}

void left() {
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
}

void right() {
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, HIGH) ;
  digitalWrite(L_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
}

void stop () {
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_A, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
}

void set_motor_speed() {
  analogWrite(R_motor_pwm, right_motor_speed) ;
  analogWrite(L_motor_pwm, left_motor_speed) ;
   
  }
  
void PID() {
  error = 0 ;

  for (int i=0; i<=5; i++) {
    error += IR_val[i]*IR_weight[i] ;
    }

  p = error ;
  i = i + error ;
  d = error - prev_error ;
  prev_error = error ;

  float PID_val = (p*KP + i*KI + d*KD) ;
  //int PID_val_int = round(PID_val);

  right_motor_speed = motor_mid_speed - PID_val ;
  left_motor_speed = motor_mid_speed + PID_val ;

  if (right_motor_speed < 0)         {right_motor_speed = 0 ; }
  if (right_motor_speed > max_speed) {right_motor_speed = max_speed ; }
  if (left_motor_speed < 0)          {left_motor_speed = 0 ; }
  if (left_motor_speed > max_speed)  {left_motor_speed = max_speed ; }
 
  }


  void PID_wall(){
  delay(50);
  int distanceL = sonarL.ping_cm();
  if (distanceL==0 || distanceL>60 ){
    distanceL=60;
  }
  error_wall = setpoint - distanceL ;
  i_wall = i_wall + error_wall ;
  d_wall = error_wall - prev_error_wall ;
  prev_error_wall = error_wall ;  
  
   
  
  float PID_val_wall = (error_wall*KP_wall + i_wall*KI_wall + d_wall*KD_wall) ;

  PID_val_wall=constrain(PID_val_wall,-50,50);

  Serial.println(PID_val_wall);
  right_motor_speed_wall = motor_mid_speed_wall - PID_val_wall ;
  left_motor_speed_wall = motor_mid_speed_wall + PID_val_wall ;

  /*if (right_motor_speed_wall < 0)         {right_motor_speed_wall = 0 ; }
  if (left_motor_speed_wall < 0)          {left_motor_speed_wall = 0 ; }
  if (right_motor_speed_wall > max_speed) {right_motor_speed_wall = max_speed ; }
  if (left_motor_speed_wall > max_speed)  {left_motor_speed_wall = max_speed ; }
  */
}




void turnRight(){
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, HIGH) ;
  digitalWrite(L_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
}

void set_motor_speed_wall() {
  analogWrite(R_motor_pwm, right_motor_speed_wall) ;
  analogWrite(L_motor_pwm, left_motor_speed_wall) ;

}
