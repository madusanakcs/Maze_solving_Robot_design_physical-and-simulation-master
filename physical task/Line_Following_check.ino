
#define R_motor_A 2                    //  4,7,2,3,   6,5    - when castor is in front
#define R_motor_B 3                    //  3,2,7,4,   5,6    - when castor is behind ? 
#define L_motor_A 4
#define L_motor_B 7

// void forward() {
//  digitalWrite(R_motor_A, HIGH) ;
//  digitalWrite(R_motor_B, LOW) ;
//  digitalWrite(L_motor_B, HIGH) ;
//  digitalWrite(L_motor_A, LOW) ;

#define R_motor_pwm 5
#define L_motor_pwm 6

int IR_val[6] = {0,0,0,0,0,0} ;
int IR_weight[6] = {-20,-10,-5,5,10,20};

int max_speed = 150 ;
int motor_mid_speed = 100 ;
int left_motor_speed = motor_mid_speed ;
int right_motor_speed = motor_mid_speed ;

float p , d , error ;
float i = 0 ;
float prev_error = 0 ;
float KP = 5.5 ;
float KI = 0 ;
float KD = 10 ;

  
void setup() {
  Serial.begin(9600) ;
  pinMode(13, INPUT) ;
  pinMode(12, INPUT) ;
  pinMode(11, INPUT) ;   //12?
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
  delay(500) ;
  
}

void loop() {
  IR_val[0] = digitalRead(8) ;

  
  IR_val[1] = digitalRead(9) ;

  IR_val[2] = digitalRead(10) ;   

  
  IR_val[3] = digitalRead(11) ;

  
  IR_val[4] = digitalRead(12) ;

  
  IR_val[5] = digitalRead(13) ;



  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){
      stop() ;
      while(1){}
      }

  PID() ;
  set_motor_speed() ;

  
}

void forward() {
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
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
  int PID_val_int = round(PID_val);

  right_motor_speed = motor_mid_speed - PID_val ;
  left_motor_speed = motor_mid_speed + PID_val ;

  if (right_motor_speed < 0)         {right_motor_speed = 0 ; }
  if (right_motor_speed > max_speed) {right_motor_speed = max_speed ; }
  if (left_motor_speed < 0)          {left_motor_speed = 0 ; }
  if (left_motor_speed > max_speed)  {left_motor_speed = max_speed ; }
 
  }
