#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 64
#define timeStep 64
#define MAX_SPEED 2
#define IR_SENSOR_NUM 6
#define OBSTACLE_THRESHOLD 120
#define Max_Speed 3
#define DESIRED_DISTANCE 10


using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // Get the motor devices
  Motor *left_motor = robot->getMotor("leftwheelM");
  Motor *right_motor = robot->getMotor("rightwheelM");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
  
  
  Motor *servo = robot->getMotor("arm_hrightM");
  Motor *servo1 = robot->getMotor("arm_hleftM");
  Motor *servoh = robot->getMotor("arm_verticalM");
  
  
  
  DistanceSensor * USFL = robot->getDistanceSensor("US_front_left");
 USFL->enable(TIME_STEP);
  
 DistanceSensor * USF_low = robot->getDistanceSensor("US_front_low");
 USF_low->enable(TIME_STEP);

 DistanceSensor * USFR = robot->getDistanceSensor("US_front_right");
USFR->enable(TIME_STEP);
  
DistanceSensor * USAL = robot->getDistanceSensor("US_arm_left");
USAL->enable(TIME_STEP);

DistanceSensor * USAR = robot->getDistanceSensor("US_arm_right");
USAR->enable(TIME_STEP);

DistanceSensor * USL1 = robot->getDistanceSensor("US_left1");
USL1->enable(TIME_STEP);

  DistanceSensor * USR1 = robot->getDistanceSensor("US_right1");
USL1->enable(TIME_STEP);
  
  
  DistanceSensor *ToF = robot->getDistanceSensor("laser");
  ToF->enable(TIME_STEP);

  DistanceSensor *ToF2 = robot->getDistanceSensor("laser2");
  ToF2->enable(TIME_STEP);

  DistanceSensor * USF = robot->getDistanceSensor("US_front");
  USF->enable(TIME_STEP);

  DistanceSensor * USB = robot->getDistanceSensor("US_back");
  USB->enable(TIME_STEP);

  DistanceSensor *irc = robot->getDistanceSensor("irchess");
  irc->enable(TIME_STEP);

  DistanceSensor *ir3 = robot->getDistanceSensor("ir3");
  ir3->enable(TIME_STEP);

  DistanceSensor *ir5b = robot->getDistanceSensor("ir5b");
  ir5b->enable(TIME_STEP);
  
  
  
  
  
    //US SENSORS
  DistanceSensor *frontSensor = robot->getDistanceSensor("US_front");
  DistanceSensor *leftSensor = robot->getDistanceSensor("US_left1");
  DistanceSensor *rightSensor = robot->getDistanceSensor("US_right1");
  frontSensor->enable(TIME_STEP);
  leftSensor->enable(TIME_STEP);
  rightSensor->enable(TIME_STEP);


  DistanceSensor *irSensor[IR_SENSOR_NUM];
  char irSensorName[IR_SENSOR_NUM][10] = {"ir1", "ir2", "ir3", "ir4", "ir5", "ir6"};

  for (int i = 0; i < IR_SENSOR_NUM; i++)
  {
    irSensor[i] = robot->getDistanceSensor(irSensorName[i]);
    irSensor[i]->enable(TIME_STEP);
  }

  // PID constants
  double kp = 1;
  double ki = 0.001;
  double kd = 0.01;
  double integral = 0.01;
  double last_error = 0.0;
  char colour = 'r';


 ///STARTING SQUARE TRAVERSE
  double leftSpeed = MAX_SPEED;
  double rightSpeed = MAX_SPEED;
  left_motor->setVelocity(leftSpeed);
  right_motor->setVelocity(rightSpeed); 
  robot-> step(2000);
    
  //STARTING LINE FOLLOW////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(TIME_STEP) != -1) 
  {
    double sensorValue[IR_SENSOR_NUM];
    double error = 0.0;

    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
      sensorValue[i] = irSensor[i]->getValue();
      
      if (sensorValue[i] > 600){sensorValue[i] = 1;}
      else{sensorValue[i]=0;}
      //std::cout << sensorValue[i]<< std::endl;
      error += sensorValue[i] * (i - 2.5);
    }
    std::cout << sensorValue[0] << " " <<sensorValue[1]<< " "  <<sensorValue[2] << " " <<sensorValue[3]  << " "<<sensorValue[4] << " " <<sensorValue[5] << std::endl ;
    
    if ((sensorValue[0] && sensorValue[1] && sensorValue[2] && sensorValue[3] && sensorValue[4] && sensorValue[5])){break;}//all black

    // PID controller
    integral += error;
    double derivative = error - last_error;
    double output = kp * error + ki * integral + kd * derivative;
    last_error = error;


         //std::cout << output << std::endl;
    // Set motor speeds based on PID output
     leftSpeed = MAX_SPEED + output;
     rightSpeed = MAX_SPEED - output;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
     //std::cout << "leftSpeed" << leftSpeed << " rightSpeed " << rightSpeed << std::endl;  
  }
  
  
  //WALL FOLLOW///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while (robot->step(TIME_STEP) != -1)
    
     {
    double sensorValue[IR_SENSOR_NUM];
    double sensorVal[IR_SENSOR_NUM];
    std::cout << "wall follow"<< std::endl;
    //std::cout << sensorValue[0] << " " <<sensorValue[1]<< " "  <<sensorValue[2] << " " <<sensorValue[3]  << " "<<sensorValue[4] << " " <<sensorValue[5] << std::endl ;
    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++)
     {
      sensorVal[i] = irSensor[i]->getValue();
      sensorValue[i] = irSensor[i]->getValue();
     
      if (sensorValue[i] > 700){sensorValue[i] = 1;}
      else{sensorValue[i]=0;} 
    }
    double frontReading = frontSensor->getValue();
    double leftReading = leftSensor->getValue();
    double rightReading = rightSensor->getValue();
    //all white?
    if(!(sensorValue[0] || sensorValue[1] || sensorValue[2] || sensorValue[3] || sensorValue[4] || sensorValue[5])){
    leftSpeed = 0;
    rightSpeed = 0;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed); 
    robot-> step(2000);      
    break;
    }
    // if there is an obstacle in front, turn left
    if (frontReading > 0 && frontReading < OBSTACLE_THRESHOLD) {
      if (leftReading > 0 && leftReading < OBSTACLE_THRESHOLD) {
        
        left_motor->setVelocity(MAX_SPEED);
        right_motor->setVelocity(-MAX_SPEED);
      }
      // if there is an obstacle on the right, turn left
      else if (rightReading > 0 && rightReading < OBSTACLE_THRESHOLD) {
        left_motor->setVelocity(-MAX_SPEED);
        right_motor->setVelocity(MAX_SPEED);
      }
      
      else {
        left_motor->setVelocity(-MAX_SPEED);
        right_motor->setVelocity(MAX_SPEED);
      }
    }
    // if there is an obstacle on the left, turn right
    else if (leftReading > 0 && leftReading < OBSTACLE_THRESHOLD) {
      left_motor->setVelocity(MAX_SPEED);
      right_motor->setVelocity(MAX_SPEED / 2.0);
    }
    // if there is an obstacle on the right, turn left
    else if (rightReading > 0 && rightReading < OBSTACLE_THRESHOLD) {
      left_motor->setVelocity(MAX_SPEED / 2.0);
      right_motor->setVelocity(MAX_SPEED);
    }
    // if there are no obstacles, go straight
    else {
      left_motor->setVelocity(MAX_SPEED);
      right_motor->setVelocity(MAX_SPEED);
    }
   }
   
   ///SQURE PASS //////////////////////////////////////////////////////////////////////////////////
  
    leftSpeed = MAX_SPEED ;
     rightSpeed = MAX_SPEED ;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
    robot-> step(5000); 


 /// LINE FOLLOW///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(TIME_STEP) != -1)
   {
    double sensorValue[IR_SENSOR_NUM];
    double sensorVal[IR_SENSOR_NUM];
    double error = 0.0;

    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++)
     {
      sensorVal[i] = irSensor[i]->getValue();
      sensorValue[i] = irSensor[i]->getValue();
      

      if (sensorValue[i] > 700){sensorValue[i] = 1;}
      else{sensorValue[i]=0;} 
      
      error += sensorValue[i] * (i - 2.5);
    }
    //std::cout << sensorValue[0] << " " <<sensorValue[1]<< " "  <<sensorValue[2] << " " <<sensorValue[3]  << " "<<sensorValue[4] << " " <<sensorValue[5] << std::endl ;
    //std::cout << sensorVal[0] << " " <<sensorVal[1]<< " "  <<sensorVal[2] << " " <<sensorVal[3]  << " "<<sensorVal[4] << " " <<sensorVal[5] << std::endl ;
    
    if (!(sensorValue[0] || sensorValue[1] || sensorValue[2] || sensorValue[3] || sensorValue[4] || sensorValue[5]))
    {
    leftSpeed = 0;
    rightSpeed = 0;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed); 
    robot-> step(2000);   
    break;
    }
    // PID controller
    integral += error;
    double derivative = error - last_error;
    double output = kp * error + ki * integral + kd * derivative;
    last_error = error;


        // std::cout << output << std::endl;
    // Set motor speeds based on PID output
    leftSpeed = MAX_SPEED + output;
    rightSpeed = MAX_SPEED - output;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
     //std::cout << "leftSpeed" << leftSpeed << " rightSpeed " << rightSpeed << std::endl;

  }
  
  // COLOUR DETECT AND TURN TO THE SIDE/////////////////////////////////////////////////////////////////////////////////
  double ir_left = irSensor[5]->getValue();
  double ir_right = irSensor[0]->getValue();
  if (colour == 'r')
  {
  if (ir_left<300)
  {
    leftSpeed = -MAX_SPEED ;
    rightSpeed = MAX_SPEED ;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
    robot-> step(2000);
  }
  else
  {
    leftSpeed = MAX_SPEED ;
    rightSpeed = -MAX_SPEED ;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
    robot-> step(2000);
  }
  } 
  
  else 
   {
   if ((ir_left<550)&&(ir_left>400))
   {
     leftSpeed = -MAX_SPEED ;
     rightSpeed = MAX_SPEED ;
     left_motor->setVelocity(leftSpeed);
     right_motor->setVelocity(rightSpeed);
     robot-> step(2000);
   }
   else
   {
     leftSpeed = MAX_SPEED ;
     rightSpeed = -MAX_SPEED ;
     left_motor->setVelocity(leftSpeed);
     right_motor->setVelocity(rightSpeed);
     robot-> step(2000);
   }  
   }




// COLOURED LINE FOLLOWING ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(TIME_STEP) != -1) 
  {
    double sensorValue[IR_SENSOR_NUM];
    double error = 0.0;

    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
      sensorValue[i] = irSensor[i]->getValue();
      
      if (sensorValue[i] > 600){sensorValue[i] = 1;}
      else{sensorValue[i]=0;}
     // std::cout << sensorValue[i]<< std::endl;
      error += sensorValue[i] * (i - 2.5);
    }
    bool all_white = !(sensorValue[0] || sensorValue[1] || sensorValue[2] || sensorValue[3] || sensorValue[4] || sensorValue[5]);
    bool all_black = (sensorValue[0] && sensorValue[1] && sensorValue[2] && sensorValue[3] && sensorValue[4] && sensorValue[5]);
    
    if (all_white)
    {
     left_motor->setVelocity(0);
     right_motor->setVelocity(0);
     robot-> step(5000);
     break;
    }
    //std::cout << sensorValue[0] << " " <<sensorValue[1]<< " "  <<sensorValue[2] << " " <<sensorValue[3]  << " "<<sensorValue[4] << " " <<sensorValue[5] << std::endl ;
    // PID controller
    integral += error;
    double derivative = error - last_error;
    double output = kp * error + ki * integral + kd * derivative;
    last_error = error;
    
    if(ir_right<700)
    {
     leftSpeed = MAX_SPEED ;
     rightSpeed = -MAX_SPEED ;
     left_motor->setVelocity(leftSpeed);
     right_motor->setVelocity(rightSpeed);
     
    }

    else if(ir_left<700)
    {
     leftSpeed = -MAX_SPEED ;
     rightSpeed = MAX_SPEED ;
     left_motor->setVelocity(leftSpeed);
     right_motor->setVelocity(rightSpeed);
     
    }    
    
        // std::cout << output << std::endl;
    // Set motor speeds based on PID output
    leftSpeed = MAX_SPEED + output;
    rightSpeed = MAX_SPEED - output;
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
     //std::cout << "leftSpeed" << leftSpeed << " rightSpeed " << rightSpeed << std::endl;

  }

//// CHESS BOARD TRAVERSE ////////////////////////////////////////////////////////////////////////////////////////////######################################+++++++++++++++++++++++++++++++++++++++++++++++++++
//                                            SIDE LINE 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

servoh->setPosition(0.0);
  robot->step(1000);
   servo->setPosition(-0.01); // set to 90 degrees
  servo1->setPosition(-0.01);
     robot->step(1000);
while (robot->step(64) != -1){
      
  // set the position of the servo motor
  std::cout << "chess board"<< std::endl;
   if ((USF->getValue())/10 < 1) {
   left_motor->setVelocity(0);
   right_motor->setVelocity(0);
  robot->step(1000);
   servo->setPosition(0.015); // set to 90 degrees
  servo1->setPosition(0.015);
   robot->step(1000); 
  servoh->setPosition(0.1);
  robot->step(1000);
  
  
                 break;
  }
  
  else{ 
            left_motor->setVelocity(Max_Speed);
            right_motor->setVelocity(Max_Speed);
            //robot->step(7000);
            
            }





};


                  left_motor->setVelocity(2.0); // clockwise rotation
                 right_motor->setVelocity(2.0);  // counterclockwise rotation
                 robot->step(5000); 




                  left_motor->setVelocity(1.0); // clockwise rotation
                 right_motor->setVelocity(-1.0);  // counterclockwise rotation
                 robot->step(7400); // ad

 const double B= (USB-> getValue())/10;

double k1 =0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controll
  while (robot->step(timeStep) != -1) {
    
   const double ToF_val = (ToF-> getValue())/2;
   const double ToF_val2 = (ToF2-> getValue())/2;
   const double irc_val = irc-> getValue();
   const double usf_val = (USF-> getValue())/10;
    const double usb_val = (USB-> getValue())/10;
     const double ir3_val = ir3-> getValue();
    std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<"      " <<usb_val<<std::endl;
 
 
 
 
    
    if (ToF_val<500){
    
          if(ToF_val2>ToF_val-10 ){
                left_motor->setVelocity(0);
                right_motor->setVelocity(0);
                robot->step(1000);
                
                 left_motor->setVelocity(2.0); // clockwise rotation
                 right_motor->setVelocity(2.0); 
                  robot->step(1500);
                
                  left_motor->setVelocity(-1.0); // clockwise rotation
                 right_motor->setVelocity(1.0);  // counterclockwise rotation
                 robot->step(7500); // adjust the duration as necessary
            
              // stop the motors
                  left_motor->setVelocity(0.0);
                  right_motor->setVelocity(0.0);
                  robot->step(1000);
                  
                  
                  
                  
                   while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                 const double usf_val = (USF-> getValue())/10;
                                 
                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                                  
    
                                          
                                          if  ( usf_val<20  )
                                          
                                       
                                          
                                                  
                                                  {if (irc_val<700)  //white chase piese 
                                                  
                                                         { left_motor->setVelocity(0);
                                                            right_motor->setVelocity(0);
                                                            robot->step(4000);
                                                            
                                              
                                                            
                                                              left_motor->setVelocity(-1.0); // clockwise rotation
                                                             right_motor->setVelocity(1.0);  // counterclockwise rotation
                                                                      
                                                            // run the simulation for some time to let the motors rotate
                                                            robot->step(7500*2); // adjust the duration as necessary
                                                          
                                                            // stop the motors
                                                            left_motor->setVelocity(0.0);
                                                            right_motor->setVelocity(0.0);
                                                            robot->step(1000);
                                                            
                                                            
                                                             while (robot->step(timeStep) != -1  )
                                                                                  { 
                                                                        const double irc_val = irc-> getValue();
                                                                       const double usf_val = (USF-> getValue())/10;
                                                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                                                     
                                                                         left_motor->setVelocity(Max_Speed);
                                                                        right_motor->setVelocity(Max_Speed);
                                                                                  if  (usf_val < 50){break;}
                                                                                  }
                                              
                                              
                                                                // place the box
                                                                        
                                                             left_motor->setVelocity(0);
                                                             right_motor->setVelocity(0);
                                                            robot->step(4000);
                                                            
                                                            
                                                            
                                                             left_motor->setVelocity(-1.0); // clockwise rotation
                                                             right_motor->setVelocity(1.0);  // counterclockwise rotation
                                                                      
                                                            // run the simulation for some time to let the motors rotate
                                                            robot->step(7340*2); // adjust the duration as necessary
                                                          
                                                            // stop the motors
                                                            left_motor->setVelocity(-2.0);
                                                            right_motor->setVelocity(-2.0);
                                                            robot->step(4000);
                                                            
                                                            
                                                            
                                                            
                                                            
                                                             servoh->setPosition(0);
                                                          
                                                             robot->step(1000); 
                                                             
                                                             servo->setPosition(0); 
                                                            servo1->setPosition(0);
                                                            
                                                            robot->step(1000); 
                                                            
                                                            
                                                             left_motor->setVelocity(-2.0);
                                                            right_motor->setVelocity(-2.0);
                                                            robot->step(3500);
                                                            
                                                            
                                                            
                                                                     
                                                
                                                              left_motor->setVelocity(1.0); // clockwise rotation
                                                             right_motor->setVelocity(-1.0);  // counterclockwise rotation
                                                        
                                                          // run the simulation for some time to let the motors rotate
                                                              robot->step(7000); // adjust the duration as necessary
                                                        
                                                           while (robot->step(timeStep) != -1  )
                                                                                  { 
                                                                        const double irc_val = irc-> getValue();
                                                                       const double usf_val = (USF-> getValue())/10;
                                                                       const double ir3_val = ir3-> getValue();
                                                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                                                     
                                                                         left_motor->setVelocity(Max_Speed);
                                                                        right_motor->setVelocity(Max_Speed);
                                                                                  if  (usf_val < 15){
                                                                                  
                                                                                        k1=1;
                                                                                         goto end_sideline;
                                                                                         left_motor->setVelocity(-1.0); // clockwise rotation
                                                                                         right_motor->setVelocity(1.0);
                                                                                         robot->step(7340); 
                                                                                         
                                                                        
                                                                                         
                                                                                           left_motor->setVelocity(Max_Speed);
                                                                                        right_motor->setVelocity(Max_Speed);
                                                                                        
                                                                                         if (ir3_val>800){
                                                                                 
                                                                                       left_motor->setVelocity(0);
                                                                                        right_motor->setVelocity(0);
                                                                                         robot->step(3000);
                                                                                           }
                                                                                  
                                                                                  
                                                                                 }
                                                                                  }
                                                            
                                                            
                                                  
                                                            
                                                          //  servoh->setPosition(0.1);
                                              
                                              //              robot->step(1000);   ///////////////////            ---------->>>>>>>>        GO TO THE DOOR
                                              
                                                              }
                                                      else
                                          
                                          
                                                            {
                                                            
                                                             { left_motor->setVelocity(0);
                                                                  right_motor->setVelocity(0);
                                                                  robot->step(4000);
                                                                  
                                                    
                                                                  
                                                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                                                              
                                                                // run the simulation for some time to let the motors rotate
                                                                robot->step(7500*2); // adjust the duration as necessary
                                                              
                                                                // stop the motors
                                                                left_motor->setVelocity(0.0);
                                                                right_motor->setVelocity(0.0);
                                                                robot->step(1000);
                                                               
                                                                
                                                                 while (robot->step(timeStep) != -1  )
                                                                                { 
                                                                                  const double irc_val = irc-> getValue();
                                                                                 const double usf_val = (USF-> getValue())/10;
                                                                                  std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                                                               
                                                                                   left_motor->setVelocity(Max_Speed);
                                                                                  right_motor->setVelocity(Max_Speed);
                                                                                            if  (usf_val < 50){break;}
                                                                                            }
                                                                
                                                                left_motor->setVelocity(0);
                                                                  right_motor->setVelocity(0);
                                                                  robot->step(1000);
                                                                  
                                                                   left_motor->setVelocity(2.0); // clockwise rotation
                                                                   right_motor->setVelocity(2.0); 
                                                                    robot->step(2000);
                                                                  
                                                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                                                              
                                                                // run the simulation for some time to let the motors rotate
                                                               
                                                          robot->step(7500); // adjust the duration as necessary
                                                                     // stop the motors
                                                                left_motor->setVelocity(0.0);
                                                                right_motor->setVelocity(0.0);
                                                                robot->step(1000);
                                                                
                                                                left_motor->setVelocity(2.0);
                                                                right_motor->setVelocity(2.0);
                                                                robot->step(1000);
                                                                break;
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                }
                                                                
                                                                
                                                                }
                                                                
                                              
                                
                                          
                                          }
                                      
                                                                      
                                                                      
                                      
                                      
                                      
                                      //////////////////////////////////////////////////////*********************************************
                                                  
                                                  
                                                  
                                                  
                                                  }
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  }
      

    
                  
    
                  
              else{  
              
              
              
          
              
                  left_motor->setVelocity(Max_Speed);
                  right_motor->setVelocity(Max_Speed);
                  
                  }
                  
    //const double ToF_val = 1000;
    }
    
    else{ 
        
        if (usf_val<15 or ir3_val>320 ){
        
         left_motor->setVelocity(0);
         right_motor->setVelocity(0);
        robot->step(1000);
        
          
          
                                                                 while (robot->step(timeStep) != -1  )
                                                                                { 
                                                                                  const double irc_val = irc-> getValue();
                                                                                 const double usb_val = (USB-> getValue())/10;
                                                                                  std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usb_val<<std::endl;
                                                                               
                                                                                   left_motor->setVelocity(-Max_Speed);
                                                                                  right_motor->setVelocity(-Max_Speed);
                                                                                            if  (usb_val < B-2){
                                                                                            
                                                                                             left_motor->setVelocity(1.0); // clockwise rotation
                                                                                           right_motor->setVelocity(-1.0);  // counterclockwise rotation
                                                              
                                                                                              robot->step(7400); // adjust the duration as necessary
                                                              
                                                                                            
                                                                                            break;}
                                                                                            
                                                                                            }
                                                                                            
                                                                                            
                                                                                            break;
          
          
          
     
                }
    
        else {
          left_motor->setVelocity(Max_Speed);
          right_motor->setVelocity(Max_Speed);
          }
          }
    
       //   left_motor->setVelocity(Max_Speed);
        //  right_motor->setVelocity(Max_Speed);
        
        /*
          
          if  ( usf_val<20  )
          
       
          
                  
                  {if (irc_val<700)  //white chase piese 
                  
                         { left_motor->setVelocity(0);
                            right_motor->setVelocity(0);
                            robot->step(4000);
                            
              
                            
                              left_motor->setVelocity(-1.0); // clockwise rotation
                             right_motor->setVelocity(1.0);  // counterclockwise rotation
                                      
                            // run the simulation for some time to let the motors rotate
                            robot->step(7450*2); // adjust the duration as necessary
                          
                            // stop the motors
                            left_motor->setVelocity(0.0);
                            right_motor->setVelocity(0.0);
                            robot->step(1000);
                            
                            
                             while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                       const double usf_val = (USF-> getValue())/10;
                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                                  if  (usf_val < 50){break;}
                                                  }
              
              
                                // place the box
                                        
                             left_motor->setVelocity(0);
                             right_motor->setVelocity(0);
                            robot->step(4000);
                            
                             servoh->setPosition(0);
                          
                             robot->step(1000); 
                             
                             servo->setPosition(0); 
                            servo1->setPosition(0);
                            
                            robot->step(1000);        
                                        
                            servoh->setPosition(0.1);
              
                            robot->step(1000);   ///////////////////            ---------->>>>>>>>        GO TO THE DOOR
              
                              }
                      else
          
          
                            {
                            
                             { left_motor->setVelocity(0);
                                  right_motor->setVelocity(0);
                                  robot->step(4000);
                                  
                    
                                  
                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                              
                                // run the simulation for some time to let the motors rotate
                                robot->step(7450*2); // adjust the duration as necessary
                              
                                // stop the motors
                                left_motor->setVelocity(0.0);
                                right_motor->setVelocity(0.0);
                                robot->step(1000);
                               
                                
                                 while (robot->step(timeStep) != -1  )
                                                { 
                                                  const double irc_val = irc-> getValue();
                                                 const double usf_val = (USF-> getValue())/10;
                                                  std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                               
                                                   left_motor->setVelocity(Max_Speed);
                                                  right_motor->setVelocity(Max_Speed);
                                                            if  (usf_val < 45){break;}
                                                            }
                                
                                left_motor->setVelocity(0);
                                  right_motor->setVelocity(0);
                                  robot->step(1000);
                                  
                                   left_motor->setVelocity(2.0); // clockwise rotation
                                   right_motor->setVelocity(2.0); 
                                    robot->step(2000);
                                  
                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                              
                                // run the simulation for some time to let the motors rotate
                                robot->step(7450); // adjust the duration as necessary
                              
                                // stop the motors
                                left_motor->setVelocity(0.0);
                                right_motor->setVelocity(0.0);
                                robot->step(1000);
                                
                                left_motor->setVelocity(2.0);
                                right_motor->setVelocity(2.0);
                                robot->step(1000);
                                
                                
                                }
                                
                                
                                }
                                
              

          
          }
      */
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };
  
  
  
  
  
  
  
  
  
  
 end_sideline:
               
 std::cout <<  "    cccccccccccccccc  " <<std::endl;
 
 
         
  
  
  
  
  
  
  
      int n=0;
      int a=0;
      int b=0;


 while (robot->step(timeStep) != -1) {
   if (k1==1){break;}
   const double ToF_val = (ToF-> getValue())/2;
   const double ToF_val2 = (ToF2-> getValue())/2;
   const double irc_val = irc-> getValue();
   const double usf_val = (USF-> getValue())/10;
   const double usb_val = (USB-> getValue())/10;
   const double ir3_val = ir3-> getValue();
   
     double L;
    std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
 
 
 
 
    
    if (ToF_val<500){
    
          if(ToF_val2>ToF_val-10 ){
                left_motor->setVelocity(0);
                right_motor->setVelocity(0);
                robot->step(1000);
                
                 left_motor->setVelocity(1.0); // clockwise rotation
                 right_motor->setVelocity(1.0); 
                  robot->step(2000);
                
                  left_motor->setVelocity(-1.0); // clockwise rotation
                 right_motor->setVelocity(1.0);  // counterclockwise rotation
            
              // run the simulation for some time to let the motors rotate
                  robot->step(7500); // adjust the duration as necessary
            
              // stop the motors
                  left_motor->setVelocity(0.0);
                  right_motor->setVelocity(0.0);
                  robot->step(1000);
      
                  
                  
                  
                   while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                       const double usf_val = (USF-> getValue())/10;
                                         const double ir3_val = ir3-> getValue();
                                          const double ir5b_val = ir5b-> getValue();
                                          double irValue1;
                                           
                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                        
/*
/////////////////////////////kotu/////////////////////////////////////

                                 //irValue1=  ir3_val;

                              if (ir3_val>500 &&    ir5b_val<500      )
                                      {n++;}
                                  else if (ir3_val<500 &&    ir5b_val>500  )   
                                        {n++;}
                                      
                                  
                                  
                                
                                  
                            std::cout   << n <<"  "<<ir3_val <<"  "<<ir5b_val << std::endl;
            

               ///////////////////////////////////////////////////////////                         
                                        */
                                                  if  (usf_val < 15){
                                                  
                                                    L=ToF_val;

                                                  break;}
                                                  }
   
                  }
    
                  
              else{  
              
              
              
    
                  left_motor->setVelocity(-Max_Speed);
                  right_motor->setVelocity(-Max_Speed);
                  
                  
                  }
                  
    //const double ToF_val = 1000;
    }
    
    else{ 
          left_motor->setVelocity(-Max_Speed);
          right_motor->setVelocity(-Max_Speed);
          }
    
          left_motor->setVelocity(-Max_Speed);
          right_motor->setVelocity(-Max_Speed);
          
          
          
          if (usb_val<8 or  ir3_val>320 ){
                  
                     left_motor->setVelocity(2.0); // clockwise rotation
                 right_motor->setVelocity(2.0); 
                  robot->step(7000);
                
                  left_motor->setVelocity(-1.0); // clockwise rotation
                 right_motor->setVelocity(1.0);  // counterclockwise rotation
            
              // run the simulation for some time to let the motors rotate
                  robot->step(7500*2);
                  
                  
                 
                 
                 
  while (robot->step(timeStep) != -1  )
        { 
                                   
   const double ir3_val = ir3-> getValue();
  
   
  left_motor->setVelocity(-Max_Speed);
  right_motor->setVelocity(-Max_Speed);
  
  std::cout<< ir3_val   <<std::endl;
  
  if (ir3_val>320)
                 {while (robot->step(timeStep) != -1) {
    
                         const double ToF_val = (ToF-> getValue())/2;
                          
                           const double usf_val = (USF-> getValue())/10;
                          
                         
                         
                           std::cout<< ToF_val   <<std::endl;
                           
                            if (ToF_val<500){
                            
                             left_motor->setVelocity(0);
                             right_motor->setVelocity(0);
                            robot->step(2000);
                            
                            left_motor->setVelocity(-2);
                             right_motor->setVelocity(-2);
                            robot->step(2000);
                            
                            left_motor->setVelocity(0);
                             right_motor->setVelocity(0);
                            robot->step(2000);
                            
                             servoh->setPosition(0);
                          
                             robot->step(1000); 
                             
                             servo->setPosition(0); 
                            servo1->setPosition(0);
                            
                            robot->step(1000);        
                                        
                            servoh->setPosition(0.1);
              
                            robot->step(1000);  
                            
                            break;
                            
                            
                            
                            
                            }
                          // break;
                         
                          
                          
                          
                          
                         if (usf_val<16 )
                        {
                        
    while (robot->step(timeStep) != -1  )
        { 
                                   
   const double ir3_val = ir3-> getValue();
  
   
  left_motor->setVelocity(-Max_Speed);
  right_motor->setVelocity(-Max_Speed);
  
  std::cout<<" giyaaaaaaa "  <<std::endl;
  
  if (ir3_val>320)
                        
             {
             
                             left_motor->setVelocity(2);
                             right_motor->setVelocity(2);
                            robot->step(6000);
                            
                            
                            left_motor->setVelocity(-1);
                             right_motor->setVelocity(1);
                            robot->step(7500);
                            
                             left_motor->setVelocity(2);
                             right_motor->setVelocity(2);
                            robot->step(1000);
                            
                            servoh->setPosition(0);
                          
                             robot->step(1000); 
                             
                             servo->setPosition(0); 
                            servo1->setPosition(0);
                            
                            robot->step(1000);        
                                        
                            servoh->setPosition(0.1);
              
                            robot->step(1000);  
                            
                            
                            
                            
                             left_motor->setVelocity(-2);
                             right_motor->setVelocity(-2);
                            robot->step(4000);
                            
                            
                            left_motor->setVelocity(1);
                             right_motor->setVelocity(-1);
                            robot->step(7500*2);
             
                       goto redline;
             
             
             
             
             
             }           
                        
                     }   
                        
                        
                        }
                        
                        
                        
                        
                        else{
                        
                         left_motor->setVelocity(Max_Speed);
                          right_motor->setVelocity(Max_Speed);
                        
                        }
                          
                          
                          
               }
               
               break;
                 }
 
                 //break;
                 
                 
                 
                  
                  
                   // adjust the duration as necessary
                  }
                  
                  break;
                  
                  }
          
          if  ( usf_val<20  )
          
       
          
                  
                  {if (irc_val<500)  //white chase piese 
                  
                         { left_motor->setVelocity(0);
                            right_motor->setVelocity(0);
                            robot->step(4000);
                            
                          double t =  2*2500*(L-37.5)/37.5;
                          
                          if (t>0){
                           std::cout   << t << std::endl;
                            left_motor->setVelocity(-Max_Speed);
                            right_motor->setVelocity(-Max_Speed);
                                robot->step(t);}
                           
                           
                           
                           
                           /* 
                             while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                       const double usf_val = (USF-> getValue())/10;
                                        std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                        
      
      
      //////////////////////////////////////////// KOTU //////////////////////
      
                                  double irValue1;
                                  double irValue;
                 
                 
                 
                                  irValue = ir3->getValue();
                                  
                                       left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                  
                                  
                                 if (irValue>500    &&    irValue1<500      )
                                      {a++;}
                                  else if (irValue<500 &&    irValue1>500  )   
                                        {a++;}
                                      
                                  
                                  
                                   irValue1=  irValue;
                                  
                            std::cout   << a << "    "<< irValue<< std::endl;
   
                                        
                                        
         /////////////////////////////////////////////////////////////////                               
                                        
                                        
                                  if  (n-1==a){break;}
                                                  }
              
              */
                                // place the box
                                
                                
                                
                                 // left_motor->setVelocity(2);
                                  //right_motor->setVelocity(2);
                                  //robot->step(5500);                         
                                                    
                                                    
                                                    
                                                    
                                                    
                                left_motor->setVelocity(0);
                                  right_motor->setVelocity(0);
                                  robot->step(1000);
                              
                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                              
                                // run the simulation for some time to let the motors rotate
                                robot->step(7700); // adjust the duration as necessary
                              
                                // stop the motors
                                left_motor->setVelocity(0.0);
                                right_motor->setVelocity(0.0);
                                robot->step(1000);
                                
                                            
                                            
                                  left_motor->setVelocity(-2.0);
                                right_motor->setVelocity(-2.0);
                                robot->step(4000);
                                    
                                                    
                                        
                             left_motor->setVelocity(0);
                             right_motor->setVelocity(0);
                            robot->step(4000);
                            
                             servoh->setPosition(0);
                          
                             robot->step(1000); 
                             
                             servo->setPosition(0); 
                            servo1->setPosition(0);
                            
                            robot->step(1000);        
                                        
                            servoh->setPosition(0.1);
              
                            robot->step(1000);  
                            
                            
                            
                            
                                
                                
                            
                            
                            
                            
                                         
             
                              break;
                            
                            
                            
                            
                            
                            
                            
                             ///////////////////            ---------->>>>>>>>        GO TO THE DOOR
              
                              }
                      else
          
          
                            {
                            
                             { left_motor->setVelocity(0);
                                  right_motor->setVelocity(0);
                                  robot->step(4000);
                                  
                                  
                                  double t =  2*3500*(L-20-37.5)/37.5;
                                  
                                  
                                  if (t>0){
                            left_motor->setVelocity(-Max_Speed);
                            right_motor->setVelocity(-Max_Speed);
                                robot->step(2*3500*(L-20-37.5)/37.5);}
                    
                                  /*
                                    left_motor->setVelocity(-1.0); // clockwise rotation
                                   right_motor->setVelocity(1.0);  // counterclockwise rotation
                              
                                // run the simulation for some time to let the motors rotate
                                robot->step(7450*2); // adjust the duration as necessary
                              
                                // stop the motors
                                left_motor->setVelocity(0.0);
                                right_motor->setVelocity(0.0);
                                robot->step(1000);
                               
                                
                                 while (robot->step(timeStep) != -1  )
                                                { 
                                                  const double irc_val = irc-> getValue();
                                                 const double usf_val = (USF-> getValue())/10;
                                                  std::cout << ToF_val<< "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                               
                                                   left_motor->setVelocity(Max_Speed);
                                                  right_motor->setVelocity(Max_Speed);
                                                  
                                                  
                                                  
                                                  
         //////////////////////////////////////////////////           KOTU ///////////////////////////////
         
                                   double irValue1;
                                  double irValue;
                             
                             
                 
                                  irValue = ir3->getValue();
                                  
                                       left_motor->setVelocity(Max_Speed);
                                    right_motor->setVelocity(Max_Speed);
                                  
                                  
                                 if (irValue>500    &&    irValue1<500      )
                                      {b++;}
                                  else if (irValue<500 &&    irValue1>500  )   
                                        {b++;}
                                      
                                  
                                  
                                   irValue1=  irValue;
                                  
                            std::cout   << b<< std::endl;
   
         
         
         
         
////////////////////////////////////////////////////////////////////////////////////////////////////////////         
                                                            if  (n==b){
                                                            
                                                            n=0;
                                                            break;}
                                                            }
                                                            
                                                            
                                     */   
                                
                                left_motor->setVelocity(0);
                                  right_motor->setVelocity(0);
                                  robot->step(1000);
                                  
                                   left_motor->setVelocity(2.0); // clockwise rotation
                                   right_motor->setVelocity(2.0); 
                                    robot->step(1000);
                                  
                                    left_motor->setVelocity(1.0); // clockwise rotation
                                   right_motor->setVelocity(-1.0);  // counterclockwise rotation
                              
                                // run the simulation for some time to let the motors rotate
                                robot->step(7500); // adjust the duration as necessary
                              
                                // stop the motors
                                left_motor->setVelocity(0.0);
                                right_motor->setVelocity(0.0);
                                robot->step(1000);
                                
                                left_motor->setVelocity(-2.0);
                                right_motor->setVelocity(-2.0);
                                robot->step(3000);
                                
                                
                                }
                                
                                
                                }
                                
              

          
          }
      
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };
  
  //left_motor->setVelocity(0.0);
  //right_motor->setVelocity(0.0);
  

             
  
  while (robot->step(timeStep) != -1  )
        {    if (k1==1){break;}
                                   
   const double ir3_val = ir3-> getValue();
  
   
  left_motor->setVelocity(-Max_Speed);
  right_motor->setVelocity(-Max_Speed);
  
  std::cout<< ir3_val   <<std::endl;
  
  if (ir3_val>320)
               {break;}
               
               
                 };
 
         if (k1==0){
               left_motor->setVelocity(2.0); // clockwise rotation
                 right_motor->setVelocity(2.0); 
                  robot->step(6000);
                
               left_motor->setVelocity(1.0); // clockwise rotation
                right_motor->setVelocity(-1.0);  // counterclockwise rotation
            
              // run the simulation for some time to let the motors rotate
                  robot->step(7500); // adjust the duration as necessary
            
            redline:
                
               left_motor->setVelocity(2.0); // clockwise rotation
                 right_motor->setVelocity(2.0); 
                  robot->step(9000);
                  
                  
                  
                                                            left_motor->setVelocity(1.0);
                                                            right_motor->setVelocity(-1.0);
                                                            robot->step(7500);
                                                            
                                                            
                                                            left_motor->setVelocity(2.0);
                                                            right_motor->setVelocity(2.0);
                                                            robot->step(6000);
                                                            
                                                            
                                                            
                                                            
                                                            
                                                            
                                                                     
                                                
                                                              left_motor->setVelocity(-1.0); // clockwise rotation
                                                             right_motor->setVelocity(1.0);  // counterclockwise rotation
                                                        
                                                          // run the simulation for some time to let the motors rotate
                                                              robot->step(7500); // adjust the duration as necessary
                  
                  
                  
                  }
                  
      
                  
                     std::cout << "   elaaaaaaa   " <<std::endl;
                  
                  
             
  
  
  
                                                     
                                                           while (robot->step(timeStep) != -1  )
                                                                                  { 
                                                                        const double irc_val = irc-> getValue();
                                                                       const double usf_val = (USF-> getValue())/10;
                                                                       const double ir3_val = ir3-> getValue();
                                                                       
                                                                      std::cout << irc_val<<"      " <<usf_val<<std::endl;
                                                                     
                                                                         left_motor->setVelocity(Max_Speed);
                                                                        right_motor->setVelocity(Max_Speed);
                                                                        
                                                                       
                                                                        
                                                                       
                                                                                  if  (usf_val < 9){
                                                                                  
                                                                                         left_motor->setVelocity(-1.0); // clockwise rotation
                                                                                         right_motor->setVelocity(1.0);
                                                                                         robot->step(7600); 
                                                                                         
                                                                        
                                                                                         
                                                                                           left_motor->setVelocity(Max_Speed);
                                                                                        right_motor->setVelocity(Max_Speed);
                                                                                        robot->step(2000);
                                                                                        
                                                                                      break;
                                                                                  }
                                                                                  }
                                                                                  
                                                                                  
                                    
                                    
                                    
                                    
  DistanceSensor *rightSensor1 = robot->getDistanceSensor("US_right1");
  DistanceSensor *rightSensor2 = robot->getDistanceSensor("US_right2");
  rightSensor1->enable(timeStep);
  rightSensor2->enable(timeStep);

  Motor *leftMotor = robot->getMotor("leftwheelM");
  Motor *rightMotor = robot->getMotor("rightwheelM");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // set up the PID controller with the desired parameters
   kp = 0.01; // proportional gain
   ki = 0.0000001; // integral gain
   kd = 0.0000001; // derivative gain
  double set_point = DESIRED_DISTANCE;
  double error_sum = 0;
   last_error = 0;
  double dt = timeStep / 1000.0;

  while (robot->step(timeStep) != -1) {
    double rightSensorValue1 = rightSensor1->getValue()/10;
    double rightSensorValue2 = rightSensor2->getValue()/10;
    const double ir3_val = ir3-> getValue();
    double actual_distance = (rightSensorValue1 + rightSensorValue2) / 2.0;

    // compute the error and update the error_sum
    double error = set_point - actual_distance;
    error_sum += error * dt;

    // compute the error derivative
    double error_derivative = (error - last_error) / dt;
    last_error = error;

    // compute the PID correction
    double correction = kp * error + ki * error_sum + kd * error_derivative;

    // compute the left and right motor speeds using the correction
    double leftSpeed = Max_Speed * 0.5 - correction;
    double rightSpeed = Max_Speed * 0.5 + correction;
    std::cout<<leftSpeed<<"   "<<rightSpeed<<"  "<<correction<<std::endl;

    // ensure the speeds are within the allowable range
    if (leftSpeed > Max_Speed) {
      leftSpeed = Max_Speed;
    }
    if (rightSpeed > Max_Speed) {
      rightSpeed = Max_Speed;
    }
    if (leftSpeed < 0) {
      leftSpeed = 0;
    }
    if (rightSpeed < 0) {
      rightSpeed = 0;
    }

    // set the motor speeds
    left_motor->setVelocity(leftSpeed);
    right_motor->setVelocity(rightSpeed);
 
                                    
                                    
                                    
                                    
                                    
                                                                                 
                                                                                 
                                                                                 if (ir3_val>500){
                                                                                 
                                                                                 left_motor->setVelocity(0);
                                                                                        right_motor->setVelocity(0);
                                                                                         robot->step(3000);
                                                                                 break;}
                                                                                 
                                                                                 
                                                                                  
  
  
  
  }
  
  
  
  //////////////////////     SECRET CHAMBER /////////////////////////////////////////
  
  
  
  servoh->setPosition(0.0);
  robot->step(1000);
   servo->setPosition(-0.01); // set to 90 degrees
  servo1->setPosition(-0.01);
     robot->step(1000);
     
 left_motor->setVelocity(2);
 right_motor->setVelocity(2);
 robot->step(3000);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
   
   
   //const double ToF_val = (ToF-> getValue())/2;
   const double ToF_val2 = (ToF2-> getValue())/2;
   const double irc_val = irc-> getValue();
   const double usf_val = (USF-> getValue())/10;
   const double usb_val = (USB-> getValue())/10;
    std::cout <<  "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<"      " <<usb_val<<std::endl;
   
     double temp;
     
     left_motor->setVelocity(Max_Speed);
     right_motor->setVelocity(Max_Speed);
     
     
     
     
     
       if  ( usf_val<1  )
                                          
                                          
                                          {
                                          
                                          
                                                                                    
                                             left_motor->setVelocity(0);
                                             right_motor->setVelocity(0);
                                            robot->step(1000);
                                             servo->setPosition(0.017); // set to 90 degrees
                                            servo1->setPosition(0.017);
                                             robot->step(1000); 
                                            servoh->setPosition(0.1);
                                            robot->step(1000);
                                            goto goout;
                                            }
                                            
                                            else{ 
                                                      left_motor->setVelocity(Max_Speed);
                                                      right_motor->setVelocity(Max_Speed);
                                                      //robot->step(7000);
                                                      
                                                                                    
                                                                                    
                                                                                    
                                                                                    
                                          }
     
     
     
     
     
     
     
     if  (ToF_val2<temp-5){
     
                 
     
                left_motor->setVelocity(2);
                right_motor->setVelocity(2);
                robot->step(2000);
                
          
                  left_motor->setVelocity(-1.0); // clockwise rotation
                 right_motor->setVelocity(1.0);  // counterclockwise rotation
            
              // run the simulation for some time to let the motors rotate
                  robot->step(7450); // adjust the duration as necessary
            
              // stop the motors
                  left_motor->setVelocity(0.0);
                  right_motor->setVelocity(0.0);
                  robot->step(1000);
                  
                  
                  
                  
                   while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                 const double usf_val = (USF-> getValue())/10;
                                 
                                        std::cout <<  "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                                  
    
                                          
                                          if  ( usf_val<1  )
                                          
                                          
                                          {
                                          
                                          
                                                                                    
                                             left_motor->setVelocity(0);
                                             right_motor->setVelocity(0);
                                            robot->step(1000);
                                             servo->setPosition(0.015); // set to 90 degrees
                                            servo1->setPosition(0.015);
                                             robot->step(1000); 
                                            servoh->setPosition(0.1);
                                            robot->step(1000);
                                            break;
                                            }
                                            
                                            else{ 
                                                      left_motor->setVelocity(Max_Speed);
                                                      right_motor->setVelocity(Max_Speed);
                                                      //robot->step(7000);
                                                      
                                                                                    
                                                                                    
                                                                                    
                                                                                    
                                          }
     
                                 }
     
     
     
     
     
     
     
     
     
     
     break;
     
     
     
     
     }
     
     std::cout <<  "      " <<temp<<"      " <<ToF_val2<<"      " <<usf_val<<std::endl;
     temp=ToF_val2;
     
      
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };
  
  
  

  
   while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                 const double usb_val = (USB-> getValue())/10;
                                 
                                       // std::cout <<  "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(-Max_Speed);
                                        right_motor->setVelocity(-Max_Speed);
                                                  
    
                                          
                                          if  ( usb_val<10  )
                                          
                                          
                                          {
                                           left_motor->setVelocity(1);
                                             right_motor->setVelocity(-1);
                                               robot->step(7450);
                                               break;
                                            
                                            }
                                            
                                            
 };
  
    goout:
  
                while (robot->step(timeStep) != -1  )
                                                  { 
                                        const double irc_val = irc-> getValue();
                                 const double usf_val = (USF-> getValue())/10;
                                 
                                       // std::cout <<  "      " <<ToF_val2<<"      " <<irc_val<<"      " <<usf_val<<std::endl;
                                     
                                         left_motor->setVelocity(Max_Speed);
                                        right_motor->setVelocity(Max_Speed);
                                                  
    
                                          
                                          if  ( usf_val<11  )
  
                                                      {
                                              left_motor->setVelocity(1);
                                             right_motor->setVelocity(-1);
                                               robot->step(7450);
                                                      
                                             left_motor->setVelocity(-1);
                                             right_motor->setVelocity(-1); 
                                                       robot->step(5000);
                                                       
                                               left_motor->setVelocity(1);
                                             right_motor->setVelocity(2); 
                                                       robot->step(2500);
                                                       
                                                       
                                               left_motor->setVelocity(2);
                                             right_motor->setVelocity(1); 
                                                       robot->step(2500);        
                                                       
                                                       
                                                left_motor->setVelocity(1);
                                             right_motor->setVelocity(1); 
                                                       robot->step(10000);       
                                                            break;
                                                            }
  
  
  };
  // Enter here exit cleanup code.
  
  
  
  
  
 /////////////////////////////////  LINE FOLLOWING ////////////////////////////////
  
  
  
  
  
  
  
#define TIME_STEP 16
#define MAX_SPEED 2
#define IR_SENSOR_NUM 6


  
  



  for (int i = 0; i < IR_SENSOR_NUM; i++) {
    irSensor[i] = robot->getDistanceSensor(irSensorName[i]);
    irSensor[i]->enable(TIME_STEP);
  }

  // PID constants
  double Kp = 1;
  double Ki = 0.001;
  double Kd = 0.01;
   integral = 0.01;
  double Last_error = 0.0;

  while (robot->step(TIME_STEP) != -1) {
    double sensorValue[IR_SENSOR_NUM];
    double error = 0.0;
    
    const double usfl_val = (USFL-> getValue())/10;
    const double usf_low_val = (USF_low-> getValue())/10;
    const double usal_val = (USAL-> getValue())/10;

    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
      sensorValue[i] = irSensor[i]->getValue();
      
      if (sensorValue[i] > 750){sensorValue[i] = 1;}
      else{sensorValue[i]=0;}
      std::cout << sensorValue[i]<< std::endl;
      error += sensorValue[i] * (i - 2.5);
    }

    // PID controller
    integral += error;
    double derivative = error - Last_error;
    double output = Kp * error + Ki * integral + Kd * derivative;
    Last_error = error;


         std::cout << output << std::endl;
    // Set motor speeds based on PID output
    double leftSpeed = MAX_SPEED - output;
    double rightSpeed = MAX_SPEED + output;
    left_motor->setVelocity(leftSpeed);
    left_motor->setVelocity(rightSpeed);
     std::cout << "leftSpeed" << leftSpeed << " rightSpeed " << rightSpeed << std::endl;
     
     
     
     
     
     
     //////////////  BOX PLACING ???????????????????

if ((usfl_val>usf_low_val+5)&&(usfl_val<25))
    { 
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    robot->step(5000);
    
    leftMotor->setVelocity(1);
    rightMotor->setVelocity(1);
    robot->step(3600);
      leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
     robot->step(2000);
    
    
     std::cout<< " ascsaascascascascasca  " <<std::endl;
     
      servoh->setPosition(0.07);
  robot->step(1000);
  
  servo->setPosition(0.105); // set to 90 degrees
  servo1->setPosition(-0.08);
     robot->step(1000);
     
      //servoh->setPosition(0);
  //robot->step(2000);
     

  servo->setPosition(0.095); // set to 90 degrees
  servo1->setPosition(-0.08);
     robot->step(1000);
     
     servoh->setPosition(0.1);
     robot->step(1000);
     
     servo->setPosition(0); // set to 90 degrees
  servo1->setPosition(0);
     robot->step(1000);
     
     leftMotor->setVelocity(-4);
    rightMotor->setVelocity(-4);
    robot->step(11000);
    
    
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
     robot->step(2000);
     
     leftMotor->setVelocity(1);
    rightMotor->setVelocity(-1);
     robot->step(7500*2);
     break;


    
    }
     
     
     
     
     

  }



while (robot->step(TIME_STEP) != -1) {
  
   const double usfl_val = (USFL-> getValue())/10;
    const double usf_low_val = (USF_low-> getValue())/10;
    const double usal_val = (USAL-> getValue())/10;
    
    
    
    
    
    const double USL1_val = (USL1-> getValue())/10;
    const double USR1_val = (USR1-> getValue())/10;
  
    double sensorValue[IR_SENSOR_NUM];
    double error = 0.0;

    // Read IR sensor values
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
      sensorValue[i] = irSensor[i]->getValue();
      
      if (sensorValue[i] > 750){sensorValue[i] = 1;}
      else{sensorValue[i]=0;}
      std::cout << sensorValue[i]<< std::endl;
      error += sensorValue[i] * (i - 2.5);
    }

    // PID controller
    integral += error;
    double derivative = error - last_error;
    double output = kp * error + ki * integral + kd * derivative;
    last_error = error;


         std::cout << output << std::endl;
    // Set motor speeds based on PID output
    double leftSpeed = MAX_SPEED + output;
    double rightSpeed = MAX_SPEED - output;
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
     std::cout << "leftSpeed" << leftSpeed << " rightSpeed " << rightSpeed << std::endl;

    std::cout<<usfl_val<<"   "<<usal_val<<std::endl; 
     if ((USL1_val<15)&&(USL1_val<15))
     {break;}
    
};


 left_motor->setVelocity(0);
    right_motor->setVelocity(0);
   robot->step(1000);
  
  left_motor->setVelocity(2);
    right_motor->setVelocity(2);
   robot->step(4000);
  
  
  
  
 
  servoh->setPosition(0.0);
  robot->step(1000);
   servo->setPosition(-0.01); // set to 90 degrees
  servo1->setPosition(-0.01);
     robot->step(1000);
     
  left_motor->setVelocity(-1);
    right_motor->setVelocity(1);
   robot->step(7500);
  
  
  left_motor->setVelocity(3);
    right_motor->setVelocity(3);
   robot->step(14000);
  
  left_motor->setVelocity(1);
    right_motor->setVelocity(-1);
   robot->step(7500*2);

  
  left_motor->setVelocity(0);
    right_motor->setVelocity(0);
 robot->step(1000);
 
   left_motor->setVelocity(2);
    right_motor->setVelocity(2);
 robot->step(3000);
 
  
  
  
  /////////////////////////////////////////NOMAL BOX TO BRIDGE///////////////////////////


  delete robot;
  return 0;
}
