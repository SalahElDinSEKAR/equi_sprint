#ifndef _MOTOR_H
#define _MOTOR_H
#include <Arduino.h>


class Motor
{
  public:
        Motor();

        // Measuring_speed
        void Encoder_init();

        // void (Motor::*MOVE[4])(int speed); 
        void Control(int l_spd, int r_spd);
        
        void Stop();
        void Forward(int speed);
        void Back(int speed);
        void Left(int speed);
        void Right(int speed);


  public:
          static unsigned long encoder_count_right_a;
          static unsigned long encoder_count_left_a;
          
  private:
  
          //Motor pin
          #define AIN1 7
          #define PWMA_LEFT 5
          #define BIN1 12
          #define PWMB_RIGHT 6
          #define STBY_PIN 8
          
          //Encoder measuring speed  pin
          #define ENCODER_LEFT_A_PIN 2
          #define ENCODER_RIGHT_A_PIN 18
  

  };







#endif
