#include <Arduino.h>
#include <string.h>
#include "Motor_real.h"


Motor::Motor()
{
//  MOVE[0] = &Motor::Forward;
//  MOVE[1] = &Motor::Back;
//  MOVE[2] = &Motor::Left;
//  MOVE[3] = &Motor::Right;
}

void Motor::Stop()
{
  Serial2.print("!M "+ String(0)+" "+ String(0)+ "\r");
}

void Motor::Forward(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.println("!M "+ String(speed)+" "+ String(-speed));
}

void Motor::Back(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(-speed)+" "+ String(-speed)+ "\r");
}


void Motor::Left(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(speed)+" "+ String(0)+ "\r");
}

void Motor::Right(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(0)+" "+ String(speed)+ "\r");
}

void Motor::Control(int l_spd,int r_spd)
{
  Serial2.print("!M "+ String(l_spd)+" "+ String(r_spd)+ "\r");
}
