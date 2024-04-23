#include "Balanced_real.h"
#include "Wire.h"
#include "Motor_real.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"
#include "sbus.h"

MPU6050 MPU6050;
Mpu6050 Mpu6050;

Balanced Balanced;
KalmanFilter kalmanfilter;
Motor Motor;

int max_pwm = 600;
float angle_pre_kalman;






Balanced::Balanced()
{
  kp_balance = 0.0, kd_balance = 0.0;
  kp_speed = 0.0, ki_speed = 0.0; //kp speedd s enerve a 7.5 (c'était à cause du signe)
  kp_turn = 0.0, kd_turn = 0.0;
  offset_orientation=0.0;
}

void Balanced::Total_Control()
{
  
  if (not arret_moteur){
    //Serial.print("speed_output: ");
    //Serial.println(speed_control_output);

    //Serial.print("rotation_output: ");
    //Serial.println(rotation_control_output);

    if(angle_pre_kalman < -6){
      pwm_left = 1.0*(balance_control_output - speed_control_output - rotation_control_output);//Superposition of Vertical Velocity Steering Ring
      pwm_right = 1.0*(balance_control_output - speed_control_output + rotation_control_output);//Superposition of Vertical Velocity Steering Ring

    } else {
      pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
      pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring

    }
    
    pwm_left = constrain(pwm_left, -max_pwm, max_pwm);
    pwm_right = constrain(pwm_right, -max_pwm, max_pwm);
  
    while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
    { 
      Mpu6050.DataProcessing();
      Motor.Stop();
    }
    Motor.Control(pwm_left,pwm_right);
    //Serial.print("pwm_left: ");
    //Serial.println(pwm_left);
    //Serial.print("pwm_right: ");
    //Serial.println(pwm_right);
   }
   else{
    Motor.Stop();
  }
  
}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
                                                  Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
                                                  Motor::encoder_count_right_a;
  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}



void Balanced::Speed_control(int trans, int turn)//pour avancer et tourner en même temps
{
  setting_car_speed = trans;
  setting_turn_speed = turn;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(60);break;
    case BACK:
                  Back(60);break;
    case LEFT:
                  Left(75);break;
    case RIGHT:
                  Right(75);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}







void Balanced::Left(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = -speed;
}

void Balanced::PI_SpeedRing()
{
   //Serial.print("encoders (left | right): ");
   //Serial.print(encoder_left_pulse_num_speed);
   //Serial.print(" | ");
   //Serial.println(encoder_right_pulse_num_speed);

   //double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;//vitesse 
   double car_speed=encoder_right_pulse_num_speed;
  //  encoder_left_pulse_num_speed = 0;
  //  encoder_right_pulse_num_speed = 0;
   
   // Serial.print("car speed: ");
   // Serial.println(car_speed);

   speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
   speed_filter_old = speed_filter;

   // Serial.print("speed filter: ");
   // Serial.println(speed_filter);

   car_speed_integeral += speed_filter;
   car_speed_integeral += -setting_car_speed; 
   car_speed_integeral = constrain(car_speed_integeral, -100, 100);

    //car_speed = cqrte puissq
    //integrqteur

   // speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
   speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;

   //Serial.print("speed control output: ");
   //Serial.println(speed_control_output);
}

void Balanced::PD_VerticalRing()
{
  test_balanced=1;
  // Serial.print("Angle pre Kalman in balanced: ");
  // Serial.println(angle_pre_kalman);
  //la valeur de -6.5 a été trouvée empiriquement
  balance_control_output= kp_balance * (angle_pre_kalman - (-6.5)) + kd_balance * (kalmanfilter.Gyro_x - 0);
  // Serial.print("balanced_output: ");
  // Serial.println(balance_control_output);
  
}

void Balanced::PI_SteeringRing()
{  
  //  Serial.print("kp_turn: ");
  //  Serial.println(kp_turn);

  //  Serial.print("encoder_left_pulse_num_speed: ");
  //  Serial.println(encoder_left_pulse_num_speed);
  //  Serial.print("encoder_right_pulse_num_speed: ");
  //  Serial.println(encoder_right_pulse_num_speed);

   double car_turn=(encoder_left_pulse_num_speed - encoder_right_pulse_num_speed);
   rotation_control_output = kp_turn * (setting_turn_speed - car_turn); // + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
  //  Serial.print("car_turn: ");
  //  Serial.println(car_turn);
  //  Serial.print("rotation_control_output: ");
  //  Serial.println(rotation_control_output);
  //  Serial.print("setting_turn_speed: ");
  //  Serial.println(setting_turn_speed);
}


void Mpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
 }

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer

  angle_pre_kalman=kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
  // Serial.print("gx: ");
  // Serial.println(gx);
}



