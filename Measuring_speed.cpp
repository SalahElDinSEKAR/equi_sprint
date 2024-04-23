#include "Motor_real.h"
#include "PinChangeInt.h"

static void EncoderCountRightA();
static void EncoderCountLeftA();

void Motor::Encoder_init()
{
  // Config en entrée des pins encodeurs
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), EncoderCountLeftA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), EncoderCountRightA, CHANGE);
  // attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

unsigned long Motor::encoder_count_right_a;
//Getting Right Wheel Speed.
static void EncoderCountRightA()
{
  // Serial.print("intRight");
  Motor::encoder_count_right_a++;
}


unsigned long Motor::encoder_count_left_a;
//Getting Left Wheel Speed.
static void EncoderCountLeftA()
{
  // Serial.print("intLeft");
  Motor::encoder_count_left_a++;
}
