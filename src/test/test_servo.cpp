#include "Arduino.h"
#include <Servo.h>

Servo steer_left_forward;
Servo steer_right_forward;
Servo steer_left_backward;
Servo steer_right_backward;

void setup()
{
    steer_left_forward.attach(28);
    steer_right_forward.attach(29);
    steer_left_backward.attach(8);
    steer_right_backward.attach(7);
}

void loop()
{
    steer_left_forward.write(150);
    steer_right_forward.write(150);
    steer_left_backward.write(30);
    steer_right_backward.write(30);
    delay(1000);

    steer_left_forward.write(90);
    steer_right_forward.write(90);
    steer_left_backward.write(90);
    steer_right_backward.write(90);
    delay(1000);

    steer_left_forward.write(30);
    steer_right_forward.write(30);
    steer_left_backward.write(150);
    steer_right_backward.write(150);
    delay(1000);
}