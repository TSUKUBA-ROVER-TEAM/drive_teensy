#include "Arduino.h"
#include "C610Bus.h"
#include "velocity_pid.h"

VelocityPID robomas_1(1000.0, 0.0, 0.0, 1.0);
VelocityPID robomas_2(1000.0, 0.0, 0.0, 1.0);
VelocityPID robomas_3(1000.0, 0.0, 0.0, 1.0);
VelocityPID robomas_4(1000.0, 0.0, 0.0, 1.0);

int32_t target_current[4] = {0, 0, 0, 0};
double target_velocity[4] = {10.0, 20.0, 30.0, 15.0};

long last_time = 0;
C610Bus<CAN2> bus;

void setup() {
    Serial.begin(115200);
}

void loop() {
  bus.PollCAN();

  long now = millis();
  if (now - last_time >= 10) {
    target_current[0] = robomas_1.compute(
        target_velocity[0], bus.Get(0).Velocity(),
        (now - last_time) / 1000.0);
    target_current[1] = robomas_2.compute(
        target_velocity[1], bus.Get(1).Velocity(),
        (now - last_time) / 1000.0);
    target_current[2] = robomas_3.compute(
        target_velocity[2], bus.Get(2).Velocity(),
        (now - last_time) / 1000.0);
    target_current[3] = robomas_4.compute(
        target_velocity[3], bus.Get(3).Velocity(),
        (now - last_time) / 1000.0);
    bus.CommandTorques(target_current[0], target_current[1], target_current[2],
                       target_current[3], C610Subbus::kOneToFourBlinks);

    Serial.print("Velocity Motor 1: ");
    Serial.println(bus.Get(0).Velocity());
    Serial.print("Velocity Motor 2: ");
    Serial.println(bus.Get(1).Velocity());
    Serial.print("Velocity Motor 3: ");
    Serial.println(bus.Get(2).Velocity());
    Serial.print("Velocity Motor 4: ");
    Serial.println(bus.Get(3).Velocity());
    Serial.println("-----");

    last_time = now;
  }
}