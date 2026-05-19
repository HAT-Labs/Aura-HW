#include <mbed.h>

void setup() {
  static const PinName led = p27;
  mbed::PwmOut IRLED(led);
}

void loop() {
  // put your main code here, to run repeatedly:
}
