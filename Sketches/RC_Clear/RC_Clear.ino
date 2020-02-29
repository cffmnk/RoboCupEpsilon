#include <epsilon.h>

eps::Robot r;

void setup() {
    r.init();
}

void loop() {
  for (int i = 0; i < 4; ++i) {
    r.setSpeed(i, 100);
  }
  delay(1000);
  for (int i = 0; i < 4; ++i) {
    r.setSpeed(i, -100);
  }
  delay(1000);
}
